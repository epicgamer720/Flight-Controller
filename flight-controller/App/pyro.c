/* ============================================================
 * pyro.c — SAFETY CRITICAL (CLAUDE.md §2).
 * One channel: gate PB13 (low-side AO3400A), continuity sense
 * PC1 = ADC1_IN11 via 100K/10K divider (ratio 1/11, battery side).
 * Gate boots LOW before any other init (pyro_boot_safe, called as
 * the first line of main()). Firing only via pyro_fire(), gated on
 * g_fsm.armed or pad-test mode. Continuity sensing NEVER pulses
 * the gate — it is a passive ADC read of the divider.
 *
 * Also hosts mcu_temp_read(): the internal die-temp sensor shares ADC1,
 * so the one module that owns the ADC channel state does the borrow.
 * ============================================================ */
#include "app.h"
#include "stm32f7xx_ll_adc.h"   /* __LL_ADC_CALC_TEMPERATURE + factory cal */

/* Fire-pulse state machine. After the pulse ends the low side of the
 * e-match needs a couple of fresh CONT_HZ ADC samples before the retry
 * decision (during the pulse the FET holds the sense node at ~0 V). */
typedef enum { PY_IDLE = 0, PY_FIRING, PY_SETTLE } py_fire_state_t;

#define PY_SETTLE_MS  (2u * (1000u / CONT_HZ))   /* 200 ms: >=2 fresh samples */
#define CONT_PERIOD_MS (1000u / CONT_HZ)

static py_fire_state_t s_fire_state = PY_IDLE;
static uint8_t  s_fire_ch;
static uint32_t s_fire_t0;
static uint8_t  s_retries;
static bool     s_backup_fire;   /* current pulse is a fault-independent backup:
                                  * pyro_poll must NOT abort/inhibit it on !armed */

static uint16_t s_pin_mv[NUM_PYRO];      /* cached pin-side mV at PC1 */
static bool     s_cont[NUM_PYRO];
static uint8_t  s_fired_bits;
static bool     s_deploy_failed;         /* continuity survived fire + retry */
static uint32_t s_last_adc_ms;

/* ---- boot-safe: runs BEFORE HAL_Init/SystemClock — raw registers only ---- */
void pyro_boot_safe(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;                  /* dummy read: clock settle before GPIO access */

    /* ODR bit LOW FIRST so the pin drives 0 the instant it becomes an output */
    GPIOB->BSRR = (uint32_t)PYRO1_GATE_PIN << 16;                    /* PB13 = 0 */
    GPIOB->MODER  = (GPIOB->MODER  & ~(3u << (13 * 2))) | (1u << (13 * 2)); /* output */
    GPIOB->OTYPER &= ~(1u << 13);                                    /* push-pull */
    GPIOB->PUPDR  = (GPIOB->PUPDR  & ~(3u << (13 * 2))) | (2u << (13 * 2)); /* pulldown */
}

static void gate_write(uint8_t ch, bool on)
{
    if (ch == 0)
        HAL_GPIO_WritePin(PYRO1_GATE_PORT, PYRO1_GATE_PIN,
                          on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Per CLAUDE.md §2: fire allowed only when armed, or on-pad test mode. */
static bool fire_allowed(void)
{
    return g_fsm.armed ||
           (g_fsm.state == ST_GROUND_IDLE && g_fsm.test_enabled);
}

int pyro_init(void)
{
    /* F7 ADC has no software calibration procedure; do one throwaway
     * conversion to verify the peripheral + channel work. */
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return -1;
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        return -2;
    }
    (void)HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    return 0;
}

static int adc_read_pin_mv(uint16_t *mv)
{
    uint32_t raw;
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return -1;
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        return -1;
    }
    raw = HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    *mv = (uint16_t)((raw * 3300u) / 4095u);
    return 0;
}

void pyro_poll(void)
{
    uint32_t now = HAL_GetTick();

    /* Continuity ADC at CONT_HZ (passive divider read — never touches gate) */
    if ((now - s_last_adc_ms) >= CONT_PERIOD_MS) {
        s_last_adc_ms = now;
        uint16_t mv;
        if (adc_read_pin_mv(&mv) == 0) {          /* on ADC error keep last */
            s_pin_mv[0] = mv;
            s_cont[0]   = (mv > CONT_THRESH_MV);
        }
    }

    /* A deploy pulse aborts early ONLY on a GROUND disarm of a pad test-fire.
     * In flight the sole thing that clears armed is a FAULT (fsm_fault), during
     * which a deploy pulse MUST run to completion + retry — truncating it on the
     * fault was a lawn-dart hole (the apogee charge could get a partial pulse and
     * the backup was then inhibited by s_deploy_fired). A backup pulse never
     * aborts either. */
    bool abort_pulse = !s_backup_fire && !fire_allowed() &&
                       (g_fsm.state == ST_GROUND_IDLE || g_fsm.state == ST_INIT);

    /* Fire-pulse timing checked every pass for prompt pulse termination */
    switch (s_fire_state) {
    case PY_FIRING:
        if (abort_pulse) {                        /* pad disarm mid-test-fire */
            gate_write(s_fire_ch, false);
            s_fire_state = PY_IDLE;
            s_backup_fire = false;
            break;
        }
        if ((now - s_fire_t0) >= FIRE_PULSE_MS) {
            gate_write(s_fire_ch, false);
            s_fire_t0 = now;
            s_fire_state = PY_SETTLE;
        }
        break;

    case PY_SETTLE:
        if ((now - s_fire_t0) >= PY_SETTLE_MS) {
            if (s_cont[s_fire_ch] && s_retries < FIRE_RETRY_MAX && !abort_pulse) {
                s_retries++;                      /* match survived: one retry */
                gate_write(s_fire_ch, true);
                s_fire_t0 = now;
                s_fire_state = PY_FIRING;
            } else {
                /* Retries exhausted (or inhibited). If continuity is still
                 * present the charge never went — latch a failed-deploy so
                 * telemetry (FLAG_DEPLOY_FAIL) and the log surface it. */
                if (s_cont[s_fire_ch]) {
                    s_deploy_failed = true;
                    datalog_event("PYRO DEPLOY FAILED");
                }
                s_fire_state = PY_IDLE;
                s_backup_fire = false;
            }
        }
        break;

    default:
        break;
    }
}

int pyro_fire(uint8_t ch)
{
    if (ch >= NUM_PYRO)
        return -1;
    if (!fire_allowed())
        return -2;                                /* gate untouched */
    if (s_fire_state != PY_IDLE)
        return -3;                                /* pulse already in progress */

    s_fire_ch     = ch;
    s_retries     = 0;
    s_backup_fire = false;
    s_fired_bits |= (uint8_t)(1u << ch);
    s_fire_t0     = HAL_GetTick();
    gate_write(ch, true);
    s_fire_state  = PY_FIRING;
    return 0;
}

/* Fault-independent backup deploy: fire WITHOUT the armed/test-mode gate.
 * SAFETY (CLAUDE.md §2): this is the ONE controlled bypass of fire_allowed(),
 * called ONLY by the state machine's backup-deploy timer and only once (guarded
 * by s_deploy_fired there). Still refuses to interrupt an in-progress pulse, so
 * it can never double-fire on top of the normal apogee deploy. */
int pyro_fire_backup(uint8_t ch)
{
    if (ch >= NUM_PYRO)
        return -1;
    if (s_fire_state != PY_IDLE)
        return -3;                                /* something already firing */
    s_fire_ch     = ch;
    s_retries     = 0;
    s_backup_fire = true;                         /* pyro_poll won't abort on !armed */
    s_fired_bits |= (uint8_t)(1u << ch);
    s_fire_t0     = HAL_GetTick();
    gate_write(ch, true);
    s_fire_state  = PY_FIRING;
    return 0;
}

bool pyro_continuity(uint8_t ch)
{
    return (ch < NUM_PYRO) ? s_cont[ch] : false;
}

uint8_t pyro_cont_bits(void)
{
    uint8_t bits = 0;
    for (uint8_t i = 0; i < NUM_PYRO; i++)
        if (s_cont[i])
            bits |= (uint8_t)(1u << i);
    return bits;
}

uint8_t pyro_fired_bits(void)
{
    return s_fired_bits;
}

bool pyro_deploy_failed(void)
{
    return s_deploy_failed;
}

uint16_t pyro_sense_mv(uint8_t ch)
{
    if (ch >= NUM_PYRO)
        return 0;
    /* battery-side estimate: pin mV * divider ratio (max 3300*11 fits u16) */
    return (uint16_t)((float)s_pin_mv[ch] * PYRO_SENSE_DIV);
}

/* Borrow ADC1 for one internal die-temp conversion, then restore the
 * pyro-sense channel. Superloop-serialized with pyro_poll's own reads, so no
 * lock needed. F7 factory cal (device-correct addresses via the LL macro);
 * assumes VDDA ~= 3.3 V. Returns 0 and writes *temp_c on success. */
int mcu_temp_read(float *temp_c)
{
    ADC_ChannelConfTypeDef sc = { .Rank = ADC_REGULAR_RANK_1,
                                  .SamplingTime = ADC_SAMPLETIME_480CYCLES };
    int rc = -1;

    sc.Channel = ADC_CHANNEL_TEMPSENSOR;          /* HAL also sets TSVREFE */
    if (HAL_ADC_ConfigChannel(&hadc1, &sc) == HAL_OK &&
        HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            uint32_t raw = HAL_ADC_GetValue(&hadc1);
            if (temp_c)
                *temp_c = (float)__LL_ADC_CALC_TEMPERATURE(3300u, raw,
                                                           LL_ADC_RESOLUTION_12B);
            rc = 0;
        }
        (void)HAL_ADC_Stop(&hadc1);
    }

    sc.Channel = PYRO1_SENSE_ADC_CH;              /* restore for pyro_poll */
    (void)HAL_ADC_ConfigChannel(&hadc1, &sc);
    return rc;
}
