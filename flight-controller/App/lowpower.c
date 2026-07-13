/* ============================================================
 * lowpower.c — `sleep` console command: safe STOP-mode low power.
 *
 * Why STOP (not STANDBY): STOP retains GPIO state, so the pyro gate
 * (PB13) stays *driven* LOW exactly as the boot sequence left it — the
 * project's #1 rule (CLAUDE.md §2). (The 10K pulldown R28 backs it up
 * even if the pin ever went high-Z, but we never rely on that alone.)
 * STANDBY would reset the GPIO config → floating gate → forbidden.
 *
 * The IWDG keeps counting in STOP and cannot be stopped once started, so
 * a naive STOP would be reset in ~2.8 s. Instead we STOP in short hops:
 * the RTC (LSE, precise) wakes us every ~2 s, we pet the IWDG, and either
 * re-enter STOP or — when the countdown expires — do a clean reset. A
 * reset is the safest possible resume: app_init() re-runs the boot-safe
 * pyro init from scratch. RESET (SW1) wakes it at any time in hardware.
 *
 * Caller (console cmd_sleep) gates to GROUND_IDLE/INIT + disarmed.
 * ============================================================ */
#include "app.h"

static RTC_HandleTypeDef s_rtc;

/* RTC wakeup fires on EXTI line 22 -> this vector (weak default overridden).
 * The handler just needs to run so WFI returns; HAL clears the flags. */
void RTC_WKUP_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&s_rtc);
}

/* Bring up LSE + RTC (once; backup domain persists across the soft reset).
 * LSE start can take ~1-2 s — pet the IWDG around it. */
static int rtc_setup(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    wdg_refresh();
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    osc.PLL.PLLState   = RCC_PLL_NONE;
    osc.LSEState       = RCC_LSE_ON;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return -1;
    wdg_refresh();

    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    pclk.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) return -2;
    __HAL_RCC_RTC_ENABLE();

    s_rtc.Instance            = RTC;
    s_rtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    s_rtc.Init.AsynchPrediv   = 127;
    s_rtc.Init.SynchPrediv    = 255;      /* 32768/(128*256) = 1 Hz ck_spre */
    s_rtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    s_rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    s_rtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&s_rtc) != HAL_OK) return -3;

    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    return 0;
}

/* seconds = 0 -> sleep until RESET (SW1). Never returns (resets on wake). */
int lowpower_sleep(uint32_t seconds)
{
    if (rtc_setup() != 0)
        return -1;                 /* caller stays awake; nothing changed */

    radio_cw(false);               /* park SX1262 in standby: DIO1 quiet, less draw */
    HAL_SuspendTick();             /* SysTick IRQ would wake STOP immediately */

    const uint32_t TICK_S = 2;     /* < ~2.8 s worst-case IWDG timeout */
    uint32_t remaining = seconds;
    for (;;) {
        wdg_refresh();             /* full IWDG budget before each STOP hop */
        /* ck_spre (1 Hz): period = (counter + 1) s -> 1 gives 2 s */
        if (HAL_RTCEx_SetWakeUpTimer_IT(&s_rtc, TICK_S - 1,
                RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
            break;                 /* fail safe: reset out below */

        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

        /* --- woke: RTC tick (or any stray EXTI); clock is now HSI 16 MHz --- */
        HAL_RTCEx_DeactivateWakeUpTimer(&s_rtc);
        if (seconds != 0) {
            remaining = (remaining > TICK_S) ? (remaining - TICK_S) : 0;
            if (remaining == 0)
                break;
        }
    }

    NVIC_SystemReset();            /* clean, boot-safe resume */
    return 0;                      /* not reached */
}
