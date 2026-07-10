/* ============================================================
 * status_led.c — single WS2812B-2020, data-in on PC13 via 100R.
 * Bit-banged with the DWT cycle counter at 216 MHz. BEST EFFORT:
 * PC13 is a low-drive (~2 MHz) pad, so edges are slow — timing is
 * biased toward the center of the WS2812 tolerance windows and may
 * still be marginal on some LED batches.
 * Frame = 24 bits GRB MSB-first, ~34 us with IRQs masked; the
 * superloop cadence guarantees the >300 us low latch gap.
 * ============================================================ */
#include "app.h"

/* 216 MHz cycle counts: T1H ~700 ns, T0H ~350 ns, bit period 1250 ns */
#define WS_T1H_CYC   151u
#define WS_T0H_CYC    76u
#define WS_TBIT_CYC  270u

static uint8_t  s_last_rgb[3];
static bool     s_have_last;
static uint32_t s_override_until;    /* led_poll muted until this tick —
                                        bench `led` test command window */

void led_override(uint32_t ms)
{
    s_override_until = HAL_GetTick() + ms;   /* ms = 0 resumes patterns */
}

void led_init(void)
{
    /* Enable DWT->CYCCNT. Cortex-M7 gates DWT behind a lock register. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR    = 0xC5ACCE55u;           /* unlock (CM7 only) */
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    led_set(16, 16, 16);                 /* ST_INIT dim white */
}

void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    uint32_t pm  = __get_PRIMASK();

    __disable_irq();                     /* ~34 us: 24 bits @ 1.25 us */
    for (int i = 23; i >= 0; i--) {
        uint32_t hi = ((grb >> i) & 1u) ? WS_T1H_CYC : WS_T0H_CYC;
        uint32_t t0 = DWT->CYCCNT;
        LED_PORT->BSRR = LED_PIN;
        while ((DWT->CYCCNT - t0) < hi) { }
        LED_PORT->BSRR = (uint32_t)LED_PIN << 16;
        while ((DWT->CYCCNT - t0) < WS_TBIT_CYC) { }
    }
    if (!pm)
        __enable_irq();

    s_last_rgb[0] = r; s_last_rgb[1] = g; s_last_rgb[2] = b;
    s_have_last = true;
}

void led_poll(flight_state_t st, bool armed, bool sd_ok, bool gps_fix)
{
    static uint32_t s_next_ms;
    static uint32_t s_tick;
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - s_override_until) < 0)
        return;                          /* bench test color is showing */
    if ((int32_t)(now - s_next_ms) < 0)
        return;                          /* self rate-limit to LED_HZ */
    s_next_ms = now + (1000u / LED_HZ);
    s_tick++;

    uint8_t r = 0, g = 0, b = 0;
    bool slow_on = (s_tick % (2u * LED_HZ)) < LED_HZ; /* ~0.5 Hz blink */
    bool fast_on = (s_tick & 1u) != 0;                /* ~2.5 Hz blink */

    switch (st) {
    case ST_INIT:
        r = g = b = 16;                              /* dim white */
        break;
    case ST_GROUND_IDLE:
        if (armed) { r = 64; }                       /* armed override: red */
        else if (gps_fix) { g = 48; }                /* bright green w/ fix */
        else { g = 16; }                             /* dim green, no fix */
        break;
    case ST_ARMED:
        r = 64;                                      /* red */
        break;
    case ST_BOOST:
    case ST_COAST:
        r = 48; b = 48;                              /* magenta (red family) */
        break;
    case ST_APOGEE:
    case ST_DROGUE:
    case ST_MAIN:
    case ST_DESCENT:
        g = 40; b = 40;                              /* cyan */
        break;
    case ST_LANDED:
        if (slow_on) g = 48;                         /* slow green blink */
        break;
    case ST_FAULT:
    default:
        if (fast_on) r = 64;                         /* fast red blink */
        break;
    }

    /* No SD: steal one tick per second for a blue flash */
    if (!sd_ok && (s_tick % LED_HZ) == 0u) {
        r = 0; g = 0; b = 48;
    }

    /* Only rewrite the LED when the pattern actually changes */
    if (!s_have_last || r != s_last_rgb[0] || g != s_last_rgb[1] ||
        b != s_last_rgb[2])
        led_set(r, g, b);
}
