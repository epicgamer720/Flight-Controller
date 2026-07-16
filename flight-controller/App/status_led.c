/* ============================================================
 * status_led.c — single WS2812B-2020, data-in on PC13 via 100R.
 * Bit-banged with the DWT cycle counter at 216 MHz. BEST EFFORT:
 * PC13 is a low-drive (~2 MHz) pad, so edges are slow — timing is
 * biased toward the center of the WS2812 tolerance windows and may
 * still be marginal on some LED batches.
 * Frame = 24 bits GRB MSB-first, ~34 us with IRQs masked; the
 * superloop cadence guarantees the >300 us low latch gap.
 *
 * Bench effects (led_bench_set/led_bench_stop, driven by the console
 * `led` command): a second, higher-priority pattern source for use on
 * the bench over USB — solid/blink/breathe/rainbow/strobe colors and a
 * cycle-through-every-flight-state preview, plus a global brightness
 * scale. led_poll() force-clears any bench effect the instant `armed`
 * is true so a bench test can never mask the real ARMED indicator.
 * ============================================================ */
#include "app.h"

/* 216 MHz cycle counts: T1H ~700 ns, T0H ~350 ns, bit period 1250 ns */
#define WS_T1H_CYC   151u
#define WS_T0H_CYC    76u
#define WS_TBIT_CYC  270u

static uint8_t  s_last_rgb[3];
static bool     s_have_last;
static uint32_t s_override_until;    /* led_poll muted until this tick —
                                        bench `led <r> <g> <b>` test window */
static uint8_t  s_brightness_pct = 100;   /* global scale, 0-100 */

static led_bench_mode_t s_bench_mode;
static uint8_t  s_bench_r, s_bench_g, s_bench_b;
static uint16_t s_bench_period_ms = 1000;
static flight_state_t s_bench_state;

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
    uint8_t raw_r = r, raw_g = g, raw_b = b;   /* pre-scale, for change-tracking */
    uint32_t pct = s_brightness_pct;
    r = (uint8_t)(((uint16_t)r * pct) / 100u);
    g = (uint8_t)(((uint16_t)g * pct) / 100u);
    b = (uint8_t)(((uint16_t)b * pct) / 100u);

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

    s_last_rgb[0] = raw_r; s_last_rgb[1] = raw_g; s_last_rgb[2] = raw_b;
    s_have_last = true;
}

void led_set_brightness(uint8_t pct)
{
    if (pct > 100u) pct = 100u;
    s_brightness_pct = pct;
    s_have_last = false;    /* force the next led_set to redraw at the new scale */
}

uint8_t led_get_brightness(void) { return s_brightness_pct; }

void led_bench_set(led_bench_mode_t mode, uint8_t r, uint8_t g, uint8_t b, uint16_t period_ms)
{
    s_override_until = 0;                 /* a bench effect supersedes the raw-color test */
    s_bench_mode = mode;
    s_bench_r = r; s_bench_g = g; s_bench_b = b;
    s_bench_period_ms = (period_ms < 50u) ? 50u : period_ms;
}

void led_bench_set_state(flight_state_t st)
{
    s_override_until = 0;
    s_bench_mode = LED_BENCH_STATE;
    s_bench_state = st;
}

void led_bench_stop(void) { s_bench_mode = LED_BENCH_OFF; }
led_bench_mode_t led_bench_get(void) { return s_bench_mode; }

const char *led_bench_name(led_bench_mode_t m)
{
    switch (m) {
    case LED_BENCH_SOLID:   return "solid";
    case LED_BENCH_BLINK:   return "blink";
    case LED_BENCH_BREATHE: return "breathe";
    case LED_BENCH_RAINBOW: return "rainbow";
    case LED_BENCH_STROBE:  return "strobe";
    case LED_BENCH_CYCLE:   return "cycle";
    case LED_BENCH_STATE:   return "state";
    default:                return "off";
    }
}

/* Classic 8-bit hue wheel (0-255 -> RGB), full 255 amplitude. */
static void wheel_rgb(uint8_t pos, uint8_t *r, uint8_t *g, uint8_t *b)
{
    pos = 255 - pos;
    if (pos < 85) {
        *r = 255 - pos * 3; *g = 0; *b = pos * 3;
    } else if (pos < 170) {
        pos -= 85;
        *r = 0; *g = pos * 3; *b = 255 - pos * 3;
    } else {
        pos -= 170;
        *r = pos * 3; *g = 255 - pos * 3; *b = 0;
    }
}

/* The normal flight-state -> color mapping. Shared by led_poll() and the
 * bench LED_BENCH_CYCLE preview so the two never drift apart. */
static void state_color(flight_state_t st, bool armed, bool gps_fix,
                         uint32_t tick, uint8_t *r, uint8_t *g, uint8_t *b)
{
    bool strobe_on = (tick % LED_HZ) == 1u;
    *r = *g = *b = 0;

    switch (st) {
    case ST_INIT:
        *r = *g = *b = 16;                            /* dim white */
        break;
    case ST_GROUND_IDLE:
        if (armed) { *r = 64; }                       /* armed override: red */
        else if (gps_fix) {                           /* rainbow: disarmed w/ fix */
            uint8_t rr, gg, bb;
            wheel_rgb((uint8_t)(tick * 5u), &rr, &gg, &bb);
            *r = (uint8_t)(((uint16_t)rr * 48u) / 255u);
            *g = (uint8_t)(((uint16_t)gg * 48u) / 255u);
            *b = (uint8_t)(((uint16_t)bb * 48u) / 255u);
        }
        else { *g = 16; }                             /* dim green, no fix */
        break;
    case ST_ARMED:
        *r = 64;                                      /* red */
        break;
    case ST_BOOST:
    case ST_COAST:
        *r = 48; *b = 48;                             /* magenta (red family) */
        break;
    case ST_APOGEE:
    case ST_DROGUE:
        *g = 40; *b = 40;                             /* cyan: apogee / drogue out */
        break;
    case ST_MAIN:
    case ST_DESCENT:
        *r = 48; *g = 32;                             /* amber: main out / final descent */
        break;
    case ST_LANDED:
        if (strobe_on) *g = 255;                      /* bright green recovery strobe (found & safe) */
        break;
    case ST_FAULT:
    default:
        if (strobe_on) *r = 255;                      /* bright red recovery strobe (fault) */
        break;
    }
}

/* Bench pattern render: fills r,g,b for the active led_bench_mode_t at
 * time `now` (ms). Runs at the same LED_HZ cadence as the normal pattern
 * (called only from led_poll after its rate-limit gate). */
static void bench_color(uint32_t now, uint32_t tick, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint16_t period = s_bench_period_ms;
    *r = *g = *b = 0;

    switch (s_bench_mode) {
    case LED_BENCH_SOLID:
        *r = s_bench_r; *g = s_bench_g; *b = s_bench_b;
        break;
    case LED_BENCH_BLINK:
        if ((now % period) < (period / 2u)) { *r = s_bench_r; *g = s_bench_g; *b = s_bench_b; }
        break;
    case LED_BENCH_BREATHE: {
        uint32_t half = period / 2u;
        uint32_t phase = now % period;
        uint32_t level = (phase < half) ? (phase * 255u / half)
                                         : ((period - phase) * 255u / half);
        *r = (uint8_t)(((uint16_t)s_bench_r * level) / 255u);
        *g = (uint8_t)(((uint16_t)s_bench_g * level) / 255u);
        *b = (uint8_t)(((uint16_t)s_bench_b * level) / 255u);
        break;
    }
    case LED_BENCH_RAINBOW: {
        uint8_t pos = (uint8_t)(((now % period) * 255u) / period);
        wheel_rgb(pos, r, g, b);
        break;
    }
    case LED_BENCH_STROBE:
        if ((now % period) < 50u) { *r = s_bench_r; *g = s_bench_g; *b = s_bench_b; }
        break;
    case LED_BENCH_CYCLE: {
        uint32_t idx = (now / period) % 11u;          /* ST_INIT..ST_FAULT */
        state_color((flight_state_t)idx, false, true, tick, r, g, b);
        break;
    }
    case LED_BENCH_STATE:
        state_color(s_bench_state, false, true, tick, r, g, b);
        break;
    default:
        break;
    }
}

void led_poll(flight_state_t st, bool armed, bool sd_ok, bool gps_fix)
{
    static uint32_t s_next_ms;
    static uint32_t s_tick;
    uint32_t now = HAL_GetTick();

    if (armed) {                          /* a bench test must never mask ARMED */
        s_override_until = 0;
        s_bench_mode = LED_BENCH_OFF;
    }

    if ((int32_t)(now - s_override_until) < 0)
        return;                          /* bench raw-color test is showing */
    if ((int32_t)(now - s_next_ms) < 0)
        return;                          /* self rate-limit to LED_HZ */
    s_next_ms = now + (1000u / LED_HZ);
    s_tick++;

    uint8_t r, g, b;
    if (s_bench_mode != LED_BENCH_OFF) {
        bench_color(now, s_tick, &r, &g, &b);
    } else {
        state_color(st, armed, gps_fix, s_tick, &r, &g, &b);
        /* No SD: steal one tick per second for a blue flash */
        if (!sd_ok && (s_tick % LED_HZ) == 0u) {
            r = 0; g = 0; b = 48;
        }
    }

    /* Only rewrite the LED when the pattern actually changes */
    if (!s_have_last || r != s_last_rgb[0] || g != s_last_rgb[1] ||
        b != s_last_rgb[2])
        led_set(r, g, b);
}
