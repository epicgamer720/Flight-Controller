/* ============================================================
 * app_main.c — module bring-up + bare-metal superloop (CLAUDE.md §5.7).
 * Rates: control/log 200 Hz, LED 5 Hz, charger 1 Hz; GPS/console/
 * telemetry/pyro/SD polled every pass (all non-blocking).
 * ============================================================ */
#include "app.h"
#include <stdio.h>
#include <string.h>

#define CTRL_DT_MS (1000u / CTRL_HZ)

static uint32_t s_next_ctrl;
static uint32_t s_next_led;
static uint32_t s_next_chg;
static uint32_t s_next_sd;
static bool     s_sd_present;        /* card-detect edge tracking */

static IWDG_HandleTypeDef s_iwdg;

/* Read RCC->CSR reset flags into a short string, then clear them so the
 * next reset reports fresh causes. Call before anything else can reset. */
static void reset_cause_read(char *buf, size_t n)
{
    buf[0] = '\0';
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) strncat(buf, " LPWR", n - strlen(buf) - 1u);
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) strncat(buf, " WWDG", n - strlen(buf) - 1u);
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) strncat(buf, " IWDG", n - strlen(buf) - 1u);
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  strncat(buf, " SFT",  n - strlen(buf) - 1u);
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))  strncat(buf, " POR",  n - strlen(buf) - 1u);
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  strncat(buf, " BOR",  n - strlen(buf) - 1u);
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  strncat(buf, " PIN",  n - strlen(buf) - 1u);
    if (buf[0] == '\0')
        strncat(buf, " none", n - strlen(buf) - 1u);
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void app_init(void)
{
    char rst[40];
    reset_cause_read(rst, sizeof rst);   /* first: flags survive only until cleared */

    /* Console first so every later init can report over USB. */
    console_init();
    usb_device_init();
    led_init();
    led_set(8, 8, 8);                    /* dim white: booting */

    servo_init();
    int rc_pyro  = pyro_init();
    int rc_imu   = imu_init();
    int rc_baro  = baro_init();
    int rc_chg   = charger_init();
    int rc_radio = radio_init();
    int rc_gps   = gps_init();
    int rc_sd    = datalog_init();
    telem_init();
    fsm_init();                          /* zeroes g_fsm — flags set below */

    /* Persisted params (flash sector 7): override the compile-time default
     * if a valid record exists. ONLY main_alt_m — see param_store.c for
     * what must never be persisted. */
    {
        uint32_t alt_cm;
        if (param_load(&alt_cm) == 0) {
            g_fsm.main_alt_m = (float)alt_cm / 100.0f;
            char msg[40];
            snprintf(msg, sizeof msg, "PARAM main_alt=%lu cm",
                     (unsigned long)alt_cm);
            datalog_event(msg);
        }
    }

    g_fsm.imu_ok   = (rc_imu == 0);
    g_fsm.baro_ok  = (rc_baro == 0);
    g_fsm.sd_ok    = (rc_sd == 0) && datalog_ok();
    g_fsm.radio_ok = (rc_radio == 0) && radio_ok();
    g_fsm.chg_ok   = (rc_chg == 0);
    g_fsm.gps_ok   = (rc_gps == 0);

    /* IWDG after every slow init above: LSI 32 kHz nominal (up to ~47 kHz
     * spec) / 32 with full reload = ~4.1 s nominal, ~2.8 s worst case.
     * Refreshed once per app_loop() pass AND at the SD liveness point in
     * sd_diskio.c (a healthy-but-slow card can legally hold one FatFs call
     * for seconds — that is progress, not a hang). */
    s_iwdg.Instance       = IWDG;
    s_iwdg.Init.Prescaler = IWDG_PRESCALER_32;
    s_iwdg.Init.Reload    = 4095u;
    s_iwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    int rc_wdg = (HAL_IWDG_Init(&s_iwdg) == HAL_OK) ? 0 : -1;

    console_printf("\r\n=== FC boot ===\r\n");
    console_printf("reset:%s\r\n", rst);
    console_printf("imu:%s baro:%s chg:%s radio:%s(%s) gps:%s sd:%s pyro:%s wdg:%s\r\n",
                   rc_imu   == 0 ? "ok" : "FAIL",
                   rc_baro  == 0 ? "ok" : "FAIL",
                   rc_chg   == 0 ? "ok" : "FAIL",
                   rc_radio == 0 ? "ok" : "FAIL",
                   radio_using_tcxo() ? "tcxo" : "xtal",
                   rc_gps   == 0 ? "ok" : "FAIL",
                   g_fsm.sd_ok ? "ok" : "none",
                   rc_pyro  == 0 ? "ok" : "FAIL",
                   rc_wdg   == 0 ? "ok" : "FAIL");
    datalog_event("BOOT");
    {
        char msg[24];
        snprintf(msg, sizeof msg, "RST:%s", rst);
        datalog_event(msg);
    }
    if (rc_wdg != 0)
        datalog_event("WDG INIT FAIL");

    uint32_t now = HAL_GetTick();
    s_next_ctrl = now + CTRL_DT_MS;
    s_next_led  = now + (1000u / LED_HZ);
    s_next_chg  = now + 1000u;
    s_next_sd   = now + SD_RETRY_PERIOD_MS;
    s_sd_present = datalog_card_present();  /* boot attempt covered this
                                               insertion — no blind retry */
}

static void push_log_record(uint32_t now_ms)
{
    log_record_t r;
    memset(&r, 0, sizeof r);
    r.magic      = LOG_RECORD_MAGIC;
    r.state      = (uint8_t)g_fsm.state;
    r.flags      = (uint16_t)((g_fsm.gps.fix       ? FLAG_GPS_FIX   : 0) |
                              (g_fsm.imu.accel_sat ? FLAG_ACCEL_SAT : 0) |
                              (g_fsm.armed         ? FLAG_ARMED     : 0) |
                              (g_fsm.sd_ok         ? FLAG_SD_OK     : 0) |
                              (g_fsm.chg_ok        ? FLAG_CHG_OK    : 0) |
                              (g_fsm.gps_ok        ? FLAG_GPS_OK    : 0));
    r.t_ms       = now_ms;
    r.ax_g       = g_fsm.imu.ax;
    r.ay_g       = g_fsm.imu.ay;
    r.az_g       = g_fsm.imu.az;
    r.gx_dps     = g_fsm.imu.gx;
    r.gy_dps     = g_fsm.imu.gy;
    r.gz_dps     = g_fsm.imu.gz;
    r.press_pa   = g_fsm.press_pa;
    r.temp_c     = g_fsm.temp_c;
    r.agl_m      = g_fsm.agl_m;
    r.vel_ms     = g_fsm.vel_ms;
    r.lat_e7     = g_fsm.gps.lat_e7;
    r.lon_e7     = g_fsm.gps.lon_e7;
    r.alt_gps_cm = g_fsm.gps.alt_msl_cm;
    r.sats       = g_fsm.gps.sats;
    r.pyro_cont  = pyro_cont_bits();
    r.pyro_fired = pyro_fired_bits();
    r.batt_mv    = g_fsm.chg.vbat_mv;
    datalog_push(&r);                    /* datalog computes the CRC */
}

void app_loop(void)
{
    uint32_t now = HAL_GetTick();

    HAL_IWDG_Refresh(&s_iwdg);           /* worst single pass << 4.1 s timeout */

    /* 200 Hz control + logging tick (drift-free; resync if badly late) */
    if ((int32_t)(now - s_next_ctrl) >= 0) {
        if ((int32_t)(now - s_next_ctrl) > 100)
            s_next_ctrl = now;           /* SD hiccup etc. — resync, don't burst */
        s_next_ctrl += CTRL_DT_MS;
        fsm_step(now);
        push_log_record(now);
    }

    /* every pass, all non-blocking */
    gps_poll();
    pyro_poll();
    telem_poll(now);
    datalog_poll(now);
    console_poll();

    if ((int32_t)(now - s_next_led) >= 0) {
        s_next_led = now + (1000u / LED_HZ);
        g_fsm.sd_ok = datalog_ok();
        led_poll(g_fsm.state, g_fsm.armed, g_fsm.sd_ok, g_fsm.gps.fix);
        if (g_fsm.state == ST_GROUND_IDLE && !g_fsm.armed)
            baro_ground_track();         /* absorb weather/thermal drift */
    }

    if ((int32_t)(now - s_next_chg) >= 0) {
        s_next_chg = now + 1000u;
        charger_poll(&g_fsm.chg);
    }

    /* SD auto-retry: EDGE-TRIGGERED — one attempt per physical card
     * insertion (remove -> insert observed while running), never a blind
     * timer. A half-seated card can hang HAL_SD_Init past the IWDG
     * budget, and since RAM counters reset with each watchdog reboot, a
     * periodic retry turns one bad card into an infinite ~9 s reboot
     * loop (observed on the bench, 2026-07-10). Boot still attempts once;
     * a card that misses boot heals via reseat or console/deck `log
     * start`. GROUND STATES ONLY. */
    if ((int32_t)(now - s_next_sd) >= 0) {
        s_next_sd = now + SD_RETRY_PERIOD_MS;
        bool present = datalog_card_present();
        if (present && !s_sd_present && !g_fsm.sd_ok &&
            (g_fsm.state == ST_INIT || g_fsm.state == ST_GROUND_IDLE ||
             g_fsm.state == ST_LANDED)) {
            wdg_refresh();               /* deliberate ground-state work */
            if (datalog_init() == 0) {
                g_fsm.sd_ok = datalog_ok();
                datalog_event("SD REINIT OK");
            }
            wdg_refresh();
        }
        s_sd_present = present;
    }
}

/* Kick the watchdog from a known-liveness point outside app_loop (the SD
 * card-busy wait in sd_diskio.c). Safe before init: Instance still NULL. */
void wdg_refresh(void)
{
    if (s_iwdg.Instance != NULL)
        (void)HAL_IWDG_Refresh(&s_iwdg);
}
