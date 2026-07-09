/* ============================================================
 * app_main.c — module bring-up + bare-metal superloop (CLAUDE.md §5.7).
 * Rates: control/log 200 Hz, LED 5 Hz, charger 1 Hz; GPS/console/
 * telemetry/pyro/SD polled every pass (all non-blocking).
 * ============================================================ */
#include "app.h"
#include <string.h>

#define CTRL_DT_MS (1000u / CTRL_HZ)

static uint32_t s_next_ctrl;
static uint32_t s_next_led;
static uint32_t s_next_chg;

void app_init(void)
{
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

    g_fsm.imu_ok   = (rc_imu == 0);
    g_fsm.baro_ok  = (rc_baro == 0);
    g_fsm.sd_ok    = (rc_sd == 0) && datalog_ok();
    g_fsm.radio_ok = (rc_radio == 0) && radio_ok();

    console_printf("\r\n=== FC boot ===\r\n");
    console_printf("imu:%s baro:%s chg:%s radio:%s(%s) gps:%s sd:%s pyro:%s\r\n",
                   rc_imu   == 0 ? "ok" : "FAIL",
                   rc_baro  == 0 ? "ok" : "FAIL",
                   rc_chg   == 0 ? "ok" : "FAIL",
                   rc_radio == 0 ? "ok" : "FAIL",
                   radio_using_tcxo() ? "tcxo" : "xtal",
                   rc_gps   == 0 ? "ok" : "FAIL",
                   g_fsm.sd_ok ? "ok" : "none",
                   rc_pyro  == 0 ? "ok" : "FAIL");
    datalog_event("BOOT");

    uint32_t now = HAL_GetTick();
    s_next_ctrl = now + CTRL_DT_MS;
    s_next_led  = now + (1000u / LED_HZ);
    s_next_chg  = now + 1000u;
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
                              (g_fsm.sd_ok         ? FLAG_SD_OK     : 0));
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
    }

    if ((int32_t)(now - s_next_chg) >= 0) {
        s_next_chg = now + 1000u;
        charger_poll(&g_fsm.chg);
    }
}
