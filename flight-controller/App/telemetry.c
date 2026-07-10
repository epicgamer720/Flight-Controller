/* ============================================================
 * telemetry.c — LoRa downlink + pad-only command uplink.
 * Contract (CLAUDE.md §4): downlink-primary at a state-dependent
 * cadence; a short RX window opens ONLY on the pad between TX
 * frames; once in flight the FC is strictly TX-only.
 * Superloop context only.
 * ============================================================ */
#include "app.h"
#include <string.h>

#define NONCE_HIST 8

static uint32_t s_last_tx_ms;
static bool     s_tx_pending;       /* radio_send issued, TxDone not seen  */
static bool     s_rx_open;          /* pad RX window currently armed       */
static uint32_t s_nonce[NONCE_HIST];
static uint8_t  s_nonce_idx;
static uint32_t s_last_reinit_ms;   /* pacing for on-ground radio re-init  */
static uint32_t s_reinit_attempts;
static uint32_t s_tx_timeouts;      /* stuck-TX force-clears (lost TxDone) */

void telem_init(void)
{
    s_last_tx_ms = 0;
    s_tx_pending = false;
    s_rx_open    = false;
    s_nonce_idx  = 0;
    memset(s_nonce, 0, sizeof s_nonce);
    s_last_reinit_ms  = 0;
    s_reinit_attempts = 0;
    s_tx_timeouts     = 0;
}

void telem_debug(uint32_t *tx_timeouts, uint32_t *reinit_attempts)
{
    if (tx_timeouts)     *tx_timeouts     = s_tx_timeouts;
    if (reinit_attempts) *reinit_attempts = s_reinit_attempts;
}

static bool on_pad(void)
{
    return g_fsm.state == ST_INIT || g_fsm.state == ST_GROUND_IDLE ||
           g_fsm.state == ST_ARMED;
}

static uint32_t period_ms(void)
{
    switch (g_fsm.state) {
    case ST_BOOST: case ST_COAST: case ST_APOGEE:
    case ST_DROGUE: case ST_MAIN: case ST_DESCENT:
        return TELEM_PERIOD_FLIGHT_MS;
    case ST_LANDED:
        return TELEM_PERIOD_LANDED_MS;   /* raised GPS rate for recovery */
    default:
        return TELEM_PERIOD_PAD_MS;      /* pad states + FAULT */
    }
}

static int16_t clamp_i16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

void telem_build(telem_packet_t *p)
{
    memset(p, 0, sizeof *p);
    p->magic   = LINK_MAGIC;
    p->version = PROTO_VERSION;
    p->type    = PKT_TELEMETRY;
    p->state   = (uint8_t)g_fsm.state;
    p->t_ms    = HAL_GetTick();
    p->lat_e7  = g_fsm.gps.lat_e7;
    p->lon_e7  = g_fsm.gps.lon_e7;
    p->alt_gps_cm  = g_fsm.gps.alt_msl_cm;
    p->alt_baro_cm = (int32_t)(g_fsm.agl_m * 100.0f);
    p->vel_up_cms  = clamp_i16(g_fsm.vel_ms * 100.0f);
    p->accel_g_x100[0] = clamp_i16(g_fsm.imu.ax * 100.0f);
    p->accel_g_x100[1] = clamp_i16(g_fsm.imu.ay * 100.0f);
    p->accel_g_x100[2] = clamp_i16(g_fsm.imu.az * 100.0f);
    p->gyro_dps_x10[0] = clamp_i16(g_fsm.imu.gx * 10.0f);
    p->gyro_dps_x10[1] = clamp_i16(g_fsm.imu.gy * 10.0f);
    p->gyro_dps_x10[2] = clamp_i16(g_fsm.imu.gz * 10.0f);
    p->batt_mv    = g_fsm.chg.vbat_mv;
    p->pyro_cont  = pyro_cont_bits();
    p->pyro_fired = pyro_fired_bits();
    p->sats       = g_fsm.gps.sats;
    p->flags      = (uint8_t)((g_fsm.gps.fix       ? FLAG_GPS_FIX   : 0) |
                              (g_fsm.imu.accel_sat ? FLAG_ACCEL_SAT : 0) |
                              (g_fsm.armed         ? FLAG_ARMED     : 0) |
                              (g_fsm.sd_ok         ? FLAG_SD_OK     : 0));
    p->crc16 = crc16_ccitt((const uint8_t *)p, sizeof *p - 2);
}

static void send_packet_type(uint8_t type, uint8_t state_field)
{
    telem_packet_t p;
    telem_build(&p);
    p.type  = type;
    p.state = state_field;
    p.crc16 = crc16_ccitt((const uint8_t *)&p, sizeof p - 2);
    if (radio_send((const uint8_t *)&p, sizeof p) == 0) {
        s_tx_pending = true;
        s_rx_open    = false;
        s_last_tx_ms = HAL_GetTick();
    }
}

void telem_event(uint8_t event_state)
{
    /* Always into the on-board log (works with no radio/GS in range)... */
    datalog_event(fsm_state_name((flight_state_t)event_state));
    /* ...and best-effort immediate PKT_EVENT downlink. */
    if (g_fsm.radio_ok && !s_tx_pending)
        send_packet_type(PKT_EVENT, event_state);
}

static bool nonce_fresh(uint32_t n)
{
    for (int i = 0; i < NONCE_HIST; i++)
        if (s_nonce[i] == n)
            return false;
    s_nonce[s_nonce_idx] = n;
    s_nonce_idx = (uint8_t)((s_nonce_idx + 1) % NONCE_HIST);
    return true;
}

static void handle_command(const uint8_t *buf, int len)
{
    command_packet_t c;
    if (len != (int)sizeof c)
        return;
    memcpy(&c, buf, sizeof c);
    if (c.magic != LINK_MAGIC || c.version != PROTO_VERSION ||
        c.type != PKT_COMMAND)
        return;
    if (crc16_ccitt(buf, sizeof c - 2) != c.crc16)
        return;
    if (!nonce_fresh(c.nonce))
        return;                              /* anti-replay */

    bool ack = true;
    switch ((command_id_t)c.cmd) {
    case CMD_ARM:
        ack = (fsm_request_arm(true) == 0);
        break;
    case CMD_DISARM:
        fsm_disarm();
        break;
    case CMD_TEST_FIRE:
        /* Honored ONLY on pad, disarmed-but-test-enabled, passcode in the
         * upper half of arg, channel in the lower (CLAUDE.md §2). */
        ack = false;
        if (g_fsm.state == ST_GROUND_IDLE && !g_fsm.armed &&
            g_fsm.test_enabled &&
            (c.arg >> 16) == TEST_FIRE_PASSCODE &&
            (c.arg & 0xFFFFu) < NUM_PYRO) {
            ack = (pyro_fire((uint8_t)(c.arg & 0xFFFFu)) == 0);
            if (ack)
                datalog_event("TEST FIRE (radio)");
        }
        break;
    case CMD_SET_MAIN_ALT: {
        float m = (float)c.arg / 100.0f;     /* arg is cm */
        if (m >= 30.0f && m <= 2000.0f)
            g_fsm.main_alt_m = m;
        else
            ack = false;
        break;
    }
    case CMD_ZERO_BARO:
        if (on_pad()) baro_zero(); else ack = false;
        break;
    case CMD_REBOOT:
        send_packet_type(PKT_ACK, (uint8_t)g_fsm.state);
        HAL_Delay(100);
        NVIC_SystemReset();
        break;                               /* not reached */
    case CMD_ENTER_BOOTLOADER:
        if (g_fsm.state == ST_GROUND_IDLE && !g_fsm.armed) {
            send_packet_type(PKT_ACK, (uint8_t)g_fsm.state);
            HAL_Delay(100);
            datalog_close();
            dfu_enter_bootloader();          /* no return */
        }
        ack = false;
        break;
    case CMD_NOP:
    default:
        break;
    }

    if (ack)
        send_packet_type(PKT_ACK, (uint8_t)g_fsm.state);
}

void telem_poll(uint32_t now_ms)
{
    g_fsm.radio_ok = radio_ok();
    if (!g_fsm.radio_ok) {
        /* Dead radio: retry init periodically so a transient fault (or a
         * hardware fix) recovers without a reboot. GROUND STATES ONLY —
         * radio_init() blocks ~250 ms, longer than the 150 ms launch
         * debounce, so it must never run in ST_ARMED or any flight state. */
        if ((g_fsm.state == ST_INIT || g_fsm.state == ST_GROUND_IDLE ||
             g_fsm.state == ST_LANDED) &&
            (now_ms - s_last_reinit_ms) >= RADIO_REINIT_PERIOD_MS) {
            s_last_reinit_ms = now_ms;
            s_reinit_attempts++;
            if (radio_init() == 0) {
                s_tx_pending = false;      /* stale pre-failure state */
                s_rx_open    = false;
                g_fsm.radio_ok = radio_ok();
                datalog_event("RADIO REINIT OK");
            }
        }
        if (!g_fsm.radio_ok)
            return;
    }

    /* TX watchdog: a lost TxDone must not wedge the link forever. */
    if (s_tx_pending && (now_ms - s_last_tx_ms) >= TELEM_TX_TIMEOUT_MS) {
        s_tx_pending = false;              /* next send re-enters STANDBY */
        s_tx_timeouts++;
        datalog_event("TELEM TX TIMEOUT");
    }

    /* TX completion -> on pad, open the single RX window. */
    if (s_tx_pending && radio_tx_done()) {
        s_tx_pending = false;
        if (on_pad()) {
            if (radio_rx_start(PAD_RX_WINDOW_MS) == 0)
                s_rx_open = true;
        }
    }

    /* Drain any received command (pad only — RX never opens in flight). */
    if (s_rx_open) {
        uint8_t buf[64];
        int n = radio_rx_get(buf, sizeof buf);
        if (n > 0) {
            s_rx_open = false;
            handle_command(buf, n);
        }
    }

    /* Cadenced telemetry frame. */
    if (!s_tx_pending && (now_ms - s_last_tx_ms) >= period_ms())
        send_packet_type(PKT_TELEMETRY, (uint8_t)g_fsm.state);
}
