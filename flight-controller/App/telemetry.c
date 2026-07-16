/* ============================================================
 * telemetry.c: LoRa downlink + pad-only command uplink.
 * Contract (CLAUDE.md §4): downlink-primary at a state-dependent
 * cadence; a short RX window opens ONLY on the pad between TX
 * frames; once in flight the FC is strictly TX-only.
 * Superloop context only.
 * ============================================================ */
#include "app.h"
#include <string.h>

#define NONCE_HIST 32               /* replay-dedup depth: a captured command
                                     * cannot be re-accepted for this many
                                     * distinct newer commands */
#define EVQ_LEN    8                /* PKT_EVENT queue depth (drop-oldest) */

static uint32_t s_last_tx_ms;
static bool     s_tx_pending;       /* radio_send issued, TxDone not seen  */
static bool     s_rx_open;          /* pad RX window currently armed       */
static uint32_t s_nonce[NONCE_HIST];
static uint8_t  s_nonce_idx;
static uint8_t  s_nonce_cnt;        /* valid entries (saturates at HIST)   */
static uint32_t s_last_reinit_ms;   /* pacing for on-ground radio re-init  */
static uint32_t s_reinit_attempts;
static uint32_t s_tx_timeouts;      /* stuck-TX force-clears (lost TxDone) */
static uint32_t s_tx_count;         /* frames handed to radio_send OK      */
static uint32_t s_rx_count;         /* frames pulled off the radio (any)   */
static bool     s_monitor;          /* live per-packet console print       */
static uint32_t s_cw_until_ms;      /* 0 = normal; else CW carrier auto-off */
static uint16_t s_seq;              /* v3: outbound frame counter (PDR)     */
static uint8_t  s_last_evt_state;   /* v3: state of most recent PKT_EVENT   */
static uint8_t  s_evt_count;        /* v3: total PKT_EVENTs emitted         */

/* Pending PKT_EVENT states. Free-running uint8_t indices (EVQ_LEN divides
 * 256); telem_poll drains at most one per pass, ahead of cadence frames. */
static uint8_t  s_evq[EVQ_LEN];
static uint8_t  s_evq_head, s_evq_tail;

static int send_packet_type(uint8_t type, uint8_t state_field);

void telem_init(void)
{
    s_last_tx_ms = 0;
    s_tx_pending = false;
    s_rx_open    = false;
    s_nonce_idx  = 0;
    s_nonce_cnt  = 0;
    memset(s_nonce, 0, sizeof s_nonce);
    s_last_reinit_ms  = 0;
    s_reinit_attempts = 0;
    s_tx_timeouts     = 0;
    s_tx_count = s_rx_count = 0;
    s_monitor      = false;
    s_cw_until_ms  = 0;
    s_evq_head = s_evq_tail = 0;
    s_seq = 0;
    s_last_evt_state = 0;
    s_evt_count = 0;
}

void telem_debug(uint32_t *tx_timeouts, uint32_t *reinit_attempts)
{
    if (tx_timeouts)     *tx_timeouts     = s_tx_timeouts;
    if (reinit_attempts) *reinit_attempts = s_reinit_attempts;
}

void telem_stats(uint32_t *tx, uint32_t *rx)
{
    if (tx) *tx = s_tx_count;
    if (rx) *rx = s_rx_count;
}

void telem_monitor(bool on)        { s_monitor = on; }
bool telem_monitor_active(void)    { return s_monitor; }
bool telem_cw_active(void)         { return s_cw_until_ms != 0; }

static const char *pkt_name(uint8_t t)
{
    switch (t) {
    case PKT_TELEMETRY: return "TELEM";
    case PKT_EVENT:     return "EVENT";
    case PKT_ACK:       return "ACK";
    case PKT_COMMAND:   return "CMD";
    default:            return "?";
    }
}

/* Bench carrier for supply-current / power checks. secs=0 stops. Suspends
 * normal traffic while up; telem_poll auto-stops at the deadline. Caller
 * (console) gates to pad + disarmed. */
int telem_cw(uint32_t secs)
{
    if (secs == 0) {
        if (s_cw_until_ms) { radio_cw(false); s_cw_until_ms = 0; datalog_event("CW OFF"); }
        return 0;
    }
    if (radio_cw(true) != 0) return -1;
    s_tx_pending = false;               /* abandon any in-flight normal TX */
    s_rx_open    = false;
    s_cw_until_ms = HAL_GetTick() + secs * 1000u;
    if (s_cw_until_ms == 0) s_cw_until_ms = 1;   /* 0 is the "off" sentinel */
    datalog_event("CW ON");
    return 0;
}

/* Force one telemetry frame out ASAP (manual "packets out" trigger).
 * 0 = sent, -1 = busy (CW up / TX already pending) or radio down. */
int telem_send_now(void)
{
    if (s_cw_until_ms || s_tx_pending) return -1;
    return send_packet_type(PKT_TELEMETRY, (uint8_t)g_fsm.state);
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
                              (g_fsm.sd_ok         ? FLAG_SD_OK     : 0) |
                              (g_fsm.chg_ok        ? FLAG_CHG_OK    : 0) |
                              (g_fsm.gps_ok        ? FLAG_GPS_OK    : 0) |
                              (g_fsm.chg.vbat_mv < ARM_MIN_VBAT_MV ? FLAG_LOW_BATT : 0) |
                              (pyro_deploy_failed() ? FLAG_DEPLOY_FAIL : 0));
    { float tc;
      p->mcu_temp_c10 = (mcu_temp_read(&tc) == 0) ? clamp_i16(tc * 10.0f)
                                                  : (int16_t)MCU_TEMP_NODATA; }
    p->seq            = s_seq++;                 /* v3: per-frame, wraps at 65535 */
    p->last_evt_state = s_last_evt_state;
    p->last_evt_count = s_evt_count;
    p->main_alt_m     = (uint16_t)(g_fsm.main_alt_m + 0.5f);
    p->crc16 = crc16_ccitt((const uint8_t *)p, sizeof *p - 2);
}

static int send_packet_type(uint8_t type, uint8_t state_field)
{
    telem_packet_t p;
    telem_build(&p);
    p.type  = type;
    p.state = state_field;
    p.crc16 = crc16_ccitt((const uint8_t *)&p, sizeof p - 2);
    s_rx_open = false;   /* radio_send SET_STANDBYs, so the RX window dies even on failure */
    int rc = radio_send((const uint8_t *)&p, sizeof p);
    if (rc == 0) {
        s_tx_pending = true;
        s_last_tx_ms = HAL_GetTick();
        s_tx_count++;
        if (s_monitor)
            console_printf("[tx] %s len=%u #%lu\r\n",
                           pkt_name(type), (unsigned)sizeof p,
                           (unsigned long)s_tx_count);
    }
    return rc;
}

void telem_event(uint8_t event_state)
{
    /* v3: latch for the telemetry echo so a lost PKT_EVENT still surfaces the
     * deploy/state on the next cadence frame. */
    s_last_evt_state = event_state;
    s_evt_count++;
    /* Always into the on-board log (works with no radio/GS in range)... */
    datalog_event(fsm_state_name((flight_state_t)event_state));
    /* ...and queue the PKT_EVENT downlink (drop-oldest when full). The
     * packet is built at drain time so it carries the pyro_fired /
     * continuity bits current at TX, not at enqueue. */
    if ((uint8_t)(s_evq_head - s_evq_tail) >= EVQ_LEN)
        s_evq_tail++;                            /* drop-oldest */
    s_evq[s_evq_head % EVQ_LEN] = event_state;
    s_evq_head++;
}

/* Anti-replay: reject an exact repeat of any of the last NONCE_HIST distinct
 * commands. Deliberately NOT a monotonic "reject <= max" floor, since the GS seeds
 * its nonce from a RANDOM 32-bit value on every restart (gs.ino send_cmd), so
 * a post-restart nonce can legitimately be lower than the last one seen; a
 * floor would permanently lock out a restarted GS. Only filled ring slots are
 * compared, so an incoming nonce of 0 is no longer falsely matched by an
 * empty (zero-initialised) slot. */
static bool nonce_fresh(uint32_t n)
{
    for (uint8_t i = 0; i < s_nonce_cnt; i++)
        if (s_nonce[i] == n)
            return false;
    s_nonce[s_nonce_idx] = n;
    s_nonce_idx = (uint8_t)((s_nonce_idx + 1) % NONCE_HIST);
    if (s_nonce_cnt < NONCE_HIST)
        s_nonce_cnt++;
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
            if (ack) {
                datalog_event("TEST FIRE (radio)");
                telem_event((uint8_t)g_fsm.state); /* post-decision: PKT_EVENT carries fired bits */
            }
        }
        break;
    case CMD_SET_MAIN_ALT: {
        float m = (float)c.arg / 100.0f;     /* arg is cm */
        /* Refused while ARMED: deploy params must not change on an armed
         * vehicle, and the flash persist is state-gated off then anyway,
         * so a RAM-only change ACKed as success would silently revert on
         * reboot. NAK also if the ground-state persist itself fails. */
        if (g_fsm.armed || m < 30.0f || m > 2000.0f) {
            ack = false;
        } else {
            g_fsm.main_alt_m = m;
            params_t pp;
            fsm_params_snapshot(&pp);
            if (param_save(&pp) != 0)
                ack = false;                 /* applied to RAM, not persisted */
        }
        break;
    }
    case CMD_ZERO_BARO:
        if (on_pad()) baro_zero(); else ack = false;
        break;
    case CMD_SET_TX_POWER:
        /* arg = signed dBm; radio_set_tx_power clamps to -9..22. Commands
         * are only received in a pad RX window, so this is inherently
         * pad-only (never in flight, where the FC is strictly TX-only). */
        ack = (radio_set_tx_power((int)(int32_t)c.arg) == 0);
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
         * hardware fix) recovers without a reboot. GROUND STATES ONLY:
         * radio_init() blocks ~250 ms, longer than the 150 ms launch
         * debounce, so it must never run in ST_ARMED or any flight state. */
        if ((g_fsm.state == ST_INIT || g_fsm.state == ST_GROUND_IDLE ||
             g_fsm.state == ST_LANDED) &&
            (now_ms - s_last_reinit_ms) >= RADIO_REINIT_PERIOD_MS) {
            s_last_reinit_ms = now_ms;
            s_reinit_attempts++;
            /* A marginal chip (half-broken joint) can stretch init well
             * past the nominal ~250 ms; this is deliberate ground-state
             * work, not a hang, so keep the IWDG fed around it. */
            wdg_refresh();
            if (radio_init() == 0) {
                s_tx_pending = false;      /* stale pre-failure state */
                s_rx_open    = false;
                g_fsm.radio_ok = radio_ok();
                datalog_event("RADIO REINIT OK");
            }
            wdg_refresh();
        }
        if (!g_fsm.radio_ok)
            return;
    }

    /* Bench CW carrier up: suspend all normal traffic until its deadline
     * (signed diff = wrap-safe). Nothing else touches the radio meanwhile. */
    if (s_cw_until_ms) {
        if ((int32_t)(now_ms - s_cw_until_ms) >= 0) {
            radio_cw(false);
            s_cw_until_ms = 0;
            datalog_event("CW AUTO-OFF");
        } else {
            return;
        }
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

    /* Drain any received command (pad only; RX never opens in flight). */
    if (s_rx_open) {
        uint8_t buf[64];
        int n = radio_rx_get(buf, sizeof buf);
        if (n > 0) {
            s_rx_open = false;
            s_rx_count++;
            if (s_monitor) {
                int rssi = 0, snr = 0;
                radio_last_rx_status(&rssi, &snr);
                console_printf("[rx] len=%d rssi=%d snr=%d #%lu\r\n",
                               n, rssi, snr, (unsigned long)s_rx_count);
            }
            handle_command(buf, n);
        }
    }

    /* Queued PKT_EVENTs drain first (at most one per poll) so state
     * transitions / pyro actions are never starved by cadence frames.
     * Peek-then-advance: a transient radio_send failure keeps the event
     * queued for retry; a few consecutive failures drop it so a half-dead
     * radio cannot starve cadence frames forever (SD log has it anyway). */
    if (!s_tx_pending && s_evq_head != s_evq_tail) {
        static uint8_t tries;
        uint8_t ev = s_evq[s_evq_tail % EVQ_LEN];
        if (send_packet_type(PKT_EVENT, ev) == 0 || ++tries > 3u) {
            s_evq_tail++;
            tries = 0;
        }
        return;
    }

    /* Cadenced telemetry frame. */
    if (!s_tx_pending && (now_ms - s_last_tx_ms) >= period_ms())
        (void)send_packet_type(PKT_TELEMETRY, (uint8_t)g_fsm.state);
}
