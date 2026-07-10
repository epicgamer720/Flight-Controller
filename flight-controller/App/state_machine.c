/* ============================================================
 * state_machine.c — flight state machine + sensor-fusion glue.
 * SAFETY-CRITICAL (CLAUDE.md §2, §5.4). Every pyro decision is made
 * here and only here; radio commands can never fire in flight.
 *
 * Runs at CTRL_HZ (200 Hz) from app_loop. Superloop context only —
 * nothing here is touched from an ISR, so g_fsm needs no volatile.
 * ============================================================ */
#include "app.h"
#include <math.h>
#include <string.h>

fsm_t g_fsm;

/* ---- derived scheduling constants ---- */
#define CTRL_DT_MS        (1000u / CTRL_HZ)              /* 5 ms          */
#define CTRL_DT_S         (1.0f / (float)CTRL_HZ)
#define BARO_DECIM        (CTRL_HZ / BARO_HZ)            /* every 4 steps */
#define GPS_DECIM         (CTRL_HZ / 5)                  /* gps copy 5 Hz */
#define LAUNCH_ACC_N      (LAUNCH_DEBOUNCE_MS / CTRL_DT_MS)   /* 30 */
#define LAUNCH_BARO_N     4        /* consecutive rising-AGL samples       */
#define BURNOUT_N         (BURNOUT_DEBOUNCE_MS / CTRL_DT_MS)  /* 40 */
#define INIT_OK_N         20       /* 100 ms of healthy sensors -> IDLE    */
#define IMU_FAIL_N        (CTRL_HZ / 2)   /* 0.5 s of consecutive failures */
#define BARO_FAIL_N       (BARO_HZ / 2)   /* 0.5 s of consecutive failures */
#define APOGEE_PEAK_MARGIN_M  0.25f /* "below peak" noise guard            */
#define LAND_ALT_BAND_M       2.0f  /* alt considered stable within this   */

/* ---- private filter + debounce state ---- */
static kalman_t s_kal;
static uint32_t s_step;
static uint16_t s_imu_fail, s_baro_fail;
static uint16_t s_init_ok_cnt;
static uint16_t s_launch_acc_cnt, s_launch_baro_cnt;
static uint16_t s_burnout_cnt;
static uint16_t s_apogee_cnt;
static float    s_peak_agl;
static bool     s_land_win;         /* landing debounce window open */
static uint32_t s_land_start_ms;
static float    s_land_ref_agl;

const char *fsm_state_name(flight_state_t s)
{
    static const char *const names[] = {
        "INIT", "GROUND_IDLE", "ARMED", "BOOST", "COAST",
        "APOGEE", "DROGUE", "MAIN", "DESCENT", "LANDED", "FAULT"
    };
    if ((unsigned)s >= sizeof(names) / sizeof(names[0]))
        return "?";
    return names[s];
}

/* Transition + PKT_EVENT + log entry (telem_event also writes the
 * datalog sidecar with the state name). */
static void fsm_set_state(flight_state_t s)
{
    if (g_fsm.state == s)
        return;
    g_fsm.state = s;
    telem_event((uint8_t)s);
}

static void fsm_fault(const char *why)
{
    if (g_fsm.state == ST_FAULT)
        return;
    g_fsm.armed = false;            /* safe pyros: pyro_fire ignores !armed */
    datalog_event(why);
    datalog_flush();                /* bounded drain + sync — FAULT may precede power loss */
    fsm_set_state(ST_FAULT);        /* logging + telemetry keep running */
}

void fsm_init(void)
{
    memset(&g_fsm, 0, sizeof g_fsm);
    g_fsm.state        = ST_INIT;
    g_fsm.main_alt_m   = MAIN_ALT_M_DEFAULT;
    g_fsm.test_enabled = false;     /* enabled only via console, never radio */
    kal_reset(&s_kal);
    s_step = 0;
    s_imu_fail = s_baro_fail = 0;
    s_init_ok_cnt = 0;
    s_launch_acc_cnt = s_launch_baro_cnt = 0;
    s_burnout_cnt = s_apogee_cnt = 0;
    s_peak_agl = 0.0f;
    s_land_win = false;
}

int fsm_request_arm(bool via_radio)
{
    /* Full arming gate per CLAUDE.md §2: explicit command + sensors
     * healthy + on-pad sanity. ALL conditions required. */
    if (g_fsm.state != ST_GROUND_IDLE)          return -1;
    if (!g_fsm.imu_ok || !g_fsm.baro_ok)        return -2;
    if (!imu_gyro_cal_done())                   return -3;  /* pad cal must finish */
    if (fabsf(g_fsm.vel_ms) >= ARM_MAX_VEL_MS)  return -4;
    float amag = sqrtf(g_fsm.imu.ax * g_fsm.imu.ax +
                       g_fsm.imu.ay * g_fsm.imu.ay +
                       g_fsm.imu.az * g_fsm.imu.az);
    if (g_fsm.imu.accel_sat)                    return -5;  /* impossible on pad */
    if (fabsf(amag - 1.0f) >= ARM_MAX_TILT_G)   return -5;

    g_fsm.armed = true;
    datalog_event(via_radio ? "ARM (radio)" : "ARM (console)");
    s_launch_acc_cnt = s_launch_baro_cnt = 0;
    fsm_set_state(ST_ARMED);        /* emits PKT_EVENT + state-name log */
    return 0;
}

void fsm_disarm(void)
{
    switch (g_fsm.state) {
    case ST_ARMED:
        g_fsm.armed = false;
        datalog_event("DISARM");
        fsm_set_state(ST_GROUND_IDLE);
        imu_gyro_cal_start();       /* fresh pad cal before any re-arm */
        break;
    case ST_INIT:
    case ST_GROUND_IDLE:
    case ST_LANDED:
    case ST_FAULT:
        g_fsm.armed = false;        /* ground states: just drop the flag */
        break;
    default:
        break;                      /* in flight: NEVER disarm autonomy */
    }
}

void fsm_step(uint32_t now_ms)
{
    /* ---- 1. sensors ---- */
    imu_sample_t s;
    if (imu_read(&s) == 0) {
        g_fsm.imu = s;
        s_imu_fail = 0;
    } else if (s_imu_fail < 0xFFFF) {
        s_imu_fail++;
    }
    g_fsm.imu_ok = imu_ok() && (s_imu_fail < IMU_FAIL_N);

    if ((s_step % BARO_DECIM) == 0) {
        float p, t;
        if (baro_read(&p, &t) == 0) {
            g_fsm.press_pa = p;
            g_fsm.temp_c   = t;
            s_baro_fail = 0;
            kal_update_baro(&s_kal, baro_altitude_m(p));
        } else if (s_baro_fail < 0xFFFF) {
            s_baro_fail++;
        }
        g_fsm.baro_ok = baro_ok() && (s_baro_fail < BARO_FAIL_N);
    }

    if ((s_step % GPS_DECIM) == 0)
        gps_get(&g_fsm.gps);
    /* g_fsm.chg is refreshed by app_loop's 1 Hz charger_poll. */

    /* ---- 2. filter ----
     * Conservative choice (CLAUDE.md §5.3): accel is NEVER used as the
     * Kalman control input (accel_valid = false always). The ±16 g part
     * saturates in boost and the vertical-axis approximation
     * (|a|-1g)*9.81 is only valid while ascending vertically; baro-only
     * prediction is strictly safer and apogee must not depend on accel. */
    kal_predict(&s_kal, 0.0f, CTRL_DT_S, false);
    g_fsm.agl_m  = s_kal.alt_m;
    g_fsm.vel_ms = s_kal.vel_ms;

    /* ---- 3. health -> FAULT (safe pyros, keep logging + telemetry) ---- */
    if (g_fsm.state != ST_FAULT &&
        (s_imu_fail >= IMU_FAIL_N || s_baro_fail >= BARO_FAIL_N)) {
        fsm_fault(s_imu_fail >= IMU_FAIL_N ? "FAULT: IMU LOST" : "FAULT: BARO LOST");
    }

    float amag = sqrtf(g_fsm.imu.ax * g_fsm.imu.ax +
                       g_fsm.imu.ay * g_fsm.imu.ay +
                       g_fsm.imu.az * g_fsm.imu.az);

    /* ---- 4. transitions (CLAUDE.md §5.4) ---- */
    switch (g_fsm.state) {

    case ST_INIT:
        if (g_fsm.imu_ok && g_fsm.baro_ok) {
            if (++s_init_ok_cnt >= INIT_OK_N) {
                imu_gyro_cal_start();   /* stationary pad bias cal */
                baro_zero();            /* current pressure = AGL 0 */
                kal_reset(&s_kal);
                fsm_set_state(ST_GROUND_IDLE);
            }
        } else {
            s_init_ok_cnt = 0;
        }
        break;

    case ST_GROUND_IDLE:
        /* Arming only via fsm_request_arm (radio CMD_ARM or console). */
        break;

    case ST_ARMED: {
        /* Launch detect: |a| > LAUNCH_G sustained, OR baro AGL rising
         * above LAUNCH_BARO_ALT_M. Accel saturation = "definitely
         * thrusting" (never a numeric value, CLAUDE.md §2/§5.3). */
        bool thrusting = g_fsm.imu.accel_sat || (amag > LAUNCH_G);
        s_launch_acc_cnt = thrusting ? (uint16_t)(s_launch_acc_cnt + 1) : 0;
        bool baro_rising = (g_fsm.agl_m > LAUNCH_BARO_ALT_M) && (g_fsm.vel_ms > 0.5f);
        s_launch_baro_cnt = baro_rising ? (uint16_t)(s_launch_baro_cnt + 1) : 0;

        if (s_launch_acc_cnt >= LAUNCH_ACC_N || s_launch_baro_cnt >= LAUNCH_BARO_N) {
            g_fsm.t_launch_ms = now_ms;
            s_peak_agl   = g_fsm.agl_m;
            s_burnout_cnt = 0;
            s_apogee_cnt  = 0;
            datalog_event("LAUNCH DETECT");
            fsm_set_state(ST_BOOST);
        }
        break;
    }

    case ST_BOOST: {
        if (g_fsm.agl_m > s_peak_agl)
            s_peak_agl = g_fsm.agl_m;
        /* Saturated accel = definitely thrusting, never counts as burnout. */
        bool low_acc = !g_fsm.imu.accel_sat && (amag < BURNOUT_G);
        s_burnout_cnt = low_acc ? (uint16_t)(s_burnout_cnt + 1) : 0;
        if (s_burnout_cnt >= BURNOUT_N ||
            (now_ms - g_fsm.t_launch_ms) >= MAX_BURN_MS) {
            s_apogee_cnt = 0;
            fsm_set_state(ST_COAST);
        }
        break;
    }

    case ST_COAST: {
        if (g_fsm.agl_m > s_peak_agl)
            s_peak_agl = g_fsm.agl_m;
        /* Apogee: vel <= 0 for APOGEE_DEBOUNCE_N consecutive samples AND
         * altitude below peak; backup timer from launch. NEVER before
         * MIN_DROGUE_DELAY_MS after launch (CLAUDE.md §2). */
        bool descending = (g_fsm.vel_ms <= 0.0f) &&
                          (g_fsm.agl_m < s_peak_agl - APOGEE_PEAK_MARGIN_M);
        s_apogee_cnt = descending ? (uint16_t)(s_apogee_cnt + 1) : 0;
        bool timeout = (now_ms - g_fsm.t_launch_ms) >= APOGEE_TIMEOUT_MS;

        if ((now_ms - g_fsm.t_launch_ms) >= MIN_DROGUE_DELAY_MS &&
            (s_apogee_cnt >= APOGEE_DEBOUNCE_N || timeout)) {
            fsm_set_state(ST_APOGEE);
            if (timeout && s_apogee_cnt < APOGEE_DEBOUNCE_N)
                datalog_event("APOGEE (backup timer)");
            /* Single pyro channel on this board = drogue / single deploy. */
            if (pyro_fire(0) == 0)
                datalog_event("PYRO0 FIRED (deploy)");
            else
                datalog_event("PYRO0 FIRE REFUSED");
        }
        break;
    }

    case ST_APOGEE:
        /* Fire happened on entry; move on next control period. */
        fsm_set_state(ST_DROGUE);
        break;

    case ST_DROGUE:
        /* Main-deploy point: descending AND AGL <= main_alt_m. This board
         * has NO second pyro channel (single AO3400A on PB13, already
         * fired at apogee) — so this is an event + log marker only,
         * no pyro action. Upward velocity inhibits per CLAUDE.md §2. */
        if (g_fsm.vel_ms < 0.0f && g_fsm.agl_m <= g_fsm.main_alt_m) {
            datalog_event("MAIN ALT (no 2nd pyro ch — marker only)");
            fsm_set_state(ST_MAIN);
        }
        break;

    case ST_MAIN:
        fsm_set_state(ST_DESCENT);  /* immediately */
        s_land_win = false;
        break;

    case ST_DESCENT:
        /* Landed: |vel| small and altitude stable for LAND_DEBOUNCE_MS. */
        if (!s_land_win) {
            s_land_win      = true;
            s_land_start_ms = now_ms;
            s_land_ref_agl  = g_fsm.agl_m;
        }
        if (fabsf(g_fsm.vel_ms) < LAND_VEL_MS &&
            fabsf(g_fsm.agl_m - s_land_ref_agl) < LAND_ALT_BAND_M) {
            if ((now_ms - s_land_start_ms) >= LAND_DEBOUNCE_MS) {
                g_fsm.armed = false;            /* safe for recovery crew */
                fsm_set_state(ST_LANDED);
                datalog_close();                /* flush + close log file */
            }
        } else {
            s_land_start_ms = now_ms;           /* restart window */
            s_land_ref_agl  = g_fsm.agl_m;
        }
        break;

    case ST_LANDED:
    case ST_FAULT:
    default:
        break;      /* terminal; telemetry/logging continue from app_loop */
    }

    s_step++;
}
