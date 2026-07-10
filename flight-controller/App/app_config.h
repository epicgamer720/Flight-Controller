#pragma once
/* ============================================================
 * app_config.h — tunable constants (thresholds per CLAUDE.md §5.4)
 * All flight thresholds must be re-tuned to the motor/airframe.
 * ============================================================ */

/* ---- Scheduling (superloop, HAL tick = 1 kHz) ---- */
#define CTRL_HZ              200     /* sensor sample + state machine */
#define BARO_HZ              50      /* BMP580 continuous-mode readout */
#define CONT_HZ              10      /* pyro continuity ADC poll */
#define CHARGER_HZ           1       /* BQ25883 poll */
#define LED_HZ               5
#define LOG_FLUSH_MS         500     /* f_sync cadence */

/* ---- Telemetry cadence (per CLAUDE.md §4) ---- */
#define TELEM_PERIOD_PAD_MS     1000
#define TELEM_PERIOD_FLIGHT_MS  100
#define TELEM_PERIOD_LANDED_MS  500   /* raised rate for recovery */
#define PAD_RX_WINDOW_MS        300   /* RX window after each pad TX; TX-only in flight */
#define TELEM_TX_TIMEOUT_MS     2000  /* force-clear a stuck TX (airtime ~40 ms) */
#define RADIO_REINIT_PERIOD_MS  5000  /* retry radio_init while dead — ground states only */

/* ---- Flight detection thresholds ---- */
#define LAUNCH_G             3.0f    /* |accel| sustained above this ... */
#define LAUNCH_DEBOUNCE_MS   150     /* ... for this long => BOOST */
#define LAUNCH_BARO_ALT_M    10.0f   /* OR baro AGL rising above this (accel may be saturated) */
#define BURNOUT_G            0.5f    /* axial accel below this ... */
#define BURNOUT_DEBOUNCE_MS  200     /* ... for this long => COAST */
#define MAX_BURN_MS          6000    /* backup: force COAST */
#define APOGEE_DEBOUNCE_N    10      /* consecutive 200 Hz samples with vel<=0 + alt falling */
#define APOGEE_TIMEOUT_MS    30000   /* backup timer from launch: force apogee event */
#define MAIN_ALT_M_DEFAULT   150.0f  /* main-deploy AGL (runtime-settable, CMD_SET_MAIN_ALT) */
#define LAND_VEL_MS          1.0f    /* |vertical vel| below this ... */
#define LAND_DEBOUNCE_MS     10000   /* ... with stable alt for this long => LANDED */
#define MIN_DROGUE_DELAY_MS  3000    /* never fire drogue sooner than this after launch */

/* ---- Pyro (per CLAUDE.md §2 — safety-critical) ---- */
#define FIRE_PULSE_MS        1000
#define FIRE_RETRY_MAX       1       /* single retry if continuity persists */
#define CONT_THRESH_MV       300     /* >this at PC1 (pin mV) = continuity */
#define TEST_FIRE_PASSCODE   0x52A5u /* upper 16 bits of CMD_TEST_FIRE arg; low 16 = channel */

/* ---- Sensors ---- */
#define ACCEL_FS_G           16      /* ISM330DHCX max; saturation => boolean-only in boost */
#define ACCEL_SAT_G          15.5f   /* any axis beyond this => FLAG_ACCEL_SAT */
#define GYRO_FS_DPS          2000
#define GYRO_CAL_SAMPLES     400     /* pad gyro bias cal (~2 s at 200 Hz), stationary */
#define SEA_LEVEL_PA_DEFAULT 101325.0f

/* ---- Kalman (1-D altitude/velocity, baro-driven) ---- */
#define KAL_Q_ACCEL          4.0f    /* process noise (m/s^2)^2 */
#define KAL_R_BARO           1.5f    /* baro meas noise m^2 */
#define KAL_GATE_SIGMA       5.0f    /* innovation gate: reject y^2 > N^2*S */
#define KAL_GATE_MAX_REJECT  25      /* ~0.5 s at 50 Hz, then accept (real step) */

/* ---- Logging ---- */
#define LOG_RECORD_MAGIC     0xA5
#define LOG_RATE_HZ          200
#define LOG_RING_BYTES       8192

/* ---- Arming gate (per CLAUDE.md §2): ALL required ---- */
#define ARM_MAX_VEL_MS       2.0f    /* near-zero velocity on pad */
#define ARM_MAX_TILT_G       0.5f    /* |a|-1g plausibility band */
