/* ============================================================
 * ism330dhcx.c — ST ISM330DHCX 6-axis IMU on shared SPI1, CS = PA4.
 * Part supports SPI mode 0 and mode 3; bus runs mode 0 (SX1262 mandate).
 * Config: XL 416 Hz ±16 g, G 416 Hz ±2000 dps (ODR > CTRL_HZ = 200).
 * FS_XL encoding on this family: 00=±2g, 01=±16g, 10=±4g, 11=±8g —
 * ±16 g is code '01', NOT '11'. FS_G: 11=2000 dps.
 * Sensitivities: 0.488 mg/LSB @ ±16 g, 70 mdps/LSB @ 2000 dps.
 * Polled from the superloop only — no ISR shares this state.
 * ============================================================ */
#include "app.h"
#include <math.h>
#include <string.h>

/* ---- Register map (DS: ISM330DHCX) ---- */
#define REG_WHO_AM_I    0x0F
#define WHO_AM_I_VAL    0x6B
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_CTRL3_C     0x12
#define REG_CTRL9_XL    0x18
#define REG_OUTX_L_G    0x22   /* gyro X/Y/Z then accel X/Y/Z, 12 B, int16 LE */

#define CTRL3_SW_RESET  0x01   /* self-clearing */
#define CTRL3_BDU_IFINC 0x44   /* BDU | IF_INC */
#define CTRL1_XL_VAL    0x64   /* ODR_XL=0110 (416 Hz), FS_XL=01 (±16 g) */
#define CTRL2_G_VAL     0x6C   /* ODR_G =0110 (416 Hz), FS_G =11 (2000 dps) */
#define CTRL9_XL_VAL    0xE2   /* DEN defaults (0xE0) + DEVICE_CONF=1
                                * (ST datasheet: recommended set during config) */

#define ACCEL_G_PER_LSB   0.000488f  /* 0.488 mg/LSB @ ±16 g */
#define GYRO_DPS_PER_LSB  0.070f     /* 70 mdps/LSB  @ 2000 dps */

static bool     s_init_ok;
static bool     s_read_ok;
static float    s_gbias[3];          /* dps, latched by pad calibration */
static bool     s_cal_active;
static bool     s_cal_done;
static uint32_t s_cal_n;
static float    s_cal_sum[3];
static float    s_cal_min[3], s_cal_max[3];  /* per-axis extent during cal */
static bool     s_cal_moved;         /* accel strayed from 1 g during cal    */
static uint8_t  s_prev_raw[12];      /* last burst, for freeze detection     */
static uint8_t  s_same_cnt;          /* consecutive byte-identical bursts    */
static bool     s_have_prev;

static int reg_write(uint8_t reg, uint8_t val)
{
    uint8_t hdr = reg & 0x7Fu;                       /* MSB clear = write */
    return spi_bus_txrx2(IMU_CS_PORT, IMU_CS_PIN, &hdr, 1, &val, NULL, 1);
}

static int reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t hdr = reg | 0x80u;   /* MSB set = read; IF_INC auto-increments */
    return spi_bus_txrx2(IMU_CS_PORT, IMU_CS_PIN, &hdr, 1, NULL, buf, len);
}

int imu_init(void)
{
    uint8_t v = 0;
    int i;

    s_init_ok = false;
    s_read_ok = false;

    for (i = 0; i < 3; i++) {                        /* WHO_AM_I, retry 3x */
        if (reg_read(REG_WHO_AM_I, &v, 1) == 0 && v == WHO_AM_I_VAL)
            break;
        HAL_Delay(10);
    }
    if (i == 3)
        return -1;                                   /* not found / wrong part */

    if (reg_write(REG_CTRL3_C, CTRL3_SW_RESET) != 0)
        return -2;
    HAL_Delay(50);                                   /* reset settle, generous */

    if (reg_write(REG_CTRL3_C,  CTRL3_BDU_IFINC) != 0) return -2;
    if (reg_write(REG_CTRL9_XL, CTRL9_XL_VAL)    != 0) return -2;
    if (reg_write(REG_CTRL1_XL, CTRL1_XL_VAL)    != 0) return -2;
    if (reg_write(REG_CTRL2_G,  CTRL2_G_VAL)     != 0) return -2;

    /* Read-back verify: catches MISO wiring / mode faults early. */
    if (reg_read(REG_CTRL1_XL, &v, 1) != 0 || v != CTRL1_XL_VAL) return -3;
    if (reg_read(REG_CTRL2_G,  &v, 1) != 0 || v != CTRL2_G_VAL)  return -3;

    s_init_ok = true;
    s_read_ok = true;   /* no read has failed yet */
    return 0;
}

static void cal_reset_accum(void)
{
    int i;
    for (i = 0; i < 3; i++) {
        s_cal_sum[i] =  0.0f;
        s_cal_min[i] =  1.0e9f;
        s_cal_max[i] = -1.0e9f;
    }
    s_cal_n     = 0;
    s_cal_moved = false;
}

int imu_read(imu_sample_t *s)
{
    uint8_t raw[12];
    int16_t w[6];
    float   g[3], a[3];
    int     i;

    if (!s_init_ok || s == NULL)
        return -1;

    if (reg_read(REG_OUTX_L_G, raw, sizeof raw) != 0) {
        s_read_ok = false;
        return -2;
    }

    /* A stuck bus returns HAL_OK with rail bytes. Real bursts always have
     * structure (accel Z at rest ≈ +1 g ≈ 0x0800 LSB; gyro noise is never
     * exactly 0 on all 12 bytes) — all-0x00 / all-0xFF is a bus fault, so
     * count it as a failed read and keep the previous outputs. */
    {
        uint8_t and_all = 0xFFu, or_all = 0x00u;
        for (i = 0; i < (int)sizeof raw; i++) {
            and_all &= raw[i];
            or_all  |= raw[i];
        }
        if ((or_all == 0x00u) || (and_all == 0xFFu)) {
            s_read_ok = false;
            return -3;
        }
    }

    /* Freeze detect: a stuck bus/sensor can return HAL_OK with a fixed
     * non-rail value. BDU is on and live gyro noise never repeats, so N
     * byte-identical bursts in a row is a fault — fail the read (feeds the
     * s_imu_fail -> FAULT path) and keep the previous outputs. */
    if (s_have_prev && memcmp(raw, s_prev_raw, sizeof raw) == 0) {
        if (s_same_cnt < 0xFFu) s_same_cnt++;
        if (s_same_cnt >= IMU_FREEZE_N) {
            s_read_ok = false;
            return -4;
        }
    } else {
        s_same_cnt = 0;
    }
    memcpy(s_prev_raw, raw, sizeof raw);
    s_have_prev = true;

    s_read_ok = true;

    for (i = 0; i < 6; i++)
        w[i] = (int16_t)((uint16_t)raw[2 * i] | ((uint16_t)raw[2 * i + 1] << 8));

    for (i = 0; i < 3; i++) {
        g[i] = (float)w[i]     * GYRO_DPS_PER_LSB;   /* gyro first in burst */
        a[i] = (float)w[i + 3] * ACCEL_G_PER_LSB;
    }

    if (s_cal_active) {                              /* stationary pad cal */
        /* Reject a cal taken while the vehicle is handled/moving: track per-
         * axis gyro peak-to-peak and accel-magnitude stray; if either is out
         * of band at the end of the window, discard and restart rather than
         * latching a bad bias. Arm precondition (-3) never clears until a
         * genuinely stationary cal completes. */
        float amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if (fabsf(amag - 1.0f) > GYRO_CAL_ACC_BAND_G)
            s_cal_moved = true;
        for (i = 0; i < 3; i++) {
            s_cal_sum[i] += g[i];
            if (g[i] < s_cal_min[i]) s_cal_min[i] = g[i];
            if (g[i] > s_cal_max[i]) s_cal_max[i] = g[i];
        }
        if (++s_cal_n >= GYRO_CAL_SAMPLES) {
            bool steady = !s_cal_moved;
            for (i = 0; i < 3; i++)
                if ((s_cal_max[i] - s_cal_min[i]) > GYRO_CAL_MAX_PP_DPS)
                    steady = false;
            if (steady) {
                for (i = 0; i < 3; i++)
                    s_gbias[i] = s_cal_sum[i] / (float)s_cal_n;
                s_cal_active = false;
                s_cal_done   = true;
            } else {
                cal_reset_accum();               /* moved: try again */
            }
        }
    }

    s->gx = g[0] - s_gbias[0];
    s->gy = g[1] - s_gbias[1];
    s->gz = g[2] - s_gbias[2];
    s->ax = a[0];
    s->ay = a[1];
    s->az = a[2];
    /* Railed axis => boolean-only during boost (CLAUDE.md §5.3). */
    s->accel_sat = (fabsf(a[0]) > ACCEL_SAT_G) ||
                   (fabsf(a[1]) > ACCEL_SAT_G) ||
                   (fabsf(a[2]) > ACCEL_SAT_G);
    return 0;
}

void imu_gyro_cal_start(void)
{
    cal_reset_accum();
    s_cal_done   = false;
    s_cal_active = true;    /* accumulation happens inside imu_read() */
}

bool imu_gyro_cal_done(void)
{
    return s_cal_done;
}

bool imu_ok(void)
{
    return s_init_ok && s_read_ok;
}
