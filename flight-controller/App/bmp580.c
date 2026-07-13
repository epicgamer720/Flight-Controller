/* ============================================================
 * bmp580.c — Bosch BMP580 barometer on I2C1 (polled, non-blocking)
 *
 * BMP580 shares the BMP581 register map (Bosch BMP5 driver family):
 *   0x01 CHIP_ID (0x50 primary / 0x51 secondary variant)
 *   0x15 INT_SOURCE   0x1D..0x1F TEMP 24-bit LE   0x20..0x22 PRESS 24-bit LE
 *   0x27 INT_STATUS (clear-on-read)   0x28 STATUS   0x36 OSR_CONFIG
 *   0x37 ODR_CONFIG   0x7E CMD
 * Config: press OSR x8, temp OSR x1, press_en; normal mode @ 50 Hz.
 * temp_c = s24/65536, press_pa = u24/64.
 * ============================================================ */
#include "app.h"
#include <math.h>

/* ---- registers / bits ---- */
#define REG_CHIP_ID      0x01
#define REG_INT_SOURCE   0x15
#define REG_TEMP_XLSB    0x1D   /* burst: T xlsb,lsb,msb then P xlsb,lsb,msb */
#define REG_INT_STATUS   0x27
#define REG_STATUS       0x28
#define REG_OSR_CONFIG   0x36
#define REG_ODR_CONFIG   0x37
#define REG_CMD          0x7E

#define CHIP_ID_PRIM     0x50
#define CHIP_ID_SEC      0x51   /* accepted too (BMP5 family secondary id) */
#define CMD_SOFT_RESET   0xB6

#define INTST_DRDY       0x01   /* data ready, data registers */
#define INTST_POR_DONE   0x10   /* POR / soft-reset complete */
#define STAT_NVM_RDY     0x02
#define STAT_NVM_ERR     0x04

/* OSR_CONFIG: press_en(b6) | osr_p x8 (0b011 << 3) | osr_t x1 (0b000) */
#define OSR_CFG_VAL      0x58
/* ODR_CONFIG: deep_dis(b7) | ODR code 0x0F = 50.06 Hz (<<2) | pwr normal(0b01) */
#define ODR_CFG_VAL      (0x80u | (0x0Fu << 2) | 0x01u)

#define I2C_TMO_MS       5
#define ERR_LIMIT        8      /* consecutive I2C/data failures -> !baro_ok */
#define STALE_MS         500    /* no fresh conversion for this long -> !baro_ok */

/* ISA standard sea-level temperature (15 degC). The 44330 altitude form
 * bakes this in; baro_altitude_m() rescales by T_pad/ISA_T0_K for the real
 * pad temperature. Keep it named — this path is hardware-tuning-sensitive. */
#define ISA_T0_K         288.15f

static uint8_t  s_addr8;        /* HAL 8-bit address (7-bit << 1), 0 = not found */
static bool     s_init_ok;
static uint8_t  s_err_cnt;
static bool     s_have_data;
static uint32_t s_fresh_ms;     /* HAL_GetTick of last new conversion */
static float    s_press_pa;
static float    s_temp_c;
static float    s_p0_pa = SEA_LEVEL_PA_DEFAULT;
static float    s_tpad_k;       /* pad temp (K) latched at baro_zero(); 0 = unset */

static int rd(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c1, s_addr8, reg, I2C_MEMADD_SIZE_8BIT,
                         buf, len, I2C_TMO_MS) != HAL_OK)
        return -1;
    return 0;
}

static int wr(uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(&hi2c1, s_addr8, reg, I2C_MEMADD_SIZE_8BIT,
                          &val, 1, I2C_TMO_MS) != HAL_OK)
        return -1;
    return 0;
}

/* Burst-read the 6 data bytes and update the cache. 0 = new sample. */
static int fetch_sample(void)
{
    uint8_t d[6];
    if (rd(REG_TEMP_XLSB, d, 6) != 0)
        return -1;
    /* temp: signed 24-bit LE, explicit sign extension */
    int32_t traw = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                             ((uint32_t)d[2] << 16));
    if (traw & 0x00800000L)
        traw -= 0x01000000L;
    /* press: unsigned 24-bit LE */
    uint32_t praw = (uint32_t)d[3] | ((uint32_t)d[4] << 8) |
                    ((uint32_t)d[5] << 16);
    if (praw == 0)              /* all-zero data = not a real conversion */
        return -1;
    float temp_c   = (float)traw / 65536.0f;
    float press_pa = (float)praw / 64.0f;
    /* Plausibility gate: a stuck bus or corrupt burst must read as a
     * failed sample, not poison the altitude filter. 12–110 kPa covers
     * ~-650 m to ~15 km MSL; temp range is the sensor's rated span. */
    if ((press_pa < 12000.0f) || (press_pa > 110000.0f) ||
        (temp_c < -40.0f) || (temp_c > 85.0f))
        return -1;
    s_temp_c    = temp_c;
    s_press_pa  = press_pa;
    s_have_data = true;
    s_fresh_ms  = HAL_GetTick();
    return 0;
}

int baro_init(void)
{
    static const uint8_t addrs[2] = { 0x46, 0x47 };
    uint8_t v;

    s_init_ok   = false;
    s_have_data = false;
    s_err_cnt   = 0;
    s_addr8     = 0;

    /* probe 0x46 then 0x47 by CHIP_ID */
    for (int i = 0; i < 2; i++) {
        s_addr8 = (uint8_t)(addrs[i] << 1);
        if (rd(REG_CHIP_ID, &v, 1) == 0 &&
            (v == CHIP_ID_PRIM || v == CHIP_ID_SEC))
            break;
        s_addr8 = 0;
    }
    if (s_addr8 == 0)
        return -1;

    /* soft reset, then settle (Bosch spec ~2 ms; 5 ms conservative) */
    if (wr(REG_CMD, CMD_SOFT_RESET) != 0)
        return -2;
    HAL_Delay(5);

    if (rd(REG_CHIP_ID, &v, 1) != 0 || (v != CHIP_ID_PRIM && v != CHIP_ID_SEC))
        return -3;

    /* NVM must be ready and error-free after reset */
    if (rd(REG_STATUS, &v, 1) != 0)
        return -4;
    if (!(v & STAT_NVM_RDY) || (v & STAT_NVM_ERR))
        return -4;

    /* POR/soft-reset-complete must be asserted (clear-on-read) */
    if (rd(REG_INT_STATUS, &v, 1) != 0 || !(v & INTST_POR_DONE))
        return -5;

    /* configure while still in standby (reset default) */
    if (wr(REG_OSR_CONFIG, OSR_CFG_VAL) != 0)
        return -6;
    /* route DRDY into INT_STATUS for polling; INT pin stays disabled
     * (INT_CONFIG left at reset default, int_en = 0) */
    if (wr(REG_INT_SOURCE, INTST_DRDY) != 0)
        return -6;
    /* normal (continuous) mode, 50 Hz, deep standby disabled */
    if (wr(REG_ODR_CONFIG, ODR_CFG_VAL) != 0)
        return -6;

    s_init_ok = true;
    return 0;
}

/* Non-blocking: polls DRDY once; on new data updates the cache, otherwise
 * returns the previous conversion (caller paces at BARO_HZ = ODR). */
int baro_read(float *press_pa, float *temp_c)
{
    if (!s_init_ok)
        return -1;

    uint8_t st;
    if (rd(REG_INT_STATUS, &st, 1) == 0) {
        if (st & INTST_DRDY) {
            if (fetch_sample() == 0)
                s_err_cnt = 0;
            else if (s_err_cnt < 0xFF)
                s_err_cnt++;
        }
        /* no DRDY this poll: not an error, ODR and poll rate free-run */
    } else if (s_err_cnt < 0xFF) {
        s_err_cnt++;
    }

    if (!s_have_data)
        return -2;
    /* Stale or error-limited data must read as a FAILURE, not as the
     * frozen cache: the FSM's 0.5 s baro-fail debounce (-> ST_FAULT) and
     * the Kalman update key off this return code, and a frozen altitude
     * served as fresh pins velocity to 0 and can fake a landing. */
    if ((s_err_cnt >= ERR_LIMIT) ||
        ((HAL_GetTick() - s_fresh_ms) > STALE_MS))
        return -3;
    if (press_pa) *press_pa = s_press_pa;
    if (temp_c)   *temp_c   = s_temp_c;
    return 0;
}

/* Slow ground-reference tracking, called at LED_HZ from the superloop
 * ONLY in GROUND_IDLE + disarmed: absorbs weather/thermal drift so pad
 * AGL holds ~0 without manual re-zeroing (tau ~= 1 min). The reference
 * freezes the moment the vehicle arms — flight altitudes are measured
 * against the last pre-arm ground level, exactly like `zero`. */
void baro_ground_track(void)
{
    if (!s_init_ok || !s_have_data || s_p0_pa <= 0.0f)
        return;
    s_p0_pa += BARO_TRACK_ALPHA * (s_press_pa - s_p0_pa);
}

float baro_altitude_m(float press_pa)
{
    if (press_pa <= 0.0f || s_p0_pa <= 0.0f)
        return 0.0f;
    float isa = 44330.0f * (1.0f - powf(press_pa / s_p0_pa, 0.190295f));
    /* Temperature-compensate: the ISA form assumes T0 = ISA_T0_K (15 degC).
     * True AGL scales by T_pad/T0, where T_pad is latched at baro_zero().
     * Unset (0) or implausible T_pad -> factor 1.0 (plain ISA). */
    float scale = 1.0f;
    if (s_tpad_k > 200.0f && s_tpad_k < 360.0f)
        scale = s_tpad_k / ISA_T0_K;
    return isa * scale;
}

/* Latch current pressure as AGL zero: average up to 8 fresh conversions.
 * Pad-only operation; bounded to ~400 ms via HAL_GetTick deadline. */
void baro_zero(void)
{
    if (!s_init_ok)
        return;

    float    sum = 0.0f;
    int      n   = 0;
    uint32_t deadline = HAL_GetTick() + 400;   /* 8 samples @ 50 Hz + margin */

    while (n < 8 && (int32_t)(deadline - HAL_GetTick()) > 0) {
        uint8_t st;
        if (rd(REG_INT_STATUS, &st, 1) == 0 && (st & INTST_DRDY)) {
            if (fetch_sample() == 0) {
                sum += s_press_pa;
                n++;
            }
        }
    }
    if (n > 0)
        s_p0_pa = sum / (float)n;
    else if (s_have_data)
        s_p0_pa = s_press_pa;      /* fallback: last known conversion */

    /* Latch pad temperature (K) for baro_altitude_m()'s ISA correction.
     * fetch_sample() refreshed s_temp_c above; skip if we never got data
     * (s_tpad_k stays 0 -> altitude falls back to plain ISA). */
    if (s_have_data)
        s_tpad_k = s_temp_c + 273.15f;
}

bool baro_ok(void)
{
    if (!s_init_ok || s_err_cnt >= ERR_LIMIT)
        return false;
    /* healthy only if conversions keep arriving (sensor stuck/standby = stale) */
    if (s_have_data && (HAL_GetTick() - s_fresh_ms) > STALE_MS)
        return false;
    return true;
}
