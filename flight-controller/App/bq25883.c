/* ============================================================
 * bq25883.c — TI BQ25883 2-cell boost charger, I2C1, 7-bit 0x6B.
 * Monitor-only: never touches charge current/voltage limits.
 * Register map from BQ2588x datasheet knowledge (see notes):
 *   0x0B Charger Status 1  (CHRG_STAT[2:0])
 *   0x0D NTC Status, 0x0E FAULT Status
 *   0x15 ADC Control       (bit7 ADC_EN, bit6 ADC_RATE 0=continuous)
 *   0x1D/0x1E VBAT ADC MSB/LSB, 1 mV/LSB
 *   0x25 Part Information  (bit7 REG_RST, [6:3] PN, [2:0] DEV_REV)
 * ============================================================ */
#include "app.h"

#define BQ_ADDR8          (0x6Bu << 1)   /* HAL wants 8-bit address */
#define BQ_I2C_TMO_MS     20

#define BQ_REG_CHG_STAT1  0x0Bu
#define BQ_REG_NTC_STAT   0x0Du
#define BQ_REG_FAULT_STAT 0x0Eu
#define BQ_REG_ADC_CTRL   0x15u
#define BQ_REG_VBAT_MSB   0x1Du
#define BQ_REG_PART_INFO  0x25u

#define BQ_ADC_EN         0x80u          /* ADC_EN=1, ADC_RATE=0 (continuous) */
#define BQ_PN_EXPECTED    0x03u          /* PN[3:0] guess for BQ25883; mismatch tolerated */

/* CHRG_STAT[2:0]: 0=not charging, 1=trickle, 2=precharge, 3=CC, 4=CV,
 * 5=top-off, 6=done, 7=rsvd. "Charging" = 1..5. */

static charger_status_t s_last;          /* last-good values, kept on I2C error */
static bool s_pn_ok;                     /* part-number sanity result (informational) */

static int bq_rd(uint8_t reg, uint8_t *dst, uint16_t n)
{
    return (HAL_I2C_Mem_Read(&hi2c1, BQ_ADDR8, reg, I2C_MEMADD_SIZE_8BIT,
                             dst, n, BQ_I2C_TMO_MS) == HAL_OK) ? 0 : -1;
}

static int bq_wr(uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(&hi2c1, BQ_ADDR8, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, BQ_I2C_TMO_MS) == HAL_OK) ? 0 : -1;
}

int charger_init(void)
{
    uint8_t pi;

    if (bq_rd(BQ_REG_PART_INFO, &pi, 1) != 0)
        return -1;                       /* chip absent / bus dead */
    s_pn_ok = (((pi >> 3) & 0x0Fu) == BQ_PN_EXPECTED); /* tolerate mismatch, don't brick */

    /* Enable internal ADC, continuous conversion. Everything else stays at
     * chip defaults — monitor-only per CLAUDE.md §5.6. */
    if (bq_wr(BQ_REG_ADC_CTRL, BQ_ADC_EN) != 0)
        return -2;
    return 0;
}

int charger_poll(charger_status_t *st)
{
    uint8_t s1, ff[2], vb[2];
    int err = 0;

    if (bq_rd(BQ_REG_CHG_STAT1, &s1, 1) == 0) {
        uint8_t cs = s1 & 0x07u;
        s_last.chrg_stat = cs;
        s_last.charging  = (cs >= 1u && cs <= 5u);
    } else {
        err = -1;
    }

    /* 0x0D NTC status + 0x0E fault status are contiguous: one 2-byte read */
    if (bq_rd(BQ_REG_NTC_STAT, ff, 2) == 0)
        s_last.fault = (ff[0] != 0u) || (ff[1] != 0u);
    else
        err = -1;

    if (bq_rd(BQ_REG_VBAT_MSB, vb, 2) == 0) {
        uint16_t mv = (uint16_t)(((uint16_t)vb[0] << 8) | vb[1]); /* 1 mV/LSB */
        if (mv != 0u)                    /* 0 = conversion not ready; keep last */
            s_last.vbat_mv = mv;
    } else {
        err = -1;
    }

    /* The chip's I2C watchdog (default 40 s) resets registers to defaults,
     * which clears ADC_EN — re-assert it every poll (1 Hz, cheap). */
    if (bq_wr(BQ_REG_ADC_CTRL, BQ_ADC_EN) != 0)
        err = -1;

    if (st)
        *st = s_last;                    /* last-good values even on error */
    return err;
}
