/* ============================================================
 * sx126x.c: SX1262 LoRa driver (Wio-SX1262 module) on shared SPI1.
 * CLAUDE.md §3/§3.1: BUSY (PB1) wait before EVERY command; NRESET
 * pulse on init; DIO2-as-RF-switch ON (module internal switch);
 * TCXO 1.8 V via DIO3 with automatic fallback to plain XTAL config;
 * PB2/RF_SW1 never touched. Link params from protocol.h.
 * All SPI through spi_bus_* under LORA_CS. Mode 0 (bus default).
 * ============================================================ */
#include "app.h"

/* ---- SX126x opcodes ---- */
#define OP_SET_STANDBY          0x80
#define OP_SET_PACKET_TYPE      0x8A
#define OP_SET_RF_FREQUENCY     0x86
#define OP_SET_PA_CONFIG        0x95
#define OP_SET_TX_PARAMS        0x8E
#define OP_SET_BUF_BASE_ADDR    0x8F
#define OP_SET_MOD_PARAMS       0x8B
#define OP_SET_PKT_PARAMS       0x8C
#define OP_SET_DIO_IRQ_PARAMS   0x08
#define OP_WRITE_REGISTER       0x0D
#define OP_CALIBRATE            0x89
#define OP_CALIBRATE_IMAGE      0x98
#define OP_DIO2_AS_RF_SWITCH    0x9D
#define OP_DIO3_AS_TCXO_CTRL    0x97
#define OP_SET_RX               0x82
#define OP_SET_TX               0x83
#define OP_GET_IRQ_STATUS       0x12
#define OP_CLEAR_IRQ_STATUS     0x02
#define OP_WRITE_BUFFER         0x0E
#define OP_READ_BUFFER          0x1E
#define OP_GET_RX_BUF_STATUS    0x13
#define OP_GET_DEVICE_ERRORS    0x17
#define OP_CLEAR_DEVICE_ERRORS  0x07
#define OP_GET_STATUS           0xC0
#define OP_GET_PKT_STATUS       0x14
#define OP_SET_TX_CONT_WAVE     0xD1

/* ---- IRQ bits ---- */
#define IRQ_TX_DONE             0x0001u
#define IRQ_RX_DONE             0x0002u
#define IRQ_HEADER_ERR          0x0020u
#define IRQ_CRC_ERR             0x0040u
#define IRQ_TIMEOUT             0x0200u
#define IRQ_ALL                 0x03FFu

/* ---- Device error bits (GetDeviceErrors) ---- */
#define ERR_XOSC_START          0x0020u

#define REG_LORA_SYNC_MSB       0x0740

#define BUSY_TIMEOUT_MS         50u
/* HAL tick may be stalled inside the EXTI ISR (SysTick can be lower
 * priority), so also bound the wait by iteration count (~0.1 s worst). */
#define BUSY_LOOP_MAX           600000u
#define RESET_BUSY_TIMEOUT_MS   100u

/* ---- State (ISR-shared => volatile; SPSC discipline) ---- */
static bool             s_ok         = false;
static bool             s_using_tcxo = false;
static volatile bool    s_tx_done    = false;
static volatile bool    s_rx_ready   = false;
static volatile bool    s_irq_pend   = false;  /* DIO1 fired while bus busy */
static volatile uint8_t s_rx_len     = 0;
static volatile uint8_t s_rx_buf[256];
static uint8_t          s_zeros[256];          /* TX filler for reads */
static uint8_t          s_junk[256];           /* RX sink for writes */
static int8_t           s_tx_dbm     = LORA_TX_DBM;  /* current SetTxParams power */
static int              s_last_rssi, s_last_snr;     /* GetPacketStatus of last RxDone */
static bool             s_have_rx_status;

/* ---------------------------------------------------------- */
static int busy_wait(void)
{
    uint32_t t0 = HAL_GetTick(), n = 0;
    while (LORA_BUSY_PORT->IDR & LORA_BUSY_PIN) {
        if ((HAL_GetTick() - t0) > BUSY_TIMEOUT_MS || ++n > BUSY_LOOP_MAX) {
            s_ok = false;
            return -1;
        }
    }
    return 0;
}

static int sx_cmd(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (busy_wait() < 0) return -1;
    return spi_bus_txrx(LORA_CS_PORT, LORA_CS_PIN, tx, rx, len);
}

static int sx_cmd2(const uint8_t *hdr, uint16_t hlen,
                   const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (busy_wait() < 0) return -1;
    return spi_bus_txrx2(LORA_CS_PORT, LORA_CS_PIN, hdr, hlen, tx, rx, len);
}

static int sx_get_irq(uint16_t *irq)
{
    uint8_t tx[4] = { OP_GET_IRQ_STATUS, 0, 0, 0 }, rx[4] = {0};
    if (sx_cmd(tx, rx, 4) < 0) return -1;
    *irq = ((uint16_t)rx[2] << 8) | rx[3];
    return 0;
}

static int sx_clear_irq(uint16_t mask)
{
    uint8_t tx[3] = { OP_CLEAR_IRQ_STATUS, (uint8_t)(mask >> 8), (uint8_t)mask };
    uint8_t rx[3];
    return sx_cmd(tx, rx, 3);
}

static int sx_get_errors(uint16_t *e)
{
    uint8_t tx[4] = { OP_GET_DEVICE_ERRORS, 0, 0, 0 }, rx[4] = {0};
    if (sx_cmd(tx, rx, 4) < 0) return -1;
    *e = ((uint16_t)rx[2] << 8) | rx[3];
    return 0;
}

static int sx_clear_errors(void)
{
    uint8_t tx[3] = { OP_CLEAR_DEVICE_ERRORS, 0, 0 }, rx[3];
    return sx_cmd(tx, rx, 3);
}

/* payload_len also sets max accepted length on RX (explicit header) */
static int sx_set_pkt_params(uint8_t payload_len)
{
    uint8_t tx[7] = { OP_SET_PKT_PARAMS,
                      (uint8_t)(LORA_PREAMBLE >> 8), (uint8_t)LORA_PREAMBLE,
                      0x00,          /* explicit header */
                      payload_len,
                      0x01,          /* CRC on */
                      0x00 };        /* IQ normal */
    uint8_t rx[7];
    return sx_cmd(tx, rx, 7);
}

/* True while the shared bus is mid-transaction in main context; the
 * DIO1 ISR must not start SPI then (CS low or HAL busy). */
static bool bus_busy(void)
{
    if (hspi1.State != HAL_SPI_STATE_READY) return true;
    if (!(IMU_CS_PORT->IDR & IMU_CS_PIN))   return true;
    if (!(LORA_CS_PORT->IDR & LORA_CS_PIN)) return true;
    return false;
}

/* Read+clear IRQ status, latch flags/packet. Runs in the EXTI ISR when
 * the bus is free, or deferred to main context via s_irq_pend.
 * Any failure DEFERS (s_irq_pend) instead of dropping: sx_cmd can lose
 * the bus_lock race against a main-context transaction that has locked
 * but not yet pulled CS low, and DIO1 is edge-triggered, so a dropped
 * service here would never re-fire and the TxDone/RxDone would be lost. */
static void radio_service(void)
{
    uint16_t irq;
    uint8_t  guard = 0;

    do {
        if (sx_get_irq(&irq) < 0)  { s_irq_pend = true; return; }
        if (sx_clear_irq(irq) < 0) { s_irq_pend = true; return; }

        if (irq & IRQ_TX_DONE)
            s_tx_done = true;

        if ((irq & IRQ_RX_DONE) && !(irq & (IRQ_CRC_ERR | IRQ_HEADER_ERR))) {
            uint8_t tx[4] = { OP_GET_RX_BUF_STATUS, 0, 0, 0 }, rx[4] = {0};
            if (sx_cmd(tx, rx, 4) < 0) { s_irq_pend = true; return; }
            uint8_t len = rx[2], off = rx[3];
            if (len > 0) {
                uint8_t hdr[3] = { OP_READ_BUFFER, off, 0x00 };
                if (sx_cmd2(hdr, 3, s_zeros, (uint8_t *)s_rx_buf, len) < 0) {
                    s_irq_pend = true;
                    return;
                }
                s_rx_len   = len;   /* set len before ready flag */
                s_rx_ready = true;
                /* Link quality of this packet (LoRa): RssiPkt = -val/2 dBm,
                 * SnrPkt = signed/4 dB. Latched for `tx` / the monitor. */
                uint8_t pt[5] = { OP_GET_PKT_STATUS, 0, 0, 0, 0 }, pr[5] = {0};
                if (sx_cmd(pt, pr, 5) == 0) {
                    s_last_rssi = -(int)pr[2] / 2;
                    s_last_snr  = (int)((int8_t)pr[3]) / 4;
                    s_have_rx_status = true;
                }
            }
        }
        /* IRQ_TIMEOUT: chip is back in STDBY_RC; nothing latched. */
    } while ((LORA_IRQ_PORT->IDR & LORA_IRQ_PIN) && ++guard < 3);
}

/* Process an IRQ that the ISR had to defer (bus was busy). Main ctx.
 * Also catches a lost edge outright: DIO1 is a LEVEL until the IRQ
 * status is cleared, so a high pin with no pending flag still means
 * there is unserviced radio work. */
static void service_pending(void)
{
    if (!s_ok || bus_busy())
        return;
    if (s_irq_pend || (LORA_IRQ_PORT->IDR & LORA_IRQ_PIN)) {
        s_irq_pend = false;
        radio_service();
    }
}

/* ---------------------------------------------------------- */
static int radio_reset(void)
{
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    uint32_t t0 = HAL_GetTick();
    while (LORA_BUSY_PORT->IDR & LORA_BUSY_PIN)
        if ((HAL_GetTick() - t0) > RESET_BUSY_TIMEOUT_MS) return -1;
    return 0;
}

static int radio_configure(bool use_tcxo)
{
    uint8_t rx[10];
    uint16_t err;

    s_ok = true;                     /* let busy_wait run; cleared on fail */
    if (radio_reset() < 0) { s_ok = false; return -1; }

    { uint8_t tx[2] = { OP_SET_STANDBY, 0x00 };            /* STDBY_RC */
      if (sx_cmd(tx, rx, 2) < 0) return -2; }

    /* presence sanity: dead/floating MISO reads 0x00 or 0xFF */
    { uint8_t tx[2] = { OP_GET_STATUS, 0x00 };
      if (sx_cmd(tx, rx, 2) < 0) return -2;
      if (rx[1] == 0x00 || rx[1] == 0xFF) { s_ok = false; return -3; } }

    if (use_tcxo) {
        /* 1.8 V (code 0x02), delay in 15.625 us steps: ms * 64 */
        uint32_t d = (uint32_t)LORA_TCXO_DELAYMS * 64u;
        uint8_t tx[5] = { OP_DIO3_AS_TCXO_CTRL, 0x02,
                          (uint8_t)(d >> 16), (uint8_t)(d >> 8), (uint8_t)d };
        if (sx_cmd(tx, rx, 5) < 0) return -4;
    }

    { uint8_t tx[2] = { OP_CALIBRATE, 0x7F };               /* all blocks */
      if (sx_cmd(tx, rx, 2) < 0) return -5; }

    /* XOSC_START_ERR is expected once after TCXO switch: clear, re-check */
    if (sx_get_errors(&err) < 0) return -6;
    if (sx_clear_errors() < 0)   return -6;
    if (sx_get_errors(&err) < 0) return -6;
    if (err != 0) { s_ok = false; return -7; }

    { uint8_t tx[2] = { OP_SET_PACKET_TYPE, 0x01 };         /* LoRa */
      if (sx_cmd(tx, rx, 2) < 0) return -8; }

    { uint8_t tx[3] = { OP_CALIBRATE_IMAGE, 0xE1, 0xE9 };   /* 902-928 MHz */
      if (sx_cmd(tx, rx, 3) < 0) return -9; }

    { /* frf = freq_hz * 2^25 / 32e6 */
      uint32_t frf = (uint32_t)(((uint64_t)LORA_FREQ_HZ << 25) / 32000000ULL);
      uint8_t tx[5] = { OP_SET_RF_FREQUENCY, (uint8_t)(frf >> 24),
                        (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf };
      if (sx_cmd(tx, rx, 5) < 0) return -10; }

    { uint8_t tx[5] = { OP_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 }; /* SX1262 +22 dBm */
      if (sx_cmd(tx, rx, 5) < 0) return -11; }

    { uint8_t tx[3] = { OP_SET_TX_PARAMS, (uint8_t)LORA_TX_DBM, 0x04 }; /* 200 us ramp */
      if (sx_cmd(tx, rx, 3) < 0) return -12; }

    { uint8_t tx[3] = { OP_SET_BUF_BASE_ADDR, 0x00, 0x00 };
      if (sx_cmd(tx, rx, 3) < 0) return -13; }

    { /* SF8, BW250k, CR4/5, LDRO off (Tsym 1.02 ms << 16.38 ms) */
      uint8_t tx[5] = { OP_SET_MOD_PARAMS, LORA_SF, 0x05,
                        (uint8_t)(LORA_CR - 4), 0x00 };
      if (sx_cmd(tx, rx, 5) < 0) return -14; }

    if (sx_set_pkt_params(0xFF) < 0) return -15;

    { uint8_t tx[2] = { OP_DIO2_AS_RF_SWITCH, 0x01 };       /* REQUIRED */
      if (sx_cmd(tx, rx, 2) < 0) return -16; }

    { /* irqMask adds CrcErr|HeaderErr (readable, not routed);
       * DIO1 <- TxDone|RxDone|Timeout. DIO2/3 unrouted. */
      uint16_t en = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT |
                    IRQ_CRC_ERR | IRQ_HEADER_ERR;
      uint16_t d1 = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT;
      uint8_t tx[9] = { OP_SET_DIO_IRQ_PARAMS,
                        (uint8_t)(en >> 8), (uint8_t)en,
                        (uint8_t)(d1 >> 8), (uint8_t)d1,
                        0x00, 0x00, 0x00, 0x00 };
      if (sx_cmd(tx, rx, 9) < 0) return -17; }

    { /* sync word 0x12 -> regs 0x0740/41 = 0x14, 0x24 */
      uint8_t tx[5] = { OP_WRITE_REGISTER,
                        (uint8_t)(REG_LORA_SYNC_MSB >> 8),
                        (uint8_t)REG_LORA_SYNC_MSB,
                        (uint8_t)((LORA_SYNC_WORD & 0xF0) | 0x04),
                        (uint8_t)(((LORA_SYNC_WORD & 0x0F) << 4) | 0x04) };
      if (sx_cmd(tx, rx, 5) < 0) return -18; }

    if (sx_clear_irq(IRQ_ALL) < 0) return -19;

    if (sx_get_errors(&err) < 0) return -20;
    if (err != 0) {                  /* e.g. IMG_CALIB_ERR */
        sx_clear_errors();
        s_ok = false;
        return -21;
    }
    return 0;
}

static int s_err_tcxo, s_err_xtal;   /* radio_configure codes, 0 = ok / not run */

int radio_init(void)
{
    s_tx_done = false; s_rx_ready = false; s_irq_pend = false; s_rx_len = 0;
    s_err_tcxo = s_err_xtal = 0;
    s_tx_dbm = LORA_TX_DBM;          /* configure() re-issues SetTxParams at this */

    s_using_tcxo = true;
    s_err_tcxo = radio_configure(true);
    if (s_err_tcxo == 0) { s_ok = true; return 0; }

    /* TCXO path failed: re-init as plain XTAL (CLAUDE.md §3.1) */
    s_using_tcxo = false;
    s_err_xtal = radio_configure(false);
    if (s_err_xtal == 0) { s_ok = true; return 0; }

    s_ok = false;
    return -1;
}

void radio_debug(int *tcxo_err, int *xtal_err)
{
    if (tcxo_err) *tcxo_err = s_err_tcxo;
    if (xtal_err) *xtal_err = s_err_xtal;
}

bool radio_ok(void)          { return s_ok; }
bool radio_using_tcxo(void)  { return s_ok && s_using_tcxo; }

int radio_send(const uint8_t *buf, uint8_t len)
{
    if (!s_ok || buf == NULL || len == 0) return -1;
    service_pending();               /* don't lose a deferred RX */

    uint8_t rx[4];
    { uint8_t tx[2] = { OP_SET_STANDBY, 0x00 };  /* abort any RX window */
      if (sx_cmd(tx, rx, 2) < 0) return -2; }

    s_tx_done = false;
    if (sx_clear_irq(IRQ_ALL) < 0) return -2;

    { uint8_t hdr[2] = { OP_WRITE_BUFFER, 0x00 };
      if (sx_cmd2(hdr, 2, buf, s_junk, len) < 0) return -3; }

    if (sx_set_pkt_params(len) < 0) return -4;

    { uint8_t tx[4] = { OP_SET_TX, 0x00, 0x00, 0x00 };  /* no TX timeout */
      if (sx_cmd(tx, rx, 4) < 0) return -5; }
    return 0;
}

bool radio_tx_done(void)
{
    service_pending();
    return s_tx_done;
}

int radio_rx_start(uint32_t timeout_ms)
{
    if (!s_ok) return -1;
    service_pending();

    uint8_t rx[4];
    { uint8_t tx[2] = { OP_SET_STANDBY, 0x00 };
      if (sx_cmd(tx, rx, 2) < 0) return -2; }

    s_rx_ready = false; s_rx_len = 0;
    if (sx_clear_irq(IRQ_ALL) < 0) return -2;

    if (sx_set_pkt_params(0xFF) < 0) return -3;   /* accept any length */

    /* timeout: 15.625 us steps => ms * 64. 0 = single RX, no timeout.
     * Clamp below 0xFFFFFF (continuous mode); never continuous here. */
    uint32_t t = (timeout_ms > 262143u) ? 0xFFFFFEu : timeout_ms * 64u;
    { uint8_t tx[4] = { OP_SET_RX, (uint8_t)(t >> 16),
                        (uint8_t)(t >> 8), (uint8_t)t };
      if (sx_cmd(tx, rx, 4) < 0) return -4; }
    return 0;
}

int radio_rx_get(uint8_t *buf, uint8_t maxlen)
{
    if (!s_ok || buf == NULL) return -1;
    service_pending();

    if (!s_rx_ready) return 0;
    uint8_t n = s_rx_len;
    if (n > maxlen) n = maxlen;      /* caller's CRC check drops truncations */
    for (uint8_t i = 0; i < n; i++)
        buf[i] = s_rx_buf[i];
    s_rx_ready = false;              /* latched packet returned once */
    return (int)n;
}

/* Set TX power live (-9..+22 dBm for the SX1262 hi-power PA). We keep the
 * +22 PA config from init and vary only the SetTxParams power byte across the
 * whole range, the standard SX1262 approach (RadioLib does the same). Takes
 * effect on the next TX (and on `radio_cw`). */
int radio_set_tx_power(int dbm)
{
    if (!s_ok) return -1;
    if (dbm < -9) dbm = -9;
    if (dbm > 22) dbm = 22;
    service_pending();
    uint8_t rx[3];
    uint8_t tx[3] = { OP_SET_TX_PARAMS, (uint8_t)(int8_t)dbm, 0x04 }; /* 200 us ramp */
    if (sx_cmd(tx, rx, 3) < 0) return -2;
    s_tx_dbm = (int8_t)dbm;
    return 0;
}

int radio_get_tx_power(void) { return s_tx_dbm; }

/* Unmodulated continuous carrier at the current freq/power until the next
 * SetStandby. Bench only, for supply-current / full-power verification.
 * DIO2 drives the RF switch to TX automatically. Caller gates to pad+disarmed. */
int radio_cw(bool on)
{
    if (!s_ok) return -1;
    service_pending();
    uint8_t rx[2];
    { uint8_t st[2] = { OP_SET_STANDBY, 0x00 };
      if (sx_cmd(st, rx, 2) < 0) return -2; }
    if (on) {
        uint8_t cw[1] = { OP_SET_TX_CONT_WAVE };
        if (sx_cmd(cw, rx, 1) < 0) return -3;
    } else {
        s_tx_done = false;
        (void)sx_clear_irq(IRQ_ALL);
    }
    return 0;
}

/* RSSI/SNR (dBm/dB) of the most recent good RX. <0 until one arrives. */
int radio_last_rx_status(int *rssi_dbm, int *snr_db)
{
    if (!s_have_rx_status) return -1;
    if (rssi_dbm) *rssi_dbm = s_last_rssi;
    if (snr_db)   *snr_db   = s_last_snr;
    return 0;
}

/* EXTI15_10 -> DIO1. Uses SPI from the ISR: acceptable only because the
 * superloop never holds the bus across scheduling points; if the bus IS
 * mid-transaction (CS low / HAL busy), defer to main context instead. */
void radio_irq(void)
{
    if (!s_ok) return;
    if (bus_busy()) {
        s_irq_pend = true;
        return;
    }
    radio_service();
}
