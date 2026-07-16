/* ============================================================
 * spi_bus.c: shared SPI1 transaction helpers (IMU CS=PA4, LoRa CS=PA9)
 * Mode 0, ~6.75 MHz, soft GPIO CS, blocking HAL with 100 ms timeout.
 * The superloop serializes bus use naturally; a test-and-set flag
 * catches nesting bugs (e.g. an SPI call from an ISR while a polled
 * transfer is mid-flight) and returns -1 instead of corrupting the bus.
 * SX1262 BUSY-line etiquette is the radio driver's job BEFORE calling
 * these helpers; spi_bus is device-agnostic.
 * ============================================================ */
#include "app.h"

#define SPI_TIMEOUT_MS  100u
#define SPI_CHUNK       64u    /* scratch size for NULL-tx/rx substitution */

static volatile uint8_t s_busy;

/* Atomic test-and-set: PRIMASK save/restore so this is safe whether or
 * not it is entered with interrupts already disabled. */
static int bus_lock(void)
{
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    if (s_busy) {
        if (!prim) __enable_irq();
        return -1;
    }
    s_busy = 1;
    if (!prim) __enable_irq();
    return 0;
}

static void bus_unlock(void)
{
    s_busy = 0;
}

/* Full-duplex transfer. tx==NULL sends 0x00 fill; rx==NULL discards.
 * HAL_SPI_TransmitReceive needs non-NULL buffers, so NULL sides run
 * through static scratch in SPI_CHUNK pieces (len itself is unlimited).
 * Returns 0 OK / -2 on HAL error or timeout. */
static int xfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    static const uint8_t fill[SPI_CHUNK];   /* all-zero TX filler (.rodata) */
    static uint8_t       dump[SPI_CHUNK];   /* RX discard bin */

    while (len) {
        uint16_t n = len;
        if ((tx == NULL || rx == NULL) && n > SPI_CHUNK)
            n = SPI_CHUNK;
        const uint8_t *t = tx ? tx : fill;
        uint8_t       *r = rx ? rx : dump;
        if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)t, r, n,
                                    SPI_TIMEOUT_MS) != HAL_OK)
            return -2;
        if (tx) tx += n;
        if (rx) rx += n;
        len -= n;
    }
    return 0;
}

int spi_bus_txrx2(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                  const uint8_t *hdr, uint16_t hdr_len,
                  const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    int rc;

    if (bus_lock() != 0)
        return -1;                              /* nested use */

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);   /* select */

    rc = xfer(hdr, NULL, hdr_len);              /* header phase, RX discarded */
    if (rc == 0)
        rc = xfer(tx, rx, len);                 /* payload phase */

    /* Always deselect, including on HAL error/timeout. */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    bus_unlock();
    return rc;
}

int spi_bus_txrx(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                 const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    return spi_bus_txrx2(cs_port, cs_pin, NULL, 0, tx, rx, len);
}
