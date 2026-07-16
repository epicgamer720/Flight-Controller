/* ============================================================
 * sd_diskio.c: FatFs Diskio_drvTypeDef backed by polled HAL_SD
 * on SDMMC1 (hsd1). No DMA: CPU-driven FIFO transfers, so no
 * D-cache coherency concerns (only I-cache is enabled anyway).
 * All waits are bounded by HAL_GetTick timeouts, so it never blocks
 * the superloop indefinitely.
 * ============================================================ */
#include "app.h"
#include "sd_diskio.h"

#define SD_RW_TIMEOUT_MS   1000U   /* per HAL read/write call */
#define SD_XFER_TIMEOUT_MS 1000U   /* wait for card back in TRANSFER state */

static volatile DSTATUS sd_stat = STA_NOINIT;

/* Card-detect switch closes to GND when a card is inserted (internal
 * pull-up configured in MX_GPIO_Init): HIGH = no card. */
static bool sd_card_present(void)
{
  return HAL_GPIO_ReadPin(SD_CD_PORT, SD_CD_PIN) == GPIO_PIN_RESET;
}

/* Poll until the card returns to TRANSFER state (write/erase done). */
static int sd_wait_transfer(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
    wdg_refresh();     /* card busy = progress, not a hang: a slow but
                          healthy card may legally sit here for seconds
                          and must not trip the IWDG (app_main.c) */
    if ((HAL_GetTick() - t0) >= timeout_ms)
      return -1;
  }
  return 0;
}

static DSTATUS sd_initialize(BYTE lun)
{
  (void)lun;
  sd_stat = STA_NOINIT;

  if (!sd_card_present()) {
    sd_stat = (DSTATUS)(STA_NODISK | STA_NOINIT);
    return sd_stat;
  }

  /* Allow clean re-init after a previous failed/aborted session. */
  if (hsd1.State != HAL_SD_STATE_RESET)
    (void)HAL_SD_DeInit(&hsd1);

  if (HAL_SD_Init(&hsd1) != HAL_OK)
    return sd_stat;                              /* STA_NOINIT */

  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK) {
    (void)HAL_SD_DeInit(&hsd1);
    return sd_stat;                              /* STA_NOINIT */
  }

  if (sd_wait_transfer(SD_XFER_TIMEOUT_MS) != 0)
    return sd_stat;                              /* STA_NOINIT */

  sd_stat = 0;
  return sd_stat;
}

static DSTATUS sd_status(BYTE lun)
{
  (void)lun;
  return sd_stat;
}

static DRESULT sd_read(BYTE lun, BYTE *buff, LBA_t sector, UINT count)
{
  (void)lun;
  if (sd_stat & STA_NOINIT)
    return RES_NOTRDY;

  if (HAL_SD_ReadBlocks(&hsd1, (uint8_t *)buff, (uint32_t)sector,
                        (uint32_t)count, SD_RW_TIMEOUT_MS) != HAL_OK)
    return RES_ERROR;

  if (sd_wait_transfer(SD_XFER_TIMEOUT_MS) != 0)
    return RES_ERROR;
  return RES_OK;
}

static DRESULT sd_write(BYTE lun, const BYTE *buff, LBA_t sector, UINT count)
{
  (void)lun;
  if (sd_stat & STA_NOINIT)
    return RES_NOTRDY;

  /* HAL API takes non-const; it only reads from the buffer. */
  if (HAL_SD_WriteBlocks(&hsd1, (uint8_t *)(uintptr_t)buff, (uint32_t)sector,
                         (uint32_t)count, SD_RW_TIMEOUT_MS) != HAL_OK)
    return RES_ERROR;

  if (sd_wait_transfer(SD_XFER_TIMEOUT_MS) != 0)
    return RES_ERROR;
  return RES_OK;
}

static DRESULT sd_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  (void)lun;
  HAL_SD_CardInfoTypeDef ci;

  if (sd_stat & STA_NOINIT)
    return RES_NOTRDY;

  switch (cmd) {
  case CTRL_SYNC:
    return (sd_wait_transfer(SD_XFER_TIMEOUT_MS) == 0) ? RES_OK : RES_ERROR;

  case GET_SECTOR_COUNT:
    if (HAL_SD_GetCardInfo(&hsd1, &ci) != HAL_OK)
      return RES_ERROR;
    *(LBA_t *)buff = (LBA_t)ci.LogBlockNbr;
    return RES_OK;

  case GET_SECTOR_SIZE:
    if (HAL_SD_GetCardInfo(&hsd1, &ci) != HAL_OK)
      return RES_ERROR;
    *(WORD *)buff = (WORD)ci.LogBlockSize;       /* 512 */
    return RES_OK;

  case GET_BLOCK_SIZE:
    if (HAL_SD_GetCardInfo(&hsd1, &ci) != HAL_OK)
      return RES_ERROR;
    /* Erase block size in units of sectors; >=1. */
    *(DWORD *)buff = (ci.LogBlockSize >= 512U) ? (ci.LogBlockSize / 512U) : 1U;
    return RES_OK;

  default:
    return RES_PARERR;
  }
}

const Diskio_drvTypeDef SD_Driver = {
  sd_initialize,
  sd_status,
  sd_read,
  sd_write,
  sd_ioctl,
};
