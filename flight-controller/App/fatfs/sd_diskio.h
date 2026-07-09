#pragma once
/* ============================================================
 * sd_diskio.h — FatFs diskio driver for SDMMC1 (polled HAL_SD)
 * Linked into FatFs via FATFS_LinkDriver(&SD_Driver, path).
 * ============================================================ */
#ifdef __cplusplus
extern "C" {
#endif

#include "ff_gen_drv.h"

extern const Diskio_drvTypeDef SD_Driver;

#ifdef __cplusplus
}
#endif
