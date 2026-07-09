/* ============================================================
 * usbd_conf.h — USB Device library configuration (OTG_FS, CDC)
 * Flight Controller, STM32F722RET6. Static allocation only.
 * ============================================================ */
#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Library config ---- */
#define USBD_MAX_NUM_INTERFACES        2U     /* CDC: comm + data */
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_STR_DESC_SIZ          512U
#define USBD_DEBUG_LEVEL               0U
#define USBD_LPM_ENABLED               0U
#define USBD_SELF_POWERED              1U     /* battery-powered board */
#define USBD_CDC_INTERVAL              2000U

/* ---- Memory management: static only, no heap ---- */
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#define USBD_malloc         (void *)USBD_static_malloc
#define USBD_free           USBD_static_free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

/* ---- Debug logging disabled (no printf from USB stack) ---- */
#define USBD_UsrLog(...)    do {} while (0)
#define USBD_ErrLog(...)    do {} while (0)
#define USBD_DbgLog(...)    do {} while (0)

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */
