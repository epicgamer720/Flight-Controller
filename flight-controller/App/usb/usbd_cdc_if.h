/* ============================================================
 * usbd_cdc_if.h: CDC class interface (console transport)
 * ============================================================ */
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE  512U
#define APP_TX_DATA_SIZE  512U

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* Non-blocking single-shot transmit. Data is copied into an internal
 * buffer, so `buf` may be reused immediately.
 * Returns 0 = started, -1 = previous TX still pending (retry later),
 * -2 = not configured / not ready, -3 = bad length (> APP_TX_DATA_SIZE). */
int cdc_transmit(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H */
