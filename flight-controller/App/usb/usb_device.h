/* ============================================================
 * usb_device.h: CDC device on OTG_FS (device mode)
 * ============================================================ */
#ifndef __USB_DEVICE_H
#define __USB_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

void usb_device_init(void);     /* also declared in app.h */
bool usb_cdc_connected(void);   /* dev_state == CONFIGURED */

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE_H */
