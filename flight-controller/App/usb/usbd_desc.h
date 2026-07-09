/* ============================================================
 * usbd_desc.h — USB device descriptors (FS CDC console)
 * ============================================================ */
#ifndef __USBD_DESC_H
#define __USBD_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_def.h"

#define DEVICE_FS               0U

/* STM32F72x/F73x 96-bit unique ID (RM0431): base 0x1FF07A10 */
#define DEVICE_ID1              0x1FF07A10UL
#define DEVICE_ID2              0x1FF07A14UL
#define DEVICE_ID3              0x1FF07A18UL

#define USB_SIZ_STRING_SERIAL   0x1AU

extern USBD_DescriptorsTypeDef FS_Desc;

#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC_H */
