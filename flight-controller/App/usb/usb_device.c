/* ============================================================
 * usb_device.c: USB device stack bring-up (CDC on OTG_FS).
 * Init failure is non-fatal: the FC must fly without a console.
 * ============================================================ */
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "app.h"

USBD_HandleTypeDef hUsbDeviceFS;

static bool usb_init_ok = false;

void usb_device_init(void)
{
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    return;
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != (uint8_t)USBD_OK)
  {
    return;
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != (uint8_t)USBD_OK)
  {
    return;
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    return;
  }
  usb_init_ok = true;
}

bool usb_cdc_connected(void)
{
  return usb_init_ok && (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}
