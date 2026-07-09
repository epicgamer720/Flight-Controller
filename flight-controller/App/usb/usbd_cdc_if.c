/* ============================================================
 * usbd_cdc_if.c — CDC ACM interface callbacks.
 * RX: OUT-EP data -> console_rx() (ISR context) -> re-arm endpoint.
 * TX: cdc_transmit() copies into a static buffer; busy until
 *     TransmitCplt fires from the OTG_FS ISR.
 * ============================================================ */
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "app.h"

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

__ALIGN_BEGIN static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE] __ALIGN_END;
__ALIGN_BEGIN static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE] __ALIGN_END;

static volatile uint8_t tx_busy;      /* set in cdc_transmit, cleared in ISR */

/* Line coding is cosmetic for a native CDC console; report 115200 8N1. */
static USBD_CDC_LineCodingTypeDef line_coding =
{
  115200U, /* bitrate     */
  0x00U,   /* 1 stop bit  */
  0x00U,   /* no parity   */
  0x08U    /* 8 data bits */
};

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
  tx_busy = 0U;
  (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0U);
  (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return 0;
}

static int8_t CDC_DeInit_FS(void)
{
  tx_busy = 0U;
  return 0;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd)
  {
    case CDC_SET_LINE_CODING:
      if (length >= 7U)
      {
        line_coding.bitrate    = (uint32_t)pbuf[0] | ((uint32_t)pbuf[1] << 8) |
                                 ((uint32_t)pbuf[2] << 16) | ((uint32_t)pbuf[3] << 24);
        line_coding.format     = pbuf[4];
        line_coding.paritytype = pbuf[5];
        line_coding.datatype   = pbuf[6];
      }
      break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(line_coding.bitrate);
      pbuf[1] = (uint8_t)(line_coding.bitrate >> 8);
      pbuf[2] = (uint8_t)(line_coding.bitrate >> 16);
      pbuf[3] = (uint8_t)(line_coding.bitrate >> 24);
      pbuf[4] = line_coding.format;
      pbuf[5] = line_coding.paritytype;
      pbuf[6] = line_coding.datatype;
      break;

    /* OK-stubs */
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SET_CONTROL_LINE_STATE:
    case CDC_SEND_BREAK:
    default:
      break;
  }
  return 0;
}

/* ISR context (OTG_FS). Hand bytes to the console RX ring, re-arm EP. */
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len)
{
  console_rx(pbuf, *Len);
  (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return 0;
}

/* ISR context. Previous IN transfer finished. */
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(pbuf);
  UNUSED(Len);
  UNUSED(epnum);
  tx_busy = 0U;
  return 0;
}

int cdc_transmit(const uint8_t *buf, uint16_t len)
{
  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassDataCmsit[0];

  if ((hcdc == NULL) || (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED))
  {
    return -2;
  }
  if ((len == 0U) || (len > APP_TX_DATA_SIZE))
  {
    return -3;
  }
  if ((tx_busy != 0U) || (hcdc->TxState != 0U))
  {
    return -1;
  }

  tx_busy = 1U;
  memcpy(UserTxBufferFS, buf, len);
  (void)USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, len);
  if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != (uint8_t)USBD_OK)
  {
    tx_busy = 0U;
    return -1;
  }
  return 0;
}
