/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_composite.c
  * @brief          : USB Composite Device class driver (CDC + MIDI).
  *
  * This file provides a single USBD_ClassTypeDef that handles both the
  * CDC ACM (Virtual COM Port) and USB MIDI 1.0 functions in one composite
  * configuration descriptor.  It bypasses CubeMX's single-class limitation
  * as specified in the project plan.
  *
  * The CDC path uses a dedicated ring buffer and is given processing priority
  * over MIDI to ensure control commands are never blocked by MIDI floods.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_composite.h"
#include "usbd_ctlreq.h"
#include <string.h>

/* ---------------------------------------------------------------------------*/
/*                    Forward Declarations (Class Callbacks)                   */
/* ---------------------------------------------------------------------------*/
static uint8_t USBD_Composite_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Composite_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Composite_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_Composite_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_Composite_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_Composite_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t *USBD_Composite_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_Composite_GetDeviceQualifierDesc(uint16_t *length);

/* ---------------------------------------------------------------------------*/
/*                          Class Structure                                   */
/* ---------------------------------------------------------------------------*/
USBD_ClassTypeDef USBD_Composite =
{
  USBD_Composite_Init,
  USBD_Composite_DeInit,
  USBD_Composite_Setup,
  NULL,                          /* EP0_TxSent      */
  USBD_Composite_EP0_RxReady,
  USBD_Composite_DataIn,
  USBD_Composite_DataOut,
  NULL,                          /* SOF             */
  NULL,                          /* IsoINIncomplete */
  NULL,                          /* IsoOUTIncomplete*/
  NULL,                          /* GetHSConfigDescriptor  (FS only) */
  USBD_Composite_GetFSCfgDesc,
  NULL,                          /* GetOtherSpeedConfigDescriptor */
  USBD_Composite_GetDeviceQualifierDesc,
};

/* ---------------------------------------------------------------------------*/
/*          Composite Configuration Descriptor (CDC + MIDI)                   */
/* ---------------------------------------------------------------------------*/
/*
 * Total descriptor contents:
 *   9  Configuration Descriptor
 *   ---- CDC Function (IAD + 2 interfaces) ----
 *   8  IAD (Interface Association Descriptor) for CDC
 *   9  IF0: CDC Communication Interface
 *   5  CDC Header Functional Descriptor
 *   5  CDC Call Management Functional Descriptor
 *   4  CDC Abstract Control Management Functional Descriptor
 *   5  CDC Union Functional Descriptor
 *   7  EP2 IN: Notification endpoint
 *   9  IF1: CDC Data Interface
 *   7  EP1 OUT: Data OUT
 *   7  EP1 IN:  Data IN
 *   ---- MIDI Function (2 interfaces) ----
 *   9  IF2: Audio Control Interface (zero-bandwidth)
 *   9  AC Header Descriptor
 *   9  IF3: MIDI Streaming Interface
 *   7  MS Header Descriptor
 *   6  MIDI IN Jack (Embedded)
 *   6  MIDI IN Jack (External)
 *   9  MIDI OUT Jack (Embedded)
 *   9  MIDI OUT Jack (External)
 *   9  EP3 OUT: MIDI OUT endpoint + CS descriptor
 *   5  CS Endpoint Descriptor (associated MIDI IN Jack)
 *   9  EP3 IN:  MIDI IN endpoint + CS descriptor
 *   5  CS Endpoint Descriptor (associated MIDI OUT Jack)
 *
 * Total = 9+8+9+5+5+4+5+7+9+7+7 + 9+9+9+7+6+6+9+9+9+5+9+5 = 167
 */

#define USBD_COMPOSITE_CONFIG_DESC_SIZE   167U

__ALIGN_BEGIN static uint8_t USBD_Composite_CfgDesc[USBD_COMPOSITE_CONFIG_DESC_SIZE] __ALIGN_END =
{
  /* ======================================================================= */
  /*                     CONFIGURATION DESCRIPTOR                            */
  /* ======================================================================= */
  0x09,                              /* bLength                 */
  USB_DESC_TYPE_CONFIGURATION,       /* bDescriptorType         */
  LOBYTE(USBD_COMPOSITE_CONFIG_DESC_SIZE),  /* wTotalLength (low)  */
  HIBYTE(USBD_COMPOSITE_CONFIG_DESC_SIZE),  /* wTotalLength (high) */
  COMPOSITE_NUM_INTERFACES,          /* bNumInterfaces          */
  0x01,                              /* bConfigurationValue     */
  0x00,                              /* iConfiguration          */
  0xC0,                              /* bmAttributes: Self-powered */
  0x32,                              /* bMaxPower: 100 mA       */

  /* ======================================================================= */
  /*               IAD (Interface Association Descriptor) for CDC             */
  /* ======================================================================= */
  0x08,                              /* bLength                 */
  USB_DESC_TYPE_IAD,                 /* bDescriptorType (0x0B)  */
  CDC_COMM_INTERFACE,                /* bFirstInterface         */
  0x02,                              /* bInterfaceCount         */
  0x02,                              /* bFunctionClass: CDC     */
  0x02,                              /* bFunctionSubClass: ACM  */
  0x01,                              /* bFunctionProtocol: AT   */
  0x00,                              /* iFunction               */

  /* ======================================================================= */
  /*            IF0: CDC Communication Interface Descriptor                  */
  /* ======================================================================= */
  0x09,                              /* bLength                 */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType         */
  CDC_COMM_INTERFACE,                /* bInterfaceNumber        */
  0x00,                              /* bAlternateSetting       */
  0x01,                              /* bNumEndpoints           */
  0x02,                              /* bInterfaceClass: CDC    */
  0x02,                              /* bInterfaceSubClass: ACM */
  0x01,                              /* bInterfaceProtocol: AT  */
  0x00,                              /* iInterface              */

  /* --- CDC Header Functional Descriptor --- */
  0x05,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x00,                              /* bDescriptorSubtype: Header */
  0x10, 0x01,                        /* bcdCDC: 1.10            */

  /* --- CDC Call Management Functional Descriptor --- */
  0x05,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x01,                              /* bDescriptorSubtype: Call Management */
  0x00,                              /* bmCapabilities          */
  CDC_DATA_INTERFACE,                /* bDataInterface          */

  /* --- CDC Abstract Control Management Functional Descriptor --- */
  0x04,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x02,                              /* bDescriptorSubtype: ACM */
  0x02,                              /* bmCapabilities: line coding + serial state */

  /* --- CDC Union Functional Descriptor --- */
  0x05,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x06,                              /* bDescriptorSubtype: Union */
  CDC_COMM_INTERFACE,                /* bMasterInterface        */
  CDC_DATA_INTERFACE,                /* bSlaveInterface0        */

  /* --- EP2 IN: CDC Notification Endpoint --- */
  0x07,                              /* bLength                 */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType         */
  CDC_CMD_EP,                        /* bEndpointAddress: 0x82  */
  0x03,                              /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),       /* wMaxPacketSize (low)    */
  HIBYTE(CDC_CMD_PACKET_SIZE),       /* wMaxPacketSize (high)   */
  0x10,                              /* bInterval: 16 ms        */

  /* ======================================================================= */
  /*                IF1: CDC Data Interface Descriptor                       */
  /* ======================================================================= */
  0x09,                              /* bLength                 */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType         */
  CDC_DATA_INTERFACE,                /* bInterfaceNumber        */
  0x00,                              /* bAlternateSetting       */
  0x02,                              /* bNumEndpoints           */
  0x0A,                              /* bInterfaceClass: CDC Data */
  0x00,                              /* bInterfaceSubClass      */
  0x00,                              /* bInterfaceProtocol      */
  0x00,                              /* iInterface              */

  /* --- EP1 OUT: CDC Data OUT --- */
  0x07,                              /* bLength                 */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType         */
  CDC_DATA_OUT_EP,                   /* bEndpointAddress: 0x01  */
  0x02,                              /* bmAttributes: Bulk      */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval               */

  /* --- EP1 IN: CDC Data IN --- */
  0x07,                              /* bLength                 */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType         */
  CDC_DATA_IN_EP,                    /* bEndpointAddress: 0x81  */
  0x02,                              /* bmAttributes: Bulk      */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval               */

  /* ======================================================================= */
  /*       IF2: Standard Audio Control Interface (zero-bandwidth)            */
  /*       Required by USB MIDI 1.0 spec even when no audio streaming        */
  /* ======================================================================= */
  0x09,                              /* bLength                 */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType         */
  AUDIO_CTRL_INTERFACE,              /* bInterfaceNumber        */
  0x00,                              /* bAlternateSetting       */
  0x00,                              /* bNumEndpoints           */
  0x01,                              /* bInterfaceClass: Audio  */
  0x01,                              /* bInterfaceSubClass: AudioControl */
  0x00,                              /* bInterfaceProtocol      */
  0x00,                              /* iInterface              */

  /* --- Class-Specific AC Interface Header Descriptor --- */
  0x09,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x01,                              /* bDescriptorSubtype: HEADER */
  0x00, 0x01,                        /* bcdADC: 1.00            */
  0x09, 0x00,                        /* wTotalLength: 9 (header only) */
  0x01,                              /* bInCollection: 1 streaming IF */
  MIDI_STREAM_INTERFACE,             /* baInterfaceNr(1)        */

  /* ======================================================================= */
  /*            IF3: MIDI Streaming Interface Descriptor                     */
  /* ======================================================================= */
  0x09,                              /* bLength                 */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType         */
  MIDI_STREAM_INTERFACE,             /* bInterfaceNumber        */
  0x00,                              /* bAlternateSetting       */
  0x02,                              /* bNumEndpoints           */
  0x01,                              /* bInterfaceClass: Audio  */
  0x03,                              /* bInterfaceSubClass: MIDIStreaming */
  0x00,                              /* bInterfaceProtocol      */
  0x00,                              /* iInterface              */

  /* --- Class-Specific MS Interface Header Descriptor --- */
  0x07,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x01,                              /* bDescriptorSubtype: MS_HEADER */
  0x00, 0x01,                        /* bcdMSC: 1.00            */
  LOBYTE(7+6+6+9+9+9+5+9+5),        /* wTotalLength (low)  = 65 */
  HIBYTE(7+6+6+9+9+9+5+9+5),        /* wTotalLength (high) */

  /* --- MIDI IN Jack (Embedded) - ID 1 --- */
  /* Represents an input inside the device (from host's perspective, data flows OUT to device) */
  0x06,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x02,                              /* bDescriptorSubtype: MIDI_IN_JACK */
  0x01,                              /* bJackType: Embedded     */
  0x01,                              /* bJackID: 1              */
  0x00,                              /* iJack                   */

  /* --- MIDI IN Jack (External) - ID 2 --- */
  0x06,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x02,                              /* bDescriptorSubtype: MIDI_IN_JACK */
  0x02,                              /* bJackType: External     */
  0x02,                              /* bJackID: 2              */
  0x00,                              /* iJack                   */

  /* --- MIDI OUT Jack (Embedded) - ID 3 --- */
  /* Represents an output inside the device (data flows IN from device to host) */
  0x09,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x03,                              /* bDescriptorSubtype: MIDI_OUT_JACK */
  0x01,                              /* bJackType: Embedded     */
  0x03,                              /* bJackID: 3              */
  0x01,                              /* bNrInputPins: 1         */
  0x02,                              /* BaSourceID(1): External IN Jack 2 */
  0x01,                              /* BaSourcePin(1)          */
  0x00,                              /* iJack                   */

  /* --- MIDI OUT Jack (External) - ID 4 --- */
  0x09,                              /* bLength                 */
  0x24,                              /* bDescriptorType: CS_INTERFACE */
  0x03,                              /* bDescriptorSubtype: MIDI_OUT_JACK */
  0x02,                              /* bJackType: External     */
  0x04,                              /* bJackID: 4              */
  0x01,                              /* bNrInputPins: 1         */
  0x01,                              /* BaSourceID(1): Embedded IN Jack 1 */
  0x01,                              /* BaSourcePin(1)          */
  0x00,                              /* iJack                   */

  /* --- EP3 OUT: MIDI Data OUT Endpoint (Standard) --- */
  0x09,                              /* bLength (audio EP = 9)  */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType         */
  MIDI_DATA_OUT_EP,                  /* bEndpointAddress: 0x03  */
  0x02,                              /* bmAttributes: Bulk      */
  LOBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  HIBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval               */
  0x00,                              /* bRefresh                */
  0x00,                              /* bSynchAddress           */

  /* --- Class-Specific MS Bulk Data OUT Endpoint Descriptor --- */
  0x05,                              /* bLength                 */
  0x25,                              /* bDescriptorType: CS_ENDPOINT */
  0x01,                              /* bDescriptorSubtype: MS_GENERAL */
  0x01,                              /* bNumEmbMIDIJack: 1      */
  0x01,                              /* BaAssocJackID(1): Embedded IN Jack 1 */

  /* --- EP3 IN: MIDI Data IN Endpoint (Standard) --- */
  0x09,                              /* bLength (audio EP = 9)  */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType         */
  MIDI_DATA_IN_EP,                   /* bEndpointAddress: 0x83  */
  0x02,                              /* bmAttributes: Bulk      */
  LOBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  HIBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval               */
  0x00,                              /* bRefresh                */
  0x00,                              /* bSynchAddress           */

  /* --- Class-Specific MS Bulk Data IN Endpoint Descriptor --- */
  0x05,                              /* bLength                 */
  0x25,                              /* bDescriptorType: CS_ENDPOINT */
  0x01,                              /* bDescriptorSubtype: MS_GENERAL */
  0x01,                              /* bNumEmbMIDIJack: 1      */
  0x03,                              /* BaAssocJackID(1): Embedded OUT Jack 3 */
};

/* ---------------------------------------------------------------------------*/
/*                   Device Qualifier Descriptor (FS-only stub)               */
/* ---------------------------------------------------------------------------*/
__ALIGN_BEGIN static uint8_t USBD_Composite_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00, 0x02,                        /* bcdUSB: 2.00            */
  0xEF,                              /* bDeviceClass: Misc      */
  0x02,                              /* bDeviceSubClass: Common */
  0x01,                              /* bDeviceProtocol: IAD    */
  USB_MAX_EP0_SIZE,
  0x01,                              /* bNumConfigurations      */
  0x00,
};

/* ---------------------------------------------------------------------------*/
/*                    Static Class Data Instance                              */
/* ---------------------------------------------------------------------------*/
static USBD_Composite_HandleTypeDef hComposite;

/* ---------------------------------------------------------------------------*/
/*                       Ring Buffer Helpers                                  */
/* ---------------------------------------------------------------------------*/
static inline uint16_t ring_available(volatile uint16_t head, volatile uint16_t tail, uint16_t size)
{
  return (uint16_t)((head - tail) & (size - 1U));   /* size must be power of 2 */
}

static inline uint16_t ring_free(volatile uint16_t head, volatile uint16_t tail, uint16_t size)
{
  return (uint16_t)(size - 1U - ring_available(head, tail, size));
}

static inline void ring_push(uint8_t *buf, volatile uint16_t *head, uint16_t size, uint8_t byte)
{
  buf[*head] = byte;
  *head = (uint16_t)((*head + 1U) & (size - 1U));
}

static inline uint8_t ring_pop(uint8_t *buf, volatile uint16_t *tail, uint16_t size)
{
  uint8_t b = buf[*tail];
  *tail = (uint16_t)((*tail + 1U) & (size - 1U));
  return b;
}

/* ---------------------------------------------------------------------------*/
/*                       Class Callbacks Implementation                      */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialize the composite class (open endpoints, prepare RX).
 */
static uint8_t USBD_Composite_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_Composite_HandleTypeDef *hc = &hComposite;
  memset(hc, 0, sizeof(*hc));
  pdev->pClassData = (void *)hc;

  /* Default line coding: 115200 8N1 */
  hc->cdc_line_coding[0] = LOBYTE(115200);
  hc->cdc_line_coding[1] = LOBYTE(115200 >> 8);
  hc->cdc_line_coding[2] = LOBYTE(115200 >> 16);
  hc->cdc_line_coding[3] = LOBYTE(115200 >> 24);
  hc->cdc_line_coding[4] = 0x00;   /* 1 stop bit   */
  hc->cdc_line_coding[5] = 0x00;   /* No parity     */
  hc->cdc_line_coding[6] = 0x08;   /* 8 data bits   */

  /* --- Open CDC endpoints --- */
  USBD_LL_OpenEP(pdev, CDC_DATA_IN_EP,  USBD_EP_TYPE_BULK, CDC_DATA_FS_MAX_PACKET_SIZE);
  pdev->ep_in[CDC_DATA_IN_EP & 0x0FU].is_used = 1U;

  USBD_LL_OpenEP(pdev, CDC_DATA_OUT_EP, USBD_EP_TYPE_BULK, CDC_DATA_FS_MAX_PACKET_SIZE);
  pdev->ep_out[CDC_DATA_OUT_EP & 0x0FU].is_used = 1U;

  USBD_LL_OpenEP(pdev, CDC_CMD_EP,      USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  pdev->ep_in[CDC_CMD_EP & 0x0FU].is_used = 1U;

  /* --- Open MIDI endpoints --- */
  USBD_LL_OpenEP(pdev, MIDI_DATA_IN_EP,  USBD_EP_TYPE_BULK, MIDI_DATA_FS_MAX_PACKET_SIZE);
  pdev->ep_in[MIDI_DATA_IN_EP & 0x0FU].is_used = 1U;

  USBD_LL_OpenEP(pdev, MIDI_DATA_OUT_EP, USBD_EP_TYPE_BULK, MIDI_DATA_FS_MAX_PACKET_SIZE);
  pdev->ep_out[MIDI_DATA_OUT_EP & 0x0FU].is_used = 1U;

  /* --- Prepare to receive on CDC and MIDI OUT endpoints --- */
  USBD_LL_PrepareReceive(pdev, CDC_DATA_OUT_EP, hc->cdc_rx_packet, CDC_DATA_FS_MAX_PACKET_SIZE);
  USBD_LL_PrepareReceive(pdev, MIDI_DATA_OUT_EP, hc->midi_rx_packet, MIDI_DATA_FS_MAX_PACKET_SIZE);

  return USBD_OK;
}

/**
 * @brief  De-initialize the composite class (close all endpoints).
 */
static uint8_t USBD_Composite_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_LL_CloseEP(pdev, CDC_DATA_IN_EP);
  USBD_LL_CloseEP(pdev, CDC_DATA_OUT_EP);
  USBD_LL_CloseEP(pdev, CDC_CMD_EP);
  USBD_LL_CloseEP(pdev, MIDI_DATA_IN_EP);
  USBD_LL_CloseEP(pdev, MIDI_DATA_OUT_EP);

  pdev->ep_in[CDC_DATA_IN_EP & 0x0FU].is_used = 0U;
  pdev->ep_out[CDC_DATA_OUT_EP & 0x0FU].is_used = 0U;
  pdev->ep_in[CDC_CMD_EP & 0x0FU].is_used = 0U;
  pdev->ep_in[MIDI_DATA_IN_EP & 0x0FU].is_used = 0U;
  pdev->ep_out[MIDI_DATA_OUT_EP & 0x0FU].is_used = 0U;

  pdev->pClassData = NULL;

  return USBD_OK;
}

/**
 * @brief  Handle SETUP requests (control transfers for CDC and MIDI).
 */
static uint8_t USBD_Composite_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  uint16_t len = 0U;
  uint8_t ifalt = 0U;

  if (hc == NULL)
  {
    return USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* ---- Class-specific requests ---- */
    case USB_REQ_TYPE_CLASS:
      /* Route based on interface number */
      if ((req->wIndex == CDC_COMM_INTERFACE) || (req->wIndex == CDC_DATA_INTERFACE))
      {
        /* CDC ACM class requests */
        if (req->wLength != 0U)
        {
          if ((req->bmRequest & 0x80U) != 0U)
          {
            /* Device-to-Host (GET) */
            if (req->bRequest == CDC_GET_LINE_CODING)
            {
              USBD_CtlSendData(pdev, hc->cdc_line_coding, 7U);
            }
            else
            {
              USBD_CtlSendData(pdev, hc->cdc_cmd_buf, req->wLength);
            }
          }
          else
          {
            /* Host-to-Device (SET) — receive data in EP0_RxReady */
            pdev->pClassData = hc; /* ensure pointer is valid */
            USBD_CtlPrepareRx(pdev, hc->cdc_cmd_buf,
                              (req->wLength > CDC_CMD_PACKET_SIZE) ? CDC_CMD_PACKET_SIZE : req->wLength);
          }
        }
        else
        {
          /* No data phase */
          if (req->bRequest == CDC_SET_CONTROL_LINE_STATE)
          {
            hc->cdc_dtr = (req->wValue & 0x01U) ? 1U : 0U;
            hc->cdc_rts = (req->wValue & 0x02U) ? 1U : 0U;
          }
        }
      }
      /* MIDI/Audio class requests — typically none needed for MIDI 1.0 */
      break;

    /* ---- Standard requests ---- */
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_INTERFACE:
          USBD_CtlSendData(pdev, &ifalt, 1U);
          break;

        case USB_REQ_SET_INTERFACE:
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          return USBD_FAIL;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      return USBD_FAIL;
  }

  UNUSED(len);
  return USBD_OK;
}

/**
 * @brief  Handle EP0 RX Ready (CDC SET_LINE_CODING data phase complete).
 */
static uint8_t USBD_Composite_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if (hc == NULL)
  {
    return USBD_FAIL;
  }

  /* If we just received SET_LINE_CODING data, it's in cdc_cmd_buf */
  if (pdev->request.bRequest == CDC_SET_LINE_CODING)
  {
    memcpy(hc->cdc_line_coding, hc->cdc_cmd_buf, 7U);
  }

  return USBD_OK;
}

/**
 * @brief  Handle data IN transfer complete (device -> host).
 */
static uint8_t USBD_Composite_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if (hc == NULL)
  {
    return USBD_FAIL;
  }

  if (epnum == (CDC_DATA_IN_EP & 0x0FU))
  {
    /* CDC TX complete — mark as not busy, flush more data if available */
    hc->cdc_tx_busy = 0U;
    USBD_Composite_CDC_TxFlush(pdev);
  }
  /* MIDI IN: nothing to do for now (device-to-host MIDI not critical) */

  return USBD_OK;
}

/**
 * @brief  Handle data OUT transfer complete (host -> device).
 *         This is where CDC and MIDI data arrive from the host.
 *         CDC data is pushed into a priority ring buffer.
 */
static uint8_t USBD_Composite_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if (hc == NULL)
  {
    return USBD_FAIL;
  }

  if (epnum == (CDC_DATA_OUT_EP & 0x0FU))
  {
    /* ---- CDC Data Received (HIGH PRIORITY) ---- */
    uint32_t rxLen = USBD_LL_GetRxDataSize(pdev, CDC_DATA_OUT_EP);

    /* Push into CDC ring buffer */
    for (uint32_t i = 0; i < rxLen; i++)
    {
      if (ring_free(hc->cdc_rx_head, hc->cdc_rx_tail, CDC_RX_RING_SIZE) > 0U)
      {
        ring_push(hc->cdc_rx_ring, &hc->cdc_rx_head, CDC_RX_RING_SIZE, hc->cdc_rx_packet[i]);
      }
      /* else: overflow — drop byte (ring guarantees no corruption) */
    }

    /* Re-arm the endpoint for next transfer */
    USBD_LL_PrepareReceive(pdev, CDC_DATA_OUT_EP, hc->cdc_rx_packet, CDC_DATA_FS_MAX_PACKET_SIZE);
  }
  else if (epnum == (MIDI_DATA_OUT_EP & 0x0FU))
  {
    /* ---- MIDI Data Received (lower priority processed in main loop) ---- */
    uint32_t rxLen = USBD_LL_GetRxDataSize(pdev, MIDI_DATA_OUT_EP);

    /* Push into MIDI ring buffer (4-byte USB-MIDI event packets) */
    for (uint32_t i = 0; i < rxLen; i++)
    {
      if (ring_free(hc->midi_rx_head, hc->midi_rx_tail, MIDI_RX_RING_SIZE) > 0U)
      {
        ring_push(hc->midi_rx_ring, &hc->midi_rx_head, MIDI_RX_RING_SIZE, hc->midi_rx_packet[i]);
      }
    }

    /* Re-arm */
    USBD_LL_PrepareReceive(pdev, MIDI_DATA_OUT_EP, hc->midi_rx_packet, MIDI_DATA_FS_MAX_PACKET_SIZE);
  }

  return USBD_OK;
}

/**
 * @brief  Return the FS configuration descriptor.
 */
static uint8_t *USBD_Composite_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_Composite_CfgDesc);
  return USBD_Composite_CfgDesc;
}

/**
 * @brief  Return the device qualifier descriptor (FS-only stub).
 */
static uint8_t *USBD_Composite_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_Composite_DeviceQualifierDesc);
  return USBD_Composite_DeviceQualifierDesc;
}

/* =========================================================================*/
/*                        PUBLIC API IMPLEMENTATION                         */
/* =========================================================================*/

uint8_t USBD_Composite_CDC_ReadByte(USBD_HandleTypeDef *pdev, uint8_t *byte)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if ((hc == NULL) || (ring_available(hc->cdc_rx_head, hc->cdc_rx_tail, CDC_RX_RING_SIZE) == 0U))
  {
    return 0U;
  }
  *byte = ring_pop(hc->cdc_rx_ring, &hc->cdc_rx_tail, CDC_RX_RING_SIZE);
  return 1U;
}

uint16_t USBD_Composite_CDC_Available(USBD_HandleTypeDef *pdev)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if (hc == NULL)
  {
    return 0U;
  }
  return ring_available(hc->cdc_rx_head, hc->cdc_rx_tail, CDC_RX_RING_SIZE);
}

uint16_t USBD_Composite_CDC_Transmit(USBD_HandleTypeDef *pdev, const uint8_t *buf, uint16_t len)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if (hc == NULL)
  {
    return 0U;
  }

  uint16_t queued = 0U;
  for (uint16_t i = 0U; i < len; i++)
  {
    if (ring_free(hc->cdc_tx_head, hc->cdc_tx_tail, CDC_TX_RING_SIZE) == 0U)
    {
      break;
    }
    ring_push(hc->cdc_tx_ring, &hc->cdc_tx_head, CDC_TX_RING_SIZE, buf[i]);
    queued++;
  }

  /* Kick a flush if not already transmitting */
  if (!hc->cdc_tx_busy)
  {
    USBD_Composite_CDC_TxFlush(pdev);
  }

  return queued;
}

void USBD_Composite_CDC_TxFlush(USBD_HandleTypeDef *pdev)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if ((hc == NULL) || (hc->cdc_tx_busy))
  {
    return;
  }

  uint16_t avail = ring_available(hc->cdc_tx_head, hc->cdc_tx_tail, CDC_TX_RING_SIZE);
  if (avail == 0U)
  {
    return;
  }

  /* Copy up to one packet from ring to TX packet buffer */
  uint16_t toSend = (avail > CDC_DATA_FS_MAX_PACKET_SIZE) ? CDC_DATA_FS_MAX_PACKET_SIZE : avail;
  for (uint16_t i = 0; i < toSend; i++)
  {
    hc->cdc_tx_packet[i] = ring_pop(hc->cdc_tx_ring, &hc->cdc_tx_tail, CDC_TX_RING_SIZE);
  }

  hc->cdc_tx_busy = 1U;
  USBD_LL_Transmit(pdev, CDC_DATA_IN_EP, hc->cdc_tx_packet, toSend);
}

uint8_t USBD_Composite_MIDI_ReadPacket(USBD_HandleTypeDef *pdev, uint8_t packet[4])
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if ((hc == NULL) || (ring_available(hc->midi_rx_head, hc->midi_rx_tail, MIDI_RX_RING_SIZE) < 4U))
  {
    return 0U;
  }
  for (int i = 0; i < 4; i++)
  {
    packet[i] = ring_pop(hc->midi_rx_ring, &hc->midi_rx_tail, MIDI_RX_RING_SIZE);
  }
  return 1U;
}

uint16_t USBD_Composite_MIDI_Available(USBD_HandleTypeDef *pdev)
{
  USBD_Composite_HandleTypeDef *hc = (USBD_Composite_HandleTypeDef *)pdev->pClassData;
  if (hc == NULL)
  {
    return 0U;
  }
  return ring_available(hc->midi_rx_head, hc->midi_rx_tail, MIDI_RX_RING_SIZE) / 4U;
}
