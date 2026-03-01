/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_composite.h
  * @brief          : Header for USB Composite Device (CDC + MIDI) class driver.
  *
  * This custom class bypasses CubeMX's single-class limitation to provide
  * a composite USB device with:
  *   - CDC ACM (Virtual COM Port) for control commands and telemetry
  *   - USB MIDI 1.0 for real-time note streaming from a DAW
  *
  * Endpoint Layout (STM32F407 USB OTG FS, 4 endpoint pairs):
  *   EP0        : Control (default)
  *   EP1 IN/OUT : CDC Data      (Bulk, 64 bytes)
  *   EP2 IN     : CDC Notify    (Interrupt, 8 bytes)
  *   EP3 IN/OUT : MIDI Streaming (Bulk, 64 bytes)
  *
  * Interface Layout:
  *   IF0 : CDC Communication Interface (ACM)
  *   IF1 : CDC Data Interface
  *   IF2 : Audio Control Interface (zero-bandwidth, required by USB MIDI spec)
  *   IF3 : MIDI Streaming Interface
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USBD_COMPOSITE_H
#define __USBD_COMPOSITE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"

/* ---------------------------------------------------------------------------*/
/*                         CDC Endpoint Definitions                           */
/* ---------------------------------------------------------------------------*/
#define CDC_DATA_IN_EP                  0x81U   /* EP1 IN  - Bulk  */
#define CDC_DATA_OUT_EP                 0x01U   /* EP1 OUT - Bulk  */
#define CDC_CMD_EP                      0x82U   /* EP2 IN  - Interrupt (Notification) */

#define CDC_DATA_FS_MAX_PACKET_SIZE     64U
#define CDC_CMD_PACKET_SIZE             8U

/* ---------------------------------------------------------------------------*/
/*                         MIDI Endpoint Definitions                          */
/* ---------------------------------------------------------------------------*/
#define MIDI_DATA_IN_EP                 0x83U   /* EP3 IN  - Bulk  */
#define MIDI_DATA_OUT_EP                0x03U   /* EP3 OUT - Bulk  */

#define MIDI_DATA_FS_MAX_PACKET_SIZE    64U

/* ---------------------------------------------------------------------------*/
/*                         Interface Numbers                                  */
/* ---------------------------------------------------------------------------*/
#define CDC_COMM_INTERFACE              0x00U
#define CDC_DATA_INTERFACE              0x01U
#define AUDIO_CTRL_INTERFACE            0x02U
#define MIDI_STREAM_INTERFACE           0x03U

#define COMPOSITE_NUM_INTERFACES        4U

/* ---------------------------------------------------------------------------*/
/*                        CDC ACM Line Coding                                 */
/* ---------------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND   0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE   0x01U
#define CDC_SET_COMM_FEATURE            0x02U
#define CDC_GET_COMM_FEATURE            0x03U
#define CDC_CLEAR_COMM_FEATURE          0x04U
#define CDC_SET_LINE_CODING             0x20U
#define CDC_GET_LINE_CODING             0x21U
#define CDC_SET_CONTROL_LINE_STATE      0x22U
#define CDC_SEND_BREAK                  0x23U

/* ---------------------------------------------------------------------------*/
/*                        Ring Buffer for CDC RX                              */
/* ---------------------------------------------------------------------------*/
#define CDC_RX_RING_SIZE                1024U
#define CDC_TX_RING_SIZE                1024U

/* ---------------------------------------------------------------------------*/
/*                        MIDI RX Buffer                                      */
/* ---------------------------------------------------------------------------*/
#define MIDI_RX_RING_SIZE               512U

/* ---------------------------------------------------------------------------*/
/*                         Composite Handle                                   */
/* ---------------------------------------------------------------------------*/
typedef struct
{
  /* CDC state */
  uint8_t  cdc_rx_packet[CDC_DATA_FS_MAX_PACKET_SIZE];    /* USB OUT packet buffer     */
  uint8_t  cdc_tx_packet[CDC_DATA_FS_MAX_PACKET_SIZE];    /* USB IN  packet buffer     */
  uint8_t  cdc_cmd_buf[CDC_CMD_PACKET_SIZE];               /* Control request buffer    */
  uint8_t  cdc_line_coding[7];                             /* DTERate(4) CharFormat(1) ParityType(1) DataBits(1) */
  volatile uint8_t  cdc_tx_busy;                           /* TX transfer in progress   */

  /* CDC RX Ring Buffer (priority path for control commands) */
  uint8_t  cdc_rx_ring[CDC_RX_RING_SIZE];
  volatile uint16_t cdc_rx_head;
  volatile uint16_t cdc_rx_tail;

  /* CDC TX Ring Buffer */
  uint8_t  cdc_tx_ring[CDC_TX_RING_SIZE];
  volatile uint16_t cdc_tx_head;
  volatile uint16_t cdc_tx_tail;

  /* MIDI state */
  uint8_t  midi_rx_packet[MIDI_DATA_FS_MAX_PACKET_SIZE];   /* USB OUT packet buffer     */
  uint8_t  midi_rx_ring[MIDI_RX_RING_SIZE];
  volatile uint16_t midi_rx_head;
  volatile uint16_t midi_rx_tail;

  /* CDC DTR/RTS state */
  volatile uint8_t  cdc_dtr;
  volatile uint8_t  cdc_rts;

} USBD_Composite_HandleTypeDef;

/* ---------------------------------------------------------------------------*/
/*                         Exported Variables                                 */
/* ---------------------------------------------------------------------------*/
extern USBD_ClassTypeDef USBD_Composite;

/* ---------------------------------------------------------------------------*/
/*                         Public API                                         */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Read one byte from the CDC RX ring buffer.
 * @param  pdev  USB device handle
 * @param  byte  Pointer to store the read byte
 * @retval 1 if a byte was read, 0 if buffer empty
 */
uint8_t USBD_Composite_CDC_ReadByte(USBD_HandleTypeDef *pdev, uint8_t *byte);

/**
 * @brief  Get number of bytes available in CDC RX ring buffer.
 * @param  pdev  USB device handle
 * @retval Number of bytes available
 */
uint16_t USBD_Composite_CDC_Available(USBD_HandleTypeDef *pdev);

/**
 * @brief  Transmit data via CDC (non-blocking, ring-buffered).
 * @param  pdev  USB device handle
 * @param  buf   Data to send
 * @param  len   Number of bytes
 * @retval Number of bytes actually queued
 */
uint16_t USBD_Composite_CDC_Transmit(USBD_HandleTypeDef *pdev, const uint8_t *buf, uint16_t len);

/**
 * @brief  Flush pending CDC TX ring data to USB endpoint (call from main loop).
 * @param  pdev  USB device handle
 */
void USBD_Composite_CDC_TxFlush(USBD_HandleTypeDef *pdev);

/**
 * @brief  Read one MIDI USB event packet (4 bytes) from the MIDI RX ring.
 * @param  pdev   USB device handle
 * @param  packet 4-byte output buffer
 * @retval 1 if a packet was read, 0 if buffer empty
 */
uint8_t USBD_Composite_MIDI_ReadPacket(USBD_HandleTypeDef *pdev, uint8_t packet[4]);

/**
 * @brief  Get number of complete MIDI event packets available.
 * @param  pdev  USB device handle
 * @retval Number of 4-byte packets available
 */
uint16_t USBD_Composite_MIDI_Available(USBD_HandleTypeDef *pdev);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_COMPOSITE_H */
