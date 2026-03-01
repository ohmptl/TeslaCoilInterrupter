/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : debug_uart.h
  * @brief          : USART1 debug logging ("firehose") interface.
  *
  * Provides non-blocking, ISR-safe debug output via USART1 (PA9 TX / PA10 RX)
  * at 115200 baud.  This is the "un-killable" debug log stream that works
  * even when USB is disconnected in high-EMI environments.
  *
  * All output is prefixed with a millisecond timestamp for correlation
  * with oscilloscope captures.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DEBUG_UART_H
#define __DEBUG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* ---------------------------------------------------------------------------*/
/*                            Configuration                                   */
/* ---------------------------------------------------------------------------*/
#define DEBUG_TX_BUF_SIZE   512U   /* Transmit ring buffer size (power of 2) */

/* ---------------------------------------------------------------------------*/
/*                            Public API                                      */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialize the debug UART module.
 *         Must be called after MX_USART1_UART_Init().
 */
void Debug_Init(void);

/**
 * @brief  Send a fixed string with timestamp prefix.
 *         Non-blocking; drops data if buffer is full.
 * @param  msg  Null-terminated string
 */
void Debug_Log(const char *msg);

/**
 * @brief  Printf-style debug output with timestamp prefix.
 *         Non-blocking; drops data if buffer is full.
 * @param  fmt  printf format string
 */
void Debug_Printf(const char *fmt, ...);

/**
 * @brief  Send raw bytes without timestamp or newline.
 * @param  data  Pointer to data
 * @param  len   Number of bytes
 */
void Debug_SendRaw(const uint8_t *data, uint16_t len);

/**
 * @brief  Flush pending debug TX data.
 *         Call this from the main loop to keep data flowing.
 *         Uses polling HAL_UART_Transmit with a short timeout.
 */
void Debug_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_UART_H */
