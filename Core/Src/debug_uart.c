/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : debug_uart.c
  * @brief          : USART1 debug logging implementation.
  *
  * Uses a simple ring buffer + polling flush to avoid blocking the main loop.
  * The flush function sends data in small chunks so the main loop remains
  * responsive even at 115200 baud.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "debug_uart.h"
#include "usart.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---------------------------------------------------------------------------*/
/*                          Private Variables                                  */
/* ---------------------------------------------------------------------------*/
static uint8_t  tx_ring[DEBUG_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;

/* ---------------------------------------------------------------------------*/
/*                       Ring Buffer Helpers                                  */
/* ---------------------------------------------------------------------------*/
static inline uint16_t dbg_ring_avail(void)
{
  return (uint16_t)((tx_head - tx_tail) & (DEBUG_TX_BUF_SIZE - 1U));
}

static inline uint16_t dbg_ring_free(void)
{
  return (uint16_t)(DEBUG_TX_BUF_SIZE - 1U - dbg_ring_avail());
}

static inline void dbg_ring_push(uint8_t b)
{
  tx_ring[tx_head] = b;
  tx_head = (uint16_t)((tx_head + 1U) & (DEBUG_TX_BUF_SIZE - 1U));
}

static inline uint8_t dbg_ring_pop(void)
{
  uint8_t b = tx_ring[tx_tail];
  tx_tail = (uint16_t)((tx_tail + 1U) & (DEBUG_TX_BUF_SIZE - 1U));
  return b;
}

/* ---------------------------------------------------------------------------*/
/*                       Private Helpers                                      */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Queue a string into the TX ring (no blocking).
 */
static void dbg_queue_str(const char *str, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    if (dbg_ring_free() == 0U)
    {
      break; /* Drop if full — never block */
    }
    dbg_ring_push((uint8_t)str[i]);
  }
}

/**
 * @brief  Queue a timestamp prefix "[%08lu] ".
 */
static void dbg_queue_timestamp(void)
{
  char ts[16];
  int len = snprintf(ts, sizeof(ts), "[%08lu] ", HAL_GetTick());
  if (len > 0)
  {
    dbg_queue_str(ts, (uint16_t)len);
  }
}

/* ---------------------------------------------------------------------------*/
/*                           Public API                                       */
/* ---------------------------------------------------------------------------*/

void Debug_Init(void)
{
  tx_head = 0;
  tx_tail = 0;

  /* Send boot banner immediately (blocking, since this is startup) */
  const char *banner = "\r\n=== TC-Interrupter Debug Console ===\r\n"
                       "USART1 @ 115200 8N1\r\n"
                       "Build: " __DATE__ " " __TIME__ "\r\n\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)banner, (uint16_t)strlen(banner), 100);
}

void Debug_Log(const char *msg)
{
  dbg_queue_timestamp();
  dbg_queue_str(msg, (uint16_t)strlen(msg));
  dbg_queue_str("\r\n", 2);
}

void Debug_Printf(const char *fmt, ...)
{
  char buf[192];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len > 0)
  {
    dbg_queue_timestamp();
    dbg_queue_str(buf, (uint16_t)len);
    dbg_queue_str("\r\n", 2);
  }
}

void Debug_SendRaw(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    if (dbg_ring_free() == 0U)
    {
      break;
    }
    dbg_ring_push(data[i]);
  }
}

void Debug_Flush(void)
{
  /* Send up to 64 bytes per flush call to keep main loop responsive */
  uint8_t chunk[64];
  uint16_t avail = dbg_ring_avail();
  if (avail == 0U)
  {
    return;
  }

  uint16_t toSend = (avail > sizeof(chunk)) ? (uint16_t)sizeof(chunk) : avail;
  for (uint16_t i = 0; i < toSend; i++)
  {
    chunk[i] = dbg_ring_pop();
  }

  /* Polling transmit with short timeout (non-blocking-ish) */
  HAL_UART_Transmit(&huart1, chunk, toSend, 5);
}
