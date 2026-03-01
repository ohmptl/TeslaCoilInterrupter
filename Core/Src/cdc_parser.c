/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : cdc_parser.c
  * @brief          : CDC command parser implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cdc_parser.h"
#include "usbd_composite.h"
#include "debug_uart.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/
static char cmd_buf[CDC_PARSER_MAX_CMD_LEN];
static uint16_t cmd_pos = 0;

/* Firmware version string */
#define FW_VERSION  "TC-Interrupter v1.0.0"

/* ---------------------------------------------------------------------------*/
/*                     Forward Declarations                                   */
/* ---------------------------------------------------------------------------*/
static void CDC_Parser_Dispatch(USBD_HandleTypeDef *pdev, const char *cmd, uint16_t len);

/* ---------------------------------------------------------------------------*/
/*                           Public API                                       */
/* ---------------------------------------------------------------------------*/

void CDC_Parser_Init(void)
{
  cmd_pos = 0;
  memset(cmd_buf, 0, sizeof(cmd_buf));
}

void CDC_Parser_Process(USBD_HandleTypeDef *pdev)
{
  uint8_t byte;

  /* Drain the CDC RX ring buffer, assembling lines */
  while (USBD_Composite_CDC_ReadByte(pdev, &byte))
  {
    if (byte == '\n' || byte == '\r')
    {
      if (cmd_pos > 0)
      {
        /* Null-terminate and dispatch */
        cmd_buf[cmd_pos] = '\0';
        CDC_Parser_Dispatch(pdev, cmd_buf, cmd_pos);
        cmd_pos = 0;
      }
      /* Ignore empty lines / lone CR/LF */
    }
    else
    {
      if (cmd_pos < (CDC_PARSER_MAX_CMD_LEN - 1U))
      {
        cmd_buf[cmd_pos++] = (char)byte;
      }
      else
      {
        /* Line too long — reset and discard */
        Debug_Log("[CDC] ERR: Command too long, discarding");
        cmd_pos = 0;
      }
    }
  }
}

int CDC_Printf(USBD_HandleTypeDef *pdev, const char *fmt, ...)
{
  char buf[256];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len > 0)
  {
    uint16_t queued = USBD_Composite_CDC_Transmit(pdev, (const uint8_t *)buf, (uint16_t)len);
    return (int)queued;
  }
  return 0;
}

/* ---------------------------------------------------------------------------*/
/*                  Command Dispatch (Private)                                */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  String comparison helper (case-insensitive, up to n chars).
 */
static int strnicmp_local(const char *a, const char *b, int n)
{
  for (int i = 0; i < n; i++)
  {
    char ca = a[i];
    char cb = b[i];
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return (int)(ca - cb);
    if (ca == '\0') return 0;
  }
  return 0;
}

/**
 * @brief  Dispatch a completed command line.
 */
static void CDC_Parser_Dispatch(USBD_HandleTypeDef *pdev, const char *cmd, uint16_t len)
{
  Debug_Printf("[CDC] RX cmd: %s", cmd);

  /* --- PING --- */
  if (strnicmp_local(cmd, "PING", 4) == 0 && len == 4)
  {
    CDC_Printf(pdev, "PONG\r\n");
    Debug_Log("[CDC] TX: PONG");
    return;
  }

  /* --- VERSION --- */
  if (strnicmp_local(cmd, "VERSION", 7) == 0 && len == 7)
  {
    CDC_Printf(pdev, "%s\r\n", FW_VERSION);
    Debug_Printf("[CDC] TX: %s", FW_VERSION);
    return;
  }

  /* --- ESTOP? --- */
  if (strnicmp_local(cmd, "ESTOP?", 6) == 0)
  {
    uint8_t estop_state = !HAL_GPIO_ReadPin(ESTOP_BUTTON_GPIO_Port, ESTOP_BUTTON_Pin);
    CDC_Printf(pdev, "ESTOP=%u\r\n", estop_state);
    Debug_Printf("[CDC] TX: ESTOP=%u", estop_state);
    return;
  }

  /* --- STATUS --- */
  if (strnicmp_local(cmd, "STATUS", 6) == 0 && len == 6)
  {
    uint8_t estop = !HAL_GPIO_ReadPin(ESTOP_BUTTON_GPIO_Port, ESTOP_BUTTON_Pin);
    CDC_Printf(pdev, "{\"fw\":\"%s\",\"estop\":%u,\"uptime_ms\":%lu}\r\n",
               FW_VERSION, estop, HAL_GetTick());
    Debug_Log("[CDC] TX: STATUS");
    return;
  }

  /* --- ECHO <text> --- */
  if (strnicmp_local(cmd, "ECHO ", 5) == 0 && len > 5)
  {
    CDC_Printf(pdev, "%s\r\n", cmd + 5);
    Debug_Printf("[CDC] TX echo: %s", cmd + 5);
    return;
  }

  /* --- Unknown command --- */
  CDC_Printf(pdev, "ERR:UNKNOWN_CMD\r\n");
  Debug_Printf("[CDC] Unknown cmd: %s", cmd);
}
