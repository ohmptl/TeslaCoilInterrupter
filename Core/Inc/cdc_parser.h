/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : cdc_parser.h
  * @brief          : CDC command parser for the Tesla Coil Interrupter.
  *
  * Receives newline-terminated ASCII commands from the CDC Virtual COM Port
  * and dispatches responses.  The parser is designed to be polled from the
  * main loop for deterministic behavior (no ISR processing of commands).
  *
  * Priority Guarantee:
  *   The CDC ring buffer in usbd_composite is filled from the USB ISR and
  *   drained here in the main loop BEFORE any MIDI data is processed,
  *   ensuring control commands are never starved by MIDI floods.
  *
  * Supported commands (Milestone 1):
  *   PING          -> "PONG\r\n"
  *   VERSION       -> "TC-Interrupter v1.0.0\r\n"
  *   ESTOP?        -> "ESTOP=0\r\n" or "ESTOP=1\r\n"
  *   STATUS        -> JSON-like status string
  *   ECHO <text>   -> "<text>\r\n"
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CDC_PARSER_H
#define __CDC_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------*/
/*                            Configuration                                   */
/* ---------------------------------------------------------------------------*/
#define CDC_PARSER_MAX_CMD_LEN     128U   /* Max command line length (bytes) */

/* ---------------------------------------------------------------------------*/
/*                            Public API                                      */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialize the CDC command parser.
 *         Must be called once before CDC_Parser_Process().
 */
void CDC_Parser_Init(void);

/**
 * @brief  Poll and process incoming CDC data.
 *         Call this from the main while(1) loop.
 *         Drains the CDC RX ring buffer, assembles lines, and dispatches.
 * @param  pdev  USB device handle
 */
void CDC_Parser_Process(USBD_HandleTypeDef *pdev);

/**
 * @brief  Send a printf-style formatted string via CDC.
 *         Non-blocking (uses the TX ring buffer).
 * @param  pdev  USB device handle
 * @param  fmt   printf format string
 * @retval Number of bytes queued
 */
int CDC_Printf(USBD_HandleTypeDef *pdev, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __CDC_PARSER_H */
