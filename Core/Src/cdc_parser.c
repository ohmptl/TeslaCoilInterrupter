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
#include "safety.h"
#include "scheduler.h"
#include "coil_driver.h"
#include "midi_engine.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

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
    uint8_t estop_state = Safety_IsEStopped();
    CDC_Printf(pdev, "ESTOP=%u\r\n", estop_state);
    Debug_Printf("[CDC] TX: ESTOP=%u", estop_state);
    return;
  }

  /* --- STATUS --- */
  if (strnicmp_local(cmd, "STATUS", 6) == 0 && len == 6)
  {
    uint8_t estop = Safety_IsEStopped();
    SchedulerStats_t ss;
    Scheduler_GetStats(&ss);
    CDC_Printf(pdev, "{\"fw\":\"%s\",\"estop\":%u,\"uptime_ms\":%lu,"
               "\"sched_running\":%u,\"sched_us\":%lu,\"sched_ticks\":%lu}\r\n",
               FW_VERSION, estop, HAL_GetTick(),
               ss.running, (unsigned long)ss.current_us,
               (unsigned long)ss.tick_count);
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

  /* ================================================================ */
  /*                   Milestone 2 Commands                           */
  /* ================================================================ */

  /* --- FIRE <coil> <freq_hz> <ontime_us> --- */
  if (strnicmp_local(cmd, "FIRE ", 5) == 0)
  {
    unsigned int coil = 0, freq = 0, ontime = 0;
    int parsed = sscanf(cmd + 5, "%u %u %u", &coil, &freq, &ontime);
    if (parsed != 3 || coil >= NUM_COILS || freq == 0 || ontime == 0)
    {
      CDC_Printf(pdev, "ERR:FIRE_ARGS (usage: FIRE <coil 0-5> <freq_hz> <ontime_us>)\r\n");
      return;
    }

    uint32_t period_us = 1000000U / freq;
    uint16_t clamped = Safety_ClampOntime((uint8_t)coil, (uint16_t)ontime);

    /* Use test-tone markers (note=0xFF, channel=0xFF) */
    int8_t slot = Scheduler_AddTone((uint8_t)coil, 0xFF, 0xFF,
                                     127, clamped, period_us);
    if (slot >= 0)
    {
      if (clamped != (uint16_t)ontime)
      {
        CDC_Printf(pdev, "OK:FIRE coil=%u freq=%u ontime=%u (clamped from %u)\r\n",
                   coil, freq, clamped, ontime);
      }
      else
      {
        CDC_Printf(pdev, "OK:FIRE coil=%u freq=%u ontime=%u\r\n",
                   coil, freq, clamped);
      }
      Debug_Printf("[CDC] FIRE coil=%u freq=%u ot=%u per=%lu slot=%d",
                   coil, freq, clamped, (unsigned long)period_us, (int)slot);
    }
    else
    {
      CDC_Printf(pdev, "ERR:FIRE_FULL (coil %u has no free voice slots)\r\n", coil);
    }
    return;
  }

  /* --- STOPALL --- (check before STOP to avoid prefix collision) */
  if (strnicmp_local(cmd, "STOPALL", 7) == 0 && len == 7)
  {
    Scheduler_RemoveAllTonesAllCoils();
    CDC_Printf(pdev, "OK:STOPALL\r\n");
    Debug_Log("[CDC] STOPALL");
    return;
  }

  /* --- STOP <coil> --- */
  if (strnicmp_local(cmd, "STOP ", 5) == 0)
  {
    unsigned int coil = 0;
    if (sscanf(cmd + 5, "%u", &coil) != 1 || coil >= NUM_COILS)
    {
      CDC_Printf(pdev, "ERR:STOP_ARGS (usage: STOP <coil 0-5>)\r\n");
      return;
    }
    Scheduler_RemoveAllTones((uint8_t)coil);
    CDC_Printf(pdev, "OK:STOP coil=%u\r\n", coil);
    Debug_Printf("[CDC] STOP coil=%u", coil);
    return;
  }

  /* --- LIMITS? <coil> --- */
  if (strnicmp_local(cmd, "LIMITS?", 7) == 0)
  {
    unsigned int coil = 0;
    if (len > 8 && sscanf(cmd + 8, "%u", &coil) == 1 && coil < NUM_COILS)
    {
      SafetyLimits_t lim;
      Safety_GetLimits((uint8_t)coil, &lim);
      CDC_Printf(pdev, "LIMITS coil=%u max_ontime=%u duty_permil=%u min_offtime=%u\r\n",
                 coil, lim.max_ontime_us, lim.max_duty_permil, lim.min_offtime_us);
    }
    else
    {
      CDC_Printf(pdev, "ERR:LIMITS?_ARGS (usage: LIMITS? <coil 0-5>)\r\n");
    }
    return;
  }

  /* --- LIMITS <coil> <max_ontime> <duty_permil> <min_offtime> --- */
  if (strnicmp_local(cmd, "LIMITS ", 7) == 0)
  {
    unsigned int coil = 0, max_ot = 0, duty = 0, min_off = 0;
    int parsed = sscanf(cmd + 7, "%u %u %u %u", &coil, &max_ot, &duty, &min_off);
    if (parsed != 4 || coil >= NUM_COILS)
    {
      CDC_Printf(pdev, "ERR:LIMITS_ARGS (usage: LIMITS <coil> <max_ot_us> <duty_permil> <min_off_us>)\r\n");
      return;
    }

    SafetyLimits_t lim;
    lim.max_ontime_us   = (uint16_t)max_ot;
    lim.max_duty_permil = (uint16_t)duty;
    lim.min_offtime_us  = (uint16_t)min_off;

    Safety_SetLimits((uint8_t)coil, &lim);
    Scheduler_UpdateLimitCache((uint8_t)coil);

    /* Read back actual (may have been clamped to absolute limits) */
    Safety_GetLimits((uint8_t)coil, &lim);
    CDC_Printf(pdev, "OK:LIMITS coil=%u max_ontime=%u duty_permil=%u min_offtime=%u\r\n",
               coil, lim.max_ontime_us, lim.max_duty_permil, lim.min_offtime_us);
    Debug_Printf("[CDC] LIMITS coil=%u ot=%u duty=%u off=%u",
                 coil, lim.max_ontime_us, lim.max_duty_permil, lim.min_offtime_us);
    return;
  }

  /* --- COILS? --- */
  if (strnicmp_local(cmd, "COILS?", 6) == 0 && len == 6)
  {
    CDC_Printf(pdev, "{\"num_coils\":%u,\"coils\":[", NUM_COILS);
    for (uint8_t c = 0; c < NUM_COILS; c++)
    {
      CoilSchedState_t cs;
      Scheduler_GetCoilState(c, &cs);
      uint8_t voices = Scheduler_GetActiveVoices(c);
      uint8_t active = CoilDriver_IsActive(c);
      if (c > 0) CDC_Printf(pdev, ",");
      CDC_Printf(pdev, "{\"id\":%u,\"en\":%u,\"voices\":%u,\"pulses\":%lu,\"active\":%u}",
                 c, cs.enabled, voices, (unsigned long)cs.pulse_count, active);
    }
    CDC_Printf(pdev, "]}\r\n");
    Debug_Log("[CDC] TX: COILS?");
    return;
  }

  /* --- SCHED? --- */
  if (strnicmp_local(cmd, "SCHED?", 6) == 0 && len == 6)
  {
    SchedulerStats_t ss;
    Scheduler_GetStats(&ss);
    CDC_Printf(pdev, "{\"running\":%u,\"ticks\":%lu,\"us\":%lu,\"coils\":[",
               ss.running, (unsigned long)ss.tick_count, (unsigned long)ss.current_us);
    for (uint8_t c = 0; c < NUM_COILS; c++)
    {
      CoilSchedState_t cs;
      Scheduler_GetCoilState(c, &cs);
      uint8_t voices = Scheduler_GetActiveVoices(c);
      if (c > 0) CDC_Printf(pdev, ",");
      CDC_Printf(pdev, "{\"id\":%u,\"en\":%u,\"voices\":%u,\"pulses\":%lu,"
                 "\"duty_us\":%lu,\"budget\":%lu}",
                 c, cs.enabled, voices, (unsigned long)cs.pulse_count,
                 (unsigned long)cs.duty_accum_us, (unsigned long)cs.max_ontime_window);
    }
    CDC_Printf(pdev, "]}\r\n");
    Debug_Log("[CDC] TX: SCHED?");
    return;
  }

  /* --- ROUTE <channel> <coil> --- */
  if (strnicmp_local(cmd, "ROUTE ", 6) == 0)
  {
    unsigned int channel = 0, coil = 0;
    int parsed = sscanf(cmd + 6, "%u %u", &channel, &coil);
    if (parsed != 2 || channel >= MIDI_MAX_CHANNELS)
    {
      CDC_Printf(pdev, "ERR:ROUTE_ARGS (usage: ROUTE <ch 0-15> <coil 0-5 or 255>)\r\n");
      return;
    }
    uint8_t coil_id = (coil >= NUM_COILS) ? MIDI_CHANNEL_UNMAP : (uint8_t)coil;
    MidiEngine_SetChannelCoil((uint8_t)channel, coil_id);
    CDC_Printf(pdev, "OK:ROUTE ch=%u coil=%u\r\n", channel, (unsigned)coil_id);
    Debug_Printf("[CDC] ROUTE ch=%u coil=%u", channel, (unsigned)coil_id);
    return;
  }

  /* --- Unknown command --- */
  CDC_Printf(pdev, "ERR:UNKNOWN_CMD\r\n");
  Debug_Printf("[CDC] Unknown cmd: %s", cmd);
}
