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
#include "qcw.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/
static char cmd_buf[CDC_PARSER_MAX_CMD_LEN];
static uint16_t cmd_pos = 0;

/** Set to 1 after the first CDC command is received (GUI connected). */
static volatile uint8_t g_gui_connected = 0;

/* Firmware version string */
#define FW_VERSION  "TC-Interrupter v2.0.0"

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
  g_gui_connected = 0;
  memset(cmd_buf, 0, sizeof(cmd_buf));
}

uint8_t CDC_Parser_IsGUIConnected(void)
{
  return g_gui_connected;
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
        g_gui_connected = 1;  /* GUI has sent its first command */
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
  /*                   Milestone 2/3 Commands                         */
  /* ================================================================ */

  /* --- MODE <coil> <mode_id> --- */
  if (strnicmp_local(cmd, "MODE ", 5) == 0)
  {
    unsigned int coil = 0, mode = 0;
    int parsed = sscanf(cmd + 5, "%u %u", &coil, &mode);
    if (parsed != 2 || coil == 0 || coil > NUM_COILS)
    {
      CDC_Printf(pdev, "ERR:MODE_ARGS (usage: MODE <coil 1-6> <mode 0=Off 1=Pulse 2=MIDI 3=QCW>)\r\n");
      return;
    }
    uint8_t c_idx = (uint8_t)(coil - 1);
    Scheduler_SetUIMode(c_idx, (uint8_t)mode);
    CDC_Printf(pdev, "OK:MODE coil=%u mode=%u\r\n", coil, mode);
    Debug_Printf("[CDC] MODE coil=%u mode=%u", coil, mode);
    return;
  }

  /* --- FIRE <coil> <freq_hz> <ontime_us> --- */
  if (strnicmp_local(cmd, "FIRE ", 5) == 0)
  {
    unsigned int coil = 0, freq = 0, ontime = 0;
    int parsed = sscanf(cmd + 5, "%u %u %u", &coil, &freq, &ontime);
    if (parsed != 3 || coil == 0 || coil > NUM_COILS || freq == 0 || ontime == 0)
    {
      CDC_Printf(pdev, "ERR:FIRE_ARGS (usage: FIRE <coil 1-6> <freq_hz> <ontime_us>)\r\n");
      return;
    }

    uint8_t c_idx = (uint8_t)(coil - 1);
    uint32_t period_us = 1000000U / freq;
    uint16_t clamped = Safety_ClampOntime(c_idx, (uint16_t)ontime);

    /* Use test-tone markers (note=0xFF, channel=0xFF) */
    int8_t slot = Scheduler_AddTone(c_idx, 0xFF, 0xFF,
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
    if (sscanf(cmd + 5, "%u", &coil) != 1 || coil == 0 || coil > NUM_COILS)
    {
      CDC_Printf(pdev, "ERR:STOP_ARGS (usage: STOP <coil 1-6>)\r\n");
      return;
    }
    uint8_t c_idx = (uint8_t)(coil - 1);
    Scheduler_RemoveAllTones(c_idx);
    CDC_Printf(pdev, "OK:STOP coil=%u\r\n", coil);
    Debug_Printf("[CDC] STOP coil=%u", coil);
    return;
  }

  /* --- LIMITS? <coil> --- */
  if (strnicmp_local(cmd, "LIMITS?", 7) == 0)
  {
    unsigned int coil = 0;
    if (len > 8 && sscanf(cmd + 8, "%u", &coil) == 1 && coil > 0 && coil <= NUM_COILS)
    {
      uint8_t c_idx = (uint8_t)(coil - 1);
      SafetyLimits_t lim;
      Safety_GetLimits(c_idx, &lim);
      CDC_Printf(pdev, "LIMITS coil=%u max_ontime=%u duty_permil=%u min_offtime=%u\r\n",
                 coil, lim.max_ontime_us, lim.max_duty_permil, lim.min_offtime_us);
    }
    else
    {
      CDC_Printf(pdev, "ERR:LIMITS?_ARGS (usage: LIMITS? <coil 1-6>)\r\n");
    }
    return;
  }

  /* --- LIMITS <coil> <max_ontime> <duty_permil> <min_offtime> --- */
  if (strnicmp_local(cmd, "LIMITS ", 7) == 0)
  {
    unsigned int coil = 0, max_ot = 0, duty = 0, min_off = 0;
    int parsed = sscanf(cmd + 7, "%u %u %u %u", &coil, &max_ot, &duty, &min_off);
    if (parsed != 4 || coil == 0 || coil > NUM_COILS)
    {
      CDC_Printf(pdev, "ERR:LIMITS_ARGS (usage: LIMITS <coil 1-6> <max_ot_us> <duty_permil> <min_off_us>)\r\n");
      return;
    }

    uint8_t c_idx = (uint8_t)(coil - 1);
    SafetyLimits_t lim;
    lim.max_ontime_us   = (uint16_t)max_ot;
    lim.max_duty_permil = (uint16_t)duty;
    lim.min_offtime_us  = (uint16_t)min_off;

    Safety_SetLimits(c_idx, &lim);
    Scheduler_UpdateLimitCache(c_idx);

    /* Read back actual (may have been clamped to absolute limits) */
    Safety_GetLimits(c_idx, &lim);
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
                 c + 1U, cs.enabled, voices, (unsigned long)cs.pulse_count, active);
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
                 c + 1U, cs.enabled, voices, (unsigned long)cs.pulse_count,
                 (unsigned long)cs.duty_accum_us, (unsigned long)cs.max_ontime_window);
    }
    CDC_Printf(pdev, "]}\r\n");
    Debug_Log("[CDC] TX: SCHED?");
    return;
  }

  /* --- ROUTE? --- (query full routing table, check before ROUTE) */
  if (strnicmp_local(cmd, "ROUTE?", 6) == 0 && len == 6)
  {
    CDC_Printf(pdev, "{\"routes\":[");
    for (uint8_t ch = 0; ch < MIDI_MAX_CHANNELS; ch++)
    {
      uint8_t coil = MidiEngine_GetChannelCoil(ch);
      if (ch > 0) CDC_Printf(pdev, ",");
      
      uint8_t out_coil = (coil == MIDI_CHANNEL_UNMAP) ? 255 : (coil + 1U);
      CDC_Printf(pdev, "{\"ch\":%u,\"coil\":%u}", ch + 1U, (unsigned)out_coil);
    }
    CDC_Printf(pdev, "]}\r\n");
    Debug_Log("[CDC] TX: ROUTE?");
    return;
  }

  /* --- ROUTE <channel> <coil> --- */
  if (strnicmp_local(cmd, "ROUTE ", 6) == 0)
  {
    unsigned int channel = 0, coil = 0;
    int parsed = sscanf(cmd + 6, "%u %u", &channel, &coil);
    if (parsed != 2 || channel == 0 || channel > MIDI_MAX_CHANNELS)
    {
      CDC_Printf(pdev, "ERR:ROUTE_ARGS (usage: ROUTE <ch 1-16> <coil 1-6 or 255>)\r\n");
      return;
    }
    uint8_t ch_idx = (uint8_t)(channel - 1);
    uint8_t coil_idx = (coil == 0 || coil > NUM_COILS) ? MIDI_CHANNEL_UNMAP : (uint8_t)(coil - 1);
    MidiEngine_SetChannelCoil(ch_idx, coil_idx);
    CDC_Printf(pdev, "OK:ROUTE ch=%u coil=%u\r\n", channel, (unsigned)coil);
    Debug_Printf("[CDC] ROUTE ch=%u coil=%u", channel, (unsigned)coil);
    return;
  }

  /* ================================================================ */
  /*                   Milestone 3 Commands                           */
  /* ================================================================ */

  /* --- ENABLE <coil> --- */
  if (strnicmp_local(cmd, "ENABLE ", 7) == 0)
  {
    unsigned int coil = 0;
    if (sscanf(cmd + 7, "%u", &coil) != 1 || coil == 0 || coil > NUM_COILS)
    {
      CDC_Printf(pdev, "ERR:ENABLE_ARGS (usage: ENABLE <coil 1-6>)\r\n");
      return;
    }
    uint8_t c_idx = (uint8_t)(coil - 1);
    Scheduler_SetCoilEnabled(c_idx, 1U);
    CDC_Printf(pdev, "OK:ENABLE coil=%u\r\n", coil);
    Debug_Printf("[CDC] ENABLE coil=%u", coil);
    return;
  }

  /* --- DISABLE <coil> --- */
  if (strnicmp_local(cmd, "DISABLE ", 8) == 0)
  {
    unsigned int coil = 0;
    if (sscanf(cmd + 8, "%u", &coil) != 1 || coil == 0 || coil > NUM_COILS)
    {
      CDC_Printf(pdev, "ERR:DISABLE_ARGS (usage: DISABLE <coil 1-6>)\r\n");
      return;
    }
    uint8_t c_idx = (uint8_t)(coil - 1);
    Scheduler_RemoveAllTones(c_idx);
    Scheduler_SetCoilEnabled(c_idx, 0U);
    CDC_Printf(pdev, "OK:DISABLE coil=%u\r\n", coil);
    Debug_Printf("[CDC] DISABLE coil=%u", coil);
    return;
  }

  /* --- ROUTEALL <coil> --- route all 16 MIDI channels to one coil */
  if (strnicmp_local(cmd, "ROUTEALL ", 9) == 0)
  {
    unsigned int coil = 0;
    if (sscanf(cmd + 9, "%u", &coil) != 1)
    {
      CDC_Printf(pdev, "ERR:ROUTEALL_ARGS (usage: ROUTEALL <coil 1-6 or 255>)\r\n");
      return;
    }
    uint8_t coil_idx = (coil == 0 || coil > NUM_COILS) ? MIDI_CHANNEL_UNMAP : (uint8_t)(coil - 1);
    for (uint8_t ch = 0; ch < MIDI_MAX_CHANNELS; ch++)
    {
      MidiEngine_SetChannelCoil(ch, coil_idx);
    }
    CDC_Printf(pdev, "OK:ROUTEALL coil=%u\r\n", (unsigned)coil);
    Debug_Printf("[CDC] ROUTEALL coil=%u", (unsigned)coil);
    return;
  }

  /* --- ROUTERESET --- restore default routing (ch N -> coil N for 0-5) */
  if (strnicmp_local(cmd, "ROUTERESET", 10) == 0 && len == 10)
  {
    for (uint8_t ch = 0; ch < MIDI_MAX_CHANNELS; ch++)
    {
      MidiEngine_SetChannelCoil(ch, (ch < NUM_COILS) ? ch : MIDI_CHANNEL_UNMAP);
    }
    CDC_Printf(pdev, "OK:ROUTERESET\r\n");
    Debug_Log("[CDC] ROUTERESET");
    return;
  }

  /* ================================================================ */
  /*                   QCW Commands                                    */
  /* ================================================================ */

  /* --- QCW? --- (query QCW channel states, check before QCW_MODE etc.) */
  if (strnicmp_local(cmd, "QCW?", 4) == 0 && len == 4)
  {
    CDC_Printf(pdev, "{\"qcw_channels\":%u,\"channels\":[", QCW_NUM_CHANNELS);
    for (uint8_t i = 0; i < QCW_NUM_CHANNELS; i++)
    {
      QCW_Channel_t st;
      QCW_GetChannelState(i, &st);
      if (i > 0) CDC_Printf(pdev, ",");
      CDC_Printf(pdev, "{\"ch\":%u,\"active\":%u,\"state\":%u,\"duty\":%u,"
                 "\"tmin1\":%u,\"tmax\":%u,\"tmin2\":%u,"
                 "\"tramp1_ms\":%u,\"tramp2_ms\":%u,\"thold_ms\":%u}",
                 i + 1, st.active, (unsigned)st.state, st.current_duty,
                 st.config.tmin1, st.config.tmax, st.config.tmin2,
                 st.config.tramp1_ms, st.config.tramp2_ms, st.config.thold_ms);
    }
    CDC_Printf(pdev, "]}\r\n");
    Debug_Log("[CDC] TX: QCW?");
    return;
  }

  /* --- QCW_MODE <ch> <0|1> --- */
  if (strnicmp_local(cmd, "QCW_MODE ", 9) == 0)
  {
    unsigned int ch_1based = 0, state = 0;
    int parsed = sscanf(cmd + 9, "%u %u", &ch_1based, &state);
    if (parsed != 2 || ch_1based < 1 || ch_1based > QCW_NUM_CHANNELS)
    {
      CDC_Printf(pdev, "ERR:QCW_MODE_ARGS (usage: QCW_MODE <ch 1-%u> <0|1>)\r\n",
                 QCW_NUM_CHANNELS);
      return;
    }
    uint8_t ch_0 = (uint8_t)(ch_1based - 1);
    int8_t rc = QCW_SetMode(ch_0, state ? 1 : 0);
    if (rc == 0)
    {
      CDC_Printf(pdev, "OK:QCW_MODE ch=%u state=%u\r\n", ch_1based, state ? 1 : 0);
      Debug_Printf("[CDC] QCW_MODE ch=%u state=%u", ch_1based, state ? 1 : 0);
    }
    else
    {
      CDC_Printf(pdev, "ERR:QCW_MODE_FAIL ch=%u\r\n", ch_1based);
    }
    return;
  }

  /* --- QCW_CONFIG <ch> <tmin1> <tmax> <tmin2> <tramp1_ms> <tramp2_ms> <thold_ms> --- */
  if (strnicmp_local(cmd, "QCW_CONFIG ", 11) == 0)
  {
    unsigned int ch_1based = 0, tmin1 = 0, tmax = 0, tmin2 = 0;
    unsigned int tramp1 = 0, tramp2 = 0, thold = 0;
    int parsed = sscanf(cmd + 11, "%u %u %u %u %u %u %u",
                        &ch_1based, &tmin1, &tmax, &tmin2,
                        &tramp1, &tramp2, &thold);
    if (parsed < 6 || ch_1based < 1 || ch_1based > QCW_NUM_CHANNELS)
    {
      CDC_Printf(pdev, "ERR:QCW_CONFIG_ARGS (usage: QCW_CONFIG <ch> "
                 "<tmin1> <tmax> <tmin2> <tramp1_ms> <tramp2_ms> [thold_ms])\r\n");
      return;
    }
    /* thold is optional (default 0) */
    if (parsed < 7) thold = 0;

    QCW_Config_t cfg;
    cfg.tmin1     = (uint16_t)tmin1;
    cfg.tmax      = (uint16_t)tmax;
    cfg.tmin2     = (uint16_t)tmin2;
    cfg.tramp1_ms = (uint16_t)tramp1;
    cfg.tramp2_ms = (uint16_t)tramp2;
    cfg.thold_ms  = (uint16_t)thold;

    uint8_t ch_0 = (uint8_t)(ch_1based - 1);
    int8_t rc = QCW_Configure(ch_0, &cfg);
    if (rc == 0)
    {
      CDC_Printf(pdev, "OK:QCW_CONFIG ch=%u tmin1=%u tmax=%u tmin2=%u "
                 "tramp1=%u tramp2=%u thold=%u\r\n",
                 ch_1based, tmin1, tmax, tmin2, tramp1, tramp2, thold);
      Debug_Printf("[CDC] QCW_CONFIG ch=%u", ch_1based);
    }
    else
    {
      CDC_Printf(pdev, "ERR:QCW_CONFIG_FAIL ch=%u\r\n", ch_1based);
    }
    return;
  }

  /* --- QCW_FIRE <ch> --- */
  if (strnicmp_local(cmd, "QCW_FIRE ", 9) == 0)
  {
    unsigned int ch_1based = 0;
    if (sscanf(cmd + 9, "%u", &ch_1based) != 1 ||
        ch_1based < 1 || ch_1based > QCW_NUM_CHANNELS)
    {
      CDC_Printf(pdev, "ERR:QCW_FIRE_ARGS (usage: QCW_FIRE <ch 1-%u>)\r\n",
                 QCW_NUM_CHANNELS);
      return;
    }
    uint8_t ch_0 = (uint8_t)(ch_1based - 1);
    int8_t rc = QCW_Fire(ch_0);
    if (rc == 0)
    {
      CDC_Printf(pdev, "OK:QCW_FIRE ch=%u\r\n", ch_1based);
      Debug_Printf("[CDC] QCW_FIRE ch=%u", ch_1based);
    }
    else
    {
      CDC_Printf(pdev, "ERR:QCW_FIRE_FAIL ch=%u (not active, already firing, or E-Stop)\r\n",
                 ch_1based);
    }
    return;
  }

  /* --- QCW_ABORT <ch> --- (check before generic abort) */
  if (strnicmp_local(cmd, "QCW_ABORT ", 10) == 0)
  {
    unsigned int ch_1based = 0;
    if (sscanf(cmd + 10, "%u", &ch_1based) != 1 ||
        ch_1based < 1 || ch_1based > QCW_NUM_CHANNELS)
    {
      CDC_Printf(pdev, "ERR:QCW_ABORT_ARGS (usage: QCW_ABORT <ch 1-%u>)\r\n",
                 QCW_NUM_CHANNELS);
      return;
    }
    uint8_t ch_0 = (uint8_t)(ch_1based - 1);
    QCW_Abort(ch_0);
    CDC_Printf(pdev, "OK:QCW_ABORT ch=%u\r\n", ch_1based);
    Debug_Printf("[CDC] QCW_ABORT ch=%u", ch_1based);
    return;
  }

  /* --- Unknown command --- */
  CDC_Printf(pdev, "ERR:UNKNOWN_CMD\r\n");
  Debug_Printf("[CDC] Unknown cmd: %s", cmd);
}
