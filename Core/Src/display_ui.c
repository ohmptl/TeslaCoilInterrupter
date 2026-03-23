/**
  ******************************************************************************
  * @file    display_ui.c
  * @brief   OLED display UI renderer for the TC-Interrupter (u8g2 version).
  *
  *          Layout (128 x 64 pixels):
  *          ┌──────────────────────────────────────────────┐  y=0
  *          │  Header: SYS OK  or  *E-STOP*               │  12px
  *          ├──────────────────────────────────────────────┤  y=12
  *          │  1 zone  │  3 zone  │  5 zone               │
  *          │          │          │                        │  25px
  *          ├──────────────────────────────────────────────┤  y=38
  *          │  2 zone  │  4 zone  │  6 zone               │
  *          │          │          │                        │  26px
  *          └──────────────────────────────────────────────┘  y=64
  *
  *  Grid ordering: column-major (down-then-across) to align with QCW pairs:
  *    Col 1: Coil 1 (top), Coil 2 (bot) = QCW Ch 1
  *    Col 2: Coil 3 (top), Coil 4 (bot) = QCW Ch 2
  *    Col 3: Coil 5 (top), Coil 6 (bot) = QCW Ch 3
  *
  *  When QCW is active on a channel, its entire column is merged into
  *  a single tall block showing QCW-specific telemetry.
  *
  *  Uses u8g2 full-buffer mode for flicker-free rendering.
  *  Rate-limited to ~15 FPS via HAL_GetTick().
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "display_ui.h"
#include "ssd1309.h"
#include "scheduler.h"
#include "safety.h"
#include "coil_driver.h"
#include "cdc_parser.h"
#include "qcw.h"
#include "debug_uart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------*/
/*                         Private Defines                                    */
/* ---------------------------------------------------------------------------*/

/** Minimum milliseconds between display refreshes (~15 FPS). */
#define DISPLAY_REFRESH_INTERVAL_MS   66U

/** Layout constants */
#define HEADER_HEIGHT   12U
#define BODY_Y          (HEADER_HEIGHT + 1U)  /* 13px */
#define BODY_HEIGHT     (64U - BODY_Y)        /* 51px */
#define COIL_COLS       3U
#define COIL_ROWS       2U
#define COIL_ZONE_W     (128U / COIL_COLS)    /* 42px */
#define COIL_ZONE_H     (BODY_HEIGHT / COIL_ROWS) /* 25px */

/**
 * Column-major coil mapping:
 *   grid_pos 0 → coil 0 (user "1")  Col 0, Row 0  (top-left)
 *   grid_pos 1 → coil 1 (user "2")  Col 0, Row 1  (bot-left)
 *   grid_pos 2 → coil 2 (user "3")  Col 1, Row 0
 *   grid_pos 3 → coil 3 (user "4")  Col 1, Row 1
 *   grid_pos 4 → coil 4 (user "5")  Col 2, Row 0
 *   grid_pos 5 → coil 5 (user "6")  Col 2, Row 1
 */
static const uint8_t g_grid_to_coil[6] = { 0, 1, 2, 3, 4, 5 };
/* grid col = pos / 2,  grid row = pos % 2 */

/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/

/** Timestamp of last display refresh. */
static uint32_t s_last_refresh_ms = 0;

/** Previous pulse counts for activity detection. */
static uint32_t s_prev_pulse_count[NUM_COILS] = {0};

/** Activity flash state per coil. */
static uint8_t  s_activity_flash[NUM_COILS] = {0};

/** Boot state: 0 = showing splash, 1 = showing "Connected", 2 = main screen */
static uint8_t  s_boot_state = 0;

/** Timestamp when "Connected" was first shown. */
static uint32_t s_connected_shown_ms = 0;

/* ---------------------------------------------------------------------------*/
/*                     Private Helper Functions                               */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Render the status header bar.
 *         Minimal: just SYS OK or *E-STOP*.
 */
static void DisplayUI_DrawHeader(u8g2_t *u8g2)
{
  u8g2_SetFont(u8g2, u8g2_font_5x7_tr);

  if (Safety_IsEStopped())
  {
    const char *msg = "*E-STOP*";
    uint8_t w = u8g2_GetStrWidth(u8g2, msg);
    uint8_t x = (128 - w) / 2;
    /* Inverted E-STOP banner for visibility */
    u8g2_SetDrawColor(u8g2, 1);
    u8g2_DrawBox(u8g2, 0, 0, 128, HEADER_HEIGHT);
    u8g2_SetDrawColor(u8g2, 0);
    u8g2_DrawStr(u8g2, x, 9, msg);
    u8g2_SetDrawColor(u8g2, 1);
  }
  else
  {
    const char *msg = "SYS OK";
    uint8_t w = u8g2_GetStrWidth(u8g2, msg);
    uint8_t x = (128 - w) / 2;
    u8g2_DrawStr(u8g2, x, 9, msg);
  }

  /* Separator line under header */
  u8g2_DrawHLine(u8g2, 0, HEADER_HEIGHT, 128);
}

/**
 * @brief  Render a single coil zone (standard Wave/MIDI mode).
 *         Label is just the 1-based number, drawn bottom-right.
 */
static void DisplayUI_DrawCoilZone(u8g2_t *u8g2, uint8_t coil_id,
                                    uint8_t zone_x, uint8_t zone_y,
                                    uint8_t zone_w, uint8_t zone_h)
{
  char buf[16];
  uint8_t tx = zone_x + 2;     /* Text X offset within zone */
  uint8_t ty = zone_y + 8;     /* First text baseline */

  /* Ensure text color and font are consistently clean per zone */
  u8g2_SetDrawColor(u8g2, 1);
  u8g2_SetFont(u8g2, u8g2_font_5x7_tr);

  /* --- 1-based coil number label (bottom-right of zone) --- */
  snprintf(buf, sizeof(buf), "%u", coil_id + 1U);
  {
    uint8_t lbl_w = u8g2_GetStrWidth(u8g2, buf);
    u8g2_DrawStr(u8g2, zone_x + zone_w - lbl_w - 2,
                 zone_y + zone_h - 2, buf);
  }

  /* Get coil state */
  CoilSchedState_t cs;
  Scheduler_GetCoilState(coil_id, &cs);
  uint8_t voices = Scheduler_GetActiveVoices(coil_id);

  uint8_t ui_mode = Scheduler_GetUIMode(coil_id);

  if (ui_mode == 0 || !cs.enabled)
  {
    u8g2_DrawStr(u8g2, tx, ty, "OFF");
  }
  else if (ui_mode == 1) /* Pulse */
  {
    /* Check if a test tone is active */
    uint8_t is_wave = 0;
    uint32_t freq_hz = 0;
    uint16_t ontime_us = 0;
    for (uint8_t v = 0; v < MAX_VOICES_PER_COIL; v++)
    {
      SchedulerTone_t tone;
      Scheduler_GetToneInfo(coil_id, v, &tone);
      if (tone.active && tone.midi_note == 0xFF)
      {
        is_wave = 1;
        if (tone.period_us > 0) freq_hz = 1000000U / tone.period_us;
        ontime_us = tone.ontime_us;
        break;
      }
    }

    if (is_wave)
    {
      snprintf(buf, sizeof(buf), "%luHz", (unsigned long)freq_hz);
      u8g2_DrawStr(u8g2, tx, ty, buf);
      snprintf(buf, sizeof(buf), "%uus", ontime_us);
      u8g2_DrawStr(u8g2, tx, ty + 9, buf);
    }
    else
    {
      u8g2_DrawStr(u8g2, tx, ty, "Pulse");
    }
  }
  else if (ui_mode == 2) /* MIDI */
  {
    u8g2_DrawStr(u8g2, tx, ty, "MIDI");
    if (voices > 0)
    {
      snprintf(buf, sizeof(buf), "%uv", voices);
      u8g2_DrawStr(u8g2, tx, ty + 9, buf);
    }
  }
  else if (ui_mode == 3) /* QCW */
  {
    u8g2_DrawStr(u8g2, tx, ty, "QCW");
  }

  /* Activity pulse indicator (top-right dot) */
  {
    uint32_t current_pulses = cs.pulse_count;
    if (current_pulses != s_prev_pulse_count[coil_id])
    {
      s_activity_flash[coil_id] = 1U;
      s_prev_pulse_count[coil_id] = current_pulses;
    }
    else
    {
      s_activity_flash[coil_id] = 0U; /* Clear if no pulses occurred since last frame */
    }

    if (s_activity_flash[coil_id])
    {
      u8g2_DrawBox(u8g2, zone_x + zone_w - 7, zone_y + 2, 5, 5);
    }
  }
}

/**
 * @brief  Render a QCW merged column (two coil zones merged vertically).
 */
static void DisplayUI_DrawQCWColumn(u8g2_t *u8g2, uint8_t qcw_ch,
                                     uint8_t col_x, uint8_t col_w)
{
  char buf[16];
  uint8_t full_h = BODY_HEIGHT;
  uint8_t tx = col_x + 2;
  uint8_t ty = BODY_Y + 8;

  u8g2_SetFont(u8g2, u8g2_font_5x7_tr);

  /* QCW channel label */
  snprintf(buf, sizeof(buf), "QCW %u", qcw_ch + 1U);
  u8g2_DrawStr(u8g2, tx, ty, buf);

  /* Get QCW state */
  QCW_Channel_t qstate;
  QCW_GetChannelState(qcw_ch, &qstate);

  /* State text */
  const char *state_str = "IDLE";
  switch (qstate.state)
  {
    case QCW_STATE_RAMP_UP:   state_str = "RAMP UP"; break;
    case QCW_STATE_HOLD:      state_str = "HOLD";    break;
    case QCW_STATE_RAMP_DOWN: state_str = "RAMP DN"; break;
    default:                  state_str = "IDLE";    break;
  }
  u8g2_DrawStr(u8g2, tx, ty + 10, state_str);

  /* Current duty cycle */
  snprintf(buf, sizeof(buf), "%u.%u%%",
           qstate.current_duty / 10U,
           qstate.current_duty % 10U);
  u8g2_DrawStr(u8g2, tx, ty + 20, buf);

  /* Column number labels at bottom-right */
  {
    uint8_t coil_top = qcw_ch * 2;       /* 0-based */
    uint8_t coil_bot = qcw_ch * 2 + 1;
    snprintf(buf, sizeof(buf), "%u/%u", coil_top + 1, coil_bot + 1);
    uint8_t lbl_w = u8g2_GetStrWidth(u8g2, buf);
    u8g2_DrawStr(u8g2, col_x + col_w - lbl_w - 2,
                 BODY_Y + full_h - 2, buf);
  }

  /* Vertical separator on right edge (unless last column) */
  if (qcw_ch < (QCW_NUM_CHANNELS - 1))
  {
    u8g2_DrawVLine(u8g2, col_x + col_w - 1, BODY_Y, full_h);
  }
}

/**
 * @brief  Render the 6-coil body area with QCW column merging.
 */
static void DisplayUI_DrawBody(u8g2_t *u8g2)
{
  for (uint8_t col = 0; col < COIL_COLS; col++)
  {
    uint8_t col_x = col * COIL_ZONE_W;

    /* Check if this column's QCW channel is active */
    if (QCW_IsActive(col))  /* col index == qcw_ch index */
    {
      /* Draw merged QCW column (no horizontal separator) */
      DisplayUI_DrawQCWColumn(u8g2, col, col_x, COIL_ZONE_W);
    }
    else
    {
      /* Draw two normal coil zones (top and bottom) */
      uint8_t coil_top = col * 2;       /* Column-major: col*2 = top */
      uint8_t coil_bot = col * 2 + 1;   /* col*2+1 = bottom */

      uint8_t zy_top = BODY_Y;
      uint8_t zy_bot = BODY_Y + COIL_ZONE_H + 1;  /* +1 for separator */

      DisplayUI_DrawCoilZone(u8g2, coil_top, col_x, zy_top,
                             COIL_ZONE_W, COIL_ZONE_H);
      DisplayUI_DrawCoilZone(u8g2, coil_bot, col_x, zy_bot,
                             COIL_ZONE_W, COIL_ZONE_H);

      /* Horizontal separator between top and bottom zones */
      u8g2_DrawHLine(u8g2, col_x, BODY_Y + COIL_ZONE_H, COIL_ZONE_W);

      /* Vertical separator (right edge, unless last column) */
      if (col < (COIL_COLS - 1))
      {
        u8g2_DrawVLine(u8g2, col_x + COIL_ZONE_W - 1, BODY_Y, BODY_HEIGHT);
      }
    }
  }

  /* Bottom edge only — no left/right/top borders to avoid doubling
     with header HLine or column-0 content at X=0. */
  u8g2_DrawHLine(u8g2, 0, 63, 128);
}

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

void DisplayUI_Init(void)
{
  /* Staggered boot delay: prevent inrush current brownout on 3.3V rail. */
  HAL_Delay(200);

  SSD1309_Init();
  Debug_Log("[DISP] OLED initialized (SSD1309 128x64, u8g2, SPI2)");

  /* ---- Boot splash: clean, minimal, no overlapping borders ---- */
  u8g2_t *u8g2 = SSD1309_GetU8g2();
  u8g2_ClearBuffer(u8g2);

  /* "ArcOS" title — inverted bar centered on screen */
  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  const char *title = "ArcOS";
  uint8_t tw = u8g2_GetStrWidth(u8g2, title);
  uint8_t bar_w = tw + 16;                 /* padding around text */
  uint8_t bar_x = (128 - bar_w) / 2;
  uint8_t bar_y = 18;
  uint8_t bar_h = 14;

  u8g2_SetDrawColor(u8g2, 1);
  u8g2_DrawBox(u8g2, bar_x, bar_y, bar_w, bar_h);  /* solid white bar */
  u8g2_SetDrawColor(u8g2, 0);
  u8g2_DrawStr(u8g2, (128 - tw) / 2, bar_y + 11, title);  /* black text */
  u8g2_SetDrawColor(u8g2, 1);

  /* "Waiting for GUI..." status line below */
  u8g2_SetFont(u8g2, u8g2_font_5x7_tr);
  const char *stat = "Waiting for GUI...";
  uint8_t sw = u8g2_GetStrWidth(u8g2, stat);
  u8g2_DrawStr(u8g2, (128 - sw) / 2, 48, stat);

  u8g2_SendBuffer(u8g2);

  s_boot_state = 0;   /* Waiting for GUI */
  s_last_refresh_ms = HAL_GetTick();
}

void DisplayUI_Update(void)
{
  uint32_t now = HAL_GetTick();

  /* Rate-limit to ~15 FPS */
  if ((now - s_last_refresh_ms) < DISPLAY_REFRESH_INTERVAL_MS)
    return;

  s_last_refresh_ms = now;

  u8g2_t *u8g2 = SSD1309_GetU8g2();

  /* --- Boot state machine --- */
  if (s_boot_state == 0)
  {
    /* Waiting for GUI connection */
    if (CDC_Parser_IsGUIConnected())
    {
      /* Flash "Connected" briefly */
      u8g2_ClearBuffer(u8g2);
      u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
      u8g2_DrawStr(u8g2, 28, 36, "Connected");
      u8g2_SendBuffer(u8g2);

      s_connected_shown_ms = now;
      s_boot_state = 1;
    }
    /* Otherwise keep showing splash — already drawn in Init */
    return;
  }
  else if (s_boot_state == 1)
  {
    /* Show "Connected" for 500ms, then proceed */
    if ((now - s_connected_shown_ms) >= 500U)
    {
      s_boot_state = 2;
    }
    return;
  }

  /* --- Main display (boot_state == 2) --- */
  u8g2_ClearBuffer(u8g2);
  DisplayUI_DrawHeader(u8g2);
  DisplayUI_DrawBody(u8g2);
  u8g2_SendBuffer(u8g2);
}
