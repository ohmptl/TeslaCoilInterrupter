/**
  ******************************************************************************
  * @file    display_ui.c
  * @brief   OLED display UI renderer for the TC-Interrupter (u8g2 version).
  *
  *          Layout (128 x 64 pixels):
  *          ┌──────────────────────────────────────────────┐  y=0
  *          │  Header: [SYS OK] / *E-STOP*  | USB | uptime│  12px
  *          ├──────────────────────────────────────────────┤  y=12
  *          │  C0 zone  │  C1 zone  │  C2 zone            │
  *          │           │           │                      │  25px
  *          ├──────────────────────────────────────────────┤  y=38
  *          │  C3 zone  │  C4 zone  │  C5 zone            │
  *          │           │           │                      │  26px
  *          └──────────────────────────────────────────────┘  y=64
  *
  *  Each coil zone shows:
  *    - Wave mode (test tone, midi_note=0xFF): frequency + on-time
  *    - MIDI mode: voice count
  *    - Activity pulse: block that toggles when pulse_count changes
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
#include "debug_uart.h"
#include "main.h"
#include "usbd_def.h"
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

/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/

/** Timestamp of last display refresh. */
static uint32_t s_last_refresh_ms = 0;

/** Previous pulse counts for activity detection. */
static uint32_t s_prev_pulse_count[NUM_COILS] = {0};

/** Activity flash state per coil. */
static uint8_t  s_activity_flash[NUM_COILS] = {0};

/** External USB device handle (declared in main.c). */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ---------------------------------------------------------------------------*/
/*                     Private Helper Functions                               */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Render the status header bar.
 */
static void DisplayUI_DrawHeader(u8g2_t *u8g2)
{
  /* u8g2 font: y coordinate = baseline of text */
  u8g2_SetFont(u8g2, u8g2_font_5x7_tr);

  /* --- System State --- */
  if (Safety_IsEStopped())
  {
    /* Inverted E-STOP banner for visibility */
    u8g2_SetDrawColor(u8g2, 1);
    u8g2_DrawBox(u8g2, 0, 0, 54, HEADER_HEIGHT);
    u8g2_SetDrawColor(u8g2, 0);  /* Black text on white bg */
    u8g2_DrawStr(u8g2, 2, 9, "*E-STOP*");
    u8g2_SetDrawColor(u8g2, 1);  /* Restore white */
  }
  else
  {
    u8g2_DrawStr(u8g2, 1, 9, "[SYS OK]");
  }

  /* --- USB Status --- */
  uint8_t usb_connected = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
  if (usb_connected)
  {
    u8g2_DrawStr(u8g2, 56, 9, "USB");
  }
  else
  {
    u8g2_DrawStr(u8g2, 56, 9, "---");
  }

  /* --- Uptime --- */
  {
    uint32_t uptime_s = HAL_GetTick() / 1000U;
    char buf[12];
    if (uptime_s < 3600U)
    {
      uint32_t m = uptime_s / 60U;
      uint32_t s = uptime_s % 60U;
      snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)m, (unsigned long)s);
    }
    else
    {
      uint32_t h = uptime_s / 3600U;
      uint32_t m = (uptime_s % 3600U) / 60U;
      uint32_t s = uptime_s % 60U;
      snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu",
               (unsigned long)h, (unsigned long)m, (unsigned long)s);
    }
    /* Right-align */
    uint8_t str_w = u8g2_GetStrWidth(u8g2, buf);
    u8g2_DrawStr(u8g2, 128 - str_w - 1, 9, buf);
  }

  /* --- Separator line under header --- */
  u8g2_DrawHLine(u8g2, 0, HEADER_HEIGHT, 128);
}

/**
 * @brief  Render a single coil zone.
 */
static void DisplayUI_DrawCoilZone(u8g2_t *u8g2, uint8_t coil_id,
                                    uint8_t zone_x, uint8_t zone_y,
                                    uint8_t zone_w, uint8_t zone_h)
{
  char buf[16];
  uint8_t tx = zone_x + 2;     /* Text X offset within zone */
  uint8_t ty = zone_y + 8;     /* First text baseline (y = zone_y + font height) */

  u8g2_SetFont(u8g2, u8g2_font_5x7_tr);

  /* --- Coil label --- */
  snprintf(buf, sizeof(buf), "C%u", coil_id);

  /* Get coil state */
  CoilSchedState_t cs;
  Scheduler_GetCoilState(coil_id, &cs);
  uint8_t voices = Scheduler_GetActiveVoices(coil_id);

  /* Disabled coil */
  if (!cs.enabled)
  {
    u8g2_DrawStr(u8g2, tx, ty, buf);
    u8g2_DrawStr(u8g2, tx, ty + 9, "OFF");
    return;
  }

  /* Draw coil label */
  u8g2_DrawStr(u8g2, tx, ty, buf);

  if (voices == 0)
  {
    /* Idle */
    u8g2_DrawStr(u8g2, tx, ty + 9, "idle");
  }
  else
  {
    /* Check if any active tone is a test tone (wave mode) */
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
        if (tone.period_us > 0)
          freq_hz = 1000000U / tone.period_us;
        ontime_us = tone.ontime_us;
        break;
      }
    }

    if (is_wave)
    {
      /* Wave mode: freq + ontime */
      snprintf(buf, sizeof(buf), "%luHz", (unsigned long)freq_hz);
      u8g2_DrawStr(u8g2, tx, ty + 9, buf);

      snprintf(buf, sizeof(buf), "%uus", ontime_us);
      u8g2_DrawStr(u8g2, tx, ty + 18, buf);
    }
    else
    {
      /* MIDI mode: voice count */
      snprintf(buf, sizeof(buf), "%uv", voices);
      u8g2_DrawStr(u8g2, tx, ty + 9, buf);
    }
  }

  /* --- Activity pulse indicator --- */
  {
    uint32_t current_pulses = cs.pulse_count;
    if (current_pulses != s_prev_pulse_count[coil_id])
    {
      s_activity_flash[coil_id] ^= 1U;
      s_prev_pulse_count[coil_id] = current_pulses;
    }

    if (s_activity_flash[coil_id])
    {
      u8g2_DrawBox(u8g2, zone_x + zone_w - 7, zone_y + 2, 5, 5);
    }
  }

  /* --- Column separator (right edge) --- */
  if (coil_id % COIL_COLS != (COIL_COLS - 1))
  {
    u8g2_DrawVLine(u8g2, zone_x + zone_w - 1, zone_y, zone_h);
  }
}

/**
 * @brief  Render the 6-coil body area.
 */
static void DisplayUI_DrawBody(u8g2_t *u8g2)
{
  /* Row separator between top and bottom rows */
  uint8_t row_sep_y = BODY_Y + COIL_ZONE_H;
  u8g2_DrawHLine(u8g2, 0, row_sep_y, 128);

  for (uint8_t coil = 0; coil < NUM_COILS; coil++)
  {
    uint8_t col = coil % COIL_COLS;
    uint8_t row = coil / COIL_COLS;

    uint8_t zx = col * COIL_ZONE_W;
    uint8_t zy = BODY_Y + row * (COIL_ZONE_H + 1);  /* +1 for separator */

    DisplayUI_DrawCoilZone(u8g2, coil, zx, zy, COIL_ZONE_W, COIL_ZONE_H);
  }
}

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

void DisplayUI_Init(void)
{
  /* Staggered boot delay: prevent inrush current brownout on 3.3V rail.
   * The OLED module can draw significant current on power-up. */
  HAL_Delay(200);

  SSD1309_Init();
  Debug_Log("[DISP] OLED initialized (SSD1309 128x64, u8g2, SPI2)");

  /* Draw splash screen */
  u8g2_t *u8g2 = SSD1309_GetU8g2();
  u8g2_ClearBuffer(u8g2);

  u8g2_SetFont(u8g2, u8g2_font_6x10_tr);
  u8g2_DrawStr(u8g2, 10, 28, "TC-Interrupter");
  u8g2_SetFont(u8g2, u8g2_font_5x7_tr);
  u8g2_DrawStr(u8g2, 44, 44, "v1.0.0");

  u8g2_SendBuffer(u8g2);

  /* Show splash for 500ms, then continue. */
  HAL_Delay(500);

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

  /* Rebuild frame */
  u8g2_ClearBuffer(u8g2);
  DisplayUI_DrawHeader(u8g2);
  DisplayUI_DrawBody(u8g2);
  u8g2_SendBuffer(u8g2);
}
