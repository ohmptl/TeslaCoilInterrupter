/**
  ******************************************************************************
  * @file    display_ui.h
  * @brief   OLED display UI renderer for the TC-Interrupter.
  *
  *          Renders a minimal header (SYS OK / E-STOP) and a column-major
  *          6-coil grid that supports QCW column merging.
  *          Boot screen waits for GUI connection before showing main display.
  *          Runs in the main super-loop at ~15 FPS (non-blocking).
  ******************************************************************************
  */

#ifndef __DISPLAY_UI_H
#define __DISPLAY_UI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialize the OLED display with staggered power-on delay.
 *         Must be called AFTER MX_SPI2_Init() and MX_GPIO_Init(),
 *         but BEFORE entering the main while(1) loop.
 *
 *         Internally calls HAL_Delay(200) before powering the display
 *         to prevent 3.3V rail brownout from OLED inrush current.
 */
void DisplayUI_Init(void);

/**
 * @brief  Non-blocking display update.  Call this every main-loop iteration.
 *         Internally rate-limits to ~15 FPS using HAL_GetTick().
 *         Reads scheduler telemetry, builds the UI framebuffer, and
 *         flushes to the display.
 *
 *         Safe to call at any frequency — returns immediately if the
 *         refresh interval has not elapsed.
 */
void DisplayUI_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_UI_H */
