/**
  ******************************************************************************
  * @file    ssd1309.h
  * @brief   SSD1309 128x64 OLED display driver — u8g2 HAL port for STM32.
  *
  *          Wraps the u8g2 library with STM32 HAL SPI2 transport and GPIO
  *          callbacks for CS (PB8), DC (PB9), and RES (PC7).
  *
  *  Hardware connections (from CubeMX / main.h):
  *    SCK  = PB10  (SPI2_SCK,  AF5)
  *    MOSI = PC3   (SPI2_MOSI, AF5)
  *    CS   = PB8   (GPIO, Very High Speed)
  *    DC   = PB9   (GPIO, Very High Speed)
  *    RES  = PC7   (GPIO, Very High Speed)
  ******************************************************************************
  */

#ifndef __SSD1309_H
#define __SSD1309_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "u8g2.h"

/* ---------------------------------------------------------------------------*/
/*                          Display Dimensions                                */
/* ---------------------------------------------------------------------------*/
#define SSD1309_WIDTH    128U
#define SSD1309_HEIGHT   64U

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Get the global u8g2 handle.
 *         Use this to call u8g2 drawing functions directly.
 * @return Pointer to the u8g2_t instance.
 */
u8g2_t* SSD1309_GetU8g2(void);

/**
 * @brief  Initialize the SSD1309 display via u8g2.
 *         Performs hardware reset, sends u8g2 init sequence,
 *         and turns the display on.
 *         Uses HAL_Delay() internally — call from main() only, NOT from ISR.
 */
void SSD1309_Init(void);

/**
 * @brief  Send the framebuffer to the display (u8g2_SendBuffer wrapper).
 *         Blocking SPI transfer.
 */
void SSD1309_Flush(void);

/**
 * @brief  Clear the u8g2 framebuffer.
 *         Does NOT flush — call SSD1309_Flush() afterwards.
 */
void SSD1309_Clear(void);

/**
 * @brief  Set the display contrast (brightness).
 * @param  contrast  0x00 (dimmest) to 0xFF (brightest)
 */
void SSD1309_SetContrast(uint8_t contrast);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1309_H */
