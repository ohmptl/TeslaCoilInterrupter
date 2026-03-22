/**
  ******************************************************************************
  * @file    ssd1309.c
  * @brief   SSD1309 128x64 OLED display driver — u8g2 HAL port for STM32.
  *
  *          Implements the u8g2 byte-level SPI callback and GPIO/delay
  *          callback using STM32 HAL for SPI2 and the OLED control GPIOs.
  *
  *  u8g2 setup function used:
  *    u8g2_Setup_ssd1309_128x64_noname2_f()
  *    - Full framebuffer mode (1024 byte RAM)
  *    - SSD1309 controller, 128x64 resolution
  *    - "noname2" variant — generic SSD1309 modules
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ssd1309.h"
#include "spi.h"
#include "main.h"

/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/

/** u8g2 display instance (full framebuffer mode). */
static u8g2_t g_u8g2;

/* ---------------------------------------------------------------------------*/
/*                   u8g2 HAL Callbacks (Private)                             */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  u8g2 byte-level SPI transport callback.
 *         Handles SPI data transfer and CS/DC pin control.
 */
static uint8_t u8x8_byte_stm32_hw_spi(u8x8_t *u8x8, uint8_t msg,
                                        uint8_t arg_int, void *arg_ptr)
{
  (void)u8x8;  /* Unused — we use global hspi2 */

  switch (msg)
  {
    case U8X8_MSG_BYTE_SEND:
      HAL_SPI_Transmit(&hspi2, (uint8_t *)arg_ptr, arg_int, 10);
      break;

    case U8X8_MSG_BYTE_INIT:
      /* SPI already initialized by MX_SPI2_Init() */
      break;

    case U8X8_MSG_BYTE_SET_DC:
      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin,
                         arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;

    case U8X8_MSG_BYTE_START_TRANSFER:
      HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
      break;

    case U8X8_MSG_BYTE_END_TRANSFER:
      HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
      break;

    default:
      return 0;
  }
  return 1;
}

/**
 * @brief  u8g2 GPIO and delay callback.
 *         Handles reset pin, CS, DC, and millisecond delays.
 */
static uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg,
                                           uint8_t arg_int, void *arg_ptr)
{
  (void)u8x8;
  (void)arg_ptr;

  switch (msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      /* GPIOs already initialized by MX_GPIO_Init() */
      break;

    case U8X8_MSG_DELAY_MILLI:
      HAL_Delay(arg_int);
      break;

    case U8X8_MSG_DELAY_10MICRO:
      /* ~10us delay — use a simple busy loop.
       * At 168 MHz, ~1680 cycles ≈ 10 µs. */
      {
        volatile uint32_t count = arg_int * 168U;
        while (count--) { __NOP(); }
      }
      break;

    case U8X8_MSG_DELAY_100NANO:
      /* ~100ns delay — very short, a few NOPs suffice. */
      __NOP(); __NOP(); __NOP(); __NOP();
      __NOP(); __NOP(); __NOP(); __NOP();
      __NOP(); __NOP(); __NOP(); __NOP();
      __NOP(); __NOP(); __NOP(); __NOP();
      break;

    case U8X8_MSG_GPIO_CS:
      HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin,
                         arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;

    case U8X8_MSG_GPIO_DC:
      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin,
                         arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;

    case U8X8_MSG_GPIO_RESET:
      HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin,
                         arg_int ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;

    default:
      return 0;
  }
  return 1;
}

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

u8g2_t* SSD1309_GetU8g2(void)
{
  return &g_u8g2;
}

void SSD1309_Init(void)
{
  /* ---------------------------------------------------------------
   * CRITICAL: Lower SPI2 clock speed for SSD1309 compatibility.
   *
   * CubeMX sets SPI2 prescaler to /2 → APB1(42MHz)/2 = 21 MHz.
   * SSD1309 max SPI clock is ~10 MHz; budget modules often fail
   * above 4 MHz.  u8g2 targets 4 MHz for this controller.
   *
   * Reconfigure to /16 → 42MHz/16 = 2.625 MHz (safe).
   * --------------------------------------------------------------- */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    /* If SPI re-init fails, continue anyway — the display just won't work */
  }

  /* Set up u8g2 for SSD1309 128x64, full framebuffer, hardware SPI.
   * U8G2_R0 = no rotation.
   * "noname2" is the generic SSD1309 variant (x_offset=2). */
  u8g2_Setup_ssd1309_128x64_noname2_f(
      &g_u8g2,
      U8G2_R0,
      u8x8_byte_stm32_hw_spi,
      u8x8_gpio_and_delay_stm32
  );

  /* u8g2_InitDisplay performs:
   *  1. Hardware reset (RES low 100ms, then high + 100ms wait)
   *  2. SSD1309 init command sequence (clock, mux, remap, charge pump,
   *     pre-charge, VCOMH, contrast, addressing mode, display on)  */
  u8g2_InitDisplay(&g_u8g2);

  /* Turn the display on (exit power-save / sleep mode). */
  u8g2_SetPowerSave(&g_u8g2, 0);

  /* Set contrast — 0xCF is the recommended value for SSD1309. */
  u8g2_SetContrast(&g_u8g2, 0xCF);

  /* Smoke test: fill screen white to verify pixel data path works.
   * If this produces a white screen, the init + SPI path is good. */
  u8g2_ClearBuffer(&g_u8g2);

  /* Draw a test pattern — border + text so we can confirm init worked */
  u8g2_SetFont(&g_u8g2, u8g2_font_6x10_tr);
  u8g2_DrawStr(&g_u8g2, 10, 35, "SSD1309 OK");
  u8g2_DrawFrame(&g_u8g2, 0, 0, 128, 64);

  u8g2_SendBuffer(&g_u8g2);
}

void SSD1309_Flush(void)
{
  u8g2_SendBuffer(&g_u8g2);
}

void SSD1309_Clear(void)
{
  u8g2_ClearBuffer(&g_u8g2);
}

void SSD1309_SetContrast(uint8_t contrast)
{
  u8g2_SetContrast(&g_u8g2, contrast);
}
