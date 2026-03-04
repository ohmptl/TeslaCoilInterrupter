/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : coil_driver.c
  * @brief          : Low-level OPM timer abstraction for 6 Tesla Coil outputs.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "coil_driver.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------*/
/*                     Timer Instance Table                                   */
/* ---------------------------------------------------------------------------*/
/*
 * Coil 0: TIM1   PE9   APB2  PSC=167  (168 MHz / 168 = 1 MHz)
 * Coil 1: TIM4   PD12  APB1  PSC=83   ( 84 MHz /  84 = 1 MHz)
 * Coil 2: TIM9   PE5   APB2  PSC=167
 * Coil 3: TIM10  PF6   APB2  PSC=167
 * Coil 4: TIM11  PF7   APB2  PSC=167
 * Coil 5: TIM13  PF8   APB1  PSC=83
 *
 * All timers tick at exactly 1 MHz → 1 µs per count.
 */
static TIM_TypeDef * const g_tim[NUM_COILS] = {
  TIM1,   /* Coil 0 - Advanced timer (needs MOE) */
  TIM4,   /* Coil 1 */
  TIM9,   /* Coil 2 */
  TIM10,  /* Coil 3 */
  TIM11,  /* Coil 4 */
  TIM13,  /* Coil 5 */
};

/* ---------------------------------------------------------------------------*/
/*                         Public API                                        */
/* ---------------------------------------------------------------------------*/

void CoilDriver_Init(void)
{
  /*
   * 1. Ensure One-Pulse Mode (OPM) is set for ALL coil timers.
   *    CubeMX may miss OPM on some timers (e.g. TIM13).
   *    Setting OPM when it's already set is harmless.
   */
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_tim[i]->CR1 |= TIM_CR1_OPM;
  }

  /*
   * 2. Enable Main Output Enable (MOE) for TIM1 (advanced timer).
   *    Without MOE, TIM1's output pins won't drive even if CC1E is set.
   */
  TIM1->BDTR |= TIM_BDTR_MOE;

  /*
   * 3. Force safe idle state on all timers BEFORE enabling outputs.
   *    Set CCR1=65535 so PWM2 output is LOW (CNT=0 < 65535 → inactive).
   *    Force UG to transfer preloaded CCR1 into shadow registers.
   *    MUST happen before CC1E is set to avoid any HIGH glitch.
   */
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_tim[i]->CR1 &= ~TIM_CR1_CEN;
    g_tim[i]->CNT  = 0;
    g_tim[i]->CCR1 = 65535;
    g_tim[i]->EGR  = TIM_EGR_UG;
    g_tim[i]->SR   = 0;
  }

  /*
   * 4. Enable Capture/Compare output channel 1 for all coil timers.
   *    Safe to enable now: CCR1=65535 guarantees PWM2 output is LOW.
   */
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_tim[i]->CCER |= TIM_CCER_CC1E;
  }
}

void CoilDriver_ArmPulse(uint8_t coil_id, uint16_t ontime_us)
{
  if (coil_id >= NUM_COILS || ontime_us == 0) return;

  TIM_TypeDef *tim = g_tim[coil_id];

  /*
   * Load the pulse width into preload registers:
   *   ARR  = ontime_us   → Timer counts from 0 to ARR (total ARR+1 states) and stops (OPM).
   *   CCR1 = 1           → Output becomes HIGH when CNT >= CCR1 (PWM2 mode).
   * 
   * With PWM Mode 2:
   *   CNT = 0            → 0 < 1 is true. Output INACTIVE (LOW). Lasts 1us.
   *   CNT = 1 to ARR     → CNT < 1 is false. Output ACTIVE (HIGH). Lasts ARR us.
   *   At CNT = ARR, the timeout wraps to 0 and stops.
   *   When stopped at CNT = 0, output correctly remains INACTIVE (LOW).
   *
   * Force an Update Event via EGR to transfer preloaded values into
   * shadow registers and reset CNT to 0. Clear the resulting UIF so
   * it doesn't trigger a spurious ISR.
   *
   * Then set CEN to arm the one-pulse.
   */
  tim->ARR  = (uint32_t)ontime_us;
  tim->CCR1 = 1;
  tim->EGR  = TIM_EGR_UG;    /* Load shadows, reset CNT */
  tim->SR   = 0;              /* Clear all interrupt flags */
  tim->CR1 |= TIM_CR1_CEN;   /* Start counting → output goes HIGH */
}

void CoilDriver_StopCoil(uint8_t coil_id)
{
  if (coil_id >= NUM_COILS) return;

  TIM_TypeDef *tim = g_tim[coil_id];

  /* Immediately stop the timer */
  tim->CR1 &= ~TIM_CR1_CEN;
  tim->CNT  = 0;

  /* Force output LOW: set CCR1=65535 → PWM2 output inactive */
  tim->CCR1 = 65535;
  tim->EGR  = TIM_EGR_UG;
  tim->SR   = 0;
}

void CoilDriver_StopAll(void)
{
  /*
   * E-Stop path: unrolled for minimum latency.
   * Stop all timers first (CEN clear), then clean up.
   */
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_tim[i]->CR1 &= ~TIM_CR1_CEN;
  }

  /* Now safe to clean up at leisure */
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_tim[i]->CNT  = 0;
    g_tim[i]->CCR1 = 65535;
    g_tim[i]->EGR  = TIM_EGR_UG;
    g_tim[i]->SR   = 0;
  }
}

uint8_t CoilDriver_IsActive(uint8_t coil_id)
{
  if (coil_id >= NUM_COILS) return 0;
  return (g_tim[coil_id]->CR1 & TIM_CR1_CEN) ? 1U : 0U;
}

uint8_t CoilDriver_AnyPinHigh(void)
{
  /*
   * Read GPIO Input Data Registers for all coil output pins.
   *   Coil 0: PE9,  Coil 2: PE5   → GPIOE
   *   Coil 1: PD12                → GPIOD
   *   Coil 3: PF6,  Coil 4: PF7,  Coil 5: PF8 → GPIOF
   *
   * Even in AF mode, IDR reflects the actual pin state.
   * Cost: 3 register reads — negligible.
   */
  if (GPIOE->IDR & (GPIO_PIN_9 | GPIO_PIN_5))              return 1U;
  if (GPIOD->IDR & GPIO_PIN_12)                             return 1U;
  if (GPIOF->IDR & (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)) return 1U;
  return 0U;
}
