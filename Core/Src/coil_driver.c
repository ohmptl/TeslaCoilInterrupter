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
   * 3. Enable Capture/Compare output channel 1 for all coil timers.
   *    CubeMX's HAL_TIM_PWM_ConfigChannel() sets up OC mode/polarity but
   *    doesn't enable the output channel.  CC1E makes the pin drive.
   *
   *    With CCR1=0 and CNT=0: PWM1 output = inactive (LOW) since 0 < 0 is
   *    false.  So no spurious output when enabling the channel.
   */
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_tim[i]->CCER |= TIM_CCER_CC1E;
  }

  /*
   * 4. Ensure all timers are stopped and at idle.
   */
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_tim[i]->CR1 &= ~TIM_CR1_CEN;
    g_tim[i]->CNT  = 0;
    g_tim[i]->CCR1 = 0;
  }
}

void CoilDriver_ArmPulse(uint8_t coil_id, uint16_t ontime_us)
{
  if (coil_id >= NUM_COILS || ontime_us == 0) return;

  TIM_TypeDef *tim = g_tim[coil_id];

  /*
   * Load the pulse width into preload registers:
   *   ARR  = ontime_us  → timer counts from 0 to ARR then stops (OPM)
   *   CCR1 = ontime_us  → output HIGH while CNT < CCR1 (PWM1 mode)
   *
   * Force an Update Event via EGR to transfer preloaded values into
   * shadow registers and reset CNT to 0.  Clear the resulting UIF so
   * it doesn't trigger a spurious ISR.
   *
   * Then set CEN to arm the one-pulse.
   */
  tim->ARR  = (uint32_t)ontime_us;
  tim->CCR1 = (uint32_t)ontime_us;
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

  /* Force output LOW: set CCR1=0 → PWM1 output inactive */
  tim->CCR1 = 0;
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
    g_tim[i]->CCR1 = 0;
    g_tim[i]->EGR  = TIM_EGR_UG;
    g_tim[i]->SR   = 0;
  }
}

uint8_t CoilDriver_IsActive(uint8_t coil_id)
{
  if (coil_id >= NUM_COILS) return 0;
  return (g_tim[coil_id]->CR1 & TIM_CR1_CEN) ? 1U : 0U;
}
