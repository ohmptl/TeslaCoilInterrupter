/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : coil_driver.h
  * @brief          : Low-level OPM timer abstraction for 6 Tesla Coil outputs.
  *
  * Maps coil index (0-5) to the corresponding OPM timer and output pin:
  *   Coil 0 → TIM1  CH1  PE9   (APB2, 168 MHz timer clock)
  *   Coil 1 → TIM4  CH1  PD12  (APB1,  84 MHz timer clock)
  *   Coil 2 → TIM9  CH1  PE5   (APB2, 168 MHz timer clock)
  *   Coil 3 → TIM10 CH1  PF6   (APB2, 168 MHz timer clock)
  *   Coil 4 → TIM11 CH1  PF7   (APB2, 168 MHz timer clock)
  *   Coil 5 → TIM13 CH1  PF8   (APB1,  84 MHz timer clock)
  *
  * All timers are pre-configured by CubeMX in PWM1 + One-Pulse Mode at
  * 1 µs resolution (prescaler 167 or 83 depending on bus).
  *
  * ArmPulse: loads ARR & CCR1, forces update, sets CEN → one hardware pulse.
  * StopAll : kills all timers immediately (E-Stop path).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __COIL_DRIVER_H
#define __COIL_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "safety.h"   /* NUM_COILS */

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Post-CubeMX initialization: enable OPM for all coil timers,
 *         enable output channels, set TIM1 MOE.
 *         Must be called AFTER MX_TIMx_Init() and BEFORE Scheduler_Start().
 */
void CoilDriver_Init(void);

/**
 * @brief  Arm a single One-Pulse of exactly @p ontime_us microseconds.
 *         The timer fires one pulse then auto-stops (OPM hardware guarantee).
 *         Safe to call from ISR.
 * @param  coil_id  Coil index 0..5
 * @param  ontime_us  Pulse width in microseconds (1..65535)
 */
void CoilDriver_ArmPulse(uint8_t coil_id, uint16_t ontime_us);

/**
 * @brief  Force-stop a single coil timer and set output LOW.
 */
void CoilDriver_StopCoil(uint8_t coil_id);

/**
 * @brief  Emergency stop: force-stop ALL coil timers immediately.
 *         Called from E-Stop ISR (priority 0) — must be very fast.
 */
void CoilDriver_StopAll(void);

/**
 * @brief  Check if a coil's timer is currently counting (pulse in progress).
 *         Safe to call from ISR.
 * @return 1 if timer is active (CEN set), 0 if idle.
 */
uint8_t CoilDriver_IsActive(uint8_t coil_id);

#ifdef __cplusplus
}
#endif

#endif /* __COIL_DRIVER_H */
