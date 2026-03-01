/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : safety.h
  * @brief          : Per-coil safety limits and E-Stop management.
  *
  * Every pulse fired by the scheduler passes through safety checks:
  *   1. On-time clamped to per-coil max (absolute cap: 500 µs)
  *   2. Duty cycle tracked per 100 ms window (absolute cap: 5%)
  *   3. Min off-time enforced between consecutive pulses
  *   4. Global E-Stop flag immediately halts all outputs
  *
  * Limits are configurable via CDC but cannot exceed absolute hardware caps.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SAFETY_H
#define __SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* ---------------------------------------------------------------------------*/
/*                          Constants                                         */
/* ---------------------------------------------------------------------------*/
#define NUM_COILS                   6U

/* ---------- Configurable Defaults (per-coil) ---------- */
#define SAFETY_DEF_MAX_ONTIME_US    200U    /* 200 µs default max pulse width     */
#define SAFETY_DEF_MIN_OFFTIME_US   100U    /* 100 µs default min gap             */
#define SAFETY_DEF_MAX_DUTY_PERMIL  10U     /* 1.0% default max duty cycle        */

/* ---------- Absolute Hardware Limits (cannot be exceeded) ---------- */
#define SAFETY_ABS_MAX_ONTIME_US    500U    /* 500 µs: silicon-enforced ceiling   */
#define SAFETY_ABS_MAX_DUTY_PERMIL  50U     /* 5.0%: absolute max duty            */
#define SAFETY_ABS_MIN_OFFTIME_US   20U     /* 20 µs: absolute min gap            */

/* ---------- Duty Cycle Measurement Window ---------- */
#define SAFETY_DUTY_WINDOW_US       100000U /* 100 ms sliding window              */

/* ---------------------------------------------------------------------------*/
/*                          Types                                             */
/* ---------------------------------------------------------------------------*/
typedef struct
{
    uint16_t max_ontime_us;     /* Maximum single pulse width (µs)            */
    uint16_t min_offtime_us;    /* Minimum gap between consecutive pulses (µs)*/
    uint16_t max_duty_permil;   /* Max duty cycle in 0.1% units (10 = 1.0%)   */
} SafetyLimits_t;

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialize all coil limits to safe defaults. Clear E-Stop.
 */
void Safety_Init(void);

/**
 * @brief  Set limits for a coil. Values are clamped to absolute hardware caps.
 */
void Safety_SetLimits(uint8_t coil_id, const SafetyLimits_t *limits);

/**
 * @brief  Get current limits for a coil.
 */
void Safety_GetLimits(uint8_t coil_id, SafetyLimits_t *limits);

/**
 * @brief  Clamp a requested on-time to the coil's max_ontime_us.
 *         Safe to call from ISR (no side effects, reads cached limit).
 * @return Clamped on-time in µs.
 */
uint16_t Safety_ClampOntime(uint8_t coil_id, uint16_t requested_us);

/**
 * @brief  Set the E-Stop state.  Called from EXTI ISR (priority 0).
 */
void Safety_EStopSet(uint8_t active);

/**
 * @brief  Query the E-Stop state.
 * @return 1 if E-Stop is active, 0 otherwise.
 */
uint8_t Safety_IsEStopped(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_H */
