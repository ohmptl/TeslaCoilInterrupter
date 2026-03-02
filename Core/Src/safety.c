/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : safety.c
  * @brief          : Per-coil safety limits and E-Stop management.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "safety.h"
#include <stddef.h>
/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/
static SafetyLimits_t g_limits[NUM_COILS];
static volatile uint8_t g_estop_active = 0;

/* ---------------------------------------------------------------------------*/
/*                         Public API                                        */
/* ---------------------------------------------------------------------------*/

void Safety_Init(void)
{
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_limits[i].max_ontime_us   = SAFETY_DEF_MAX_ONTIME_US;
    g_limits[i].min_offtime_us  = SAFETY_DEF_MIN_OFFTIME_US;
    g_limits[i].max_duty_permil = SAFETY_DEF_MAX_DUTY_PERMIL;
  }
  g_estop_active = 0;
}

void Safety_SetLimits(uint8_t coil_id, const SafetyLimits_t *limits)
{
  if (coil_id >= NUM_COILS || limits == NULL) return;

  /* Clamp to absolute hardware ceilings */
  g_limits[coil_id].max_ontime_us =
      (limits->max_ontime_us > SAFETY_ABS_MAX_ONTIME_US)
          ? SAFETY_ABS_MAX_ONTIME_US
          : limits->max_ontime_us;

  g_limits[coil_id].min_offtime_us =
      (limits->min_offtime_us < SAFETY_ABS_MIN_OFFTIME_US)
          ? SAFETY_ABS_MIN_OFFTIME_US
          : limits->min_offtime_us;

  g_limits[coil_id].max_duty_permil =
      (limits->max_duty_permil > SAFETY_ABS_MAX_DUTY_PERMIL)
          ? SAFETY_ABS_MAX_DUTY_PERMIL
          : limits->max_duty_permil;

  /* Floor: zero duty or zero on-time means effectively disabled
   * but don't force them — let caller set 0 deliberately. */
}

void Safety_GetLimits(uint8_t coil_id, SafetyLimits_t *limits)
{
  if (coil_id >= NUM_COILS || limits == NULL) return;
  *limits = g_limits[coil_id];
}

uint16_t Safety_ClampOntime(uint8_t coil_id, uint16_t requested_us)
{
  if (coil_id >= NUM_COILS) return 0;
  uint16_t max_ot = g_limits[coil_id].max_ontime_us;
  return (requested_us > max_ot) ? max_ot : requested_us;
}

void Safety_EStopSet(uint8_t active)
{
  g_estop_active = active ? 1U : 0U;
}

uint8_t Safety_IsEStopped(void)
{
  return g_estop_active;
}
