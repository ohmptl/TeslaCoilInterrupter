/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : scheduler.c
  * @brief          : TIM7-based master scheduler for polyphonic OPM pulse gen.
  *
  * TIM7 is reconfigured to fire every 100 µs (10 kHz).  Each tick the ISR
  * scans all coils and fires the earliest-due tone that passes safety checks.
  *
  * Design decisions:
  *   - No floating point in ISR (all integer arithmetic)
  *   - Safety limits cached in CoilSchedState_t (no function calls in hot path)
  *   - Critical sections protect tone add/remove (main loop ↔ ISR)
  *   - Duty enforced via simple 100 ms window accumulator
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "scheduler.h"
#include "coil_driver.h"
#include "safety.h"
#include "tim.h"
#include "debug_uart.h"
#include <string.h>

/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/

/** All tone slots: [coil][voice]. */
static SchedulerTone_t  g_tones[NUM_COILS][MAX_VOICES_PER_COIL];

/** Per-coil runtime state. */
static CoilSchedState_t g_coil_state[NUM_COILS];

/** Global scheduler time in microseconds (wraps every ~71 minutes). */
static volatile uint32_t g_sched_us = 0;

/** Total tick count since scheduler start. */
static volatile uint32_t g_sched_tick_count = 0;

/** Scheduler running flag. */
static volatile uint8_t  g_sched_running = 0;

/* ---------------------------------------------------------------------------*/
/*                         Public API                                        */
/* ---------------------------------------------------------------------------*/

void Scheduler_Init(void)
{
  /* Clear all tone slots */
  memset(g_tones, 0, sizeof(g_tones));
  memset(g_coil_state, 0, sizeof(g_coil_state));
  g_sched_us         = 0;
  g_sched_tick_count = 0;
  g_sched_running    = 0;

  /* Initialize per-coil state */
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    g_coil_state[i].enabled = 1U;   /* All coils enabled by default */
  }

  /* Pre-compute cached limits from Safety module */
  Scheduler_UpdateAllLimitCaches();

  /*
   * Reconfigure TIM7 for 100 µs period.
   * TIM7 is on APB1 (timer clock = 84 MHz), PSC = 83 → 1 MHz (1 µs tick).
   * ARR = 99 → period = 100 µs.
   */
  __HAL_TIM_SET_AUTORELOAD(&htim7, SCHEDULER_TICK_US - 1U);
  __HAL_TIM_SET_COUNTER(&htim7, 0);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
}

void Scheduler_Start(void)
{
  g_sched_running = 1;
  HAL_TIM_Base_Start_IT(&htim7);
  Debug_Log("[SCHED] Started (100us tick, 10kHz)");
}

void Scheduler_Stop(void)
{
  g_sched_running = 0;
  HAL_TIM_Base_Stop_IT(&htim7);
  CoilDriver_StopAll();
  Debug_Log("[SCHED] Stopped");
}

/* ---------------------------------------------------------------------------*/
/*                   ISR Tick — HOT PATH                                      */
/* ---------------------------------------------------------------------------*/

void Scheduler_Tick(void)
{
  g_sched_us += SCHEDULER_TICK_US;
  g_sched_tick_count++;

  if (!g_sched_running || Safety_IsEStopped())
    return;

  for (uint8_t c = 0; c < NUM_COILS; c++)
  {
    CoilSchedState_t *cs = &g_coil_state[c];

    if (!cs->enabled) continue;

    /* Skip if coil's OPM timer is still counting (pulse in progress) */
    if (CoilDriver_IsActive(c)) continue;

    /* Enforce min off-time since last pulse ended */
    int32_t since_end = (int32_t)(g_sched_us - cs->last_pulse_end_us);
    if (since_end < (int32_t)cs->cached_min_offtime_us) continue;

    /* ---- Refresh duty window ---- */
    uint32_t window_elapsed = g_sched_us - cs->duty_window_start;
    if (window_elapsed >= SAFETY_DUTY_WINDOW_US)
    {
      cs->duty_accum_us     = 0;
      cs->duty_window_start = g_sched_us;
    }

    /* ---- Collect ALL due tones, sort by next_fire_us ---- */
    SchedulerTone_t *due[MAX_VOICES_PER_COIL];
    uint8_t due_count = 0;

    for (uint8_t v = 0; v < MAX_VOICES_PER_COIL; v++)
    {
      SchedulerTone_t *t = &g_tones[c][v];
      if (!t->active) continue;
      if ((int32_t)(t->next_fire_us - g_sched_us) <= 0)
        due[due_count++] = t;
    }

    if (due_count == 0) continue;

    /* Insertion sort by next_fire_us (at most 8 elements) */
    for (uint8_t i = 1; i < due_count; i++)
    {
      SchedulerTone_t *key = due[i];
      int8_t j = (int8_t)i - 1;
      while (j >= 0 && (int32_t)(due[j]->next_fire_us - key->next_fire_us) > 0)
      {
        due[j + 1] = due[j];
        j--;
      }
      due[j + 1] = key;
    }

    /* ---- Merge due pulses into one superposed on-time ---- */
    /*
     * Pulse superposition (like Syntherrupter):
     * Walk the sorted due-tones and build a combined on-time.
     * Each tone's pulse is placed back-to-back after the previous
     * one ends (separated by min_offtime if configured to 0, they
     * merge seamlessly).  The merged total is capped by the duty
     * budget and max_ontime.
     */
    uint16_t merged_ot = 0;
    uint16_t max_ot    = cs->cached_max_ontime_us;
    uint32_t budget    = cs->max_ontime_window - cs->duty_accum_us;

    for (uint8_t i = 0; i < due_count; i++)
    {
      uint16_t ot = due[i]->ontime_us;
      if (ot > max_ot) ot = max_ot;

      /* Check if adding this pulse would exceed duty budget */
      if ((uint32_t)(merged_ot + ot) > budget)
      {
        /* Clamp to remaining budget */
        if (merged_ot < (uint16_t)budget)
          ot = (uint16_t)(budget - merged_ot);
        else
          ot = 0;
      }

      merged_ot += ot;

      /* Advance this tone regardless (avoid re-triggering next tick) */
      due[i]->next_fire_us += due[i]->period_us;

      /* If we missed many pulses, catch up to avoid burst */
      if ((int32_t)(due[i]->next_fire_us - g_sched_us) <
          -(int32_t)(due[i]->period_us))
      {
        due[i]->next_fire_us = g_sched_us + due[i]->period_us;
      }
    }

    /* ---- Fire the merged pulse ---- */
    if (merged_ot > 0)
    {
      /* Final clamp: merged total cannot exceed single-pulse max */
      if (merged_ot > max_ot)
        merged_ot = max_ot;

      CoilDriver_ArmPulse(c, merged_ot);
      cs->duty_accum_us     += merged_ot;
      cs->last_pulse_end_us  = g_sched_us + merged_ot;
      cs->pulse_count++;
    }
  }
}

/* ---------------------------------------------------------------------------*/
/*                   Limit Cache Management                                   */
/* ---------------------------------------------------------------------------*/

void Scheduler_UpdateLimitCache(uint8_t coil_id)
{
  if (coil_id >= NUM_COILS) return;

  SafetyLimits_t lim;
  Safety_GetLimits(coil_id, &lim);

  g_coil_state[coil_id].cached_max_ontime_us  = lim.max_ontime_us;
  g_coil_state[coil_id].cached_min_offtime_us = lim.min_offtime_us;

  /* Budget = max_duty_permil * DUTY_WINDOW_US / 1000
   * e.g. 10 permil (1%) * 100000 µs / 1000 = 1000 µs per window */
  g_coil_state[coil_id].max_ontime_window =
      (uint32_t)lim.max_duty_permil * SAFETY_DUTY_WINDOW_US / 1000U;
}

void Scheduler_UpdateAllLimitCaches(void)
{
  for (uint8_t i = 0; i < NUM_COILS; i++)
  {
    Scheduler_UpdateLimitCache(i);
  }
}

/* ---------------------------------------------------------------------------*/
/*                   Tone Management (Main Loop Context)                      */
/* ---------------------------------------------------------------------------*/

int8_t Scheduler_AddTone(uint8_t coil_id, uint8_t midi_note, uint8_t midi_channel,
                          uint8_t velocity, uint16_t ontime_us, uint32_t period_us)
{
  if (coil_id >= NUM_COILS) return -1;
  if (period_us == 0 || ontime_us == 0) return -1;

  /* Check if this note already exists on this coil (update in place) */
  for (uint8_t v = 0; v < MAX_VOICES_PER_COIL; v++)
  {
    SchedulerTone_t *t = &g_tones[coil_id][v];
    if (t->active && t->midi_note == midi_note && t->midi_channel == midi_channel)
    {
      __disable_irq();
      t->velocity  = velocity;
      t->ontime_us = ontime_us;
      t->period_us = period_us;
      __enable_irq();
      return (int8_t)v;
    }
  }

  /* Find a free slot */
  for (uint8_t v = 0; v < MAX_VOICES_PER_COIL; v++)
  {
    SchedulerTone_t *t = &g_tones[coil_id][v];
    if (!t->active)
    {
      __disable_irq();
      t->coil_id      = coil_id;
      t->midi_note    = midi_note;
      t->midi_channel = midi_channel;
      t->velocity     = velocity;
      t->ontime_us    = ontime_us;
      t->period_us    = period_us;
      t->next_fire_us = g_sched_us;  /* Fire on next tick */
      t->active       = 1;           /* Set active LAST so ISR sees consistent data */
      __enable_irq();
      return (int8_t)v;
    }
  }

  return -1;  /* No free slots */
}

void Scheduler_RemoveTone(uint8_t coil_id, uint8_t midi_note, uint8_t midi_channel)
{
  if (coil_id >= NUM_COILS) return;

  for (uint8_t v = 0; v < MAX_VOICES_PER_COIL; v++)
  {
    SchedulerTone_t *t = &g_tones[coil_id][v];
    if (t->active && t->midi_note == midi_note && t->midi_channel == midi_channel)
    {
      __disable_irq();
      t->active = 0;
      __enable_irq();
      return;
    }
  }
}

void Scheduler_RemoveAllTones(uint8_t coil_id)
{
  if (coil_id >= NUM_COILS) return;

  __disable_irq();
  for (uint8_t v = 0; v < MAX_VOICES_PER_COIL; v++)
  {
    g_tones[coil_id][v].active = 0;
  }
  /* Reset duty window so next FIRE starts with a clean budget */
  g_coil_state[coil_id].duty_accum_us     = 0;
  g_coil_state[coil_id].duty_window_start = g_sched_us;
  __enable_irq();
  CoilDriver_StopCoil(coil_id);
}

void Scheduler_RemoveAllTonesAllCoils(void)
{
  __disable_irq();
  for (uint8_t c = 0; c < NUM_COILS; c++)
  {
    for (uint8_t v = 0; v < MAX_VOICES_PER_COIL; v++)
    {
      g_tones[c][v].active = 0;
    }
    /* Reset duty window so next FIRE starts with a clean budget */
    g_coil_state[c].duty_accum_us     = 0;
    g_coil_state[c].duty_window_start = g_sched_us;
  }
  __enable_irq();
  CoilDriver_StopAll();
}

void Scheduler_SetCoilEnabled(uint8_t coil_id, uint8_t enabled)
{
  if (coil_id >= NUM_COILS) return;
  g_coil_state[coil_id].enabled = enabled ? 1U : 0U;
  if (!enabled)
  {
    Scheduler_RemoveAllTones(coil_id);
  }
}

/* ---------------------------------------------------------------------------*/
/*                          Status Queries                                    */
/* ---------------------------------------------------------------------------*/

void Scheduler_GetStats(SchedulerStats_t *stats)
{
  if (stats == NULL) return;
  __disable_irq();
  stats->tick_count = g_sched_tick_count;
  stats->current_us = g_sched_us;
  stats->running    = g_sched_running;
  __enable_irq();
}

void Scheduler_GetCoilState(uint8_t coil_id, CoilSchedState_t *state)
{
  if (coil_id >= NUM_COILS || state == NULL) return;
  __disable_irq();
  *state = g_coil_state[coil_id];
  __enable_irq();
}

uint8_t Scheduler_GetActiveVoices(uint8_t coil_id)
{
  if (coil_id >= NUM_COILS) return 0;
  uint8_t count = 0;
  for (uint8_t v = 0; v < MAX_VOICES_PER_COIL; v++)
  {
    if (g_tones[coil_id][v].active) count++;
  }
  return count;
}
