/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : scheduler.h
  * @brief          : TIM7-based master scheduler for polyphonic OPM pulse gen.
  *
  * Architecture:
  *   TIM7 fires an ISR every 100 µs (10 kHz).  Each tick, the scheduler
  *   iterates over all 6 coils × up to MAX_VOICES_PER_COIL tones and arms
  *   the OPM timer for any tone whose next-fire time has arrived.
  *
  *   Safety checks are performed in-line:
  *     - On-time clamped to per-coil max (cached from Safety module)
  *     - Duty cycle enforced via 100 ms window budget
  *     - Min off-time between consecutive pulses
  *     - Global E-Stop hard-kills all output
  *
  *   Tone management (Add/Remove) is called from the main loop
  *   with brief critical sections to protect shared data.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "safety.h"

/* ---------------------------------------------------------------------------*/
/*                          Configuration                                     */
/* ---------------------------------------------------------------------------*/
#define MAX_VOICES_PER_COIL     8U      /* Max simultaneous tones per coil    */
#define SCHEDULER_TICK_US       100U    /* TIM7 ISR period = 100 µs (10 kHz)  */

/* ---------------------------------------------------------------------------*/
/*                          Types                                             */
/* ---------------------------------------------------------------------------*/

/** One active voice (tone) on a specific coil. */
typedef struct
{
  uint8_t  active;          /**< Non-zero if this slot is in use              */
  uint8_t  coil_id;         /**< Which coil (0..NUM_COILS-1)                  */
  uint8_t  midi_note;       /**< MIDI note number (0-127, 0xFF = test tone)   */
  uint8_t  midi_channel;    /**< MIDI channel   (0-15,   0xFF = test tone)    */
  uint8_t  velocity;        /**< Note velocity (1-127)                        */
  uint16_t ontime_us;       /**< Requested pulse width in µs (pre-clamp)      */
  uint32_t period_us;       /**< Pulse period in µs (1e6 / freq_hz)           */
  uint32_t next_fire_us;    /**< Absolute time of next pulse (µs)             */
} SchedulerTone_t;

/** Per-coil scheduler runtime state. */
typedef struct
{
  /* Duty cycle tracking (100 ms window) */
  uint32_t duty_accum_us;           /**< On-time accumulated this window      */
  uint32_t duty_window_start;       /**< When this window started (µs)        */
  uint32_t max_ontime_window;       /**< Budget: max_duty_permil * window/1000*/

  /* Timing */
  uint32_t last_pulse_end_us;       /**< Estimated end of last pulse (µs)     */

  /* Cached safety limits (refreshed by Scheduler_UpdateLimitCache) */
  uint16_t cached_max_ontime_us;
  uint16_t cached_min_offtime_us;

  /* Statistics */
  uint32_t pulse_count;             /**< Total pulses fired on this coil      */

  /* Enable */
  uint8_t  enabled;                 /**< 1 = coil output allowed              */
} CoilSchedState_t;

/** Snapshot for CDC telemetry queries. */
typedef struct
{
  uint32_t tick_count;              /**< Total scheduler ticks since start    */
  uint32_t current_us;              /**< Current absolute time (µs)           */
  uint8_t  running;                 /**< 1 if scheduler is active             */
} SchedulerStats_t;

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialize scheduler state, reconfigure TIM7 for 100 µs period.
 *         Does NOT start the scheduler.  Call after MX_TIM7_Init().
 */
void Scheduler_Init(void);

/**
 * @brief  Start the scheduler (enable TIM7 interrupt).
 */
void Scheduler_Start(void);

/**
 * @brief  Stop the scheduler and force-stop all coil timers.
 */
void Scheduler_Stop(void);

/**
 * @brief  Scheduler tick — called from TIM7 Period Elapsed callback (ISR).
 *         Scans all coils/tones and fires due pulses.
 */
void Scheduler_Tick(void);

/**
 * @brief  Refresh cached safety limits for a coil.
 *         Must be called after Safety_SetLimits().
 */
void Scheduler_UpdateLimitCache(uint8_t coil_id);

/**
 * @brief  Refresh cached safety limits for ALL coils.
 */
void Scheduler_UpdateAllLimitCaches(void);

/* ---- Tone Management (call from main loop) ---- */

/**
 * @brief  Add or update a tone on a coil.
 * @return Tone slot index (0..MAX_VOICES_PER_COIL-1) on success, -1 if full.
 */
int8_t Scheduler_AddTone(uint8_t coil_id, uint8_t midi_note, uint8_t midi_channel,
                          uint8_t velocity, uint16_t ontime_us, uint32_t period_us);

/**
 * @brief  Remove a specific tone (by note + channel) from a coil.
 */
void Scheduler_RemoveTone(uint8_t coil_id, uint8_t midi_note, uint8_t midi_channel);

/**
 * @brief  Remove all tones from a specific coil and stop its timer.
 */
void Scheduler_RemoveAllTones(uint8_t coil_id);

/**
 * @brief  Remove all tones from ALL coils and stop all timers.
 */
void Scheduler_RemoveAllTonesAllCoils(void);

/**
 * @brief  Enable or disable a coil's output.
 */
void Scheduler_SetCoilEnabled(uint8_t coil_id, uint8_t enabled);

/* ---- Status Queries ---- */

void    Scheduler_GetStats(SchedulerStats_t *stats);
void    Scheduler_GetCoilState(uint8_t coil_id, CoilSchedState_t *state);
uint8_t Scheduler_GetActiveVoices(uint8_t coil_id);

/**
 * @brief  Read a copy of a tone slot (ISR-safe).
 *         Used by the display to inspect per-coil frequency/on-time.
 * @param  coil_id    Coil index 0..5
 * @param  voice_idx  Voice slot index 0..MAX_VOICES_PER_COIL-1
 * @param  tone       Output: copy of the tone slot
 */
void Scheduler_GetToneInfo(uint8_t coil_id, uint8_t voice_idx, SchedulerTone_t *tone);

#ifdef __cplusplus
}
#endif

#endif /* __SCHEDULER_H */
