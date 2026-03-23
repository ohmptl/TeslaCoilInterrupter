/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : qcw.h
  * @brief          : QCW (Quasi-Continuous Wave) mode for Tesla Coil outputs.
  *
  * QCW mode pairs two fiber outputs per channel:
  *   Enable pin  — held HIGH for the entire burst duration (GPIO output)
  *   PWM pin     — continuous PWM with a ramping duty-cycle envelope
  *
  * Channel mapping (1-based for user-facing, 0-based internally):
  *   QCW Ch 1:  Enable = Coil 0 (PE9/TIM1),   PWM = Coil 1 (PD12/TIM4)
  *   QCW Ch 2:  Enable = Coil 2 (PE5/TIM9),   PWM = Coil 3 (PF6/TIM10)
  *   QCW Ch 3:  Enable = Coil 4 (PF7/TIM11),  PWM = Coil 5 (PF8/TIM13)
  *
  * The QCW envelope is a trapezoidal waveform:
  *   IDLE → RAMP_UP → HOLD → RAMP_DOWN → IDLE
  *
  * The TIM7 scheduler tick (100 µs) drives the QCW state machine.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __QCW_H
#define __QCW_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "safety.h"   /* NUM_COILS */

/* ---------------------------------------------------------------------------*/
/*                          Configuration                                     */
/* ---------------------------------------------------------------------------*/
#define QCW_NUM_CHANNELS      3U    /* 3 QCW channels (each uses 2 coil outputs) */

/** Fixed PWM base frequency for the QCW control signal (Hz).
 *  The PWM timer runs at this frequency; the duty cycle is swept by the
 *  trapezoidal envelope.  With PSC=0 this gives high resolution:
 *    84 MHz / 20 kHz = 4200 duty levels (APB1 timers)
 *   168 MHz / 20 kHz = 8400 duty levels (APB2 timers) */
#define QCW_PWM_BASE_FREQ_HZ 20000U

/** Duty cycle resolution: 0–1000 representing 0.0%–100.0% in 0.1% steps. */
#define QCW_DUTY_MAX          1000U

/* ---------------------------------------------------------------------------*/
/*                          Types                                             */
/* ---------------------------------------------------------------------------*/

/** QCW state machine phases. */
typedef enum
{
  QCW_STATE_IDLE      = 0,
  QCW_STATE_RAMP_UP   = 1,
  QCW_STATE_HOLD      = 2,
  QCW_STATE_RAMP_DOWN = 3
} QCW_State_t;

/** QCW waveform configuration for one channel. */
typedef struct
{
  uint16_t tmin1;       /**< Start duty cycle (0–1000 = 0.0%–100.0%)         */
  uint16_t tmax;        /**< Peak  duty cycle (0–1000)                       */
  uint16_t tmin2;       /**< End   duty cycle (0–1000)                       */
  uint16_t tramp1_ms;   /**< Ramp-up duration   in milliseconds              */
  uint16_t tramp2_ms;   /**< Ramp-down duration in milliseconds              */
  uint16_t thold_ms;    /**< Hold at peak duration in ms (default 0)         */
} QCW_Config_t;

/** QCW per-channel runtime state. */
typedef struct
{
  uint8_t      active;          /**< 1 = QCW mode enabled for this channel    */
  QCW_State_t  state;           /**< Current state machine phase              */
  QCW_Config_t config;          /**< Envelope parameters                      */
  uint32_t     phase_start_us;  /**< Scheduler time when current phase began  */
  uint16_t     current_duty;    /**< Current duty cycle (0–1000)              */
} QCW_Channel_t;

/* ---------------------------------------------------------------------------*/
/*                          Public API                                        */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Initialize QCW subsystem.  All channels start in inactive/IDLE.
 *         Call once at boot, after CoilDriver_Init().
 */
void QCW_Init(void);

/**
 * @brief  Enable or disable QCW mode for a channel.
 *         When enabled, the two underlying coils are disabled in the standard
 *         scheduler and their MIDI routing is suppressed.
 *         When disabled, the coils are returned to normal OPM mode.
 * @param  qcw_ch  QCW channel index (0-based internally, user sees 1-based)
 * @param  enable  1 = activate QCW mode, 0 = deactivate
 * @retval 0 on success, -1 on invalid channel
 */
int8_t QCW_SetMode(uint8_t qcw_ch, uint8_t enable);

/**
 * @brief  Configure the QCW envelope parameters for a channel.
 * @param  qcw_ch  QCW channel index (0-based)
 * @param  cfg     Pointer to configuration struct
 * @retval 0 on success, -1 on error
 */
int8_t QCW_Configure(uint8_t qcw_ch, const QCW_Config_t *cfg);

/**
 * @brief  Trigger a single QCW burst on the specified channel.
 *         The channel must be in QCW mode (QCW_SetMode enabled).
 *         If a burst is already in progress, this call is ignored.
 * @param  qcw_ch  QCW channel index (0-based)
 * @retval 0 on success, -1 on error (channel not active or already firing)
 */
int8_t QCW_Fire(uint8_t qcw_ch);

/**
 * @brief  Abort any active QCW burst on the specified channel.
 *         Immediately drives enable LOW and stops PWM.
 * @param  qcw_ch  QCW channel index (0-based)
 */
void QCW_Abort(uint8_t qcw_ch);

/**
 * @brief  Abort ALL active QCW bursts on all channels.
 *         Called from E-Stop path — must be very fast.
 */
void QCW_AbortAll(void);

/**
 * @brief  QCW tick — called from the TIM7 scheduler ISR (every 100 µs).
 *         Advances the state machine for all active QCW channels.
 * @param  now_us  Current scheduler time in microseconds
 */
void QCW_Tick(uint32_t now_us);

/**
 * @brief  Check if a QCW channel is currently in QCW mode.
 * @return 1 if QCW mode active, 0 otherwise.
 */
uint8_t QCW_IsActive(uint8_t qcw_ch);

/**
 * @brief  Check if a QCW channel is currently mid-burst (not IDLE).
 * @return 1 if firing, 0 otherwise.
 */
uint8_t QCW_IsFiring(uint8_t qcw_ch);

/**
 * @brief  Get a snapshot of a QCW channel's state (ISR-safe copy).
 */
void QCW_GetChannelState(uint8_t qcw_ch, QCW_Channel_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __QCW_H */
