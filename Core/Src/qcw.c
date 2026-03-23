/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : qcw.c
  * @brief          : QCW (Quasi-Continuous Wave) mode implementation.
  *
  * Implements the trapezoidal envelope state machine:
  *   IDLE → RAMP_UP → HOLD → RAMP_DOWN → IDLE
  *
   * Each QCW channel pairs two coil outputs:
   *   - Enable pin: reconfigured as GPIO output, held HIGH during burst
   *   - PWM pin:    timer reconfigured for continuous high-resolution PWM.
   *                 The PWM output is a control signal for an external driver
   *                 board — its duty cycle ramps as the QCW envelope dictates.
  *
  * The QCW_Tick() function is called every 100 µs from the TIM7 ISR and
  * linearly interpolates the duty cycle along the envelope.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "qcw.h"
#include "coil_driver.h"
#include "scheduler.h"
#include "safety.h"
#include "debug_uart.h"
#include "tim.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ---------------------------------------------------------------------------*/
/*                      Coil-Pair Mapping Table                               */
/* ---------------------------------------------------------------------------*/
/*
 * Each QCW channel uses two of the 6 coil slots:
 *   [0] = enable coil index,  [1] = PWM coil index
 *
 * QCW Ch 0 (user Ch 1):  enable = coil 0 (PE9  / TIM1 ),  PWM = coil 1 (PD12 / TIM4 )
 * QCW Ch 1 (user Ch 2):  enable = coil 2 (PE5  / TIM9 ),  PWM = coil 3 (PF6  / TIM10)
 * QCW Ch 2 (user Ch 3):  enable = coil 4 (PF7  / TIM11),  PWM = coil 5 (PF8  / TIM13)
 */
static const uint8_t g_qcw_enable_coil[QCW_NUM_CHANNELS] = { 0, 2, 4 };
static const uint8_t g_qcw_pwm_coil[QCW_NUM_CHANNELS]    = { 1, 3, 5 };

/* GPIO port/pin for enable outputs (same pins used by coil timers, repurposed) */
typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin;
} GPIO_PinDef_t;

static const GPIO_PinDef_t g_enable_gpio[QCW_NUM_CHANNELS] = {
  { GPIOE, GPIO_PIN_9  },   /* Coil 0 = PE9  */
  { GPIOE, GPIO_PIN_5  },   /* Coil 2 = PE5  */
  { GPIOF, GPIO_PIN_7  },   /* Coil 4 = PF7  */
};

/* Timer instances used for PWM output */
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim13;

static TIM_TypeDef * const g_pwm_tim[QCW_NUM_CHANNELS] = {
  TIM4,    /* Coil 1 → TIM4  (APB1, 84 MHz timer clock) */
  TIM10,   /* Coil 3 → TIM10 (APB2, 168 MHz timer clock) */
  TIM13,   /* Coil 5 → TIM13 (APB1, 84 MHz timer clock) */
};

/* Timer clock frequencies (Hz) for accurate ARR calculation */
static const uint32_t g_pwm_tim_clk[QCW_NUM_CHANNELS] = {
  84000000U,   /* TIM4  on APB1 → 84 MHz timer clock */
  168000000U,  /* TIM10 on APB2 → 168 MHz timer clock */
  84000000U,   /* TIM13 on APB1 → 84 MHz timer clock */
};

/* ---------------------------------------------------------------------------*/
/*                         Private Variables                                  */
/* ---------------------------------------------------------------------------*/
static QCW_Channel_t g_qcw[QCW_NUM_CHANNELS];

/* Pre-computed ARR value for each PWM timer (set in QCW_ConfigPWMTimer) */
static uint32_t g_pwm_arr[QCW_NUM_CHANNELS];

/* Saved original PSC value so we can restore it when leaving QCW mode */
static uint32_t g_saved_psc[QCW_NUM_CHANNELS];

/* ---------------------------------------------------------------------------*/
/*                    Private Helper Functions                                */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  Reconfigure a pin between Alternate Function (timer) and GPIO output.
 * @param  pin_def   GPIO port/pin
 * @param  as_gpio   1 = set to GPIO output push-pull, 0 = restore to AF
 * @param  af_num    AF number to restore (only used when as_gpio = 0)
 *
 * AF mappings for reference:
 *   PE9  = AF1 (TIM1),  PE5  = AF3 (TIM9),  PF7  = AF3 (TIM11)
 */
static void QCW_ReconfigPin(const GPIO_PinDef_t *pd, uint8_t as_gpio)
{
  GPIO_InitTypeDef init = {0};
  init.Pin   = pd->pin;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  init.Pull  = GPIO_NOPULL;

  if (as_gpio)
  {
    init.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(pd->port, &init);
    HAL_GPIO_WritePin(pd->port, pd->pin, GPIO_PIN_RESET);
  }
  else
  {
    /* Restore to alternate function for the timer.
     * The correct AF is already configured by CubeMX init (MX_TIMx_Init).
     * Re-running HAL_TIM_PWM_ConfigChannel or MX_TIMx_Init will restore
     * the AF mapping.  For simplicity here, set the pin as AF and use
     * the known AF numbers from the CubeMX configuration. */
    init.Mode = GPIO_MODE_AF_PP;
    /* Determine AF from port/pin */
    if (pd->port == GPIOE && pd->pin == GPIO_PIN_9)
      init.Alternate = GPIO_AF1_TIM1;
    else if (pd->port == GPIOE && pd->pin == GPIO_PIN_5)
      init.Alternate = GPIO_AF3_TIM9;
    else if (pd->port == GPIOF && pd->pin == GPIO_PIN_7)
      init.Alternate = GPIO_AF3_TIM11;
    else
      init.Alternate = 0;
    HAL_GPIO_Init(pd->port, &init);
  }
}

/**
 * @brief  Configure a PWM timer for continuous high-resolution PWM.
 *
 *         The timer is reconfigured with PSC = 0 so the full timer clock
 *         drives the counter.  This maximises duty-cycle resolution:
 *
 *           APB1 timers (TIM4, TIM13)  — 84 MHz clock
 *             @ 20 kHz PWM → ARR = 4199  → 4200 duty levels
 *
 *           APB2 timers (TIM10)         — 168 MHz clock
 *             @ 20 kHz PWM → ARR = 8399  → 8400 duty levels
 *
 *         The output is a proper hardware PWM waveform that an external
 *         driver board can read to control its charging voltage.
 */
static void QCW_ConfigPWMTimer(uint8_t qcw_ch)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return;

  TIM_TypeDef *tim = g_pwm_tim[qcw_ch];
  uint32_t tim_clk = g_pwm_tim_clk[qcw_ch];

  /* Stop timer */
  tim->CR1 &= ~TIM_CR1_CEN;

  /* Save the original OPM prescaler so we can restore it later */
  g_saved_psc[qcw_ch] = tim->PSC;

  /* Remove One-Pulse Mode — set continuous mode */
  tim->CR1 &= ~TIM_CR1_OPM;

  /* Set PSC = 0 to run at full timer clock for maximum duty resolution.
   * ARR = (timer_clock / pwm_frequency) - 1 */
  tim->PSC = 0;
  uint32_t arr = (tim_clk / QCW_PWM_BASE_FREQ_HZ) - 1U;
  g_pwm_arr[qcw_ch] = arr;

  tim->ARR  = arr;
  tim->CCR1 = 0;       /* Start at 0% duty */
  tim->CNT  = 0;

  /* Configure PWM Mode 1 (output HIGH when CNT < CCR1).
   * Clear OC1M bits and set to PWM Mode 1 (110).
   * Also ensure preload is enabled for CCR1. */
  uint32_t ccmr = tim->CCMR1;
  ccmr &= ~(TIM_CCMR1_OC1M_Msk);
  ccmr |= (0x6U << TIM_CCMR1_OC1M_Pos);  /* PWM Mode 1 */
  ccmr |= TIM_CCMR1_OC1PE;                /* Preload enable */
  tim->CCMR1 = ccmr;

  /* Ensure output is enabled */
  tim->CCER |= TIM_CCER_CC1E;

  /* Force update to load shadow registers */
  tim->EGR = TIM_EGR_UG;
  tim->SR  = 0;
}

/**
 * @brief  Restore a PWM timer back to OPM mode for normal coil operation.
 *         Restores the original prescaler, OPM bit, and output compare mode.
 */
static void QCW_RestoreOPMTimer(uint8_t qcw_ch)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return;

  TIM_TypeDef *tim = g_pwm_tim[qcw_ch];

  /* Stop timer */
  tim->CR1 &= ~TIM_CR1_CEN;

  /* Restore original prescaler (1µs ticks for OPM) */
  tim->PSC = g_saved_psc[qcw_ch];

  /* Restore OPM */
  tim->CR1 |= TIM_CR1_OPM;

  /* Restore PWM Mode 2 (as used by standard OPM: HIGH when CNT >= CCR1) */
  uint32_t ccmr = tim->CCMR1;
  ccmr &= ~(TIM_CCMR1_OC1M_Msk);
  ccmr |= (0x7U << TIM_CCMR1_OC1M_Pos);  /* PWM Mode 2 */
  ccmr |= TIM_CCMR1_OC1PE;
  tim->CCMR1 = ccmr;

  /* Restore ARR to full range, CCR1 to safe idle */
  tim->ARR  = 65535;
  tim->CCR1 = 65535;
  tim->CNT  = 0;
  tim->EGR  = TIM_EGR_UG;
  tim->SR   = 0;

  /* Ensure CC1 output enabled */
  tim->CCER |= TIM_CCER_CC1E;
}

/**
 * @brief  Set the PWM duty cycle (0–1000) on the PWM timer.
 *         CCR1 = duty * (ARR+1) / 1000
 */
static inline void QCW_SetDuty(uint8_t qcw_ch, uint16_t duty)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return;
  TIM_TypeDef *tim = g_pwm_tim[qcw_ch];
  uint32_t arr = g_pwm_arr[qcw_ch];
  uint32_t ccr = ((uint32_t)duty * (arr + 1U)) / QCW_DUTY_MAX;
  tim->CCR1 = ccr;
}

/**
 * @brief  Assert the enable pin HIGH or LOW.
 */
static inline void QCW_SetEnable(uint8_t qcw_ch, uint8_t high)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return;
  HAL_GPIO_WritePin(g_enable_gpio[qcw_ch].port,
                    g_enable_gpio[qcw_ch].pin,
                    high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---------------------------------------------------------------------------*/
/*                         Public API                                        */
/* ---------------------------------------------------------------------------*/

void QCW_Init(void)
{
  memset(g_qcw, 0, sizeof(g_qcw));
  memset(g_pwm_arr, 0, sizeof(g_pwm_arr));
  Debug_Log("[QCW] Subsystem initialized (3 channels, idle)");
}

int8_t QCW_SetMode(uint8_t qcw_ch, uint8_t enable)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return -1;

  QCW_Channel_t *ch = &g_qcw[qcw_ch];

  if (enable && !ch->active)
  {
    /* --- Entering QCW mode --- */

    /* 1. Disable the two coils in the standard scheduler */
    uint8_t en_coil = g_qcw_enable_coil[qcw_ch];
    uint8_t pw_coil = g_qcw_pwm_coil[qcw_ch];

    Scheduler_RemoveAllTones(en_coil);
    Scheduler_RemoveAllTones(pw_coil);
    Scheduler_SetCoilEnabled(en_coil, 0);
    Scheduler_SetCoilEnabled(pw_coil, 0);

    /* 2. Reconfigure enable pin as GPIO output */
    QCW_ReconfigPin(&g_enable_gpio[qcw_ch], 1);

    /* 3. Reconfigure PWM timer for continuous PWM */
    QCW_ConfigPWMTimer(qcw_ch);

    /* 4. Set defaults */
    ch->active = 1;
    ch->state  = QCW_STATE_IDLE;
    ch->current_duty = 0;

    /* Set default config if not already configured */
    if (ch->config.tmax == 0)
    {
      ch->config.tmin1     = 100;   /* 10% */
      ch->config.tmax      = 500;   /* 50% */
      ch->config.tmin2     = 0;     /*  0% */
      ch->config.tramp1_ms = 15;
      ch->config.tramp2_ms = 10;
      ch->config.thold_ms  = 0;
    }

    Debug_Printf("[QCW] Ch %u: QCW mode ENABLED (coils %u+%u)",
                 qcw_ch + 1, en_coil, pw_coil);
  }
  else if (!enable && ch->active)
  {
    /* --- Leaving QCW mode --- */

    /* 1. Abort any active burst */
    QCW_Abort(qcw_ch);

    /* 2. Restore enable pin to timer AF */
    QCW_ReconfigPin(&g_enable_gpio[qcw_ch], 0);

    /* 3. Restore PWM timer to OPM */
    QCW_RestoreOPMTimer(qcw_ch);

    /* 4. Re-enable coils in the standard scheduler */
    uint8_t en_coil = g_qcw_enable_coil[qcw_ch];
    uint8_t pw_coil = g_qcw_pwm_coil[qcw_ch];
    Scheduler_SetCoilEnabled(en_coil, 1);
    Scheduler_SetCoilEnabled(pw_coil, 1);

    ch->active = 0;
    ch->state  = QCW_STATE_IDLE;

    Debug_Printf("[QCW] Ch %u: QCW mode DISABLED", qcw_ch + 1);
  }

  return 0;
}

int8_t QCW_Configure(uint8_t qcw_ch, const QCW_Config_t *cfg)
{
  if (qcw_ch >= QCW_NUM_CHANNELS || cfg == NULL) return -1;

  QCW_Channel_t *ch = &g_qcw[qcw_ch];

  /* Clamp duty values to QCW_DUTY_MAX */
  ch->config.tmin1 = (cfg->tmin1 > QCW_DUTY_MAX) ? QCW_DUTY_MAX : cfg->tmin1;
  ch->config.tmax  = (cfg->tmax  > QCW_DUTY_MAX) ? QCW_DUTY_MAX : cfg->tmax;
  ch->config.tmin2 = (cfg->tmin2 > QCW_DUTY_MAX) ? QCW_DUTY_MAX : cfg->tmin2;
  ch->config.tramp1_ms = cfg->tramp1_ms;
  ch->config.tramp2_ms = cfg->tramp2_ms;
  ch->config.thold_ms  = cfg->thold_ms;

  Debug_Printf("[QCW] Ch %u: Config tmin1=%u tmax=%u tmin2=%u ramp1=%ums ramp2=%ums hold=%ums",
               qcw_ch + 1, ch->config.tmin1, ch->config.tmax, ch->config.tmin2,
               ch->config.tramp1_ms, ch->config.tramp2_ms, ch->config.thold_ms);
  return 0;
}

int8_t QCW_Fire(uint8_t qcw_ch)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return -1;

  QCW_Channel_t *ch = &g_qcw[qcw_ch];

  if (!ch->active)           return -1;
  if (ch->state != QCW_STATE_IDLE) return -1;  /* Already firing */
  if (Safety_IsEStopped())   return -1;

  /* Start the burst:
   * 1. Assert enable HIGH
   * 2. Set initial duty = tmin1
   * 3. Start PWM timer
   * 4. Transition to RAMP_UP */

  /* Use a dummy "now" of 0 — the real timestamp will be set on the next tick.
   * We set phase_start_us to 0 as a sentinel; QCW_Tick will initialize it. */
  ch->current_duty   = ch->config.tmin1;
  ch->phase_start_us = 0;  /* sentinel: will be set on first tick */

  QCW_SetDuty(qcw_ch, ch->current_duty);
  QCW_SetEnable(qcw_ch, 1);

  /* Start the PWM timer */
  TIM_TypeDef *tim = g_pwm_tim[qcw_ch];
  tim->CNT = 0;
  tim->EGR = TIM_EGR_UG;
  tim->SR  = 0;
  tim->CR1 |= TIM_CR1_CEN;

  ch->state = QCW_STATE_RAMP_UP;

  Debug_Printf("[QCW] Ch %u: FIRED", qcw_ch + 1);
  return 0;
}

void QCW_Abort(uint8_t qcw_ch)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return;

  QCW_Channel_t *ch = &g_qcw[qcw_ch];

  /* Drive enable LOW immediately */
  QCW_SetEnable(qcw_ch, 0);

  /* Stop PWM timer */
  TIM_TypeDef *tim = g_pwm_tim[qcw_ch];
  tim->CR1 &= ~TIM_CR1_CEN;
  tim->CCR1 = 0;
  tim->CNT  = 0;

  ch->state        = QCW_STATE_IDLE;
  ch->current_duty = 0;
}

void QCW_AbortAll(void)
{
  for (uint8_t i = 0; i < QCW_NUM_CHANNELS; i++)
  {
    if (g_qcw[i].active && g_qcw[i].state != QCW_STATE_IDLE)
    {
      QCW_Abort(i);
    }
  }
}

/* ---------------------------------------------------------------------------*/
/*                     ISR Tick — HOT PATH                                    */
/* ---------------------------------------------------------------------------*/

void QCW_Tick(uint32_t now_us)
{
  for (uint8_t i = 0; i < QCW_NUM_CHANNELS; i++)
  {
    QCW_Channel_t *ch = &g_qcw[i];

    if (!ch->active || ch->state == QCW_STATE_IDLE)
      continue;

    /* E-Stop check */
    if (Safety_IsEStopped())
    {
      QCW_Abort(i);
      continue;
    }

    /* Initialize phase_start_us on first tick after fire */
    if (ch->phase_start_us == 0)
      ch->phase_start_us = now_us;

    uint32_t elapsed_us = now_us - ch->phase_start_us;

    switch (ch->state)
    {
      case QCW_STATE_RAMP_UP:
      {
        uint32_t ramp_us = (uint32_t)ch->config.tramp1_ms * 1000U;
        if (ramp_us == 0 || elapsed_us >= ramp_us)
        {
          /* Ramp complete → transition to HOLD */
          ch->current_duty   = ch->config.tmax;
          ch->state          = QCW_STATE_HOLD;
          ch->phase_start_us = now_us;
        }
        else
        {
          /* Linear interpolation: tmin1 + (tmax - tmin1) * elapsed / ramp_us */
          int32_t range = (int32_t)ch->config.tmax - (int32_t)ch->config.tmin1;
          ch->current_duty = (uint16_t)((int32_t)ch->config.tmin1 +
                             (range * (int32_t)elapsed_us) / (int32_t)ramp_us);
        }
        QCW_SetDuty(i, ch->current_duty);
        break;
      }

      case QCW_STATE_HOLD:
      {
        uint32_t hold_us = (uint32_t)ch->config.thold_ms * 1000U;
        if (elapsed_us >= hold_us)
        {
          /* Hold complete → transition to RAMP_DOWN */
          ch->state          = QCW_STATE_RAMP_DOWN;
          ch->phase_start_us = now_us;
        }
        /* Duty stays at tmax */
        break;
      }

      case QCW_STATE_RAMP_DOWN:
      {
        uint32_t ramp_us = (uint32_t)ch->config.tramp2_ms * 1000U;
        if (ramp_us == 0 || elapsed_us >= ramp_us)
        {
          /* Ramp-down complete → burst finished */
          ch->current_duty = ch->config.tmin2;
          QCW_SetDuty(i, ch->current_duty);

          /* Deassert enable, stop PWM */
          QCW_SetEnable(i, 0);
          g_pwm_tim[i]->CR1 &= ~TIM_CR1_CEN;
          g_pwm_tim[i]->CCR1 = 0;

          ch->state        = QCW_STATE_IDLE;
          ch->current_duty = 0;
        }
        else
        {
          /* Linear interpolation: tmax - (tmax - tmin2) * elapsed / ramp_us */
          int32_t range = (int32_t)ch->config.tmax - (int32_t)ch->config.tmin2;
          ch->current_duty = (uint16_t)((int32_t)ch->config.tmax -
                             (range * (int32_t)elapsed_us) / (int32_t)ramp_us);
          QCW_SetDuty(i, ch->current_duty);
        }
        break;
      }

      default:
        ch->state = QCW_STATE_IDLE;
        break;
    }
  }
}

/* ---------------------------------------------------------------------------*/
/*                          Status Queries                                    */
/* ---------------------------------------------------------------------------*/

uint8_t QCW_IsActive(uint8_t qcw_ch)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return 0;
  return g_qcw[qcw_ch].active;
}

uint8_t QCW_IsFiring(uint8_t qcw_ch)
{
  if (qcw_ch >= QCW_NUM_CHANNELS) return 0;
  return (g_qcw[qcw_ch].state != QCW_STATE_IDLE) ? 1U : 0U;
}

void QCW_GetChannelState(uint8_t qcw_ch, QCW_Channel_t *out)
{
  if (qcw_ch >= QCW_NUM_CHANNELS || out == NULL) return;
  __disable_irq();
  *out = g_qcw[qcw_ch];
  __enable_irq();
}
