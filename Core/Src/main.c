/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug_uart.h"
#include "cdc_parser.h"
#include "usbd_composite.h"
#include "safety.h"
#include "coil_driver.h"
#include "scheduler.h"
#include "midi_engine.h"
#include "display_ui.h"
#include "qcw.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Debug_Init();
  Debug_Log("System booting...");
  CDC_Parser_Init();
  Debug_Log("USB Composite (CDC+MIDI) initialized");

  /* Milestone 2: Initialize safety, coil driver, scheduler, MIDI engine */
  Safety_Init();
  Debug_Log("Safety limits initialized (defaults)");

  CoilDriver_Init();
  Debug_Log("Coil driver initialized (6 OPM timers)");

  /* SAFETY: Verify all coil outputs are LOW after initialization.
   * If any pin is still HIGH, hardware or init is broken — engage E-Stop. */
  if (CoilDriver_AnyPinHigh())
  {
    Debug_Log("[SAFETY] CRITICAL: Coil pin(s) HIGH after init! E-Stop engaged.");
    Safety_EStopSet(1);
    CoilDriver_StopAll();
  }
  else
  {
    Debug_Log("[SAFETY] Boot check passed: all coil pins LOW");
  }

  /* SAFETY: Sync E-Stop state to the actual physical button position.
   * This corrects any stale state from transient EXTI edges during boot.
   * Pull-down + Disconnect Detect: pin LOW = disconnected/cut = E-Stop active. */
  {
    uint8_t btn_pressed = (HAL_GPIO_ReadPin(ESTOP_BUTTON_GPIO_Port,
                                            ESTOP_BUTTON_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    Safety_EStopSet(btn_pressed);
    if (btn_pressed)
    {
      CoilDriver_StopAll();
      Debug_Log("[SAFETY] E-Stop button is physically pressed at boot");
    }
    else
    {
      Debug_Log("[SAFETY] E-Stop button released — system armed");
    }
  }

  MidiEngine_Init();

  /* QCW subsystem: init after CoilDriver (needs OPM timers ready) */
  QCW_Init();

  /* OLED Display: staggered init to prevent 3.3V brownout from inrush.
   * Must be called after MX_SPI2_Init() and MX_GPIO_Init(). */
  DisplayUI_Init();

  Scheduler_Init();
  Scheduler_Start();

  Debug_Log("Entering main loop");
  HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ------ CDC Command Processing (HIGHEST PRIORITY in main loop) ------ */
    /* Process CDC commands BEFORE MIDI to guarantee control is never starved */
    CDC_Parser_Process(&hUsbDeviceFS);

    /* ------ MIDI Note Processing ------ */
    /* Process USB MIDI packets → scheduler tones (lower priority than CDC) */
    MidiEngine_ProcessUSB(&hUsbDeviceFS);

    /* ------ CDC TX Flush ------ */
    USBD_Composite_CDC_TxFlush(&hUsbDeviceFS);

    /* ------ OLED Display Update (~15 FPS, non-blocking) ------ */
    DisplayUI_Update();

    /* ------ Debug UART Flush ------ */
    Debug_Flush();

    /* ------ Watchdog Feed ------ */
    HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
