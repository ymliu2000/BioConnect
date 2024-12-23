/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief 系统状态机枚举
  */
typedef enum
{
    STATE_INIT = 0,        // 开机初始状态
    STATE_ADVANCED_SELECT, // 高级处理选择：心率 or 血氧
    STATE_WORKMODE_SELECT, // 工作模式选择：RED or IR
    STATE_RUNNING,         // 正常采集并处理
    STATE_ERROR            // 错误状态
} SystemState_t;

/**
  * @brief 按键事件：无按键、短按、长按
  */
typedef enum
{
    PRESS_NONE = 0,
    PRESS_SHORT,
    PRESS_LONG
} PressType_t;

/**
  * @brief 测量类型：心率 / 血氧
  */
typedef enum
{
    MEASURE_HR = 0,  // 心率
    MEASURE_SPO2     // 血氧
} MeasureType_t;

/**
  * @brief 工作模式：RED / IR
  */
typedef enum
{
    MODE_RED = 0,
    MODE_IR
} WorkMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LONG_PRESS_THRESHOLD   1000  // 长按阈值 (1秒)
#define DEBOUNCE_INTERVAL      50    // 去抖间隔 (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// 状态机全局变量
volatile PressType_t g_pressEvent = PRESS_NONE;     // 本次“短按/长按”事件 (在主循环中处理后清空)
SystemState_t g_systemState = STATE_INIT;           // 系统当前状态
MeasureType_t g_measureType = MEASURE_HR;           // 默认测量类型 (心率)
WorkMode_t g_workMode = MODE_RED;                   // 默认工作模式 (红光)

// 这里演示用，实际项目中可能更复杂
// 例如一个滤波器、峰值检测或FFT等
static uint16_t computeHeartRate(uint32_t adcValue)
{
    // 简单映射：HR = 60 + (ADC值 mod 40)
    return 60 + (adcValue % 40);
}

static uint8_t computeSpO2(uint32_t adcValue)
{
    // 简单映射：SpO2 = 95 + (ADC值 mod 5)
    return 95 + (adcValue % 5);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void enterErrorState(void);
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
	  char msg[100];
	  uint32_t adcValue = 0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf(msg, "System Started. Current State: INIT\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  // 上电后，默认关闭红光和IR
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // 1. 读取本次的按键事件
	    PressType_t currentPress = g_pressEvent;
	    if (currentPress != PRESS_NONE)
	    {
	      // 读取到后，马上清除，避免重复触发
	      g_pressEvent = PRESS_NONE;

	      // 根据当前系统状态和按键类型，做状态转移或动作
	      switch (g_systemState)
	      {
	        case STATE_INIT:
	          // 在 INIT 下，仅识别长按 => 进入 ADVANCED_SELECT
	          if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_ADVANCED_SELECT;
	            sprintf(msg, "System -> ADVANCED_SELECT\r\n");
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	          }
	          break;

	        case STATE_ADVANCED_SELECT:
	          if (currentPress == PRESS_SHORT)
	          {
	            // 短按 => 在 心率 / 血氧 之间切换
	            g_measureType = (g_measureType == MEASURE_HR) ? MEASURE_SPO2 : MEASURE_HR;
	            if (g_measureType == MEASURE_HR)
	            {
	              sprintf(msg, "Selected: Heart Rate\r\n");
	            }
	            else
	            {
	              sprintf(msg, "Selected: SpO2\r\n");
	            }
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	          }
	          else if (currentPress == PRESS_LONG)
	          {
	            // 长按 => 进入 WORKMODE_SELECT
	            g_systemState = STATE_WORKMODE_SELECT;
	            sprintf(msg, "System -> WORKMODE_SELECT\r\n");
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	          }
	          break;

	        case STATE_WORKMODE_SELECT:
	          if (currentPress == PRESS_SHORT)
	          {
	            // 短按 => 在 RED / IR 模式之间切换
	            g_workMode = (g_workMode == MODE_RED) ? MODE_IR : MODE_RED;
	            if (g_workMode == MODE_RED)
	            {
	              sprintf(msg, "Work Mode: RED\r\n");
	            }
	            else
	            {
	              sprintf(msg, "Work Mode: IR\r\n");
	            }
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	          }
	          else if (currentPress == PRESS_LONG)
	          {
	            // 长按 => 进入 RUNNING
	            g_systemState = STATE_RUNNING;
	            sprintf(msg, "System -> RUNNING\r\n");
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	          }
	          break;

	        case STATE_RUNNING:
	          // 在 RUNNING 下，长按 => 回到 INIT
	          if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_INIT;
	            sprintf(msg, "System -> INIT\r\n");
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	            // 关闭所有LED
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	          }
	          // 短按在 RUNNING 状态下此处不做任何处理
	          break;

	        case STATE_ERROR:
	          // 在 ERROR 下，长按 => 回到 INIT
	          if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_INIT;
	            sprintf(msg, "System -> INIT (from ERROR)\r\n");
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	            // 关闭所有LED
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	          }
	          break;

	        default:
	          // 不期望进入的分支，进入 ERROR
	          enterErrorState();
	          break;
	      }
	    }

	    // 2. 根据当前状态执行“状态行为”
	    switch (g_systemState)
	    {
	      case STATE_INIT:
	        // INIT: 不做采集
	        break;

	      case STATE_ADVANCED_SELECT:
	        // 可在此进行提示或小动画，此处不做任何采集
	        break;

	      case STATE_WORKMODE_SELECT:
	        // 同上，仅选择工作模式，不做采集
	        break;

	      case STATE_RUNNING:
	      {
	        // 根据选择，打开对应LED
	        if (g_workMode == MODE_RED)
	        {
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // 打开红光(低电平有效)
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // 关闭IR
	        }
	        else
	        {
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // 关闭红光
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // 打开IR
	        }

	        // 采集ADC
	        HAL_ADC_Start(&hadc1);
	        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	        {
	          adcValue = HAL_ADC_GetValue(&hadc1);

	          // 根据测量类型，对 ADC 数据做不同的处理
	          if (g_measureType == MEASURE_HR)
	          {
	            uint16_t hrValue = computeHeartRate(adcValue);
	            sprintf(msg, "ADC: %lu -> HR: %u bpm\r\n", adcValue, hrValue);
	          }
	          else
	          {
	            uint8_t spo2Value = computeSpO2(adcValue);
	            sprintf(msg, "ADC: %lu -> SpO2: %u%%\r\n", adcValue, spo2Value);
	          }

	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	        }
	        break;
	      }

	      case STATE_ERROR:
	        // ERROR: 不断闪烁LED或等待用户长按恢复
	        // 此处仅等待用户长按来回到INIT，不做其他操作
	        break;

	      default:
	        // 防御式处理：进入 ERROR
	        enterErrorState();
	        break;
	    }

	    HAL_Delay(10); // 小延时，防止主循环过于频繁
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t pressStartTime = 0;
  static uint32_t lastInterruptTime = 0;

  if (GPIO_Pin == GPIO_PIN_13)
  {
    uint32_t now = HAL_GetTick();

    // 基础去抖判断：若距离上次中断 < DEBOUNCE_INTERVAL，则直接忽略
    if ((now - lastInterruptTime) < DEBOUNCE_INTERVAL)
      return;
    lastInterruptTime = now;

    // 判断当前引脚电平
    GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

    if (pinState == GPIO_PIN_RESET)
    {
      // 下降沿 => 按下
      pressStartTime = now;
    }
    else
    {
      // 上升沿 => 松开
      uint32_t pressDuration = now - pressStartTime;
      if (pressDuration >= LONG_PRESS_THRESHOLD)
      {
        g_pressEvent = PRESS_LONG;
      }
      else
      {
        g_pressEvent = PRESS_SHORT;
      }
    }
  }
}

/**
  * @brief 将系统切换到 ERROR 状态
  */
static void enterErrorState(void)
{
  g_systemState = STATE_ERROR;
  char msg[50];
  sprintf(msg, "System -> ERROR\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  // 关灯
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}
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

#ifdef  USE_FULL_ASSERT
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
