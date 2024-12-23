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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// System FSM enumeration
typedef enum
{
    STATE_INIT = 0,             // Initial state
    STATE_WORKMODE_SELECT,      // Select RED or IR mode
    STATE_PREPROCESS_SELECT,    // Select preprocessing algorithm
    STATE_ADVANCED_SELECT,      // Select advanced processing (HR or SpO2)
    STATE_RUNNING,              // System in active data acquisition
    STATE_ERROR                 // Error state
} SystemState_t;

// Button event types
typedef enum
{
    PRESS_NONE = 0,
    PRESS_SHORT,
    PRESS_LONG
} PressType_t;

// Advanced Processing Algorithm
typedef enum
{
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code BEGIN */
    MEASURE_HR = 0,
    MEASURE_SPO2
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code END */
} MeasureType_t;

// Workmode Select
typedef enum
{
    MODE_RED = 0,
    MODE_IR
} WorkMode_t;

// Preprocessing Algorirhm Select
typedef enum
{
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code BEGIN */
    PREPROC_SF = 0,   // Squaring Filter
    PREPROC_EMA       // Exponential Moving Average
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code END */
} PreProcessType_t;

// Function pointer types to decouple algorithms
typedef uint32_t (*PreprocFunc_t)(uint32_t); //Input uint32_t (original ADC), Output uint32_t (preprocessed)
typedef uint32_t (*AdvancedFunc_t)(uint32_t); //Input uint32_t (preprocessed), Output uint32_t (result)

// Simple ring buffer structure for storing logs
#define LOG_BUF_SIZE  256
typedef struct
{
    char  buffer[LOG_BUF_SIZE];
    int   head;
    int   tail;
} RingBuffer_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Thresholds and intervals
#define LONG_PRESS_THRESHOLD   2000
#define DEBOUNCE_INTERVAL      50

// LED pin and port
#define LED_PIN                GPIO_PIN_5
#define LED_PORT               GPIOA

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Global variables for the state machine and events
volatile PressType_t   g_pressEvent   = PRESS_NONE;
SystemState_t          g_systemState  = STATE_INIT;

MeasureType_t    g_measureType  = MEASURE_HR;
WorkMode_t       g_workMode     = MODE_RED;
PreProcessType_t g_preprocType  = PREPROC_SF;



// Function declarations for decoupled algorithms
static uint32_t applySquaringFilter(uint32_t input);
static uint32_t applyExponentialMovingAverage(uint32_t input);
static uint32_t computeHeartRate(uint32_t value);
static uint32_t computeSpO2(uint32_t value);
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code BEGIN */

/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code END */


// Function pointer tables for preprocessing and advanced processing
static PreprocFunc_t g_preprocFuncs[] =
{
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code BEGIN */
    applySquaringFilter,
    applyExponentialMovingAverage
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code END */
};

static AdvancedFunc_t g_advancedFuncs[] =
{
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code BEGIN */
    computeHeartRate,
    computeSpO2
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code END */
};




// Simple ring buffer for log text
static RingBuffer_t g_logRB = {
    .buffer = {0},
    .head   = 0,
    .tail   = 0
};

// Variable for EMA calculations
static float s_emaVal = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
// Ring buffer functions
static void RB_Init(RingBuffer_t* rb);
static int  RB_Write(RingBuffer_t* rb, const char* data, int len);
static int  RB_Read(RingBuffer_t* rb, char* out, int maxlen);

// Local log functions
static void logPrintf(const char* fmt, ...);
static void processLogBuffer(void);

// Additional utility functions
static void enterErrorState(void);
static void updateLED(SystemState_t state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code Begin */
/* Decoupled function implementations */

/**
  * @brief Squaring Filter
  * @param input Original ADC value
  * @return Squared result, clipped to 32-bit
  */
static uint32_t applySquaringFilter(uint32_t input)
{
    uint64_t sq = (uint64_t)input * (uint64_t)input;
    if (sq > 0xFFFFFFFF) sq = 0xFFFFFFFF;
    return (uint32_t)sq;
}

/**
  * @brief Exponential Moving Average
  * @param input Original ADC value
  * @return EMA result (uint32_t)
  */
static uint32_t applyExponentialMovingAverage(uint32_t input)
{
    float alpha = 0.1f;
    s_emaVal = alpha * input + (1.0f - alpha) * s_emaVal;
    return (uint32_t)s_emaVal;
}

/**
  * @brief Compute Heart Rate
  * @param value Preprocessed result
  * @return Heart Rate
  */
static uint32_t computeHeartRate(uint32_t value)
{
    // 60 + (value mod 40) for demonstration
    return 60 + (value % 40);
}

/**
  * @brief Compute SpO2
  * @param value Preprocessed result
  * @return SpO2
  */
static uint32_t computeSpO2(uint32_t value)
{
    // 95 + (value mod 5) for demonstration
    return 95 + (value % 5);
}
/* USER CODE END 0 */
/* ----------------------------------------------------------------------------*/
/* Yuchen Wang's code END */




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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize ring buffer, head/tail=0
  RB_Init(&g_logRB);

  // Print start message
  logPrintf("System Started.\r\nCurrent State: INIT\r\n");

  // Disable RED/IR by default
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    // 1. Check if there is a new button event
	    PressType_t currentPress = g_pressEvent;
	    if (currentPress != PRESS_NONE)
	    {
	      // Reset the event flag after reading
	      g_pressEvent = PRESS_NONE;

	      // State machine transitions based on button events
	      switch (g_systemState)
	      {
	        case STATE_INIT:
	          // If long press in INIT, go to WORKMODE_SELECT
	          if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_WORKMODE_SELECT;
	            logPrintf("Current State: WORKMODE_SELECT\r\n");
	          }
	          break;

	        case STATE_WORKMODE_SELECT:
	          // Short press toggles RED or IR
	          if (currentPress == PRESS_SHORT)
	          {
	            g_workMode = (g_workMode == MODE_RED) ? MODE_IR : MODE_RED;
	            if (g_workMode == MODE_RED)
	              logPrintf("Selected WorkMode: RED\r\n");
	            else
	              logPrintf("Selected WorkMode: IR\r\n");
	          }
	          // Long press moves to PREPROCESS_SELECT
	          else if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_PREPROCESS_SELECT;
	            logPrintf("Current State: PREPROCESS_SELECT\r\n");
	          }
	          break;

	        case STATE_PREPROCESS_SELECT:
	          // Short press toggles SF or EMA
	          if (currentPress == PRESS_SHORT)
	          {
	            g_preprocType = (g_preprocType == PREPROC_SF) ? PREPROC_EMA : PREPROC_SF;
	            if (g_preprocType == PREPROC_SF)
	            	/* ----------------------------------------------------------------------------*/
	            	/* Yuchen Wang's code BEGIN */
	              logPrintf("Selected PreProcessing Algorithm: Squaring Filter\r\n");
	            else
	              logPrintf("Selected PreProcessing Algorithm: ExponentialMovingAverage\r\n");
	            /* ----------------------------------------------------------------------------*/
	            /* Yuchen Wang's code BEGIN */
	          }
	          // Long press moves to ADVANCED_SELECT
	          else if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_ADVANCED_SELECT;
	            logPrintf("Current State: ADVANCED_SELECT\r\n");
	          }
	          break;

	        case STATE_ADVANCED_SELECT:
	          // Short press toggles HR or SpO2
	          if (currentPress == PRESS_SHORT)
	          {
	            g_measureType = (g_measureType == MEASURE_HR) ? MEASURE_SPO2 : MEASURE_HR;
	            if (g_measureType == MEASURE_HR)
	            	/* ----------------------------------------------------------------------------*/
	            	/* Yuchen Wang's code BEGIN */
	              logPrintf("Selected Advanced Processing Algorithm: Heart Rate\r\n");
	            else
	              logPrintf("Selected Advanced Processing Algorithm: SpO2\r\n");
	            /* ----------------------------------------------------------------------------*/
	            /* Yuchen Wang's code BEGIN */
	          }
	          // Long press moves to RUNNING
	          else if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_RUNNING;
	            logPrintf("Current State: RUNNING\r\n");
	          }
	          break;

	        case STATE_RUNNING:
	          // Long press returns to INIT, turns off LEDs
	          if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_INIT;
	            logPrintf("Current State: INIT\r\n");
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	          }
	          break;

	        case STATE_ERROR:
	          // Long press in ERROR goes back to INIT
	          if (currentPress == PRESS_LONG)
	          {
	            g_systemState = STATE_INIT;
	            logPrintf("Current State: INIT (from ERROR)\r\n");
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	          }
	          break;

	        default:
	          // If an undefined state, go to ERROR
	          enterErrorState();
	          break;
	      }
	    }

	    // 2. Perform state-specific operations
	    switch (g_systemState)
	    {
	      case STATE_INIT:
	      case STATE_WORKMODE_SELECT:
	      case STATE_PREPROCESS_SELECT:
	      case STATE_ADVANCED_SELECT:
	        // No ADC acquisition in these states
	        break;

	      case STATE_RUNNING:
	      {
	        // Turn on the correct LED based on chosen mode
	        if (g_workMode == MODE_RED)
	        {
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	        }
	        else
	        {
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	        }

	        // Single ADC poll
	        HAL_ADC_Start(&hadc1);
	        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	        {
	          uint32_t adcValue = HAL_ADC_GetValue(&hadc1);

	          // Preprocessing and advanced processing
	          uint32_t preVal = g_preprocFuncs[g_preprocType](adcValue);
	          uint32_t outVal = g_advancedFuncs[g_measureType](preVal);

	          // Format and log the result
	          char line[80];
	          if (g_measureType == MEASURE_HR)
	          {
	            sprintf(line, "ADC:%lu ->Pre:%lu ->HR:%lu bpm\r\n",
	                    adcValue, preVal, outVal);
	          }
	          else
	          {
	            sprintf(line, "ADC:%lu ->Pre:%lu ->SpO2:%lu%%\r\n",
	                    adcValue, preVal, outVal);
	          }
	          logPrintf(line);
	        }
	        break;
	      }

	      case STATE_ERROR:
	        // Could flash LED quickly or await user input
	        break;

	      default:
	        // If unknown, switch to error
	        enterErrorState();
	        break;
	    }

	    // 3. Update LED based on current state
	    updateLED(g_systemState);

	    // 4. Process log buffer to send messages via UART
	    processLogBuffer();

	    // 5. Minimal delay for main loop
	    HAL_Delay(10);
    /* USER CODE BEGIN 3 */
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
/**
  * @brief EXTI Callback to differentiate short press or long press
  *        Triggered when the user button on PC13 changes state.
  *        This function uses debouncing and press duration measurement.
  * @param GPIO_Pin Pin number that generated the EXTI interrupt
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t pressStartTime = 0;      // Records the timestamp when the button is pressed
  static uint32_t lastInterruptTime = 0;   // Used for debouncing to ignore rapid signals

  if (GPIO_Pin == GPIO_PIN_13)
  {
    uint32_t now = HAL_GetTick();          // Current system time in milliseconds
    if ((now - lastInterruptTime) < DEBOUNCE_INTERVAL)
      return;                              // Ignore if within debounce interval
    lastInterruptTime = now;

    GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    if (pinState == GPIO_PIN_RESET)
    {
      // Button is pressed down (falling edge if using rising trigger)
      pressStartTime = now;
    }
    else
    {
      // Button is released (rising edge)
      uint32_t pressDuration = now - pressStartTime;
      if (pressDuration >= LONG_PRESS_THRESHOLD)
      {
        // Classify as a long press
        g_pressEvent = PRESS_LONG;
      }
      else
      {
        // Classify as a short press
        g_pressEvent = PRESS_SHORT;
      }
    }
  }
}

/**
  * @brief Initialize the ring buffer
  *        Resets head and tail indices and clears the buffer.
  * @param rb Pointer to the RingBuffer_t structure
  */
static void RB_Init(RingBuffer_t* rb)
{
  rb->head = 0;
  rb->tail = 0;
  memset(rb->buffer, 0, LOG_BUF_SIZE);
}

/**
  * @brief Write data to the ring buffer
  *        If the buffer is full, the function stops and discards remaining data.
  * @param rb Pointer to the RingBuffer_t structure
  * @param data Pointer to the data to be written
  * @param len Number of bytes to write
  * @return The number of bytes successfully written
  */
static int RB_Write(RingBuffer_t* rb, const char* data, int len)
{
  int count = 0;
  for(int i = 0; i < len; i++)
  {
    int nextHead = (rb->head + 1) % LOG_BUF_SIZE;
    // If nextHead equals tail, it means the buffer is full
    if(nextHead == rb->tail)
      break;

    rb->buffer[rb->head] = data[i];
    rb->head = nextHead;
    count++;
  }
  return count;
}

/**
  * @brief Read data from the ring buffer
  *        Reads until the buffer is empty or out of space for the output string.
  * @param rb Pointer to the RingBuffer_t structure
  * @param out Pointer to the output buffer
  * @param maxlen Maximum number of bytes to read (including null terminator)
  * @return The number of bytes read (not counting the null terminator)
  */
static int RB_Read(RingBuffer_t* rb, char* out, int maxlen)
{
  int count = 0;
  while(rb->tail != rb->head && count < (maxlen - 1))
  {
    out[count] = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % LOG_BUF_SIZE;
    count++;
  }
  out[count] = '\0';  // Null-terminate
  return count;
}

/**
  * @brief A log function using variable arguments
  *        Formats a string and writes it to the ring buffer.
  * @param fmt Format string
  * @param ... Variable arguments
  */
static void logPrintf(const char* fmt, ...)
{
  char tmp[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  RB_Write(&g_logRB, tmp, strlen(tmp));
}

/**
  * @brief Transmit ring buffer contents via UART
  *        Reads from the buffer and sends to huart2.
  */
static void processLogBuffer(void)
{
  char out[64];
  int readCount = RB_Read(&g_logRB, out, sizeof(out));
  if(readCount > 0)
  {
    HAL_UART_Transmit(&huart2, (uint8_t*)out, readCount, HAL_MAX_DELAY);
  }
}

/**
  * @brief Enter error state
  *        Switches system state to STATE_ERROR, logs an error message,
  *        and disables RED/IR LEDs.
  */
static void enterErrorState(void)
{
  g_systemState = STATE_ERROR;
  logPrintf("System -> ERROR\r\n");

  // Disable both RED and IR
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

/**
  * @brief Update onboard LED (PA5) based on the current system state
  *        Provides different blink intervals or constant states.
  * @param state Current system state
  */
static void updateLED(SystemState_t state)
{
  static uint32_t lastToggleTime = 0;
  static GPIO_PinState ledState  = GPIO_PIN_SET;
  uint32_t now = HAL_GetTick();
  uint32_t blinkInterval = 0;

  switch (state)
  {
    case STATE_INIT:
      blinkInterval = 0;         // LED off
      break;
    case STATE_WORKMODE_SELECT:
      blinkInterval = 500;       // Slow blink
      break;
    case STATE_PREPROCESS_SELECT:
      blinkInterval = 300;       // Medium blink
      break;
    case STATE_ADVANCED_SELECT:
      blinkInterval = 200;       // Fast blink
      break;
    case STATE_RUNNING:
      blinkInterval = 0xFFFFFFFF;// Always on
      break;
    case STATE_ERROR:
      blinkInterval = 100;       // Very fast blink
      break;
    default:
      blinkInterval = 0;
      break;
  }

  if(blinkInterval == 0)
  {
    // Keep LED off
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    ledState = GPIO_PIN_SET;
  }
  else if(blinkInterval == 0xFFFFFFFF)
  {
    // Keep LED on
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    ledState = GPIO_PIN_RESET;
  }
  else
  {
    // Blink at specified interval
    if((now - lastToggleTime) >= blinkInterval)
    {
      lastToggleTime = now;
      ledState = (ledState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, ledState);
    }
  }
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
