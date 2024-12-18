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
#include <string.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RX_BUFFER_SIZE 128      // Larger buffer for receiving data
#define TX_BUFFER_SIZE 128      // Larger buffer for transmitting data
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* For UART receive interrupt (board-to-board) */
static char rxBuffer[RX_BUFFER_SIZE];    // receiving buffer from Collector Board
static volatile uint8_t rxReady = 0;     // a flag to indicate a line is received

/* For UART transmit interrupt */
static char txBufferUart1[TX_BUFFER_SIZE];
static char txBufferUart2[TX_BUFFER_SIZE];
static volatile uint8_t uart1TxBusy = 0; // indicates if UART1 is transmitting
static volatile uint8_t uart2TxBusy = 0; // indicates if UART2 is transmitting

/* USER CODE BEGIN PV */
/* State machine types */
typedef enum {
  STATE_IDLE = 0,
  STATE_SELECT,
  STATE_RUNNING,
  STATE_ERROR
} Level1State_t;

typedef enum {
  SUB_WORKMODE_SELECT=0,
  SUB_PREPROC_SELECT,
  SUB_ADVPROC_SELECT
} SelectSubState_t;

/* Global state variables */
Level1State_t level1State = STATE_IDLE;
SelectSubState_t selectSubState = SUB_WORKMODE_SELECT;
uint8_t workMode    = 0;  // IR=0, RED=1
uint8_t preprocMode = 1;  // 1=no process,2=sliding avg,3=low pass
uint8_t advMode     = 0;  // 0=heart rate,1=SpO2
uint8_t isRunning   = 0;

/* LED blink and button */
uint32_t lastBlinkTick = 0;
uint8_t blinkPhase = 0;

uint8_t  btnPressed    = 0;
uint32_t btnPressTick  = 0;
uint8_t  btnShortPress = 0;

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

void RunStateMachine(void);
void CheckButton(void);
void UpdateLedBlink(void);
void SendStartCmd(void);
void SendStopCmd(void);
void ProcessCollectorData(const char* line);

static void UART_Send_IT(UART_HandleTypeDef *huart, const char *str);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

void RunStateMachine(void);
void CheckButton(void);
void UpdateLedBlink(void);
void SendStartCmd(void);
void SendStopCmd(void);
void ProcessCollectorData(const char* line);

static void UART_Send_IT(UART_HandleTypeDef *huart, const char *str);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  UART receive complete callback for USART1
 *         We assume that we receive a full line at once (since original code used HAL_UART_Receive_IT with a block size).
 *         To make it more robust, consider using single-byte interrupt or IDLE line detection.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    // Ensure termination
    rxBuffer[RX_BUFFER_SIZE - 1] = '\0';

    // Find newline or treat the whole buffer as one message
    // If your protocol ends with '\r\n', you should find it.
    // For simplicity, assume the entire buffer is one line:
    rxReady = 1;

    // Re-arm the receive
    HAL_UART_Receive_IT(&huart1, (uint8_t*)rxBuffer, RX_BUFFER_SIZE-1);
  }
}

/**
 * @brief UART transmit complete callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1) {
    uart1TxBusy = 0; // UART1 transmission finished
  }
  else if(huart->Instance == USART2) {
    uart2TxBusy = 0; // UART2 transmission finished
  }
}

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Adjust interrupt priorities if necessary:
     * Ensure USART1 interrupts have a priority that ensures timely processing.
     * For example:
     */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    // Lower number = higher priority. Adjust as needed.

    /* Start UART receive interrupt for board-to-board communication */
    memset(rxBuffer,0,sizeof(rxBuffer));
    HAL_UART_Receive_IT(&huart1,(uint8_t*)rxBuffer,RX_BUFFER_SIZE-1);

    UART_Send_IT(&huart2,"Processing Board (3-level SM). Start in IDLE.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    CheckButton();
	    RunStateMachine();
	    UpdateLedBlink();

	    // If rxReady is set, process the received line
	    if(rxReady){
	      rxReady = 0;
	      // Make a copy to avoid issues if we re-arm reception
	      char lineCopy[RX_BUFFER_SIZE];
	      strncpy(lineCopy, rxBuffer, RX_BUFFER_SIZE-1);
	      lineCopy[RX_BUFFER_SIZE-1] = '\0';

	      // Debug print
	      char debugLine[128];
	      snprintf(debugLine, sizeof(debugLine), "[PB] RxCplt: %s\n", lineCopy);
	      UART_Send_IT(&huart2, debugLine);

	      // Parse the line
	      if(strncmp(lineCopy, "DATA,",5)==0) {
	        ProcessCollectorData(lineCopy);
	      } else if(strncmp(lineCopy,"ACK:START",9)==0) {
	        UART_Send_IT(&huart2,"Collector ACK:START\r\n");
	      } else if(strncmp(lineCopy,"ACK:STOP",8)==0) {
	        UART_Send_IT(&huart2,"Collector ACK:STOP\r\n");
	      } else if(strncmp(lineCopy,"ERROR:",6)==0) {
	        UART_Send_IT(&huart2,"Collector ERROR\r\n");
	        level1State = STATE_ERROR;
	      }
	    }

	    HAL_Delay(20);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void RunStateMachine(void)
{
  switch(level1State)
  {
    case STATE_IDLE:
      if(btnShortPress)
      {
        level1State = STATE_SELECT;
        selectSubState = SUB_WORKMODE_SELECT;

        char dbg[64];
        snprintf(dbg, sizeof(dbg), "Enter STATE_SELECT, Sub=WORKMODE_SELECT (workMode=%s)\r\n", (workMode==0?"IR":"RED"));
        UART_Send_IT(&huart2, dbg);

        btnShortPress=0;
      }
      break;

    case STATE_SELECT:
      switch(selectSubState)
      {
        case SUB_WORKMODE_SELECT:
          if(btnShortPress)
          {
            workMode = !workMode;
            btnShortPress=0;

            char dbg[64];
            snprintf(dbg,sizeof(dbg),"Now WorkMode=%s\r\n",(workMode==0?"IR":"RED"));
            UART_Send_IT(&huart2, dbg);
          }
          break;

        case SUB_PREPROC_SELECT:
          if(btnShortPress)
          {
            preprocMode++;
            if(preprocMode>3) preprocMode=1;
            btnShortPress=0;

            char dbg[64];
            snprintf(dbg,sizeof(dbg),"Now PreprocMode=%d\r\n", preprocMode);
            UART_Send_IT(&huart2, dbg);
          }
          break;

        case SUB_ADVPROC_SELECT:
          if(btnShortPress)
          {
            advMode = !advMode;
            btnShortPress=0;

            char dbg[64];
            snprintf(dbg,sizeof(dbg),"Now AdvMode=%s\r\n",(advMode==0?"HR/Arrhythmia":"SpO2"));
            UART_Send_IT(&huart2, dbg);
          }
          break;
      }
      break;

    case STATE_RUNNING:
      if(btnShortPress)
      {
        SendStopCmd();
        isRunning=0;
        level1State=STATE_IDLE;

        UART_Send_IT(&huart2,"Now STOP => Go back to IDLE\r\n");
        btnShortPress=0;
      }
      break;

    case STATE_ERROR:
      if(btnShortPress)
      {
        level1State=STATE_IDLE;
        UART_Send_IT(&huart2,"Error cleared, back to IDLE\r\n");
        btnShortPress=0;
      }
      break;
  }
}

void CheckButton(void)
{
  static uint8_t prevPin=1;
  uint8_t pinVal = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

  if(pinVal!=prevPin)
  {
    if(pinVal==0)
    {
      btnPressed=1;
      btnPressTick=HAL_GetTick();
    }
    else
    {
      if(btnPressed)
      {
        uint32_t pressDuration=HAL_GetTick()-btnPressTick;
        if(pressDuration<800) // short press
        {
          btnShortPress=1;
        }
        else
        {
          // long press => switch sub-state or enter RUNNING
          if(level1State==STATE_SELECT)
          {
            switch(selectSubState)
            {
              case SUB_WORKMODE_SELECT:
                selectSubState=SUB_PREPROC_SELECT;
                UART_Send_IT(&huart2,"Enter STATE_SELECT, Sub=PREPROC_SELECT\r\n");
                break;
              case SUB_PREPROC_SELECT:
                selectSubState=SUB_ADVPROC_SELECT;
                UART_Send_IT(&huart2,"Enter STATE_SELECT, Sub=ADVPROC_SELECT\r\n");
                break;
              case SUB_ADVPROC_SELECT:
                SendStartCmd();
                isRunning=1;
                level1State=STATE_RUNNING;
                break;
            }
          }
        }
        btnPressed=0;
      }
    }
    prevPin=pinVal;
  }
}

void UpdateLedBlink(void)
{
  static uint8_t ledOn=0;
  uint32_t now=HAL_GetTick();

  switch(level1State) {
    case STATE_IDLE:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      break;
    case STATE_SELECT:
      if(selectSubState==SUB_WORKMODE_SELECT) {
        if(now-lastBlinkTick>1000) {
          lastBlinkTick=now;
          ledOn=!ledOn;
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ledOn?GPIO_PIN_SET:GPIO_PIN_RESET);
        }
      } else if(selectSubState==SUB_PREPROC_SELECT) {
        if(now-lastBlinkTick>200) {
          lastBlinkTick=now;
          blinkPhase++;
          if(blinkPhase>=5) blinkPhase=0;
          if(blinkPhase==0||blinkPhase==2)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          else
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
      } else if(selectSubState==SUB_ADVPROC_SELECT) {
        if(now-lastBlinkTick>150) {
          lastBlinkTick=now;
          blinkPhase++;
          if(blinkPhase>=6) blinkPhase=0;
          if(blinkPhase==0||blinkPhase==2||blinkPhase==4)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          else
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
      }
      break;
    case STATE_RUNNING:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      break;
    case STATE_ERROR:
      if(now-lastBlinkTick>300) {
        lastBlinkTick=now;
        ledOn=!ledOn;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ledOn?GPIO_PIN_SET:GPIO_PIN_RESET);
      }
      break;
  }
}

/* Send START command (ASCII) using non-blocking transmit */
void SendStartCmd(void)
{
  char wStr[8];
  strcpy(wStr,(workMode==0)?"IR":"RED");

  snprintf(txBufferUart1, TX_BUFFER_SIZE, "CMD:START,%s,%d,%d\r\n", wStr, preprocMode, advMode);
  UART_Send_IT(&huart1, txBufferUart1);

  char dbg[80];
  snprintf(dbg,sizeof(dbg),"Send START: %s", txBufferUart1);
  UART_Send_IT(&huart2, dbg);
}

/* Send STOP command (ASCII) using non-blocking transmit */
void SendStopCmd(void)
{
  snprintf(txBufferUart1, TX_BUFFER_SIZE, "CMD:STOP\r\n");
  UART_Send_IT(&huart1, txBufferUart1);

  UART_Send_IT(&huart2,"Send CMD:STOP\r\n");
}

/**
 * @brief Process data line: "DATA,rawVal,preVal"
 * If preprocMode=1 (no process), treat rawVal, preVal as float.
 * Although in reality pre-processing is on the collector board side,
 * we show float handling here as requested.
 */
void ProcessCollectorData(const char* line)
{
  uint16_t rawVal, preVal;
  sscanf(line,"DATA,%hu,%hu",&rawVal,&preVal);

  // If no processing: treat them as float
  if(preprocMode == 1) {
    float rawF = (float)rawVal;
    float preF = (float)preVal;
    char dbg[128];
    if(advMode==0)
      snprintf(dbg,sizeof(dbg),"Recv DATA: raw=%.2f, pre=%.2f, [Adv=HR]\r\n", rawF, preF);
    else
      snprintf(dbg,sizeof(dbg),"Recv DATA: raw=%.2f, pre=%.2f, [Adv=SpO2]\r\n", rawF, preF);

    UART_Send_IT(&huart2, dbg);
  } else {
    // Other mode still print as integer
    char dbg[128];
    if(advMode==0)
      snprintf(dbg,sizeof(dbg),"Recv DATA: raw=%u, pre=%u, [Adv=HR]\r\n", rawVal, preVal);
    else
      snprintf(dbg,sizeof(dbg),"Recv DATA: raw=%u, pre=%u, [Adv=SpO2]\r\n", rawVal, preVal);

    UART_Send_IT(&huart2, dbg);
  }
}

/**
 * @brief Non-blocking send function using HAL_UART_Transmit_IT()
 *        Store the string in a buffer and start transmission if not busy.
 */
static void UART_Send_IT(UART_HandleTypeDef *huart, const char *str)
{
  // For simplicity, we assume str is stored in a static or global buffer
  // already done above. If we need a dynamic approach, consider double-buffering.

  if (huart->Instance == USART1) {
    // Wait if UART1 is busy (or handle in a queue)
    while(uart1TxBusy) { /* You could use a queue or semaphore here */ }
    uart1TxBusy = 1;
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)str, strlen(str));
  }
  else if(huart->Instance == USART2) {
    while(uart2TxBusy) { /* Wait or queue */ }
    uart2TxBusy = 1;
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)str, strlen(str));
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 (LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
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
	    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	    HAL_Delay(500);
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
