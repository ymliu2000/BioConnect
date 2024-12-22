/* DMA-enhanced code for STM32L476RG: Collector Board and Processing Board */

/* This code includes DMA initialization and changes for the described use case.
It includes configuration of ADC and USARTs to use DMA with the stated priorities
and data width adjustments. */

/* -------------------- Collector Board -------------------- */
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

volatile uint8_t rxBufferUart1[64];
volatile uint8_t rxBufferUart3[64];
volatile uint16_t adcData[100]; // ADC DMA buffer
volatile uint8_t isSampling = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);

/* Function to handle UART commands */
void ProcessCommandFromProcessingBoard(void);

/* Main code */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_ADC1_Init();

    // Start UART RX DMA
    HAL_UART_Receive_DMA(&huart3, rxBufferUart3, sizeof(rxBufferUart3));

    // Start ADC DMA (does not sample until triggered)
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcData, sizeof(adcData) / sizeof(adcData[0]));

    // Initialization message
    char msg[] = "Collector Board Ready.\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

    while (1)
    {
        if (isSampling)
        {
            // Simulate sending sampled data to Processing Board
            for (int i = 0; i < 100; i++)
            {
                char dataMsg[32];
                sprintf(dataMsg, "DATA:%u\r\n", adcData[i]);
                HAL_UART_Transmit(&huart1, (uint8_t *)dataMsg, strlen(dataMsg), HAL_MAX_DELAY);
                HAL_Delay(10); // Simulated delay between samples
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
// Check for commands from Processing Board
ProcessCommandFromProcessingBoard();
}
}

void ProcessCommandFromProcessingBoard(void)
{
if (strstr((char *)rxBufferUart3, "CMD:START") != NULL)
{
// Clear buffer
memset((char *)rxBufferUart3, 0, sizeof(rxBufferUart3));

// Enable ADC sampling
isSampling = 1;
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED ON for sampling

char ack[] = "ACK:START\r\n";
HAL_UART_Transmit(&huart3, (uint8_t *)ack, strlen(ack), HAL_MAX_DELAY);
}
else if (strstr((char *)rxBufferUart3, "CMD:STOP") != NULL)
{
// Clear buffer
memset((char *)rxBufferUart3, 0, sizeof(rxBufferUart3));

// Disable ADC sampling
isSampling = 0;
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF for sampling

char ack[] = "ACK:STOP\r\n";
HAL_UART_Transmit(&huart3, (uint8_t *)ack, strlen(ack), HAL_MAX_DELAY);
}
}

/* DMA initialization */
static void MX_DMA_Init(void)
{
__HAL_RCC_DMA1_CLK_ENABLE();

HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
huart1.Instance = USART1;
huart1.Init.BaudRate = 115200;
huart1.Init.WordLength = UART_WORDLENGTH_8B;
huart1.Init.StopBits = UART_STOPBITS_1;
huart1.Init.Parity = UART_PARITY_NONE;
huart1.Init.Mode = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart1.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart1) != HAL_OK)
{
Error_Handler();
}
}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{
huart3.Instance = USART3;
huart3.Init.BaudRate = 115200;
huart3.Init.WordLength = UART_WORDLENGTH_8B;
huart3.Init.StopBits = UART_STOPBITS_1;
huart3.Init.Parity = UART_PARITY_NONE;
huart3.Init.Mode = UART_MODE_TX_RX;
huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart3.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart3) != HAL_OK)
{
Error_Handler();
}
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
hadc1.Instance = ADC1;
hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
hadc1.Init.Resolution = ADC_RESOLUTION_12B;
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
hadc1.Init.LowPowerAutoWait = DISABLE;
hadc1.Init.ContinuousConvMode = DISABLE;
hadc1.Init.NbrOfConversion = 1;
hadc1.Init.DiscontinuousConvMode = DISABLE;
hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
hadc1.Init.DMAContinuousRequests = ENABLE;
hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
hadc1.Init.OversamplingMode = DISABLE;
if (HAL_ADC_Init(&hadc1) != HAL_OK)
{
Error_Handler();
}
}

/* Error Handler */
void Error_Handler(void)
{
while (1)
{
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
HAL_Delay(500);
}
}
