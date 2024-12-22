/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1; // 与处理板通信
UART_HandleTypeDef huart2; // 调试串口
UART_HandleTypeDef huart3; // 接收处理板命令
ADC_HandleTypeDef hadc1;   // ADC模块

char rxBuffer1[64];        // USART1 接收处理板数据
char rxBuffer3[64];        // USART3 接收处理板命令
uint8_t isWorking = 0;     // 标记是否在工作
char dbg[128];             // 调试信息

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
void ProcessCommand(char *cmd); // 处理处理板通过 USART3 发送的命令

/* Main Program --------------------------------------------------------------*/
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_ADC1_Init();

    // 初始化完成，打印信息
    char initMsg[] = "Collector Board Initialized. Ready to Work.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)initMsg, strlen(initMsg), HAL_MAX_DELAY);

    // 启动 USART3 接收中断
    HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer3, sizeof(rxBuffer3) - 1);

    while (1) {
        if (isWorking) {
            // 开始采集数据
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // 启动 RED LED
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
                uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
                char data_packet[64];
                sprintf(data_packet, "DATA,%u\r\n", adc_value);
                HAL_UART_Transmit(&huart1, (uint8_t *)data_packet, strlen(data_packet), HAL_MAX_DELAY);

                // 在调试串口打印
                sprintf(dbg, "Collected Data: ADC=%u\r\n", adc_value);
                HAL_UART_Transmit(&huart2, (uint8_t *)dbg, strlen(dbg), HAL_MAX_DELAY);
            }
            HAL_ADC_Stop(&hadc1);
            HAL_Delay(500);
        }
    }
}

/* USART3 中断回调：接收处理板命令 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        rxBuffer3[63] = '\0';
        ProcessCommand(rxBuffer3); // 处理命令
        memset(rxBuffer3, 0, sizeof(rxBuffer3));
        HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer3, sizeof(rxBuffer3) - 1);
    }
}

/* 处理接收到的命令 */
void ProcessCommand(char *cmd) {
    if (strncmp(cmd, "CMD:START", 9) == 0) {
        isWorking = 1;
        char dbg[] = "Command: START\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)dbg, strlen(dbg), HAL_MAX_DELAY);
    } else if (strncmp(cmd, "CMD:STOP", 8) == 0) {
        isWorking = 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // 关闭 RED LED
        char dbg[] = "Command: STOP\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)dbg, strlen(dbg), HAL_MAX_DELAY);
    } else {
        char dbg[] = "Unknown Command Received\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)dbg, strlen(dbg), HAL_MAX_DELAY);
    }
}

/* System Clock Configuration */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* ADC1 Initialization */
static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/* GPIO Initialization */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // RED LED
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USART1 Initialization */
static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
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
}

/* USART3 Initialization */
static void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void) {
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq(); // 禁用中断，防止错误扩散
  while (1) {
    // 错误指示：闪烁板载 LED (PA5)
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(500); // 500 毫秒闪烁
  }
}







