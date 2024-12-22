/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#define CMD_START "CMD:START"
#define CMD_STOP  "CMD:STOP"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;  // 处理板与采集板通信
UART_HandleTypeDef huart2;  // 处理板与PC通信
UART_HandleTypeDef huart3;  // 处理板与采集板反馈通信

char rxBuffer1[64];
char rxBuffer3[64];
uint8_t is_working = 0;
uint32_t last_command_time = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void ProcessCollectorData(char* line);
void ProcessReceivedCommand(char* cmd);
void SendStartCmd(void);
void SendStopCmd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // 接收采集板数据
    {
        rxBuffer1[63] = '\0';
        ProcessCollectorData(rxBuffer1);

        memset(rxBuffer1, 0, sizeof(rxBuffer1));
        HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuffer1, sizeof(rxBuffer1) - 1);
    }
    else if (huart->Instance == USART3) // 接收采集板命令反馈
    {
        rxBuffer3[63] = '\0';
        ProcessReceivedCommand(rxBuffer3);

        memset(rxBuffer3, 0, sizeof(rxBuffer3));
        HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer3, sizeof(rxBuffer3) - 1);
    }
}

void ProcessCollectorData(char* line)
{
    char dbg[64];
    sprintf(dbg, "Recv DATA: %s\r\n", line);
    HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
}

void ProcessReceivedCommand(char* cmd)
{
    char dbg[64];
    if (strncmp(cmd, CMD_START, strlen(CMD_START)) == 0)
    {
        sprintf(dbg, "Recv ACK: START\r\n");
    }
    else if (strncmp(cmd, CMD_STOP, strlen(CMD_STOP)) == 0)
    {
        sprintf(dbg, "Recv ACK: STOP\r\n");
    }
    else
    {
        sprintf(dbg, "Unknown CMD: %s\r\n", cmd);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
}

void SendStartCmd(void)
{
    char cmd[] = "CMD:START\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

    char dbg[] = "Send CMD: START\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
}

void SendStopCmd(void)
{
    char cmd[] = "CMD:STOP\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

    char dbg[] = "Send CMD: STOP\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    memset(rxBuffer1, 0, sizeof(rxBuffer1));
    HAL_UART_Receive_IT(&huart1, (uint8_t*)rxBuffer1, sizeof(rxBuffer1) - 1);

    memset(rxBuffer3, 0, sizeof(rxBuffer3));
    HAL_UART_Receive_IT(&huart3, (uint8_t*)rxBuffer3, sizeof(rxBuffer3) - 1);

    char initMsg[] = "Processing Board Ready.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)initMsg, strlen(initMsg), HAL_MAX_DELAY);

    while (1)
    {
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_command_time >= 5000)
        {
            if (is_working)
            {
                SendStopCmd();
            }
            else
            {
                SendStartCmd();
            }
            is_working = !is_working;
            last_command_time = current_time;
        }
        HAL_Delay(100);
    }
}

/* CubeMX Initialization Functions */

/* USER CODE END 4 */



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

/* USART2 Initialization */
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
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

/* GPIO Initialization */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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




