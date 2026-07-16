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
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile float motor_angle = 0;
volatile float motor_speed = 0;
volatile float motor_current = 0;
volatile float motor_error = 0;
volatile uint8_t new_data_flag = 0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void C610_SendCurrent(uint8_t id, int16_t current);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>


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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef sFilterConfig;
  // 1. 配置过滤器参数
  sFilterConfig.FilterBank = 0;                        // 使用过滤器组 0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;    // 掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;   // 32位位宽

  // 重点：ID 和 掩码 全部设为 0，代表"不过滤任何报文"，接收总线上的所有数据
  sFilterConfig.FilterIdHigh = 0x0000;                 // 验证码高位
  sFilterConfig.FilterIdLow = 0x0000;                  // 验证码低位
  sFilterConfig.FilterMaskIdHigh = 0x0000;             // 掩码高位
  sFilterConfig.FilterMaskIdLow = 0x0000;              // 掩码低位

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;   // 过滤后将数据放入 FIFO1
  sFilterConfig.FilterActivation = ENABLE;             // 激活该过滤器

  // 2. 将配置写入硬件寄存器
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler(); // 配置失败则进入死循环
  }

  // 3. 启动 CAN 外设 (非常重要，否则硬件不会工作)
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  // 4. 开启 CAN 接收 FIFO1 挂起中断 (非常重要，否则不会触发回调)
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }


  printf("=== System Ready ===\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t current_cmd = 2000; // 2A 转矩电流
  uint32_t last_tick = 0;
  while (1)
  {
    // 每 10ms 发送一次控制指令
    if (HAL_GetTick() - last_tick >= 10)
    {
      last_tick = HAL_GetTick();
      C610_SendCurrent(4, current_cmd);
    }

    // 收到 CAN 报文则闪烁 LED 并打印数据
    if (new_data_flag)
    {
      new_data_flag = 0;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // 闪烁 PC13 LED
      printf("Angle:%.1f Speed:%.0f Current:%.2fA Error:%d\r\n",
             motor_angle, motor_speed, motor_current, (int)motor_error);
    }
    HAL_Delay(1);
    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 发送控制指令给某个 ID 的电调，电流值范围 -10000 ~ 10000
void C610_SendCurrent(uint8_t id, int16_t current)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8] = {0};
  uint32_t tx_mailbox;

  tx_header.StdId = 0x200;   // 选择标识符
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  // 根据 ID 在数据域中的位置填充
  int offset = ((id - 1) % 4) * 2;   // 0,2,4,6
  tx_data[offset] = (current >> 8) & 0xFF;
  tx_data[offset + 1] = current & 0xFF;
  // 其他位置不变（默认为0，表示其他电调电流为0）
  // 如果控制多个电调，可将它们的电流值填入对应位置

  HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    uint16_t std_id = rx_header.StdId;
    if (std_id >= 0x201 && std_id <= 0x208)
    {
      // 解析数据
      uint16_t angle = (rx_data[0] << 8) | rx_data[1];
      int16_t speed = (rx_data[2] << 8) | rx_data[3];
      int16_t actual_current = (rx_data[4] << 8) | rx_data[5];
      uint8_t error_code = rx_data[7];

      // 存入全局变量
      motor_angle = (float)angle / 8191.0f * 360.0f;
      motor_speed = (float)speed;
      motor_current = (float)actual_current / 1000.0f;
      motor_error = (float)error_code;

      new_data_flag = 1; // 标记有新数据，通知主循环
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
