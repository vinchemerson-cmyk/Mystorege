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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * 项目自定义模块：
 *   control_input.h — USB CDC 双轴串口控制入口
 *   motor_control.h   — 双轴 GM6020 电机串级 PID 控制
 */
#include "control_input.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * 速度环调试上电自启动配置。
 *
 * SPEED_LOOP_DEBUG_BOOT_ENABLE:
 *   0 — 正常模式：上电后进入位置控制（等待串口目标角度）
 *   1 — 调试模式：上电收到 CAN 反馈后自动进入速度环调试，
 *       绕过角度环，以固定 RPM 驱动电机。
 *
 * 仅在需要整定速度环 PID 参数或测试电机机械响应时开启。
 * 正常使用时设为 0。
 *
 * SPEED_LOOP_DEBUG_AXIS:
 *   调试目标轴：GM6020_AXIS_YAW 或 GM6020_AXIS_PITCH
 *
 * SPEED_LOOP_DEBUG_TARGET_RPM:
 *   调试模式下的初始目标转速（rpm），自动限幅到 ±200 RPM。
 *   正负方向由电机安装方向及编码器标定决定。
 */
#define SPEED_LOOP_DEBUG_BOOT_ENABLE  0U
#define SPEED_LOOP_DEBUG_AXIS         GM6020_AXIS_YAW
#define SPEED_LOOP_DEBUG_TARGET_RPM   100.0f

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
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  MX_CAN2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /*
   * 初始化双轴 GM6020 电机控制器。
   * 内部流程：校验配置 → 配置 CAN 滤波器（0x205, 0x206）
   *         → 启动 CAN → 发送初始零电流命令。
   * 必须在 MX_CAN1_Init() 之后调用。
   */
  if (GM6020_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

#if SPEED_LOOP_DEBUG_BOOT_ENABLE
  /*
   * 上电自动进入速度环调试模式：
   *   1. 覆盖默认的 PID 增益（便于快速迭代调参）
   *   2. 进入速度调试模式，以固定 RPM 驱动指定轴
   *
   * 调参完成后将 SPEED_LOOP_DEBUG_BOOT_ENABLE 改回 0 即可。
   */
  (void)GM6020_SetSpeedPidGains(
      SPEED_LOOP_DEBUG_AXIS, 31.0f, 30.0f, 0.0f);
  GM6020_EnterSpeedDebug(
      SPEED_LOOP_DEBUG_AXIS, SPEED_LOOP_DEBUG_TARGET_RPM);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
     * 主循环调度（裸机，无 RTOS）：
     *
     *   1. control_in()
     *      处理 USB CDC 收到的双轴位置命令。
     *
     *   2. GM6020_Process()
     *      接收两轴 CAN 反馈 → 更新编码器 → 状态机 → PID 计算
     *      → 发送合并 0x1FE 电流命令。
     *
     *   3. control_out()
     *      每 100 ms 通过 USB CDC 上报一次双轴反馈。
     *
     * 执行顺序说明：
     *   先处理串口目标，再运行 GM6020 控制并上报最新反馈。
     */
    control_in();
    GM6020_Process();
    control_out();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
