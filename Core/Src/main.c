/**
 * ===========================================================================
 * @file    main.c
 * @brief   系统入口 — 基于 FreeRTOS 的步兵机器人云台/底盘/发射控制
 * ===========================================================================
 *
 * 【系统概述 (System Overview)】
 *   本系统是 RoboMaster 步兵机器人电控固件，运行在 STM32F407 上，
 *   基于 FreeRTOS 多任务架构。控制对象包括：
 *     - 双轴 GM6020 云台 (Yaw + Pitch)
 *     - 双 C620/M3508 摩擦轮 (发射机构)
 *     - C610/M2006 拨弹盘 (单发/连发)
 *     - CAN 底盘控制命令
 *     - BMI088 IMU 惯性测量 + Pitch 融合
 *
 * 【硬件总线拓扑 (Bus Topology)】
 *   CAN1 (PD0/PD1) — 云台 Yaw + 拨弹盘 + 底盘命令 (0x300/0x301)
 *   CAN2 (PB5/PB6) — 云台 Pitch + 双摩擦轮
 *   USART3 + DMA1  — DBUS 遥控器接收机 (100kbps 8E1)
 *   USART6 + DMA2  — 调试数据输出 (460800 8N1)
 *   SPI1           — BMI088 IMU
 *   USB OTG FS     — CDC 虚拟串口 (上位机协议)
 *   TIM6           — HAL 时基 (1ms, 替代 SysTick)
 *
 * 【架构变化 (RTOS Architecture)】
 *   早期版本为裸机 while(1) 循环；当前版本改为 FreeRTOS：
 *     main() 完成外设与业务模块初始化后启动调度器 (osKernelStart)，
 *     所有业务逻辑在 freertos.c 的 gimbalTask 中以 1 ms 周期运行。
 *
 * 【时钟树 (Clock Tree)】
 *   HSE=8MHz → PLL → SYSCLK=168MHz → HCLK=168MHz
 *   PCLK1=42MHz (CAN/APB1), PCLK2=84MHz (USART6/APB2)
 *   USB 48MHz = PLLQ=7
 *
 * 【关键配置入口】
 *   - config/gimbal_params.h        云台 CAN ID、PID、零位、限位
 *   - config/dual_m3508_params.h    摩擦轮 PID、安全参数
 *   - config/feeder_params.h        拨弹盘 PID、单发步距
 *   - config/pitch_fusion_config.h  IMU 融合、Kalman 参数
 * ===========================================================================
 */
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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * 项目自定义模块 (Project Custom Modules)：
 *   chassis_can.h       — CAN1 底盘控制命令发送 (Chassis CAN1 command transmission)
 *   bmi088.h            — BMI088 6轴 IMU SPI 驱动
 *   control_input.h     — USB CDC 双轴串口控制入口 (USB CDC serial control interface)
 *   dbus.h              — DBUS 遥控器接收与解析
 *   dual_m3508.h        — CAN2 双C620/M3508 摩擦轮控制
 *   feeder_motor.h      — CAN1 C610 ID3 / M2006 拨弹盘控制
 *   gimbal_calibration.h — 双轴上电自动机械零位采样
 *   motor_control.h     — 双轴 GM6020 电机串级 PID 控制
 *   pitch_fusion.h      — Pitch 轴 IMU/编码器融合
 *   remote_gimbal_control.h — DBUS 摇杆到双轴云台位置目标的映射
 *   uart_debug.h        — USART6 统一调试数据输出
 */
#include "chassis_can.h"
#include "bmi088.h"
#include "control_input.h"
#include "dbus.h"
#include "dual_m3508.h"
#include "feeder_motor.h"
#include "gimbal_calibration.h"
#include "motor_control.h"
#include "pitch_fusion.h"
#include "remote_gimbal_control.h"
#include "uart_debug.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point — 系统主入口。
  *
  * 【执行流程】
  *   阶段1: HAL 底层初始化 (HAL_Init → SystemClock_Config)
  *   阶段2: 外设初始化 (GPIO/SPI/DMA/CAN/USART/USB)
  *   阶段3: 业务模块初始化 (GM6020 云台/标定/摩擦轮/拨弹盘/底盘/DBUS/BMI088/融合/调试)
  *   阶段4: 启动 FreeRTOS 调度器 (osKernelStart)
  *          调度器接管后不返回，业务逻辑运行在 gimbalTask (freertos.c)
  *
  * @retval int 正常情况下不返回。
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
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ---- 阶段2：业务模块初始化 ---- */

  /*
   * 初始化双轴 GM6020 电机控制器：
   *   Yaw   → 上板 CAN1（经滑环到下板 CAN2），ID 2，反馈 0x206
   *   Pitch → 上板 CAN2，ID 2，反馈 0x206
   * 两条总线各自配置滤波器并分别发送 0x1FE 电流帧。
   */
  if (GM6020_Init(&hcan1, &hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  GimbalCalibration_Init();

  /*
   * 初始化CAN2上的双C620/M3508摩擦轮：
   *   ID1/ID2反馈0x201/0x202 -> 过滤器组15 -> FIFO1；
   *   控制帧0x200的DATA[0:3]，上电默认双零电流。
   */
  if (DualM3508_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 初始化CAN1上的C610 ID3拨弹盘：
   *   过滤器组1 -> FIFO1 -> 反馈0x203；
   *   控制帧0x200的DATA[4:5]，上电默认零电流且必须中挡重新解锁。
   */
  if (FeederMotor_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 初始化 CAN1 底盘发送通道。CAN1 已由 Yaw 电机模块启动时直接复用；
   * 底盘 0x300/0x301 与 Yaw 0x206/0x1FE 不冲突。
   */
  if (ChassisCAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 启动开发板 C 型板载 DBUS 接口：
   * DBUS -> 板载反相电路 -> PC11/USART3_RX -> DMA1 Stream1。
   */
  if (DBUS_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  RemoteGimbalControl_Init();

  /*
   * BMI088最小通信验证：分别读取加速度计和陀螺仪CHIP_ID。
   * 读取失败不阻止云台工作，诊断结果由USART6调试任务周期输出。
  */
  (void)BMI088_Init(&hspi1);
  PitchFusion_Init();
  if (UART_Debug_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* ---- 阶段4：启动 FreeRTOS 调度器 ---- */

  /* 初始化 FreeRTOS 内核 (CMSIS-RTOS v2 封装) */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */

  /* 创建 gimbalTask 及其他 RTOS 对象 (线程/互斥/队列等) */
  MX_FREERTOS_Init();

  /* 启动调度器 — 开始任务调度，main() 自此不再返回 */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop (防御性代码 — 正常情况下不会执行到这里) */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* osKernelStart() 成功后不会返回；业务调度位于 freertos.c 的 gimbalTask。 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief  System Clock Configuration — 系统时钟配置
  * @note   System Clock source    = PLL (HSE 8 MHz)
  *         SYSCLK  = HSE/PLLM×PLLN/PLLP = 8/6×168/2 = 168 MHz
  *         HCLK    = SYSCLK/1            = 168 MHz
  *         PCLK1   = HCLK/4 = 42 MHz   (APB1: CAN1/2, USART3, TIM6, SPI...)
  *         PCLK2   = HCLK/2 = 84 MHz   (APB2: USART6, ADC...)
  *         USB 48M = VCO/PLLQ = (8×168/6)/7 = 48 MHz (USB FS)
  *
  *         【FreeRTOS 相关】SysTick 由 FreeRTOS 占用用于任务调度心跳，
  *         HAL 时基改用 TIM6 (见 stm32f4xx_hal_timebase_tim.c)。
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
