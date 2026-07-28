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

/**
 * ===========================================================================
 * @file    main.c
 * @brief   主程序入口 — 双轴 GM6020 云台 + CAN 底盘控制
 * ===========================================================================
 *
 * 【系统概述】
 *   本系统是基于 STM32F407 的两轴云台控制器，使用 DJI GM6020 无刷直流
 *   电机作为 Yaw（偏航）和 Pitch（俯仰）驱动，通过 CAN 总线通信。
 *   同时通过 CAN2 向底盘发送运动控制指令。
 *
 * 【硬件架构】
 *   ┌───────────────────────────────────────────────────┐
 *   │                  STM32F407                        │
 *   │  CAN1 ──────── GM6020 Yaw  (ID1, 反馈 0x205)      │
 *   │           └─── GM6020 Pitch (ID2, 反馈 0x206)      │
 *   │  CAN2 ──────── 底盘控制器 (命令 0x300, 模式 0x301) │
 *   │  USB_OTG_FS ── USB CDC 虚拟串口 (上位机通信)       │
 *   │  USART6 ────── 串口调试 (115200-8-N-1)             │
 *   └───────────────────────────────────────────────────┘
 *
 * 【主循环调度（裸机，无 RTOS）】
 *   在 while(1) 主循环中按固定顺序执行四个模块的调度函数：
 *     1. control_in()        — 处理上位机串口命令（目标角度/急停/清除）
 *     2. GM6020_Process()    — CAN 反馈接收 → 编码器多圈累计 → 状态机
 *                              → 串级 PID → 发送电流命令 (~1 kHz)
 *     3. control_out()       — 通过 USB CDC 上报双轴角度和状态 (~1 Hz)
 *     4. ChassisCAN_Process() — CAN2 底盘控制命令周期发送 (~100 Hz)
 *
 * 【控制模式】
 *   正常模式 (SPEED_LOOP_DEBUG_BOOT_ENABLE=0):
 *     上电 → 等待 CAN 反馈 → 进入角度+速度串级位置控制 →
 *     通过 USB 虚拟串口接收 "yaw,pitch\r\n" 格式的目标角度命令。
 *
 *   调试模式 (SPEED_LOOP_DEBUG_BOOT_ENABLE=1):
 *     上电 → 等待 CAN 反馈 → 自动进入速度环调试模式 →
 *     绕过角度环，以固定 RPM 驱动指定轴（用于 PID 参数整定）。
 *
 * 【通讯协议】
 *   上位机 → MCU (USB CDC):
 *     "123.45,-15.30\r\n"  — Yaw=123.45°, Pitch=-15.30°（累计多圈+单圈）
 *     "ESTOP\r\n"          — 立即急停（锁存）
 *     "CLEAR\r\n"           — 解除急停
 *
 *   MCU → 上位机 (USB CDC):
 *     "FB,<yaw>,<pitch>,<yaw_rpm>,<pitch_rpm>,<yaw_on>,<pitch_on>,<estop>\r\n"
 *     每 1000 ms 上报一次。
 *
 * 【CAN 时钟说明】
 *   APB1 = HCLK/4 = 168/4 = 42 MHz
 *   CAN 波特率 = 42 MHz / Prescaler / (1 + BS1 + BS2)
 *              = 42 MHz / 2 / (1 + 16 + 4) = 1 Mbps
 *
 * 【关键配置】
 *   - HSE: 8 MHz → PLL: 168 MHz (HCLK)
 *   - gimbal_params.h: Yaw/Pitch CAN ID、PID 增益、零位偏置、软限位
 *   - chassis_can_config.h: 底盘 CAN ID、Q10 缩放因子、发送周期
 * ===========================================================================
 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * 项目自定义模块：
 *   chassis_can.h  — CAN2 底盘控制命令发送 (Chassis CAN2 command transmission)
 *   control_input.h — USB CDC 双轴串口控制入口 (USB CDC serial control interface)
 *   motor_control.h — 双轴 GM6020 电机串级 PID 控制 (GM6020 cascaded PID control)
 */
#include "chassis_can.h"
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

  /* ---- 阶段1：HAL 外设初始化 ---- */
  /*
   * 初始化顺序说明：
   *   GPIO → CAN → USART → USB 的顺序遵循依赖关系：
   *   1. GPIO 最底层，为其他外设提供引脚配置
   *   2. CAN1 必须在 GM6020_Init() 之前初始化（电机控制依赖 CAN1 句柄）
   *   3. CAN2 必须在 ChassisCAN_Init() 之前初始化（底盘控制依赖 CAN2 句柄）
   *   4. USART6 和 USB 顺序无关紧要（没有互相依赖）
   */
  MX_GPIO_Init();          /* 初始化所有 GPIO 引脚（时钟+复用功能） */
  MX_CAN1_Init();          /* CAN1: PD0(RX), PD1(TX), 1 Mbps  — 电机总线 */
  MX_USART6_UART_Init();   /* USART6: PG9(RX), PG14(TX), 115200 — 调试串口 */
  MX_CAN2_Init();          /* CAN2: PB5(RX), PB6(TX), 1 Mbps  — 底盘总线 */
  MX_USB_DEVICE_Init();    /* USB OTG FS: PA11(DM), PA12(DP) — 虚拟串口 */
  /* USER CODE BEGIN 2 */

  /* ---- 阶段2：业务模块初始化 ---- */

  /*
   * 初始化双轴 GM6020 电机控制器。
   * 内部流程：校验配置（CAN ID/电流槽位无冲突） → 配置 CAN 滤波器
   *         （0x205 接收 Yaw, 0x206 接收 Pitch） → 启动 CAN1
   *         → 发送初始零电流命令（0x1FE 帧）。
   * 必须在 MX_CAN1_Init() 之后调用（依赖 hcan1 句柄已初始化）。
   */
  if (GM6020_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 初始化 CAN2 底盘发送通道。
   * 校验 hcan2 为 CAN2 外设 → 启动 CAN2 → 记录时间戳基准。
   */
  if (ChassisCAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

#if SPEED_LOOP_DEBUG_BOOT_ENABLE
  /*
   * 【调试配置】上电自动进入速度环调试模式：
   *   1. 覆盖默认的 PID 增益（便于快速迭代调参无需重新编译）
   *   2. 进入速度调试模式，以固定 RPM 驱动指定轴
   *
   * 调参完成后将 SPEED_LOOP_DEBUG_BOOT_ENABLE 改回 0，重新编译即可。
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
     * ┌────────── 主循环调度（裸机循环，无 RTOS） ──────────┐
     * │                                                      │
     * │  1. control_in()          ~异步（有数据时才动作）     │
     * │     解析 USB CDC 收到的双轴位置命令 "yaw,pitch\r\n"  │
     * │     或紧急命令 "ESTOP\r\n" / "CLEAR\r\n"             │
     * │     收到有效命令后发送 ACK 回复。                     │
     * │                                                      │
     * │  2. GM6020_Process()      ~1 kHz (由电机反馈驱动)     │
     * │     接收 CAN RX FIFO 中积压的反馈帧                   │
     * │     → 匹配 StdId (0x205/0x206) → 更新编码器多圈累计  │
     * │     → 状态机 (WAIT→POSITION/SPEED_DEBUG→FAULT)       │
     * │     → 角度环 PID → 速度环 PID → 打包 0x1FE 电流帧    │
     * │                                                      │
     * │  3. control_out()         ~1 Hz (1000 ms 周期)       │
     * │     通过 USB CDC 上报反馈帧 "FB,..."                  │
     * │     包含 Yaw/Pitch 多圈角度、转速、在线状态、急停状态  │
     * │                                                      │
     * │  4. ChassisCAN_Process()  ~100 Hz (10 ms 周期)        │
     * │     通过 CAN2 发送底盘控制量帧 + 模式帧               │
     * │                                                      │
     * │  【执行顺序合理性】                                   │
     * │   先处理上位机目标 → 再运行电机控制 → 再上报反馈。    │
     * │   上位机下发命令后，同一轮主循环就能完成闭环：         │
     * │   命令解析 → PID 计算 → 电流输出 → 反馈上报。         │
     * └──────────────────────────────────────────────────────┘
     */
    control_in();
    GM6020_Process();
    control_out();
    ChassisCAN_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief  System Clock Configuration — 系统时钟配置
  * @note   System Clock source    = PLL (HSE)
  *         SYSCLK  = HSE / PLLM * PLLN / PLLP = 8 / 6 * 168 / 2 = 168 MHz
  *         HCLK    = SYSCLK / AHB_DIV = 168 / 1 = 168 MHz
  *         PCLK1   = HCLK / APB1_DIV = 168 / 4 = 42 MHz  (APB1 总线, CAN/UART/I2C/SPI)
  *         PCLK2   = HCLK / APB2_DIV = 168 / 2 = 84 MHz  (APB2 总线, 高速外设)
  *         USB/48M = PLLQ = 7 → VCO/PLLQ = (8*168/6)/7 = 48 MHz (USB FS 专用时钟)
  *
  *         关键外设时钟：
  *           CAN1/2    ← APB1 = 42 MHz
  *           USART6    ← APB2 = 84 MHz
  *           SysTick   ← HCLK/8 = 21 MHz (HAL 1ms 时基)
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
  * @brief  错误处理函数 — 当 HAL 库检测到不可恢复错误时调用。
  *         关闭全局中断后进入死循环，等待看门狗复位或手动断电。
  * @note   在生产环境中可以在此处添加：
  *         1. 保存错误现场到备份寄存器 (BKP)
  *         2. 通过 CAN 发送紧急停机广播
  *         3. 闪烁板载 LED 指示错误码
  *         4. 触发独立看门狗复位
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
