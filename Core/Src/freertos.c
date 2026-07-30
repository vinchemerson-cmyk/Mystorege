/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * ===========================================================================
 * @file    freertos.c
 * @brief   FreeRTOS 任务创建与 gimbalTask 主控制循环
 * ===========================================================================
 *
 * 【RTOS 架构说明 (RTOS Architecture)】
 *   本系统从原裸机 while(1) 主循环迁移到 FreeRTOS 架构：
 *   - gimbalTask (512×4 字节栈, osPriorityHigh) 负责云台控制
 *   - imuTask (512×4 字节栈, osPriorityAboveNormal) 负责BMI088采集
 *   - uartDebugTask (1024×4 字节栈, osPriorityLow) 负责USART6调试输出
 *   - 控制与IMU任务1 ms周期，调试任务10 ms周期
 *   - 保留原裸机主循环的调用顺序，确保行为一致性
 *
 *   当前任务架构：
 *     gimbalTask   (1 kHz) — 电机控制、上位机协议、标定、遥控器
 *     imuTask      (1 kHz) — BMI088六轴原始数据与温度采集
 *     uartDebugTask (100 Hz时隙) — USART6分时输出全部关键调试数据
 *   未来可继续扩展：
 *     safetyTask   (50 Hz) — 超时检测、看门狗喂狗、心跳 LED
 *
 * 【任务周期 (Task Period & Scheduling)】
 *   vTaskDelayUntil(&last_wake_time, 1ms) 确保精确 1 kHz 调度：
 *   - 使用绝对唤醒时间 (absolute wake time)，不会累积漂移
 *   - 控制周期 = 1 ms → 与 GM6020 反馈帧速率 (~1 kHz) 匹配
 *   - ChassisCAN_Process() 内部自己节流 (10 ms)
 *
 * 【与原裸机主循环的差异 (vs Bare-Metal)】
 *   裸机 main.c while(1)                     RTOS gimbalTask for(;;)
 *   ─────────────────────────────────       ──────────────────────
 *   control_in()                            DBUS_Process()        ← 新增
 *   GM6020_Process()                        control_in()
 *   control_out()                           GM6020_Process()
 *   ChassisCAN_Process()                    GimbalCalibration_Process() ← 新增
 *                                           RemoteGimbalControl_Process() ← 新增
 *                                           ChassisCAN_Process()
 *
 * 【FreeRTOS 钩子函数 (Hook Functions)】
 *   - vApplicationStackOverflowHook  — 栈溢出保护：关中断 → 死循环
 *   - vApplicationMallocFailedHook   — 动态内存分配失败：关中断 → 死循环
 * ===========================================================================
 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * 项目自定义模块 (Project custom modules)。
 *   原裸机所有模块在此处包含，RTOS 任务中调度。
 */
#include "chassis_can.h"         /* CAN1 底盘控制 */
#include "bmi088.h"              /* BMI088六轴原始数据采集 */
#include "control_input.h"       /* USB CDC 串口协议 */
#include "dbus.h"                /* DBUS 遥控器接收 */
#include "dual_m3508.h"          /* CAN2双C620/M3508摩擦轮 */
#include "feeder_motor.h"        /* CAN1 C610 ID3拨弹盘 */
#include "gimbal_calibration.h"  /* 云台零位 Flash 标定 */
#include "motor_control.h"       /* 双轴 GM6020 电机控制 */
#include "pitch_fusion.h"        /* Pitch轴 BMI088/编码器融合闭环 */
#include "remote_gimbal_control.h" /* DBUS 摇杆映射到云台目标 */
#include "uart_debug.h"          /* USART6统一调试数据输出 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * 云台控制任务执行周期：1 ms → 1 kHz。
 * 与 GM6020 电机反馈帧推送频率 (~1 kHz) 匹配，
 * 确保每个控制周期都能处理最新收到的反馈帧。
 */
#define GIMBAL_CONTROL_PERIOD_MS  1U    /* 控制任务周期 — control task period (ms) */
#define IMU_ACQUISITION_PERIOD_MS 1U    /* IMU采集任务周期 — acquisition period (ms) */
#define UART_DEBUG_PERIOD_MS      10U   /* 调试数据分时时隙 — debug slot period */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/*
 * ─── 云台控制任务定义 (Gimbal Control Task Definition) ───
 *
 * 栈大小: 512 × 4 = 2048 字节 (CMSIS-RTOS v2 以 uint32_t 为单位)。
 *   实际分配: 2048 bytes ≈ 2 KiB。
 *   各模块栈用量估算: 电机控制 PID 栈变量 ~200B + CAN 收发 ~100B
 *                   + USB CDC 缓冲 ~200B + FreeRTOS 上下文 ~100B
 *                   → 总 ~600B（留有充足裕度用于嵌套中断和函数调用）
 * 优先级: osPriorityHigh (仅次于系统 Tick/SVC 中断)
 */
osThreadId_t gimbalTaskHandle;                        /* 任务句柄 — task handle (for osThread* APIs) */
const osThreadAttr_t gimbalTask_attributes = {
  .name       = "gimbalTask",                         /* 任务名称 — task name (debugger display) */
  .stack_size = 512 * 4,                              /* 栈大小 — stack size in uint32_t units (2 KiB) */
  .priority   = (osPriority_t) osPriorityHigh,        /* 任务优先级 — high priority */
};

/*
 * IMU采集任务优先级低于云台控制任务。两个任务同时就绪时，
 * 先执行电机控制，再进行阻塞式SPI读取，避免IMU采集延迟控制周期。
 */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name       = "imuTask",
  .stack_size = 512 * 4,
  .priority   = (osPriority_t) osPriorityAboveNormal,
};

/*
 * UART格式化和发送放在最低优先级任务，避免阻塞1 kHz控制环。
 * 4 KiB栈用于snprintf及各调试数据快照。
 */
osThreadId_t uartDebugTaskHandle;
const osThreadAttr_t uartDebugTask_attributes = {
  .name       = "uartDebugTask",
  .stack_size = 1024 * 4,
  .priority   = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartGimbalTask(void *argument);
void StartImuTask(void *argument);
void StartUartDebugTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of gimbalTask */
  gimbalTaskHandle = osThreadNew(StartGimbalTask, NULL, &gimbalTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

  /* creation of uartDebugTask */
  uartDebugTaskHandle = osThreadNew(
      StartUartDebugTask, NULL, &uartDebugTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
  * @brief  Function implementing the gimbalTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGimbalTask */
/*
 * ─── 云台控制任务主函数 (Gimbal Control Task Main Function) ───
 *
 * FreeRTOS 在调度器启动后调用此函数。无限循环以 1 ms 固定周期运行。
 * 调度顺序与原裸机主循环保持一致，新增 DBUS 遥控器和标定模块。
 */
void StartGimbalTask(void *argument)
{
  /* ---- 任务初始化阶段 (Task Initialization) ---- */
  MX_USB_DEVICE_Init();          /* USB CDC 虚拟串口初始化 */
  /* USER CODE BEGIN StartGimbalTask */
  TickType_t last_wake_time = xTaskGetTickCount(); /* 记录调度器启动时刻作为基准 */

  (void)argument;  /* 未使用参数 — unused FreeRTOS task argument */

  /*
   * ─── 主控制循环 (Main Control Loop) ───
   *
   * 保留原裸机主循环的调用顺序，避免 RTOS 迁移改变控制行为。
   * vTaskDelayUntil() 使用绝对唤醒时间，确保 1 ms 精确周期
   * 且不累积调度漂移 (no cumulative drift)。
   *
   * 调用顺序及频率：
   *   1. DBUS_Process()                — 解析 DBUS 帧, 与反馈速率同步 (~1 kHz)
   *   2. control_in()                   — 串口命令处理, 有命令时才动作
   *   3. GM6020_Process()              — 电机控制, 与 CAN 反馈同步 (~1 kHz)
   *   4. GimbalCalibration_Process()   — 标定状态机, 空操作除非标定中
   *   5. PitchFusion_Process()         — Pitch融合状态与超时维护
   *   6. RemoteGimbalControl_Process() — 摇杆、摩擦轮和拨弹联锁
   *   7. DualM3508_Process()           — CAN2 FIFO1双摩擦轮速度环
   *   8. FeederMotor_Process()         — CAN1 FIFO1反馈与拨弹盘速度环
   *   9. ChassisCAN_Process()          — 内部节流 (10 ms=100 Hz)
   *
   * 调试数据已移到低优先级uartDebugTask，不占用本控制任务发送时间。
   */
  for(;;)
  {
    DBUS_Process();
    control_in();
    GM6020_Process();
    GimbalCalibration_Process();
    PitchFusion_Process();
    RemoteGimbalControl_Process();
    DualM3508_Process();
    FeederMotor_Process();
    /* control_out(); */  /* DBUS 调试期间暂停周期 FB 上报 (暂停后可减小 USB 带宽占用) */
    ChassisCAN_Process();

    /* 绝对延迟 1 ms — 与下一拍唤醒时间对齐，不累积漂移 */
    vTaskDelayUntil(
        &last_wake_time,
        pdMS_TO_TICKS(GIMBAL_CONTROL_PERIOD_MS));
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
  * @brief  Function implementing the imuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  TickType_t last_wake_time = xTaskGetTickCount();

  (void)argument;

  for (;;)
  {
    (void)BMI088_ReadSample();
    vTaskDelayUntil(
        &last_wake_time,
        pdMS_TO_TICKS(IMU_ACQUISITION_PERIOD_MS));
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartUartDebugTask */
/**
  * @brief  Function implementing the uartDebugTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUartDebugTask */
void StartUartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartUartDebugTask */
  TickType_t last_wake_time = xTaskGetTickCount();

  (void)argument;

  for (;;)
  {
    UART_Debug_Process();
    vTaskDelayUntil(
        &last_wake_time,
        pdMS_TO_TICKS(UART_DEBUG_PERIOD_MS));
  }
  /* USER CODE END StartUartDebugTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 * FreeRTOS 栈溢出钩子 (Stack Overflow Hook)。
 * 当任务的栈使用量超过分配值时，FreeRTOS 在任务上下文切换时调用此函数。
 * 处理方式：禁用所有中断 + 死循环（等待看门狗复位或手动重启）。
 * 生产环境中可在此处写错误码到备份寄存器或触发外部看门狗。
 */
void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char *task_name)
{
  (void)task;       /* 溢出任务句柄 — 可用于调试器或日志记录 */
  (void)task_name;  /* 溢出任务名称 — 可在调试器中查看 */
  taskDISABLE_INTERRUPTS();  /* 禁止中断 → 停止调度器 */
  for (;;)                   /* 死循环 → 等待看门狗复位 */
  {
  }
}

/*
 * FreeRTOS 内存分配失败钩子 (Malloc Failed Hook)。
 * 当 pvPortMalloc() 无法分配请求的内存时调用。
 * 处理方式：同栈溢出（关中断+死循环）。
 */
void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();  /* 禁止中断 → 停止调度器 */
  for (;;)                   /* 死循环 → 等待看门狗复位 */
  {
  }
}

/* USER CODE END Application */

