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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_can.h"
#include "control_input.h"
#include "dbus.h"
#include "dbus_monitor.h"
#include "gimbal_calibration.h"
#include "motor_control.h"
#include "remote_gimbal_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GIMBAL_CONTROL_PERIOD_MS  1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for gimbalTask */
osThreadId_t gimbalTaskHandle;
const osThreadAttr_t gimbalTask_attributes = {
  .name = "gimbalTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartGimbalTask(void *argument);

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
void StartGimbalTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartGimbalTask */
  TickType_t last_wake_time = xTaskGetTickCount();

  (void)argument;

  /*
   * 保留原裸机主循环的调用顺序，避免 RTOS 迁移改变控制行为。
   * 任务使用绝对唤醒时间，以 1 ms 周期运行且不累积调度漂移。
   */
  for(;;)
  {
    DBUS_Process();
    control_in();
    GM6020_Process();
    GimbalCalibration_Process();
    RemoteGimbalControl_Process();
    /* control_out(); */  /* DBUS 调试期间暂停周期 FB 上报 */
    DBUS_Monitor_Process();
    ChassisCAN_Process();

    vTaskDelayUntil(
        &last_wake_time,
        pdMS_TO_TICKS(GIMBAL_CONTROL_PERIOD_MS));
  }
  /* USER CODE END StartGimbalTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char *task_name)
{
  (void)task;
  (void)task_name;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

/* USER CODE END Application */

