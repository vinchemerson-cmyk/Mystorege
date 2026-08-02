/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
/**
 * ===========================================================================
 * @note  本文件由 STM32CubeMX 自动生成，包含所有外设中断服务例程 (ISR)。
 *        项目自定义 ISR:
 *          - USART6_IRQHandler   → HAL_UART_IRQHandler (串口调试)
 *          - CAN1_TX_IRQHandler  → HAL_CAN_IRQHandler (云台 CAN)
 *          - DMA1_Stream1_IRQHandler → DBUS 接收 DMA
 *          - DMA2_Stream6_IRQHandler → USART6 调试发送 DMA
 *        FreeRTOS 下中断优先级的配置见 FreeRTOSConfig.h
 * ===========================================================================
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"     /* USART6 句柄 (huart6) — USART6 handle for debug UART IRQ */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* ─── 外设句柄 extern 声明 (Global Peripheral Handles) ─── */
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;  /* USB OTG FS — CDC 虚拟串口 (usbd_conf.c 定义) */
extern DMA_HandleTypeDef hdma_usart3_rx;   /* USART3 RX DMA — DBUS 接收 (usart.c 定义) */
extern DMA_HandleTypeDef hdma_usart6_tx;   /* USART6 TX DMA — 调试发送 (usart.c 定义) */
extern UART_HandleTypeDef huart3;          /* USART3 — DBUS 遥控器 (usart.c 定义) */
extern UART_HandleTypeDef huart6;          /* USART6 — 调试串口 (usart.c 定义) */
extern TIM_HandleTypeDef htim6;            /* TIM6 — HAL 时基, 1ms Tick (hal_timebase_tim.c 定义) */

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/*     Cortex-M4 处理器中断与异常处理程序                                     */
/*     系统异常 (NMI/HardFault 等) 均在死循环中等待复位,                      */
/*     项目可在此处添加错误诊断 (如闪烁LED/保存错误码)。                      */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* STM32F4xx 外设中断处理程序                                                  */
/*                                                                             */
/* 项目使用的中断一览 (Project IRQ Usage):                                     */
/*   DMA1_Stream1 — USART3 RX DMA (DBUS 遥控器接收)  优先级 0                  */
/*   DMA2_Stream6 — USART6 TX DMA (调试串口发送)     优先级 0                  */
/*   USART3       — DBUS 遥控器接收机中断            优先级 5                  */
/*   USART6       — 调试串口中断                      优先级 5                  */
/*   TIM6         — HAL 时基 (1ms Tick)              优先级 15 (最低)          */
/*   OTG_FS       — USB CDC 虚拟串口中断              优先级 5                  */
/*                                                                             */
/* 全部通过 HAL_xxx_IRQHandler 转发给 HAL 库，由 HAL 内部状态机分发到          */
/* 对应的回调函数 (如 HAL_UARTEx_RxEventCallback → DBUS_StartReceive)。        */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  *        DMA1 流1 全局中断 — USART3 RX (DBUS 接收) DMA 完成中断。
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);  /* 转发给 HAL DMA 状态机 → USART3 接收完成回调 */
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  *        DMA2 流6 全局中断 — USART6 TX (调试输出) DMA 完成中断。
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);  /* 转发给 HAL DMA 状态机 → 发送完成 */
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  *        USART3 全局中断 — DBUS 遥控器接收 (含 Receive-to-IDLE 事件)。
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);  /* → HAL_UARTEx_RxEventCallback → DBUS_StartReceive 重启 DMA */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  *        USART6 全局中断 — 调试串口。
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);  /* 转发给 HAL UART 状态机 */
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  *        TIM6 全局中断 — HAL 时基 (1 ms Tick, 替代 SysTick)。
  *        SysTick 被 FreeRTOS 占用用于任务调度。
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);  /* 每 1 ms 触发 → HAL_IncTick() 更新 HAL 时基 */
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  *        USB OTG FS 全局中断 — USB CDC 虚拟串口。
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);  /* → USBD_LL_* 回调 → USB 协议栈处理 */
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
