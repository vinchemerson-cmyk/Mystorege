/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
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
 * @file    dma.c
 * @brief   DMA1/DMA2 控制器时钟使能与中断配置
 * ===========================================================================
 *
 * 【DMA 通道分配 (DMA Channel Allocation)】
 *   DMA1_Stream1  (CH4) — USART3 RX (DBUS 遥控器接收), 优先级 5
 *   DMA2_Stream6  (CH5) — USART6 TX (调试输出),         优先级 5
 *
 * 注意: DMA 流的实际参数 (方向/对齐/Normal等) 在 usart.c 的
 *       HAL_UART_MspInit() 中配置，本文件只负责时钟与 NVIC 中断。
 * ===========================================================================
 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* ---- DMA 控制器时钟使能 ---- */
  __HAL_RCC_DMA1_CLK_ENABLE();   /* DMA1 — USART3 RX (DBUS 遥控器) */
  __HAL_RCC_DMA2_CLK_ENABLE();   /* DMA2 — USART6 TX (调试输出) */

  /* ---- DMA 中断初始化 ---- */
  /* DMA1_Stream1_IRQn — USART3 RX 接收完成中断 */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream6_IRQn — USART6 TX 发送完成中断 */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

