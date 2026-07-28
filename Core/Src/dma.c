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
/* USER CODE END Header */

/**
 * ===========================================================================
 * @file    dma.c
 * @brief   DMA 控制器初始化 — DMA1 时钟使能 + 中断配置
 * ===========================================================================
 *
 * 【DMA 用途】
 *   DMA1_Stream1 绑定 USART3_RX，用于 DBUS 遥控器接收机数据接收。
 *   DBUS 协议波特率 100 kbps，18 字节帧，使用 Receive-to-IDLE 模式。
 *
 * 【中断优先级】DMA1_Stream1_IRQn = 0（最高优先级），确保实时接收。
 * ===========================================================================
 */

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

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

