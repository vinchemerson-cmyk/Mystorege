/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
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
 * @file    dma.h
 * @brief   DMA1/DMA2 控制器初始化 — MX_DMA_Init() 声明
 * ===========================================================================
 *
 * 【DMA 通道分配】
 *   DMA1_Stream1 (CH4) — USART3_RX (DBUS 遥控器接收), 优先级 5
 *   DMA2_Stream6 (CH5) — USART6_TX (调试输出),         优先级 5
 *
 * DMA 流的具体参数在 usart.c HAL_UART_MspInit() 中配置。
 * ===========================================================================
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

