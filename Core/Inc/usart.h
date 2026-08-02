/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
 * @file    usart.h
 * @brief   USART3/USART6 硬件初始化与句柄声明
 * ===========================================================================
 *
 * USART3: 100000-8-E-1，DBUS接收。
 * USART6: 460800-8-N-1，PG14=TX、PG9=RX，TX使用DMA2 Stream6。
 *         在开发板C型外壳上对应丝印“UART1”的3-pin接口。
 * ===========================================================================
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* ─── 全局句柄 (Global Handles, 在 usart.c 中定义) ─── */
extern UART_HandleTypeDef huart3;          /* USART3 — DBUS 遥控器接收 (RX only) */
extern UART_HandleTypeDef huart6;          /* USART6 — 调试串口 (TX/RX) */
extern DMA_HandleTypeDef hdma_usart3_rx;   /* USART3 RX DMA — DMA1_Stream1_CH4 */
extern DMA_HandleTypeDef hdma_usart6_tx;   /* USART6 TX DMA — DMA2_Stream6_CH5 */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);  /* DBUS 遥控器串口初始化 — 100kbps 8E1 RX-only */
void MX_USART6_UART_Init(void);  /* 调试串口初始化 — 460800 8N1 TX/RX */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

