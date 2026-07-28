/**
 * ===========================================================================
 * @file    can.h
 * @brief   CAN1/CAN2 硬件初始化 — 对外 API 声明
 * ===========================================================================
 *
 * 【双 CAN 总线】
 *   CAN1 (PD0/PD1, 1 Mbps) → GM6020 云台电机 (motor_control.c 管理)
 *   CAN2 (PB5/PB6, 1 Mbps) → 底盘控制器 (chassis_can.c 管理)
 *
 * 【对外句柄】
 *   extern hcan1 / hcan2 — 全局 CAN 句柄，供 motor_control.c 和
 *   chassis_can.c 引用以进行 CAN 帧的发送和接收。
 *
 * 【波特率】两路均为 1 Mbps (APB1=42 MHz / Prescaler=2 / (1+16+4))
 * ===========================================================================
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

