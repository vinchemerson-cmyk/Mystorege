/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
 * @file    can.c
 * @brief   CAN1/CAN2 硬件初始化、MSP 配置与反初始化
 * ===========================================================================
 *
 * 【双 CAN 总线拓扑】
 *   CAN1 (PD0/PD1) → GM6020 Yaw (ID1, 反馈 0x205) + Pitch (ID2, 反馈 0x206)
 *                     波特率 1 Mbps，由 motor_control.c 管理
 *   CAN2 (PB5/PB6) → 底盘控制器
 *                     波特率 1 Mbps，由 chassis_can.c 管理
 *
 * 【波特率计算 (CAN1/CAN2 相同)】
 *   APB1 时钟 = HCLK / APB1_DIV = 168 MHz / 4 = 42 MHz
 *   位时间 = Prescaler × (1 + BS1 + BS2) / APB1_CLK
 *          = 2 × (1 + 16 + 4) / 42 MHz = 42 / 42 MHz = 1 μs
 *   波特率 = 1 / 1 μs = 1 Mbps
 *
 *   CAN BS1 (TimeSeg1) = 16 TQ — 传播段 + 相位缓冲段1（采样点前）
 *   CAN BS2 (TimeSeg2) =  4 TQ — 相位缓冲段2（采样点后）
 *   CAN SJW              =  1 TQ — 同步跳转宽度（时钟偏差容限）
 *   采样点位置 = (1 + 16) / (1 + 16 + 4) ≈ 81%
 *
 * 【CAN1 vs CAN2 差异】
 *   CAN2.AutoBusOff = ENABLE  — 自动离线管理：当发送错误计数 > 255 时
 *                               硬件自动进入 Bus-Off 状态，需软件恢复
 *   CAN1.AutoBusOff = DISABLE — 不自动离线，错误由应用层处理
 *   这是 STM32F407 的特性：CAN2 共享 CAN1 的部分资源（如滤波器），
 *   因此 CAN2 配置 AutoBusOff 可以减少单点故障影响整条总线。
 *
 * 【CAN1/CAN2 共享时钟注意事项】
 *   CAN1 和 CAN2 在 STM32F407 中共享 APB1 时钟使能位。
 *   本文件使用 HAL_RCC_CAN1_CLK_ENABLED 引用计数器来正确管理
 *   共享时钟的使能和禁用，防止一边反初始化时误关另一边的时钟。
 * ===========================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * CAN 外设句柄。
 *   hcan1 — CAN1 外设，挂载 GM6020 双轴云台电机
 *   hcan2 — CAN2 外设，挂载底盘控制器
 * 声明为全局变量供其他模块引用（extern in can.h 和 motor_control.c）。
 */
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/**
 * @brief  CAN1 硬件初始化 — 云台电机总线。
 * @note   引脚: PD0=CAN1_RX, PD1=CAN1_TX (复用 AF9)
 *          波特率 1 Mbps，正常工作模式，不自动离线。
 *          AutoRetransmission=DISABLE: 发送失败后由软件决定是否重试，
 *          避免硬件自动重试占用总线带宽。
 */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /*
   * CAN1 波特率计算：
   *   APB1 CAN 时钟 = 42 MHz (HCLK=168 / AHB=1 / APB1=4)
   *   位速率 = APB1_CLK / Prescaler / (1 + BS1 + BS2)
   *          = 42 MHz / 2 / (1 + 16 + 4) = 1 Mbps
   *   采样点 = (1 + BS1) / (1 + BS1 + BS2) = 17/21 ≈ 81%
   *
   *   配置参数含义：
   *     Prescaler=2         时钟预分频，每个 TQ = 2/42MHz ≈ 47.6 ns
   *     SJW=1TQ             同步跳转宽度，用于补偿时钟偏差
   *     BS1=16TQ (TimeSeg1) 传播段+相位缓冲段1，采样点前的时间
   *     BS2=4TQ  (TimeSeg2) 相位缓冲段2，采样点后的时间
   *     Mode=Normal          正常工作模式（非 Loopback/Silent）
   *     AutoBusOff=DISABLE   不自动离线，软件处理 CAN 错误
   */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/**
 * @brief  CAN2 硬件初始化 — 底盘控制总线。
 * @note   引脚: PB5=CAN2_RX, PB6=CAN2_TX (复用 AF9)
 *          波特率 1 Mbps，正常工作模式。
 *          AutoBusOff=ENABLE: CAN2 与 CAN1 共享部分硬件资源，
 *          CAN2 自动离线可以减少单点故障影响 CAN1 云台控制。
 */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */
  /*
   * CAN2 波特率与 CAN1 相同 (1 Mbps)，配置参数含义参见 CAN1 注释。
   * 唯一差异：AutoBusOff=ENABLE，CAN2 检测到发送错误超限后自动离线。
   */
  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/*
 * CAN1/CAN2 共享时钟引用计数器。
 * STM32F407 中 CAN1 和 CAN2 共用 APB1 的同一个时钟使能位 (RCC_APB1ENR_CAN1EN)，
 * 因此 CAN2 初始化时也需要确保 CAN1 时钟已使能。此计数器追踪使能次数，
 * 避免 CAN1 反初始化时误关时钟导致 CAN2 无法工作（反之亦然）。
 * 增量为 1 时使能时钟，减量为 0 时关闭时钟。
 */
static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

/*
 * CAN MSP 初始化（MSP = MCU Support Package）。
 * HAL_CAN_Init() 内部自动调用此函数配置 GPIO 引脚和时钟。
 * CAN1 和 CAN2 走不同的 GPIO 端口和引脚配置。
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

/*
 * CAN MSP 反初始化。
 * HAL_CAN_DeInit() 内部自动调用此函数释放 GPIO 引脚和时钟资源。
 * 由于 CAN1/CAN2 共享时钟，使用引用计数器管理时钟关闭。
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

