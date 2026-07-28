/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
 * @file    gpio.c
 * @brief   GPIO 端口时钟初始化
 * ===========================================================================
 *
 * 本文件仅使能所有用到的 GPIO 端口的时钟：
 *   GPIOA — USB OTG FS, SWD 调试引脚
 *   GPIOB — CAN2 (PB5/PB6)
 *   GPIOD — CAN1 (PD0/PD1)
 *   GPIOG — USART6 (PG9/PG14)
 *   GPIOH — 板载 LED 或其他外设
 *
 * 具体的引脚复用功能配置（Mode/AF/Pull/Speed）在各外设的
 * HAL_*_MspInit() 中完成，不在本文件中处理。
 * ===========================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
