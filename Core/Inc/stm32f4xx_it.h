/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/**
 * ===========================================================================
 * @note  CubeMX 自动生成的中断向量声明。项目自定义中断:
 *          DMA1_Stream1_IRQHandler — DBUS DMA 接收完成中断 (优先级 0)
 *          USART3_IRQHandler       — DBUS 串口中断 (优先级 ?)
 *          USART6_IRQHandler       — 调试串口中断 (优先级 5)
 *          TIM6_DAC_IRQHandler     — HAL 时基中断 (优先级 15, 替代 SysTick)
 *          OTG_FS_IRQHandler       — USB OTG FS 中断 (优先级 5)
 *        Cortex-M4 系统异常 (NMI/HardFault/MemManage/BusFault/UsageFault):
 *          由 CMSIS 启动文件 (startup_stm32f407xx.s) 自动关联。
 * ===========================================================================
 */
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void DMA1_Stream1_IRQHandler(void);
void USART3_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void OTG_FS_IRQHandler(void);
/* USER CODE BEGIN EFP */

void USART6_IRQHandler(void);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
