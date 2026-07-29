/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  */
/**
 * ===========================================================================
 * @file    spi.h
 * @brief   SPI1 硬件初始化 — MX_SPI1_Init() 声明与句柄 extern
 * ===========================================================================
 *
 * SPI1 用于 BMI088 IMU 通信：
 *   - PB3=SCK, PB4=MISO, PA7=MOSI (AF5)
 *   - 主机模式, 8-bit 数据, CPOL=0/CPHA=0 (SPI Mode 0)
 *   - 预分频 16 → SPI 时钟 = 84 MHz / 16 = 5.25 MHz
 *   - 软件 NSS (两个 GPIO 片选引脚独立控制)
 * ===========================================================================
 */
/* USER CODE END Header */

#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
