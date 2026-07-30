/**
 * ===========================================================================
 * @file    uart_debug.h
 * @brief   USART6统一调试数据输出
 * ===========================================================================
 */

#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "stm32f4xx_hal.h"

/*
 * 位置调试行默认只发送Yaw实际/理论位置，先减少绘图通道和串口负担。
 * 改为1并重新编译后，会在同一行追加Pitch实际/理论位置。
 */
#ifndef UART_DEBUG_PITCH_POSITION_ENABLE
#define UART_DEBUG_PITCH_POSITION_ENABLE  0U
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 绑定已初始化且已链接TX DMA的STM32 USART6句柄。
 * 当前固定使用460800-8-N-1，只负责发送ASCII调试数据。
 */
HAL_StatusTypeDef UART_Debug_Init(UART_HandleTypeDef *uart);

/*
 * 由低优先级通信任务每10 ms调用一次。内部轮询输出POSITION、GIMBAL、
 * RC_CHASSIS、IMU_RAW和PITCH_FUSION数据，不应从1 kHz控制任务直接调用。
 */
void UART_Debug_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_DEBUG_H */
