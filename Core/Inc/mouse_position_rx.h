#ifndef __MOUSE_POSITION_RX_H__
#define __MOUSE_POSITION_RX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 初始化鼠标位置串口接收器并启动 1 字节中断接收。
 * 串口格式必须为 115200-8-N-1，与 PC 端 MouseToSerial 一致。
 */
HAL_StatusTypeDef MousePositionRx_Init(UART_HandleTypeDef *huart);

/*
 * 在主循环中高频调用。
 * 该函数把中断中收到的最新位置命令安全地交给 GM6020 位置环。
 */
void MousePositionRx_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOUSE_POSITION_RX_H__ */
