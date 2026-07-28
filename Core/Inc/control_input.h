#ifndef CONTROL_INPUT_H
#define CONTROL_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * USB CDC 接收入口。
 * 由 CDC_Receive_FS() 提交刚收到的原始字节。
 */
void control_in_receive(const uint8_t *data, uint32_t length);

/*
 * 双轴串口控制入口。
 * 在主循环中调用，解析 "yaw,pitch\r\n"。
 * Yaw 为相对启动位置的累计多圈角度，Pitch 为单圈位置目标。
 */
void control_in(void);

/*
 * 周期发送双轴反馈。
 * 在主循环中调用，默认每 100 ms 通过 USB CDC 上报一次。
 */
void control_out(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_INPUT_H */
