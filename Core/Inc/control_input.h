/**
 * ===========================================================================
 * @file    control_input.h
 * @brief   USB CDC 虚拟串口命令解析与反馈上报模块 — 对外 API 声明
 * ===========================================================================
 *
 * 【模块职责】
 *   - 接收 USB CDC 虚拟串口下发的命令（目标角度 / 急停 / 清除）
 *   - 解析并分发到 motor_control.c 和 chassis_can.c
 *   - 周期上报双轴反馈状态到上位机
 *   - 作为后续MiniPC视觉瞄准、开火命令及机器人状态回传的协议扩展点
 *
 * 【对外 API】
 *   control_in_receive()  USB CDC 接收中断回调入口
 *   control_in()          主循环命令处理
 *   control_out()         主循环反馈上报 (~1 Hz)
 *
 * 后续联调时在control_in()内增加明确的视觉报文分支，并在control_out()
 * 扩展反馈字段；在协议和控制权仲裁确定前，不在此处猜测开火格式。
 * 【协议详情】参见 control_input.c 文件头注释。
 * ===========================================================================
 */

#ifndef CONTROL_INPUT_H
#define CONTROL_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * USB CDC 接收入口 (USB CDC Receive Entry Point)。
 * 由 CDC_Receive_FS() 提交刚收到的原始字节，在中断上下文中调用。
 */
void control_in_receive(const uint8_t *data, uint32_t length);

/*
 * MiniPC/上位机命令入口 (MiniPC/Host Command Entry)。
 * 在主循环中调用，解析 "yaw,pitch\r\n"、"ESTOP\r\n"、
 * "CLEAR\r\n" 和可选诊断命令 "CALSTATUS\r\n"。
 * Yaw 为相对标定机械零点的累计多圈角度 (accumulated multi-turn angle)，
 * Pitch 为单圈位置目标 (single-turn position target)。
 */
void control_in(void);

/*
 * MiniPC/上位机状态输出 (MiniPC/Host Status Output)。
 * 在主循环中调用，默认每 1000 ms 通过 USB CDC 上报一次。
 */
void control_out(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_INPUT_H */
