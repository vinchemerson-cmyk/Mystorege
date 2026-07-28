/**
 * ===========================================================================
 * @file    chassis_can_config.h
 * @brief   底盘 CAN 控制配置参数 — CAN ID、编码格式、发送周期
 * ===========================================================================
 *
 * 【配置项说明】
 *   CHASSIS_CAN_CTRL_STD_ID  — 控制量帧 CAN ID（8 字节：vx/vy/wz/offset）
 *   CHASSIS_CAN_MODE_STD_ID  — 模式帧 CAN ID（1 字节：底盘模式枚举值）
 *   CHASSIS_Q10_SCALE        — Q10 定点数缩放因子（1024 = 2^10）
 *   CHASSIS_CAN_TX_PERIOD_MS — 发送周期（ms），默认 10 ms = 100 Hz
 *
 * 【修改指南】
 *   1. CAN ID 必须是标准帧范围 (0x000 ~ 0x7FF)，且控制/模式 ID 必须不同
 *   2. 修改 Q10_SCALE 需同时确认 chassis_can.c 中的 q10_value_is_valid()
 *      范围检查是否仍然正确
 *   3. 发送周期不应低于 5 ms（底盘控制器接收能力有限）
 *   4. 所有 #define 在编译期通过 #if/#error 检查合法性
 * ===========================================================================
 */

#ifndef CHASSIS_CAN_CONFIG_H
#define CHASSIS_CAN_CONFIG_H

/* 8字节控制量帧和1字节模式帧使用不同的标准ID。 */
#define CHASSIS_CAN_CTRL_STD_ID         0x300U   /* 控制量帧标准ID — control frame StdID */
#define CHASSIS_CAN_MODE_STD_ID         0x301U   /* 模式帧标准ID — mode frame StdID */

/* 控制量使用Q10定点数编码 (Q10 fixed-point encoding)。Q10 = 乘以1024取整。 */
#define CHASSIS_Q10_SCALE               1024.0f  /* Q10 定点缩放因子 */

/* 底盘命令发送周期：10 ms → 100 Hz。 */
#define CHASSIS_CAN_TX_PERIOD_MS        10U      /* command transmit period — 命令发送周期 */

#endif /* CHASSIS_CAN_CONFIG_H */
