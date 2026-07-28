#ifndef CHASSIS_CAN_CONFIG_H
#define CHASSIS_CAN_CONFIG_H

/* 8字节控制量帧和1字节模式帧使用不同的标准ID。 */
#define CHASSIS_CAN_CTRL_STD_ID         0x300U
#define CHASSIS_CAN_MODE_STD_ID         0x301U

/* 控制量使用Q10定点数编码。 */
#define CHASSIS_Q10_SCALE               1024.0f

/* 底盘命令发送周期：10 ms。 */
#define CHASSIS_CAN_TX_PERIOD_MS        10U

#endif /* CHASSIS_CAN_CONFIG_H */
