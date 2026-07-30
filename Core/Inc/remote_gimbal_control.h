/**
 * ===========================================================================
 * @file    remote_gimbal_control.h
 * @brief   DBUS摇杆到双轴云台位置和底盘速度目标的映射
 * ===========================================================================
 */

#ifndef REMOTE_GIMBAL_CONTROL_H
#define REMOTE_GIMBAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 初始化遥控器云台目标积分器 (Initialize remote gimbal target integrator)。
 * 清零目标角度和执行时间戳。
 */
void RemoteGimbalControl_Init(void);

/*
 * 主循环周期调用 (Main loop periodic call)。
 *
 * CH0 控制 Yaw 目标角速度 (target angular rate)，
 * CH1 控制 Pitch 目标角速度。
 * CH2 控制底盘横移速度vy，CH3控制底盘前进速度vx。
 * S2上/中/下分别选择底盘跟随、不跟随和小陀螺模式。
 * 摇杆在死区内（回中）时归一化输出为 0，目标角度保持不变。
 * DBUS掉线或急停时，底盘发送零速度和软件断电模式。
 *
 * 每轴独立接管条件：
 *   - DBUS在线 + 该轴标定完成 + 未急停 + 该轴电机在线
 * 某轴条件不满足时只暂停该轴，另一轴继续运行；恢复后该轴从实时位置
 * 重新同步目标。
 */
void RemoteGimbalControl_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_GIMBAL_CONTROL_H */
