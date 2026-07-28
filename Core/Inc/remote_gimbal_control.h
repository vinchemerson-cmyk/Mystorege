/**
 * ===========================================================================
 * @file    remote_gimbal_control.h
 * @brief   DBUS 摇杆到双轴云台位置目标的映射
 * ===========================================================================
 */

#ifndef REMOTE_GIMBAL_CONTROL_H
#define REMOTE_GIMBAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化遥控器云台目标积分器。 */
void RemoteGimbalControl_Init(void);

/*
 * 主循环周期调用。
 *
 * CH0 控制 Yaw 目标角速度，CH1 控制 Pitch 目标角速度；
 * 摇杆回中时保持当前目标位置。
 */
void RemoteGimbalControl_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_GIMBAL_CONTROL_H */
