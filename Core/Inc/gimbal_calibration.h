#ifndef GIMBAL_CALIBRATION_H
#define GIMBAL_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  GIMBAL_CALIBRATION_WAITING_FEEDBACK = 0,
  GIMBAL_CALIBRATION_WAITING_STILL,
  GIMBAL_CALIBRATION_SAMPLING,
  GIMBAL_CALIBRATION_CALIBRATED,
  GIMBAL_CALIBRATION_ERROR,
  GIMBAL_CALIBRATION_RETURNING_ZERO
} GimbalCalibrationStatus_t;

/*
 * 启动上电自动标定。
 * 必须在 GM6020_Init() 之后、主循环开始之前调用。
 */
void GimbalCalibration_Init(void);

/*
 * Yaw采集100个静止编码器反馈确认传感器稳定，然后应用配置的绝对
 * 编码器传感器零点，不使用开机姿态覆盖零点。
 * Pitch同步采集100个编码器反馈和IMU重力角，把传感器水平定义为逻辑0°，
 * 随后使用位置环斜坡目标缓慢回到该零点。
 * 任一轴可在另一轴离线时独立完成标定。
 */
void GimbalCalibration_Process(void);

/* 返回双轴汇总状态，主要用于CALSTATUS诊断。 */
GimbalCalibrationStatus_t GimbalCalibration_GetStatus(void);

/* 查询单轴标定状态或是否已经可用于控制。 */
GimbalCalibrationStatus_t GimbalCalibration_GetAxisStatus(
    GM6020_Axis_t axis);
bool GimbalCalibration_IsAxisCalibrated(
    GM6020_Axis_t axis);

bool GimbalCalibration_IsBusy(void);
uint16_t GimbalCalibration_GetYawSampleCount(void);
uint16_t GimbalCalibration_GetPitchSampleCount(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CALIBRATION_H */
