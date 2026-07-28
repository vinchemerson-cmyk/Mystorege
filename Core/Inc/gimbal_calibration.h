#ifndef GIMBAL_CALIBRATION_H
#define GIMBAL_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  GIMBAL_CALIBRATION_WAITING_FEEDBACK = 0,
  GIMBAL_CALIBRATION_WAITING_STILL,
  GIMBAL_CALIBRATION_SAMPLING,
  GIMBAL_CALIBRATION_CALIBRATED,
  GIMBAL_CALIBRATION_ERROR
} GimbalCalibrationStatus_t;

/*
 * 启动上电自动标定。
 * 必须在 GM6020_Init() 之后、主循环开始之前调用。
 */
void GimbalCalibration_Init(void);

/*
 * 采集两轴各 100 个新的静止反馈，把开机姿态设置为机械零点。
 * 标定完成后，电机控制器继续保持该位置。
 */
void GimbalCalibration_Process(void);

GimbalCalibrationStatus_t GimbalCalibration_GetStatus(void);
bool GimbalCalibration_IsBusy(void);
uint16_t GimbalCalibration_GetYawSampleCount(void);
uint16_t GimbalCalibration_GetPitchSampleCount(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CALIBRATION_H */
