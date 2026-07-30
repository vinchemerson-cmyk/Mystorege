/**
 * ===========================================================================
 * @file    pitch_fusion.h
 * @brief   Pitch 轴 BMI088 / GM6020 融合器
 * ===========================================================================
 *
 * 该模块生成完整估计值，并通过 PitchFusion_GetControlFeedback() 向
 * GM6020 Pitch串级PID提供带安全限制的角度和速度反馈。
 * ===========================================================================
 */

#ifndef PITCH_FUSION_H
#define PITCH_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  PITCH_FUSION_WAITING_DATA = 0,
  PITCH_FUSION_WAITING_STILL,
  PITCH_FUSION_CALIBRATING,
  PITCH_FUSION_RUNNING,
  PITCH_FUSION_DEGRADED
} PitchFusionStatus_t;

typedef struct
{
  PitchFusionStatus_t status;

  /* IMU计算值。accel_pitch_raw_deg 尚未减去安装偏差。 */
  float accel_pitch_raw_deg;
  float accel_pitch_aligned_deg;
  float gyro_pitch_rate_dps;
  float gyro_bias_dps;
  float accel_norm_g;

  /* GM6020机械相对量。 */
  float encoder_pitch_deg;
  float encoder_pitch_rate_dps;

  /* 惯性Pitch估计及底盘/安装架扰动角速度估计。 */
  float fused_pitch_deg;
  float fused_pitch_rate_dps;
  float base_disturbance_rate_dps;
  float accel_innovation_deg;

  uint32_t imu_sample_count;
  uint32_t fusion_update_count;
  uint16_t calibration_sample_count;
  uint16_t recovery_sample_count;

  bool imu_valid;
  bool motor_valid;
  bool accel_trusted;
  bool recovery_pending;
} PitchFusionData_t;

void PitchFusion_Init(void);
void PitchFusion_Process(void);
const PitchFusionData_t *PitchFusion_GetData(void);

/*
 * 获取GM6020控制坐标系中的Pitch融合反馈。
 *
 * 返回true表示静止标定完成、IMU和Pitch电机数据均有效，输出：
 *   pitch_angle_deg — 相对Pitch机械零位的受限融合角度
 *   pitch_speed_rpm — 受限融合角速度，已经换算为电机rpm方向
 *
 * 返回false时，调用方必须使用GM6020编码器角度和反馈rpm。
 */
bool PitchFusion_GetControlFeedback(
    float *pitch_angle_deg,
    float *pitch_speed_rpm);

#ifdef __cplusplus
}
#endif

#endif /* PITCH_FUSION_H */
