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

/*
 * 融合器状态枚举 (Fusion Status Enum)。
 */
typedef enum
{
  PITCH_FUSION_WAITING_DATA = 0,  /* 等待有效数据 — waiting for valid IMU/motor data */
  PITCH_FUSION_WAITING_STILL,     /* 等待静止 — waiting for stationary calibration */
  PITCH_FUSION_CALIBRATING,       /* 静止标定中 — collecting calibration samples */
  PITCH_FUSION_RUNNING,           /* 正常运行 — running (Kalman fusion active) */
  PITCH_FUSION_DEGRADED           /* 降级运行 — degraded (recovery from dropout) */
} PitchFusionStatus_t;

/*
 * 融合输出数据结构 (Fusion Output Data)。
 * 通过 PitchFusion_GetData() 获取，供调试/控制使用。
 */
typedef struct
{
  PitchFusionStatus_t status;  /* 当前融合状态 — current fusion status */

  /* ---- IMU 计算值 (IMU-derived) ---- */
  float accel_pitch_raw_deg;       /* 加速度计重力角 (未减安装偏差) — raw accel pitch (gravity angle) */
  float accel_pitch_aligned_deg;   /* 减去安装偏差后的加速度重力角 — accel pitch aligned to encoder zero */
  float gyro_pitch_rate_dps;       /* 陀螺 Pitch 角速度 (去零偏) — gyro pitch rate (bias-removed) */
  float gyro_bias_dps;             /* 陀螺零偏估计 — estimated gyro bias */
  float accel_norm_g;              /* 加速度模值 (重力检测用) — accel vector norm in g */

  /* ---- GM6020 机械相对量 (Encoder/Motor) ---- */
  float encoder_pitch_deg;         /* 编码器机械 Pitch 角 — encoder mechanical pitch angle */
  float encoder_pitch_rate_dps;    /* 编码器 Pitch 角速度 — encoder pitch rate (dps) */

  /* ---- 惯性融合估计 (Inertial Fusion) ---- */
  float fused_pitch_deg;           /* 融合 Pitch 角 — fused inertial pitch angle */
  float fused_pitch_rate_dps;      /* 融合 Pitch 角速度 — fused pitch rate */
  float base_disturbance_rate_dps; /* 底盘/安装架扰动角速度 = gyro - encoder — base disturbance rate */
  float accel_innovation_deg;      /* 加速度观测新息 — accel measurement innovation (deg) */

  uint32_t imu_sample_count;       /* 最新 IMU 采样序号 — latest IMU sample count */
  uint32_t fusion_update_count;    /* 融合更新次数 — fusion update counter */
  uint16_t calibration_sample_count; /* 标定已采集样本数 — calibration sample count */
  uint16_t recovery_sample_count;  /* 恢复期健康样本数 — recovery healthy sample count */

  bool imu_valid;                  /* IMU 数据有效 — IMU data valid */
  bool motor_valid;                /* 电机数据有效 — motor data valid */
  bool accel_trusted;              /* 加速度观测可信 (模值+新息门限) — accel measurement trusted */
  bool recovery_pending;           /* 恢复挂起 — recovery in progress */
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
