/**
 * ===========================================================================
 * @file    pitch_fusion.c
 * @brief   Pitch 轴 BMI088 / GM6020 融合器
 * ===========================================================================
 *
 * 融合流程：
 *   1. 等待 Pitch 电机完成上电零位标定，且 BMI088 / 电机反馈均有效；
 *   2. 静止采集约 500 ms，估计陀螺零偏和 IMU重力角到编码器零位的偏差；
 *   3. 用陀螺角速度预测，用可信的加速度重力角进行低频修正；
 *   4. 同时发布编码器角度/速度以及
 *      base_disturbance_rate = gyro_rate - encoder_rate。
 *
 * 注意：
 *   - 编码器测量云台相对安装架的机械角，IMU测量惯性运动，两者参考系不同；
 *   - 因此编码器只用于初始对齐、机械状态和扰动分离，不在运行中强行把
 *     惯性角拉回编码器角；
 *   - 对电机闭环发布的角度和速度带有独立安全限制，融合失效时调用方
 *     必须立即回退到编码器和电机RPM。
 * ===========================================================================
 */

#include "pitch_fusion.h"

#include "bmi088.h"
#include "config/pitch_fusion_config.h"
#include "gimbal_calibration.h"
#include "motor_control.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PITCH_FUSION_RAD_TO_DEG 57.29577951308232f
#define PITCH_FUSION_RPM_TO_DPS  6.0f

typedef struct
{
  PitchFusionData_t output;
  float calibration_gyro_sum_dps;
  float calibration_accel_minus_encoder_sum_deg;
  float accel_minus_encoder_offset_deg;
  uint32_t last_imu_sample_count;
  uint32_t last_imu_timestamp_ms;
  bool calibrated;
} PitchFusionContext_t;

static PitchFusionContext_t pitch_fusion;

static float pitch_fusion_abs(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float pitch_fusion_wrap_degrees(float angle_deg)
{
  while (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }
  while (angle_deg < -180.0f)
  {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static bool pitch_fusion_accel_is_trusted(float accel_norm_g)
{
  return (accel_norm_g >= PITCH_FUSION_ACCEL_NORM_MIN_G)
      && (accel_norm_g <= PITCH_FUSION_ACCEL_NORM_MAX_G);
}

static void pitch_fusion_reset_calibration(void)
{
  pitch_fusion.calibration_gyro_sum_dps = 0.0f;
  pitch_fusion.calibration_accel_minus_encoder_sum_deg = 0.0f;
  pitch_fusion.output.calibration_sample_count = 0U;
}

static bool pitch_fusion_read_motor(
    float *encoder_pitch_deg,
    float *encoder_rate_dps)
{
  const GM6020_Feedback_t *feedback =
      GM6020_GetFeedback(GM6020_AXIS_PITCH);
  float position_deg;

  if ((feedback == NULL)
      || !feedback->online
      || !GimbalCalibration_IsAxisCalibrated(
          GM6020_AXIS_PITCH)
      || !GM6020_GetMultiTurnPosition(
          GM6020_AXIS_PITCH,
          &position_deg))
  {
    return false;
  }

  *encoder_pitch_deg =
      position_deg * PITCH_FUSION_ENCODER_DIRECTION;
  *encoder_rate_dps =
      (float)feedback->speed_rpm
      * PITCH_FUSION_RPM_TO_DPS
      * PITCH_FUSION_ENCODER_DIRECTION;
  return true;
}

static bool pitch_fusion_decode_imu(
    const BMI088_Sample_t *sample,
    float *accel_pitch_raw_deg,
    float *gyro_pitch_rate_dps,
    float *accel_norm_g)
{
  float accel_g[3];
  float forward;
  float lateral;
  float up;
  float vertical_norm;
  uint32_t axis_index;
  const uint32_t lateral_axis_index =
      3U
      - PITCH_FUSION_ACCEL_FORWARD_AXIS_INDEX
      - PITCH_FUSION_ACCEL_UP_AXIS_INDEX;

  if ((sample == NULL) || !sample->valid)
  {
    return false;
  }

  for (axis_index = 0U; axis_index < 3U; ++axis_index)
  {
    accel_g[axis_index] =
        (float)sample->accel_raw[axis_index]
        / PITCH_FUSION_ACCEL_LSB_PER_G;
  }

  forward =
      accel_g[PITCH_FUSION_ACCEL_FORWARD_AXIS_INDEX]
      * PITCH_FUSION_ACCEL_FORWARD_DIRECTION;
  up =
      accel_g[PITCH_FUSION_ACCEL_UP_AXIS_INDEX]
      * PITCH_FUSION_ACCEL_UP_DIRECTION;
  lateral = accel_g[lateral_axis_index];
  vertical_norm = sqrtf(up * up + lateral * lateral);

  /*
   * 使用另外两个重力分量的合量作分母，避免云台存在Roll倾角时把
   * Roll直接耦合进Pitch重力角。Pitch机械范围远小于±90°。
   */
  *accel_pitch_raw_deg =
      atan2f(forward, vertical_norm)
      * PITCH_FUSION_RAD_TO_DEG;
  *gyro_pitch_rate_dps =
      (float)sample->gyro_raw[PITCH_FUSION_GYRO_AXIS_INDEX]
      / PITCH_FUSION_GYRO_LSB_PER_DPS
      * PITCH_FUSION_GYRO_DIRECTION;
  *accel_norm_g = sqrtf(
      accel_g[0] * accel_g[0]
      + accel_g[1] * accel_g[1]
      + accel_g[2] * accel_g[2]);

  return isfinite(*accel_pitch_raw_deg)
      && isfinite(*gyro_pitch_rate_dps)
      && isfinite(*accel_norm_g);
}

void PitchFusion_Init(void)
{
  memset(&pitch_fusion, 0, sizeof(pitch_fusion));
  pitch_fusion.output.status = PITCH_FUSION_WAITING_DATA;
}

void PitchFusion_Process(void)
{
  const BMI088_Diagnostic_t *diagnostic;
  BMI088_Sample_t sample;
  const uint32_t now = HAL_GetTick();
  float accel_pitch_raw_deg = 0.0f;
  float gyro_pitch_rate_dps = 0.0f;
  float accel_norm_g = 0.0f;
  float encoder_pitch_deg = 0.0f;
  float encoder_rate_dps = 0.0f;
  bool imu_valid;
  bool motor_valid;
  bool new_imu_sample;
  bool accel_norm_trusted;

  diagnostic = BMI088_GetDiagnostic();
  imu_valid =
      (diagnostic != NULL)
      && (diagnostic->status == BMI088_STATUS_OK)
      && BMI088_GetLatestSample(&sample)
      && ((uint32_t)(now - sample.timestamp_ms)
          <= PITCH_FUSION_IMU_TIMEOUT_MS)
      && pitch_fusion_decode_imu(
          &sample,
          &accel_pitch_raw_deg,
          &gyro_pitch_rate_dps,
          &accel_norm_g);

  motor_valid = pitch_fusion_read_motor(
      &encoder_pitch_deg,
      &encoder_rate_dps);

  pitch_fusion.output.imu_valid = imu_valid;
  pitch_fusion.output.motor_valid = motor_valid;

  if (motor_valid)
  {
    pitch_fusion.output.encoder_pitch_deg =
        encoder_pitch_deg;
    pitch_fusion.output.encoder_pitch_rate_dps =
        encoder_rate_dps;
  }

  if (!imu_valid)
  {
    pitch_fusion.output.accel_trusted = false;
    pitch_fusion.output.accel_innovation_deg = 0.0f;
    if (pitch_fusion.calibrated)
    {
      pitch_fusion.output.recovery_pending = true;
      pitch_fusion.output.recovery_sample_count = 0U;
    }
    pitch_fusion.output.status = pitch_fusion.calibrated
        ? PITCH_FUSION_DEGRADED
        : PITCH_FUSION_WAITING_DATA;
    if (motor_valid)
    {
      pitch_fusion.output.fused_pitch_deg =
          encoder_pitch_deg;
      pitch_fusion.output.fused_pitch_rate_dps =
          encoder_rate_dps;
      pitch_fusion.output.base_disturbance_rate_dps =
          0.0f;
    }
    return;
  }

  pitch_fusion.output.imu_sample_count =
      sample.sample_count;
  pitch_fusion.output.accel_pitch_raw_deg =
      accel_pitch_raw_deg;
  pitch_fusion.output.accel_pitch_aligned_deg =
      pitch_fusion.calibrated
      ? accel_pitch_raw_deg
        - pitch_fusion.accel_minus_encoder_offset_deg
      : accel_pitch_raw_deg;
  pitch_fusion.output.gyro_pitch_rate_dps =
      pitch_fusion.calibrated
      ? gyro_pitch_rate_dps
        - pitch_fusion.output.gyro_bias_dps
      : gyro_pitch_rate_dps;
  pitch_fusion.output.accel_norm_g = accel_norm_g;
  accel_norm_trusted =
      pitch_fusion_accel_is_trusted(accel_norm_g);
  pitch_fusion.output.accel_trusted = accel_norm_trusted;

  new_imu_sample =
      sample.sample_count != pitch_fusion.last_imu_sample_count;
  if (!new_imu_sample)
  {
    if (!motor_valid || pitch_fusion.output.recovery_pending)
    {
      pitch_fusion.output.status = pitch_fusion.calibrated
          ? PITCH_FUSION_DEGRADED
          : PITCH_FUSION_WAITING_DATA;
    }
    return;
  }

  pitch_fusion.last_imu_sample_count = sample.sample_count;

  if (!pitch_fusion.calibrated)
  {
    const bool stationary =
        motor_valid
        && pitch_fusion.output.accel_trusted
        && (pitch_fusion_abs(gyro_pitch_rate_dps)
            <= PITCH_FUSION_STILL_GYRO_LIMIT_DPS)
        && (pitch_fusion_abs(encoder_rate_dps)
            <= PITCH_FUSION_STILL_ENCODER_RATE_LIMIT_DPS);

    if (!motor_valid)
    {
      pitch_fusion_reset_calibration();
      pitch_fusion.output.status = PITCH_FUSION_WAITING_DATA;
      return;
    }
    if (!stationary)
    {
      pitch_fusion_reset_calibration();
      pitch_fusion.output.status = PITCH_FUSION_WAITING_STILL;
      return;
    }

    pitch_fusion.output.status = PITCH_FUSION_CALIBRATING;
    pitch_fusion.calibration_gyro_sum_dps +=
        gyro_pitch_rate_dps;
    pitch_fusion.calibration_accel_minus_encoder_sum_deg +=
        accel_pitch_raw_deg - encoder_pitch_deg;
    ++pitch_fusion.output.calibration_sample_count;

    if (pitch_fusion.output.calibration_sample_count
        >= PITCH_FUSION_CALIBRATION_SAMPLE_COUNT)
    {
      const float sample_count =
          (float)pitch_fusion.output.calibration_sample_count;

      pitch_fusion.output.gyro_bias_dps =
          pitch_fusion.calibration_gyro_sum_dps
          / sample_count;
      pitch_fusion.accel_minus_encoder_offset_deg =
          pitch_fusion.calibration_accel_minus_encoder_sum_deg
          / sample_count;
      pitch_fusion.output.accel_pitch_aligned_deg =
          accel_pitch_raw_deg
          - pitch_fusion.accel_minus_encoder_offset_deg;
      pitch_fusion.output.gyro_pitch_rate_dps = 0.0f;
      pitch_fusion.output.fused_pitch_deg =
          encoder_pitch_deg;
      pitch_fusion.output.fused_pitch_rate_dps = 0.0f;
      pitch_fusion.output.base_disturbance_rate_dps =
          -encoder_rate_dps;
      pitch_fusion.output.accel_innovation_deg = 0.0f;
      pitch_fusion.output.recovery_sample_count = 0U;
      pitch_fusion.output.recovery_pending = false;
      pitch_fusion.last_imu_timestamp_ms =
          sample.timestamp_ms;
      pitch_fusion.calibrated = true;
      pitch_fusion.output.status = PITCH_FUSION_RUNNING;
    }
    return;
  }

  if (!motor_valid)
  {
    pitch_fusion.output.recovery_pending = true;
    pitch_fusion.output.recovery_sample_count = 0U;
  }

  {
    uint32_t delta_ms =
        (uint32_t)(sample.timestamp_ms
                   - pitch_fusion.last_imu_timestamp_ms);
    float delta_time_s;
    float corrected_gyro_rate_dps;
    float predicted_pitch_deg;
    float accel_innovation_deg;

    corrected_gyro_rate_dps =
        gyro_pitch_rate_dps
        - pitch_fusion.output.gyro_bias_dps;
    pitch_fusion.output.accel_pitch_aligned_deg =
        accel_pitch_raw_deg
        - pitch_fusion.accel_minus_encoder_offset_deg;

    /*
     * IMU或电机数据恢复后的第一帧先重新对齐到机械角，不使用跨越掉线
     * 时段的旧时间戳积分。随后要求连续健康样本通过创新门限，才恢复
     * RUNNING状态。
     */
    if (pitch_fusion.output.recovery_pending
        && (pitch_fusion.output.recovery_sample_count == 0U)
        && motor_valid)
    {
      accel_innovation_deg = pitch_fusion_wrap_degrees(
          pitch_fusion.output.accel_pitch_aligned_deg
          - encoder_pitch_deg);
      pitch_fusion.output.accel_innovation_deg =
          accel_innovation_deg;
      pitch_fusion.output.accel_trusted =
          accel_norm_trusted
          && (pitch_fusion_abs(accel_innovation_deg)
              <= PITCH_FUSION_ACCEL_INNOVATION_MAX_DEG);
      pitch_fusion.output.fused_pitch_deg =
          encoder_pitch_deg;
      pitch_fusion.output.gyro_pitch_rate_dps =
          corrected_gyro_rate_dps;
      pitch_fusion.output.fused_pitch_rate_dps =
          corrected_gyro_rate_dps;
      pitch_fusion.output.base_disturbance_rate_dps =
          corrected_gyro_rate_dps - encoder_rate_dps;
      pitch_fusion.last_imu_timestamp_ms =
          sample.timestamp_ms;
      if (pitch_fusion.output.accel_trusted)
      {
        pitch_fusion.output.recovery_sample_count = 1U;
      }
      ++pitch_fusion.output.fusion_update_count;
      pitch_fusion.output.status = PITCH_FUSION_DEGRADED;
      return;
    }

    pitch_fusion.last_imu_timestamp_ms =
        sample.timestamp_ms;
    if (delta_ms == 0U)
    {
      return;
    }

    delta_time_s = (float)delta_ms * 0.001f;
    if (delta_time_s > PITCH_FUSION_MAX_DELTA_TIME_S)
    {
      delta_time_s = PITCH_FUSION_MAX_DELTA_TIME_S;
    }

    predicted_pitch_deg =
        pitch_fusion.output.fused_pitch_deg
        + corrected_gyro_rate_dps * delta_time_s;
    accel_innovation_deg = pitch_fusion_wrap_degrees(
        pitch_fusion.output.accel_pitch_aligned_deg
        - predicted_pitch_deg);
    pitch_fusion.output.accel_innovation_deg =
        accel_innovation_deg;
    pitch_fusion.output.accel_trusted =
        accel_norm_trusted
        && (pitch_fusion_abs(accel_innovation_deg)
            <= PITCH_FUSION_ACCEL_INNOVATION_MAX_DEG);

    if (pitch_fusion.output.accel_trusted)
    {
      const float gyro_weight =
          PITCH_FUSION_ACCEL_CORRECTION_TAU_S
          / (PITCH_FUSION_ACCEL_CORRECTION_TAU_S
             + delta_time_s);
      pitch_fusion.output.fused_pitch_deg =
          predicted_pitch_deg
          + (1.0f - gyro_weight)
          * accel_innovation_deg;
    }
    else
    {
      pitch_fusion.output.fused_pitch_deg =
          predicted_pitch_deg;
    }

    pitch_fusion.output.gyro_pitch_rate_dps =
        corrected_gyro_rate_dps;
    pitch_fusion.output.fused_pitch_rate_dps =
        corrected_gyro_rate_dps;
    pitch_fusion.output.base_disturbance_rate_dps =
        motor_valid
        ? corrected_gyro_rate_dps - encoder_rate_dps
        : 0.0f;

    if (pitch_fusion.output.recovery_pending)
    {
      if (motor_valid && pitch_fusion.output.accel_trusted)
      {
        if (pitch_fusion.output.recovery_sample_count
            < PITCH_FUSION_RECOVERY_HEALTHY_SAMPLE_COUNT)
        {
          ++pitch_fusion.output.recovery_sample_count;
        }
        if (pitch_fusion.output.recovery_sample_count
            >= PITCH_FUSION_RECOVERY_HEALTHY_SAMPLE_COUNT)
        {
          pitch_fusion.output.recovery_pending = false;
        }
      }
      else
      {
        pitch_fusion.output.recovery_sample_count = 0U;
      }
    }

    ++pitch_fusion.output.fusion_update_count;
    pitch_fusion.output.status =
        motor_valid && !pitch_fusion.output.recovery_pending
        ? PITCH_FUSION_RUNNING
        : PITCH_FUSION_DEGRADED;
  }
}

const PitchFusionData_t *PitchFusion_GetData(void)
{
  return &pitch_fusion.output;
}

bool PitchFusion_GetControlFeedback(
    float *pitch_angle_deg,
    float *pitch_speed_rpm)
{
#if PITCH_FUSION_CONTROL_ENABLE
  float angle_delta_deg;
  float rate_delta_dps;
  float control_angle_deg;
  float control_rate_dps;

  if ((pitch_angle_deg == NULL)
      || (pitch_speed_rpm == NULL)
      || !pitch_fusion.calibrated
      || (pitch_fusion.output.status
          != PITCH_FUSION_RUNNING)
      || !pitch_fusion.output.imu_valid
      || !pitch_fusion.output.motor_valid)
  {
    return false;
  }

  /*
   * 对控制用融合量增加编码器安全带：
   *   - 惯性角可以修正底盘扰动，但不能瞬间远离机械角；
   *   - 陀螺角速度可以提高响应，但不能瞬间远离电机反馈速度。
   */
  angle_delta_deg =
      pitch_fusion.output.fused_pitch_deg
      - pitch_fusion.output.encoder_pitch_deg;
  if (angle_delta_deg
      > PITCH_FUSION_CONTROL_MAX_ANGLE_DELTA_DEG)
  {
    angle_delta_deg =
        PITCH_FUSION_CONTROL_MAX_ANGLE_DELTA_DEG;
  }
  else if (angle_delta_deg
           < -PITCH_FUSION_CONTROL_MAX_ANGLE_DELTA_DEG)
  {
    angle_delta_deg =
        -PITCH_FUSION_CONTROL_MAX_ANGLE_DELTA_DEG;
  }

  rate_delta_dps =
      pitch_fusion.output.fused_pitch_rate_dps
      - pitch_fusion.output.encoder_pitch_rate_dps;
  if (rate_delta_dps
      > PITCH_FUSION_CONTROL_MAX_RATE_DELTA_DPS)
  {
    rate_delta_dps =
        PITCH_FUSION_CONTROL_MAX_RATE_DELTA_DPS;
  }
  else if (rate_delta_dps
           < -PITCH_FUSION_CONTROL_MAX_RATE_DELTA_DPS)
  {
    rate_delta_dps =
        -PITCH_FUSION_CONTROL_MAX_RATE_DELTA_DPS;
  }

  control_angle_deg =
      pitch_fusion.output.encoder_pitch_deg
      + angle_delta_deg;
  control_rate_dps =
      pitch_fusion.output.encoder_pitch_rate_dps
      + rate_delta_dps;

  /*
   * 融合器内部使用配置后的IMU/编码器正方向；GM6020控制器使用原始
   * 编码器方向。direction为±1，因此乘一次即可变回控制器坐标系。
   */
  *pitch_angle_deg =
      control_angle_deg * PITCH_FUSION_ENCODER_DIRECTION;
  *pitch_speed_rpm =
      control_rate_dps
      * PITCH_FUSION_ENCODER_DIRECTION
      / PITCH_FUSION_RPM_TO_DPS;

  return isfinite(*pitch_angle_deg)
      && isfinite(*pitch_speed_rpm);
#else
  (void)pitch_angle_deg;
  (void)pitch_speed_rpm;
  return false;
#endif
}
