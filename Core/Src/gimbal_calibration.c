#include "gimbal_calibration.h"

#include "config/gimbal_params.h"
#include "config/pitch_fusion_config.h"
#include "motor_control.h"
#include "pitch_fusion.h"

#include <math.h>
#include <string.h>

#define CALIBRATION_SAMPLE_COUNT        100U
#define CALIBRATION_STILL_SPEED_RPM     2
#define CALIBRATION_MAX_SPAN_ECD        64

typedef struct
{
  GimbalCalibrationStatus_t status[GM6020_AXIS_COUNT];
  uint32_t last_sequence[GM6020_AXIS_COUNT];
  uint16_t sample_count[GM6020_AXIS_COUNT];
  int32_t reference_ecd[GM6020_AXIS_COUNT];
  int32_t minimum_unwrapped_ecd[GM6020_AXIS_COUNT];
  int32_t maximum_unwrapped_ecd[GM6020_AXIS_COUNT];
  int64_t sum_unwrapped_ecd[GM6020_AXIS_COUNT];
  float pitch_sensor_angle_sum_deg;
  float pitch_return_target_deg;
  uint32_t pitch_return_motion_start_ms;
  uint32_t pitch_return_last_ms;
  uint32_t pitch_return_settle_start_ms;
  uint32_t last_pitch_imu_sample_count;
  bool pitch_return_target_valid;
  bool pitch_return_settling;
  bool reference_valid[GM6020_AXIS_COUNT];
} CalibrationContext_t;

static CalibrationContext_t calibration;

static bool calibration_axis_is_valid(GM6020_Axis_t axis)
{
  return ((uint32_t)axis < (uint32_t)GM6020_AXIS_COUNT);
}

static void calibration_reset_axis(
    GM6020_Axis_t axis,
    GimbalCalibrationStatus_t status)
{
  const GM6020_Feedback_t *feedback =
      GM6020_GetFeedback(axis);

  calibration.sample_count[axis] = 0U;
  calibration.reference_valid[axis] = false;
  calibration.sum_unwrapped_ecd[axis] = 0;
  if (axis == GM6020_AXIS_PITCH)
  {
    const PitchFusionData_t *fusion =
        PitchFusion_GetData();

    calibration.pitch_sensor_angle_sum_deg = 0.0f;
    calibration.pitch_return_target_deg = 0.0f;
    calibration.pitch_return_motion_start_ms =
        HAL_GetTick();
    calibration.pitch_return_last_ms = HAL_GetTick();
    calibration.pitch_return_settle_start_ms = 0U;
    calibration.pitch_return_target_valid = false;
    calibration.pitch_return_settling = false;
    calibration.last_pitch_imu_sample_count =
        (fusion != NULL) ? fusion->imu_sample_count : 0U;
  }
  calibration.last_sequence[axis] =
      (feedback != NULL) ? feedback->rx_sequence : 0U;
  calibration.status[axis] = status;
}

static bool calibration_sample_axis(
    GM6020_Axis_t axis,
    const GM6020_Feedback_t *feedback)
{
  const PitchFusionData_t *fusion = NULL;
  float pitch_sensor_angle_deg = 0.0f;
  int32_t unwrapped;
  int32_t delta;

  if ((feedback == NULL)
      || (calibration.sample_count[axis]
          >= CALIBRATION_SAMPLE_COUNT)
      || (feedback->rx_sequence
          == calibration.last_sequence[axis]))
  {
    return true;
  }

  if (axis == GM6020_AXIS_PITCH)
  {
    fusion = PitchFusion_GetData();
    if ((fusion == NULL)
        || !fusion->imu_valid
        || !fusion->accel_trusted
        || !isfinite(fusion->accel_pitch_raw_deg)
        || !isfinite(fusion->gyro_pitch_rate_dps))
    {
      return false;
    }
    if (fusion->imu_sample_count
        == calibration.last_pitch_imu_sample_count)
    {
      return true;
    }
    if ((fusion->gyro_pitch_rate_dps
         > PITCH_FUSION_STILL_GYRO_LIMIT_DPS)
        || (fusion->gyro_pitch_rate_dps
            < -PITCH_FUSION_STILL_GYRO_LIMIT_DPS))
    {
      return false;
    }

    pitch_sensor_angle_deg =
        fusion->accel_pitch_raw_deg
        - PITCH_SENSOR_LEVEL_OFFSET_DEG;
  }

  calibration.last_sequence[axis] = feedback->rx_sequence;
  if (axis == GM6020_AXIS_PITCH)
  {
    calibration.last_pitch_imu_sample_count =
        fusion->imu_sample_count;
  }

  if (!calibration.reference_valid[axis])
  {
    calibration.reference_ecd[axis] = feedback->angle;
    calibration.minimum_unwrapped_ecd[axis] = feedback->angle;
    calibration.maximum_unwrapped_ecd[axis] = feedback->angle;
    calibration.reference_valid[axis] = true;
  }

  delta = (int32_t)feedback->angle
      - calibration.reference_ecd[axis];
  if (delta > (int32_t)(GM6020_ENCODER_CPR / 2U))
  {
    delta -= (int32_t)GM6020_ENCODER_CPR;
  }
  else if (delta < -(int32_t)(GM6020_ENCODER_CPR / 2U))
  {
    delta += (int32_t)GM6020_ENCODER_CPR;
  }
  unwrapped = calibration.reference_ecd[axis] + delta;

  if (unwrapped < calibration.minimum_unwrapped_ecd[axis])
  {
    calibration.minimum_unwrapped_ecd[axis] = unwrapped;
  }
  if (unwrapped > calibration.maximum_unwrapped_ecd[axis])
  {
    calibration.maximum_unwrapped_ecd[axis] = unwrapped;
  }

  if ((calibration.maximum_unwrapped_ecd[axis]
       - calibration.minimum_unwrapped_ecd[axis])
      > CALIBRATION_MAX_SPAN_ECD)
  {
    return false;
  }

  calibration.sum_unwrapped_ecd[axis] += unwrapped;
  if (axis == GM6020_AXIS_PITCH)
  {
    calibration.pitch_sensor_angle_sum_deg +=
        pitch_sensor_angle_deg;
  }
  ++calibration.sample_count[axis];
  return true;
}

static int32_t calibration_average_axis_unwrapped(
    GM6020_Axis_t axis)
{
  int64_t sum = calibration.sum_unwrapped_ecd[axis];
  int32_t average;

  if (sum >= 0)
  {
    average = (int32_t)((sum + (CALIBRATION_SAMPLE_COUNT / 2U))
                        / CALIBRATION_SAMPLE_COUNT);
  }
  else
  {
    average = (int32_t)((sum - (CALIBRATION_SAMPLE_COUNT / 2U))
                        / CALIBRATION_SAMPLE_COUNT);
  }

  return average;
}

static uint16_t calibration_wrap_ecd(int32_t encoder)
{
  encoder %= (int32_t)GM6020_ENCODER_CPR;
  if (encoder < 0)
  {
    encoder += (int32_t)GM6020_ENCODER_CPR;
  }
  return (uint16_t)encoder;
}

static uint16_t calibration_calculate_zero_ecd(
    GM6020_Axis_t axis)
{
  if (axis == GM6020_AXIS_PITCH)
  {
    const int32_t average_encoder =
        calibration_average_axis_unwrapped(axis);
    const float average_sensor_angle_deg =
        calibration.pitch_sensor_angle_sum_deg
        / (float)CALIBRATION_SAMPLE_COUNT;
    const float sensor_angle_in_encoder_direction_deg =
        average_sensor_angle_deg
        * PITCH_FUSION_ENCODER_DIRECTION;
    const int32_t sensor_angle_ecd = (int32_t)lroundf(
        sensor_angle_in_encoder_direction_deg
        * (float)GM6020_ENCODER_CPR
        / 360.0f);

    /*
     * 当前编码器相对新零位的角度应等于传感器重力角：
     *   (encoder - zero) * encoder_direction = sensor_angle
     */
    return calibration_wrap_ecd(
        average_encoder - sensor_angle_ecd);
  }

  /*
   * Yaw的GM6020编码器是单圈绝对传感器。静止采样只用于确认反馈稳定，
   * 逻辑零点始终取装机配置的传感器零位，不能被本次开机姿态覆盖。
   */
  return (uint16_t)YAW_SENSOR_ZERO_ECD;
}

static float calibration_abs_float(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static bool calibration_start_pitch_return(void)
{
#if PITCH_AUTO_ZERO_RETURN_ENABLE
  float current_position_deg;

  if (!GM6020_GetMultiTurnPosition(
          GM6020_AXIS_PITCH,
          &current_position_deg)
      || !isfinite(current_position_deg)
      || (current_position_deg
          < PITCH_AUTO_ZERO_MIN_START_ANGLE_DEG)
      || (current_position_deg
          > PITCH_AUTO_ZERO_MAX_START_ANGLE_DEG))
  {
    return false;
  }

  calibration.pitch_return_target_deg =
      current_position_deg;
  calibration.pitch_return_motion_start_ms =
      HAL_GetTick();
  calibration.pitch_return_last_ms = HAL_GetTick();
  calibration.pitch_return_settle_start_ms = 0U;
  calibration.pitch_return_target_valid = true;
  calibration.pitch_return_settling = false;
  GM6020_SetTargetPosition(
      GM6020_AXIS_PITCH,
      current_position_deg);
  calibration.status[GM6020_AXIS_PITCH] =
      GIMBAL_CALIBRATION_RETURNING_ZERO;
#else
  calibration.status[GM6020_AXIS_PITCH] =
      GIMBAL_CALIBRATION_CALIBRATED;
#endif
  return true;
}

static void calibration_process_pitch_return(
    const GM6020_Feedback_t *feedback)
{
#if PITCH_AUTO_ZERO_RETURN_ENABLE
  const uint32_t now = HAL_GetTick();
  float current_position_deg;
  uint32_t delta_ms;
  float maximum_step_deg;
  float minimum_target_deg;
  float maximum_target_deg;

  if ((feedback == NULL)
      || !feedback->online
      || GM6020_IsEmergencyStopped()
      || !GM6020_GetMultiTurnPosition(
          GM6020_AXIS_PITCH,
          &current_position_deg)
      || !isfinite(current_position_deg))
  {
    calibration.pitch_return_target_valid = false;
    calibration.pitch_return_settling = false;
    calibration.pitch_return_settle_start_ms = 0U;
    calibration.pitch_return_motion_start_ms = now;
    calibration.pitch_return_last_ms = now;
    return;
  }

  /*
   * 运行软限位允许从实测机械端点外侧向内恢复，但自动回零本身仍受
   * 更外层的启动包络和总时间保护。姿态超出包络或长期无法回零时，
   * 锁存全局急停，避免持续顶住机构。
   */
  if ((current_position_deg
       < PITCH_AUTO_ZERO_MIN_START_ANGLE_DEG)
      || (current_position_deg
          > PITCH_AUTO_ZERO_MAX_START_ANGLE_DEG)
      || ((uint32_t)(
              now - calibration.pitch_return_motion_start_ms)
          > PITCH_AUTO_ZERO_RETURN_TIMEOUT_MS))
  {
    calibration.pitch_return_target_valid = false;
    calibration.pitch_return_settling = false;
    calibration.pitch_return_settle_start_ms = 0U;
    calibration.status[GM6020_AXIS_PITCH] =
        GIMBAL_CALIBRATION_ERROR;
    (void)GM6020_EmergencyStop();
    return;
  }

  /*
   * 急停解除或反馈恢复后先锁定当前位置，再从这里重新开始斜坡，
   * 防止继续追赶暂停前的旧目标。
   */
  if (!calibration.pitch_return_target_valid)
  {
    calibration.pitch_return_target_deg =
        current_position_deg;
    calibration.pitch_return_target_valid = true;
    calibration.pitch_return_motion_start_ms = now;
    calibration.pitch_return_last_ms = now;
    GM6020_SetTargetPosition(
        GM6020_AXIS_PITCH,
        current_position_deg);
    return;
  }

  /*
   * 给操作者保留上电固定和放手时间。等待阶段持续从实际位置重新
   * 锚定目标，不让位置误差在手动固定期间累积。
   */
  if ((uint32_t)(
          now - calibration.pitch_return_motion_start_ms)
      < PITCH_AUTO_ZERO_RELEASE_DELAY_MS)
  {
    calibration.pitch_return_target_deg =
        current_position_deg;
    calibration.pitch_return_last_ms = now;
    GM6020_SetTargetPosition(
        GM6020_AXIS_PITCH,
        current_position_deg);
    return;
  }

  if ((calibration_abs_float(current_position_deg)
       <= PITCH_AUTO_ZERO_TOLERANCE_DEG)
      && (feedback->speed_rpm
          <= PITCH_AUTO_ZERO_SETTLE_SPEED_RPM)
      && (feedback->speed_rpm
          >= -PITCH_AUTO_ZERO_SETTLE_SPEED_RPM))
  {
    GM6020_SetTargetPosition(GM6020_AXIS_PITCH, 0.0f);
    if (!calibration.pitch_return_settling)
    {
      calibration.pitch_return_settling = true;
      calibration.pitch_return_settle_start_ms = now;
    }
    else if ((uint32_t)(
                 now
                 - calibration.pitch_return_settle_start_ms)
             >= PITCH_AUTO_ZERO_SETTLE_TIME_MS)
    {
      calibration.status[GM6020_AXIS_PITCH] =
          GIMBAL_CALIBRATION_CALIBRATED;
    }
    return;
  }

  calibration.pitch_return_settling = false;
  calibration.pitch_return_settle_start_ms = 0U;

  delta_ms = (uint32_t)(
      now - calibration.pitch_return_last_ms);
  calibration.pitch_return_last_ms = now;
  if (delta_ms > PITCH_AUTO_ZERO_MAX_DELTA_MS)
  {
    delta_ms = PITCH_AUTO_ZERO_MAX_DELTA_MS;
  }
  if (delta_ms == 0U)
  {
    return;
  }

  maximum_step_deg =
      PITCH_AUTO_ZERO_RETURN_RATE_DPS
      * (float)delta_ms
      / 1000.0f;
  if (calibration.pitch_return_target_deg
      > maximum_step_deg)
  {
    calibration.pitch_return_target_deg -=
        maximum_step_deg;
  }
  else if (calibration.pitch_return_target_deg
           < -maximum_step_deg)
  {
    calibration.pitch_return_target_deg +=
        maximum_step_deg;
  }
  else
  {
    calibration.pitch_return_target_deg = 0.0f;
  }

  /*
   * 目标只允许领先实际位置少量角度。若机构受阻，斜坡目标会跟随
   * 实际位置暂停，不会在后台持续累积很大的角度误差。
   */
  minimum_target_deg =
      current_position_deg
      - PITCH_AUTO_ZERO_MAX_TARGET_LEAD_DEG;
  maximum_target_deg =
      current_position_deg
      + PITCH_AUTO_ZERO_MAX_TARGET_LEAD_DEG;
  if (calibration.pitch_return_target_deg
      < minimum_target_deg)
  {
    calibration.pitch_return_target_deg =
        minimum_target_deg;
  }
  else if (calibration.pitch_return_target_deg
           > maximum_target_deg)
  {
    calibration.pitch_return_target_deg =
        maximum_target_deg;
  }

  GM6020_SetTargetPosition(
      GM6020_AXIS_PITCH,
      calibration.pitch_return_target_deg);
#else
  (void)feedback;
  calibration.status[GM6020_AXIS_PITCH] =
      GIMBAL_CALIBRATION_CALIBRATED;
#endif
}

void GimbalCalibration_Init(void)
{
  uint32_t axis_index;

  memset(&calibration, 0, sizeof(calibration));
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    calibration_reset_axis(
        (GM6020_Axis_t)axis_index,
        GIMBAL_CALIBRATION_WAITING_FEEDBACK);
  }
}

void GimbalCalibration_Process(void)
{
  uint32_t axis_index;

  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    const GM6020_Axis_t axis =
        (GM6020_Axis_t)axis_index;
    const GM6020_Feedback_t *feedback;
    uint16_t zero_ecd;

    if ((calibration.status[axis]
         == GIMBAL_CALIBRATION_CALIBRATED)
        || (calibration.status[axis]
            == GIMBAL_CALIBRATION_ERROR))
    {
      continue;
    }

    feedback = GM6020_GetFeedback(axis);
    if (calibration.status[axis]
        == GIMBAL_CALIBRATION_RETURNING_ZERO)
    {
      calibration_process_pitch_return(feedback);
      continue;
    }

    if ((feedback == NULL) || !feedback->online)
    {
      calibration_reset_axis(
          axis,
          GIMBAL_CALIBRATION_WAITING_FEEDBACK);
      continue;
    }

    if ((feedback->speed_rpm
         > CALIBRATION_STILL_SPEED_RPM)
        || (feedback->speed_rpm
            < -CALIBRATION_STILL_SPEED_RPM))
    {
      calibration_reset_axis(
          axis,
          GIMBAL_CALIBRATION_WAITING_STILL);
      continue;
    }

    calibration.status[axis] =
        GIMBAL_CALIBRATION_SAMPLING;
    if (!calibration_sample_axis(axis, feedback))
    {
      calibration_reset_axis(
          axis,
          GIMBAL_CALIBRATION_WAITING_STILL);
      continue;
    }

    if (calibration.sample_count[axis]
        < CALIBRATION_SAMPLE_COUNT)
    {
      continue;
    }

    zero_ecd = calibration_calculate_zero_ecd(axis);
    if (!GM6020_SetAxisZeroOffsetEcd(axis, zero_ecd))
    {
      calibration.status[axis] =
          GIMBAL_CALIBRATION_ERROR;
    }
    else if (axis == GM6020_AXIS_PITCH)
    {
      if (!calibration_start_pitch_return())
      {
        calibration.status[axis] =
            GIMBAL_CALIBRATION_ERROR;
      }
    }
    else
    {
      calibration.status[axis] =
          GIMBAL_CALIBRATION_CALIBRATED;
    }
  }
}

GimbalCalibrationStatus_t GimbalCalibration_GetStatus(void)
{
  uint32_t axis_index;
  bool any_waiting_feedback = false;
  bool any_waiting_still = false;
  bool any_sampling = false;
  bool any_returning_zero = false;

  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    switch (calibration.status[axis_index])
    {
      case GIMBAL_CALIBRATION_ERROR:
        return GIMBAL_CALIBRATION_ERROR;
      case GIMBAL_CALIBRATION_SAMPLING:
        any_sampling = true;
        break;
      case GIMBAL_CALIBRATION_RETURNING_ZERO:
        any_returning_zero = true;
        break;
      case GIMBAL_CALIBRATION_WAITING_STILL:
        any_waiting_still = true;
        break;
      case GIMBAL_CALIBRATION_WAITING_FEEDBACK:
        any_waiting_feedback = true;
        break;
      case GIMBAL_CALIBRATION_CALIBRATED:
      default:
        break;
    }
  }

  if (any_returning_zero)
  {
    return GIMBAL_CALIBRATION_RETURNING_ZERO;
  }
  if (any_sampling)
  {
    return GIMBAL_CALIBRATION_SAMPLING;
  }
  if (any_waiting_still)
  {
    return GIMBAL_CALIBRATION_WAITING_STILL;
  }
  if (any_waiting_feedback)
  {
    return GIMBAL_CALIBRATION_WAITING_FEEDBACK;
  }
  return GIMBAL_CALIBRATION_CALIBRATED;
}

GimbalCalibrationStatus_t GimbalCalibration_GetAxisStatus(
    GM6020_Axis_t axis)
{
  if (!calibration_axis_is_valid(axis))
  {
    return GIMBAL_CALIBRATION_ERROR;
  }
  return calibration.status[axis];
}

bool GimbalCalibration_IsAxisCalibrated(
    GM6020_Axis_t axis)
{
  return GimbalCalibration_GetAxisStatus(axis)
      == GIMBAL_CALIBRATION_CALIBRATED;
}

bool GimbalCalibration_IsBusy(void)
{
  uint32_t axis_index;

  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    const GimbalCalibrationStatus_t status =
        calibration.status[axis_index];

    if ((status == GIMBAL_CALIBRATION_WAITING_FEEDBACK)
        || (status == GIMBAL_CALIBRATION_WAITING_STILL)
        || (status == GIMBAL_CALIBRATION_SAMPLING)
        || (status
            == GIMBAL_CALIBRATION_RETURNING_ZERO))
    {
      return true;
    }
  }
  return false;
}

uint16_t GimbalCalibration_GetYawSampleCount(void)
{
  return calibration.sample_count[GM6020_AXIS_YAW];
}

uint16_t GimbalCalibration_GetPitchSampleCount(void)
{
  return calibration.sample_count[GM6020_AXIS_PITCH];
}
