#include "gimbal_calibration.h"

#include "motor_control.h"

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
  calibration.last_sequence[axis] =
      (feedback != NULL) ? feedback->rx_sequence : 0U;
  calibration.status[axis] = status;
}

static bool calibration_sample_axis(
    GM6020_Axis_t axis,
    const GM6020_Feedback_t *feedback)
{
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
  calibration.last_sequence[axis] = feedback->rx_sequence;

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
  ++calibration.sample_count[axis];
  return true;
}

static uint16_t calibration_average_axis(GM6020_Axis_t axis)
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

  average %= (int32_t)GM6020_ENCODER_CPR;
  if (average < 0)
  {
    average += (int32_t)GM6020_ENCODER_CPR;
  }
  return (uint16_t)average;
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

    zero_ecd = calibration_average_axis(axis);
    calibration.status[axis] =
        GM6020_SetAxisZeroOffsetEcd(axis, zero_ecd)
        ? GIMBAL_CALIBRATION_CALIBRATED
        : GIMBAL_CALIBRATION_ERROR;
  }
}

GimbalCalibrationStatus_t GimbalCalibration_GetStatus(void)
{
  uint32_t axis_index;
  bool any_waiting_feedback = false;
  bool any_waiting_still = false;
  bool any_sampling = false;

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
        || (status == GIMBAL_CALIBRATION_SAMPLING))
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
