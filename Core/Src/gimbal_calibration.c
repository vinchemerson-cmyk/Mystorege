#include "gimbal_calibration.h"

#include "config/gimbal_params.h"
#include "motor_control.h"

#include <string.h>

#define CALIBRATION_SAMPLE_COUNT        100U
#define CALIBRATION_STILL_SPEED_RPM     2
#define CALIBRATION_MAX_SPAN_ECD        64

typedef struct
{
  GimbalCalibrationStatus_t status;
  uint32_t last_sequence[GM6020_AXIS_COUNT];
  uint16_t sample_count[GM6020_AXIS_COUNT];
  int32_t reference_ecd[GM6020_AXIS_COUNT];
  int32_t minimum_unwrapped_ecd[GM6020_AXIS_COUNT];
  int32_t maximum_unwrapped_ecd[GM6020_AXIS_COUNT];
  int64_t sum_unwrapped_ecd[GM6020_AXIS_COUNT];
  bool reference_valid[GM6020_AXIS_COUNT];
} CalibrationContext_t;

static CalibrationContext_t calibration;

static void calibration_reset_samples(void)
{
  const GM6020_Feedback_t *yaw =
      GM6020_GetFeedback(GM6020_AXIS_YAW);
  const GM6020_Feedback_t *pitch =
      GM6020_GetFeedback(GM6020_AXIS_PITCH);

  calibration.sample_count[GM6020_AXIS_YAW] = 0U;
  calibration.sample_count[GM6020_AXIS_PITCH] = 0U;
  calibration.reference_valid[GM6020_AXIS_YAW] = false;
  calibration.reference_valid[GM6020_AXIS_PITCH] = false;
  calibration.sum_unwrapped_ecd[GM6020_AXIS_YAW] = 0;
  calibration.sum_unwrapped_ecd[GM6020_AXIS_PITCH] = 0;
  calibration.last_sequence[GM6020_AXIS_YAW] =
      (yaw != NULL) ? yaw->rx_sequence : 0U;
  calibration.last_sequence[GM6020_AXIS_PITCH] =
      (pitch != NULL) ? pitch->rx_sequence : 0U;
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
  memset(&calibration, 0, sizeof(calibration));
  calibration_reset_samples();
  calibration.status = GIMBAL_CALIBRATION_WAITING_FEEDBACK;
}

void GimbalCalibration_Process(void)
{
  const GM6020_Feedback_t *feedback[GM6020_AXIS_COUNT];
  uint16_t yaw_zero_ecd;
#if !GIMBAL_YAW_ONLY_TEST_MODE
  uint16_t pitch_zero_ecd;
#endif

  if (!GimbalCalibration_IsBusy())
  {
    return;
  }

  feedback[GM6020_AXIS_YAW] =
      GM6020_GetFeedback(GM6020_AXIS_YAW);
#if !GIMBAL_YAW_ONLY_TEST_MODE
  feedback[GM6020_AXIS_PITCH] =
      GM6020_GetFeedback(GM6020_AXIS_PITCH);
#endif

#if GIMBAL_YAW_ONLY_TEST_MODE
  if ((feedback[GM6020_AXIS_YAW] == NULL)
      || !feedback[GM6020_AXIS_YAW]->online)
#else
  if ((feedback[GM6020_AXIS_YAW] == NULL)
      || (feedback[GM6020_AXIS_PITCH] == NULL)
      || !feedback[GM6020_AXIS_YAW]->online
      || !feedback[GM6020_AXIS_PITCH]->online)
#endif
  {
    calibration_reset_samples();
    calibration.status = GIMBAL_CALIBRATION_WAITING_FEEDBACK;
    return;
  }

  if ((feedback[GM6020_AXIS_YAW]->speed_rpm
       > CALIBRATION_STILL_SPEED_RPM)
      || (feedback[GM6020_AXIS_YAW]->speed_rpm
          < -CALIBRATION_STILL_SPEED_RPM)
#if !GIMBAL_YAW_ONLY_TEST_MODE
      || (feedback[GM6020_AXIS_PITCH]->speed_rpm
          > CALIBRATION_STILL_SPEED_RPM)
      || (feedback[GM6020_AXIS_PITCH]->speed_rpm
          < -CALIBRATION_STILL_SPEED_RPM))
#else
      )
#endif
  {
    calibration_reset_samples();
    calibration.status = GIMBAL_CALIBRATION_WAITING_STILL;
    return;
  }

  calibration.status = GIMBAL_CALIBRATION_SAMPLING;
  if (!calibration_sample_axis(
          GM6020_AXIS_YAW,
          feedback[GM6020_AXIS_YAW])
#if !GIMBAL_YAW_ONLY_TEST_MODE
      || !calibration_sample_axis(
          GM6020_AXIS_PITCH,
          feedback[GM6020_AXIS_PITCH]))
#else
      )
#endif
  {
    calibration_reset_samples();
    calibration.status = GIMBAL_CALIBRATION_WAITING_STILL;
    return;
  }

  if (calibration.sample_count[GM6020_AXIS_YAW]
      < CALIBRATION_SAMPLE_COUNT
#if !GIMBAL_YAW_ONLY_TEST_MODE
      || (calibration.sample_count[GM6020_AXIS_PITCH]
          < CALIBRATION_SAMPLE_COUNT))
#else
      )
#endif
  {
    return;
  }

  yaw_zero_ecd = calibration_average_axis(GM6020_AXIS_YAW);
#if GIMBAL_YAW_ONLY_TEST_MODE
  if (!GM6020_SetAxisZeroOffsetEcd(
          GM6020_AXIS_YAW, yaw_zero_ecd))
#else
  pitch_zero_ecd = calibration_average_axis(GM6020_AXIS_PITCH);
  if (!GM6020_SetZeroOffsetsEcd(
          yaw_zero_ecd, pitch_zero_ecd))
#endif
  {
    calibration.status = GIMBAL_CALIBRATION_ERROR;
    return;
  }

  calibration.status = GIMBAL_CALIBRATION_CALIBRATED;
}

GimbalCalibrationStatus_t GimbalCalibration_GetStatus(void)
{
  return calibration.status;
}

bool GimbalCalibration_IsBusy(void)
{
  return (calibration.status
          == GIMBAL_CALIBRATION_WAITING_FEEDBACK)
      || (calibration.status
          == GIMBAL_CALIBRATION_WAITING_STILL)
      || (calibration.status
          == GIMBAL_CALIBRATION_SAMPLING);
}

uint16_t GimbalCalibration_GetYawSampleCount(void)
{
  return calibration.sample_count[GM6020_AXIS_YAW];
}

uint16_t GimbalCalibration_GetPitchSampleCount(void)
{
  return calibration.sample_count[GM6020_AXIS_PITCH];
}
