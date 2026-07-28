/**
 * ===========================================================================
 * @file    remote_gimbal_control.c
 * @brief   将 DBUS 摇杆量映射为双轴云台位置目标
 * ===========================================================================
 *
 * 控制方式：
 *   CH0 → Yaw 目标角速度 → 积分得到 Yaw 多圈位置目标
 *   CH1 → Pitch 目标角速度 → 积分得到 Pitch 单圈位置目标
 *
 * 只有在以下条件全部满足时才接管云台：
 *   - DBUS 最近 100 ms 内收到过合法帧；
 *   - 上电机械零位标定完成；
 *   - 两轴 GM6020 在线并且没有急停。
 *
 * 条件不满足时不会继续积分。恢复控制时从电机实时位置重新建立目标，
 * 防止遥控器重连、急停解除或电机恢复后跳到旧目标。
 * ===========================================================================
 */

#include "remote_gimbal_control.h"

#include "config/gimbal_params.h"
#include "dbus.h"
#include "gimbal_calibration.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

/* 遥控器通道映射：右摇杆横向 CH0、右摇杆纵向 CH1 */
#define REMOTE_GIMBAL_YAW_CHANNEL          0U
#define REMOTE_GIMBAL_PITCH_CHANNEL        1U

/* 摇杆去中心值在 ±30 内不动作，滤除回中抖动 */
#define REMOTE_GIMBAL_DEADZONE             30
#define REMOTE_GIMBAL_FULL_SCALE           660

/* 满杆对应的云台逻辑目标角速度 */
#define REMOTE_GIMBAL_YAW_MAX_RATE_DPS    360.0f
#define REMOTE_GIMBAL_PITCH_MAX_RATE_DPS   30.0f

/*
 * 安装方向修正。若实机运动方向相反，只需把对应值改为 -1.0f。
 */
#define REMOTE_GIMBAL_YAW_DIRECTION         1.0f
#define REMOTE_GIMBAL_PITCH_DIRECTION       1.0f

/* 遥控器位置目标的软件范围 */
#define REMOTE_GIMBAL_YAW_MIN_DEG      (-36000.0f)
#define REMOTE_GIMBAL_YAW_MAX_DEG        36000.0f
#define REMOTE_GIMBAL_PITCH_MIN_DEG        -30.0f
#define REMOTE_GIMBAL_PITCH_MAX_DEG         30.0f

/* 限制单次积分时间，防止主循环长时间阻塞后目标突然跳变 */
#define REMOTE_GIMBAL_MAX_DELTA_MS           20U

typedef struct
{
  float yaw_target_deg;
  float pitch_target_deg;
  uint32_t last_process_ms;
  bool active;
} RemoteGimbalControlContext_t;

static RemoteGimbalControlContext_t remote_control;

static float clamp_float(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

/*
 * 将去中心后的通道值映射到 -1.0~+1.0。
 * 死区边沿从 0 平滑起步，避免刚越过死区就产生速度阶跃。
 */
static float normalize_channel(int16_t centered_value)
{
  int32_t magnitude = centered_value;
  float normalized;

  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }

  if (magnitude <= REMOTE_GIMBAL_DEADZONE)
  {
    return 0.0f;
  }

  normalized =
      (float)(magnitude - REMOTE_GIMBAL_DEADZONE)
      / (float)(REMOTE_GIMBAL_FULL_SCALE
                - REMOTE_GIMBAL_DEADZONE);
  normalized = clamp_float(normalized, 0.0f, 1.0f);

  return (centered_value < 0) ? -normalized : normalized;
}

/*
 * 判断当前是否具备遥控器接管条件。
 * DBUS 的 channel[] 保存最近一帧合法数据，因此这里只需要检查 online。
 */
static bool remote_control_is_available(const DBUS_Data_t *dbus_data)
{
  const GM6020_Feedback_t *yaw_feedback;
#if !GIMBAL_YAW_ONLY_TEST_MODE
  const GM6020_Feedback_t *pitch_feedback;
#endif

  if ((dbus_data == NULL)
      || !dbus_data->online
      || (GimbalCalibration_GetStatus()
          != GIMBAL_CALIBRATION_CALIBRATED)
      || GM6020_IsEmergencyStopped())
  {
    return false;
  }

  yaw_feedback = GM6020_GetFeedback(GM6020_AXIS_YAW);
#if GIMBAL_YAW_ONLY_TEST_MODE
  return (yaw_feedback != NULL) && yaw_feedback->online;
#else
  pitch_feedback = GM6020_GetFeedback(GM6020_AXIS_PITCH);

  return (yaw_feedback != NULL)
      && (pitch_feedback != NULL)
      && yaw_feedback->online
      && pitch_feedback->online;
#endif
}

void RemoteGimbalControl_Init(void)
{
  remote_control.yaw_target_deg = 0.0f;
  remote_control.pitch_target_deg = 0.0f;
  remote_control.last_process_ms = HAL_GetTick();
  remote_control.active = false;
}

void RemoteGimbalControl_Process(void)
{
  const uint32_t now = HAL_GetTick();
  const DBUS_Data_t *dbus_data = DBUS_GetData();
  uint32_t delta_ms;
  float yaw_input;
#if !GIMBAL_YAW_ONLY_TEST_MODE
  float pitch_input;
#endif

  if (!remote_control_is_available(dbus_data))
  {
    remote_control.active = false;
    remote_control.last_process_ms = now;
    return;
  }

  /*
   * 首次接管或故障恢复时，以编码器实时位置作为积分起点。
   * 这里只建立目标，不使用开机前或掉线前保存的旧目标。
   */
  if (!remote_control.active)
  {
    if (!GM6020_GetMultiTurnPosition(
            GM6020_AXIS_YAW,
            &remote_control.yaw_target_deg)
#if !GIMBAL_YAW_ONLY_TEST_MODE
        || !GM6020_GetMultiTurnPosition(
            GM6020_AXIS_PITCH,
            &remote_control.pitch_target_deg))
#else
        )
#endif
    {
      remote_control.last_process_ms = now;
      return;
    }

    remote_control.active = true;
    remote_control.last_process_ms = now;
    GM6020_SetMultiTurnTargetPosition(
        GM6020_AXIS_YAW,
        remote_control.yaw_target_deg);
#if !GIMBAL_YAW_ONLY_TEST_MODE
    GM6020_SetTargetPosition(
        GM6020_AXIS_PITCH,
        remote_control.pitch_target_deg);
#endif
    return;
  }

  delta_ms = (uint32_t)(now - remote_control.last_process_ms);
  if (delta_ms == 0U)
  {
    return;
  }
  remote_control.last_process_ms = now;

  if (delta_ms > REMOTE_GIMBAL_MAX_DELTA_MS)
  {
    delta_ms = REMOTE_GIMBAL_MAX_DELTA_MS;
  }

  yaw_input = normalize_channel(
      dbus_data->centered_channel[REMOTE_GIMBAL_YAW_CHANNEL]);
#if !GIMBAL_YAW_ONLY_TEST_MODE
  pitch_input = normalize_channel(
      dbus_data->centered_channel[REMOTE_GIMBAL_PITCH_CHANNEL]);
#endif

  remote_control.yaw_target_deg +=
      yaw_input
      * REMOTE_GIMBAL_YAW_DIRECTION
      * REMOTE_GIMBAL_YAW_MAX_RATE_DPS
      * (float)delta_ms
      / 1000.0f;
#if !GIMBAL_YAW_ONLY_TEST_MODE
  remote_control.pitch_target_deg +=
      pitch_input
      * REMOTE_GIMBAL_PITCH_DIRECTION
      * REMOTE_GIMBAL_PITCH_MAX_RATE_DPS
      * (float)delta_ms
      / 1000.0f;

  remote_control.yaw_target_deg = clamp_float(
      remote_control.yaw_target_deg,
      REMOTE_GIMBAL_YAW_MIN_DEG,
      REMOTE_GIMBAL_YAW_MAX_DEG);
  remote_control.pitch_target_deg = clamp_float(
      remote_control.pitch_target_deg,
      REMOTE_GIMBAL_PITCH_MIN_DEG,
      REMOTE_GIMBAL_PITCH_MAX_DEG);
#endif

  GM6020_SetMultiTurnTargetPosition(
      GM6020_AXIS_YAW,
      remote_control.yaw_target_deg);
#if !GIMBAL_YAW_ONLY_TEST_MODE
  GM6020_SetTargetPosition(
      GM6020_AXIS_PITCH,
      remote_control.pitch_target_deg);
#endif
}
