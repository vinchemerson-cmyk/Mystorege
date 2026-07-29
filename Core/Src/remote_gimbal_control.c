/**
 * ===========================================================================
 * @file    remote_gimbal_control.c
 * @brief   将 DBUS 摇杆量映射为双轴云台位置目标
 * ===========================================================================
 *
 * 控制方式：
 *   CH0 → Yaw 目标角速度 → 积分得到 Yaw 多圈位置目标
 *   CH1 → Pitch 目标角速度 → 积分得到 Pitch 单圈位置目标
 *   S1上挡 → 云台与底盘急停；S1中/下挡 → 解除急停（临时映射）
 *
 * 每个轴独立检查以下接管条件：
 *   - DBUS 最近100 ms内收到过合法帧；
 *   - 该轴已经独立完成上电机械零位标定；
 *   - 该轴GM6020在线并且全局急停未锁存。
 *
 * 条件不满足时不会继续积分。恢复控制时从电机实时位置重新建立目标，
 * 防止遥控器重连、急停解除或电机恢复后跳到旧目标。
 * ===========================================================================
 */

#include "remote_gimbal_control.h"

#include "chassis_can.h"
#include "config/gimbal_params.h"
#include "dbus.h"
#include "gimbal_calibration.h"
#include "motor_control.h"

#include <stdbool.h>
#include <stdint.h>

/* ─── 遥控器通道映射 (Remote Control Channel Mapping) ─── */
/* 上板视角：右摇杆横向 CH0 → Yaw，纵向 CH1 → Pitch */
#define REMOTE_GIMBAL_YAW_CHANNEL          0U    /* Yaw 通道索引 — yaw channel index (CH0) */
#define REMOTE_GIMBAL_PITCH_CHANNEL        1U    /* Pitch 通道索引 — pitch channel index (CH1) */

/*
 * ─── 临时S1安全开关映射 (Temporary S1 Safety Mapping) ───
 *
 * 当前测试阶段：
 *   S1上挡     → 锁存云台和底盘急停
 *   S1中/下挡  → 解除云台和底盘急停
 *
 * 此映射是临时设置，后续功能规划改变时可整体移除或替换。
 */
#define REMOTE_GIMBAL_TEMP_S1_ESTOP_ENABLE  1U
#define REMOTE_GIMBAL_S1_INDEX              0U
#define REMOTE_GIMBAL_SWITCH_UNKNOWN        0U

/* ─── 摇杆曲线参数 (Joystick Curve Parameters) ─── */
/*
 * 死区 (Deadzone): 去中心值 ±30 内不动作，滤除摇杆回中抖动。
 * DBUS 去中心值范围约 -660 ~ +660，30/660 ≈ 4.5% 死区。
 */
#define REMOTE_GIMBAL_DEADZONE             30    /* 死区阈值 — deadzone threshold (centered units) */

/*
 * 满量程 (Full Scale): 去中心值的最大幅值。
 * DBUS 通道值 364~1684 → 去中心 = -660 ~ +660。
 */
#define REMOTE_GIMBAL_FULL_SCALE           660   /* 满量程 — full scale (centered units) */

/* ─── 角速度映射 (Angular Rate Mapping) ─── */
/* 满杆时对应的云台逻辑目标角速度 (°/s) */
#define REMOTE_GIMBAL_YAW_MAX_RATE_DPS    360.0f /* Yaw 满杆角速度 — max yaw rate (degrees/sec) */
#define REMOTE_GIMBAL_PITCH_MAX_RATE_DPS   30.0f /* Pitch 满杆角速度 — max pitch rate (degrees/sec) */

/*
 * 安装方向修正 (Mounting Direction Correction)。
 * 若实机运动方向与遥控器方向相反，只需把对应值改为 -1.0f。
 * 例: 推杆向右 → 云台向左 → 设 YAW_DIRECTION = -1.0f
 */
#define REMOTE_GIMBAL_YAW_DIRECTION         1.0f  /* Yaw 方向修正 — +1=正向, -1=反向 */
#define REMOTE_GIMBAL_PITCH_DIRECTION      -1.0f  /* Pitch 安装方向与遥控器相反 */

/* ─── 软件限位 (Software Limits) ─── */
/* 遥控器积分位置目标的软件范围，防止目标角度无限累积溢出 */
#define REMOTE_GIMBAL_YAW_MIN_DEG      (-36000.0f) /* Yaw 最小角度 — min yaw angle (±100圈) */
#define REMOTE_GIMBAL_YAW_MAX_DEG        36000.0f  /* Yaw 最大角度 — max yaw angle */
#define REMOTE_GIMBAL_PITCH_MIN_DEG        -30.0f  /* Pitch 最小角度 — min pitch angle */
#define REMOTE_GIMBAL_PITCH_MAX_DEG         30.0f  /* Pitch 最大角度 — max pitch angle */

/*
 * 单次积分时间上限 (Max Integration Delta)。
 * 限制单次积分时间 ≤ 20 ms，防止主循环长时间阻塞（如 Flash 擦除）
 * 后恢复时，摇杆目标的单次积分跃变过大导致云台突然跳动。
 * 如果真实时间差 > 20 ms，只按 20 ms 计算积分值。
 */
#define REMOTE_GIMBAL_MAX_DELTA_MS           20U   /* 单次积分上限 — max delta per integration (ms) */

/*
 * ─── 遥控器控制上下文 (Remote Control Context) ───
 * 保存 Yaw/Pitch 的积分目标角度、时间戳、激活状态。
 * 整个模块使用一个静态全局实例。
 */
typedef struct
{
  float yaw_target_deg;        /* Yaw 积分目标角度 — integrated yaw target (degrees) */
  float pitch_target_deg;      /* Pitch 积分目标角度 — integrated pitch target (degrees) */
  uint32_t last_process_ms;    /* 上次处理时间戳 — last process timestamp (ms) */
  bool axis_active[GM6020_AXIS_COUNT]; /* 各轴独立接管激活标志 */
  uint8_t last_s1_position;    /* 上次S1挡位，用于中/下挡解除沿检测 */
} RemoteGimbalControlContext_t;

static RemoteGimbalControlContext_t remote_control;

/*
 * ─── 内部工具函数 (Internal Utility Functions) ───
 */

/*
 * 浮点数限幅 (Float Clamp)。
 * 将 value 限制在 [minimum, maximum] 闭区间内。
 */
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
 * 将去中心通道值归一化到 [-1.0, +1.0] (Normalize centered channel to [-1.0, +1.0])。
 *
 * 【映射曲线】
 *   Deadzone (dead)    : centered ∈ [-30, +30] → 0.0
 *   Transition (ramp)  : centered ∈ (±30, ±660] → 线性映射到 (±0, ±1]
 *
 * 【优点】
 *   死区边沿从 0 平滑起步 (smooth ramp from zero at deadzone edge)，
 *   避免刚越过死区就产生速度阶跃 (velocity step)，操作手感更线性。
 *
 * @param centered_value  DBUS 去中心值 (约 -660 ~ +660)
 * @return               归一化输出 [-1.0, +1.0]
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
 * 处理临时S1急停映射。
 *
 * 上挡使用电平优先策略：只要S1保持上挡，任何其他来源解除急停后都会
 * 在下一控制周期重新急停。中/下挡只在进入该挡位时解除一次，避免
 * 持续覆盖USB ESTOP等其他急停来源。
 */
static void process_temporary_s1_safety(
    const DBUS_Data_t *dbus_data)
{
#if REMOTE_GIMBAL_TEMP_S1_ESTOP_ENABLE
  uint8_t s1_position;

  if ((dbus_data == NULL)
      || !dbus_data->online
      || !dbus_data->last_frame_valid)
  {
    return;
  }

  s1_position =
      dbus_data->switch_value[REMOTE_GIMBAL_S1_INDEX];

  if (s1_position == DBUS_SWITCH_UP)
  {
    if (!GM6020_IsEmergencyStopped())
    {
      (void)GM6020_EmergencyStop();
      (void)ChassisCAN_EmergencyStop();
    }
  }
  else if (((s1_position == DBUS_SWITCH_MIDDLE)
            || (s1_position == DBUS_SWITCH_DOWN))
           && (s1_position != remote_control.last_s1_position))
  {
    GM6020_ClearEmergencyStop();
    ChassisCAN_ClearEmergencyStop();
  }

  remote_control.last_s1_position = s1_position;
#else
  (void)dbus_data;
#endif
}

/*
 * 判断指定轴是否具备遥控器接管条件。
 *
 * 四个必要条件全部满足才返回true：
 *   1. DBUS 在线 (dbus_data->online) — 接收机有信号
 *   2. 指定轴零位标定完成 — 逻辑角度0°对应该轴开机位置
 *   3. 云台未急停 — 没有被锁存式急停阻止
 *   4. 指定轴在线 — 收到该轴有效CAN反馈
 *
 * 另一轴离线或尚未标定不会影响本轴。
 */
static bool remote_axis_is_available(
    const DBUS_Data_t *dbus_data,
    GM6020_Axis_t axis)
{
  const GM6020_Feedback_t *feedback;

#if GIMBAL_YAW_ONLY_TEST_MODE
  if (axis == GM6020_AXIS_PITCH)
  {
    return false;
  }
#endif

  if ((dbus_data == NULL)
      || !dbus_data->online
      || !GimbalCalibration_IsAxisCalibrated(axis)
      || GM6020_IsEmergencyStopped())
  {
    return false;
  }

  feedback = GM6020_GetFeedback(axis);
  return (feedback != NULL) && feedback->online;
}

/*
 * ─── 公有 API (Public API) ───
 */

/*
 * 初始化遥控器目标积分器。
 * 清零目标角度和时间戳，等待首次接管时从电机实时位置同步。
 */
void RemoteGimbalControl_Init(void)
{
  remote_control.yaw_target_deg = 0.0f;
  remote_control.pitch_target_deg = 0.0f;
  remote_control.last_process_ms = HAL_GetTick();
  remote_control.axis_active[GM6020_AXIS_YAW] = false;
  remote_control.axis_active[GM6020_AXIS_PITCH] = false;
  remote_control.last_s1_position =
      REMOTE_GIMBAL_SWITCH_UNKNOWN;
}

/*
 * 主循环遥控器控制处理 (Main Loop Remote Control Processing)。
 *
 * 【控制流程】
 *   1. 分别检查两轴接管条件，不满足的轴单独deactivate
 *   2. 某轴首次接管 → 从该轴实时位置建立积分起点
 *   3. 已接管轴 → normalize_channel → 角速度积分 → 限幅 → 下传目标
 *
 * 【积分公式】
 *   new_target = old_target + normalized * direction * max_rate * (delta_ms / 1000)
 *
 *   例: 摇杆半推 (normalized=0.5), Yaw max_rate=360°/s, delta=1ms
 *        → 增量 = 0.5 * 1.0 * 360 * 0.001 = 0.18° / 周期
 *
 * 【恢复时重新加锁策略】
 *   当某轴接管条件从false变为true时，只为该轴读取实时位置并重新
 *   建立积分起点；另一轴继续运行，不复用离线轴的旧目标。
 */
void RemoteGimbalControl_Process(void)
{
  const uint32_t now = HAL_GetTick();
  const DBUS_Data_t *dbus_data = DBUS_GetData();
  bool yaw_available;
  bool pitch_available;
  uint32_t delta_ms;
  float yaw_input;
  float pitch_input;

  process_temporary_s1_safety(dbus_data);

  yaw_available = remote_axis_is_available(
      dbus_data, GM6020_AXIS_YAW);
  pitch_available = remote_axis_is_available(
      dbus_data, GM6020_AXIS_PITCH);

  if (!yaw_available)
  {
    remote_control.axis_active[GM6020_AXIS_YAW] = false;
  }
  if (!pitch_available)
  {
    remote_control.axis_active[GM6020_AXIS_PITCH] = false;
  }

  delta_ms = (uint32_t)(now - remote_control.last_process_ms);
  remote_control.last_process_ms = now;
  if (delta_ms > REMOTE_GIMBAL_MAX_DELTA_MS)
  {
    delta_ms = REMOTE_GIMBAL_MAX_DELTA_MS;
  }

  if (!yaw_available && !pitch_available)
  {
    return;
  }

  if (yaw_available)
  {
    if (!remote_control.axis_active[GM6020_AXIS_YAW])
    {
      if (GM6020_GetMultiTurnPosition(
              GM6020_AXIS_YAW,
              &remote_control.yaw_target_deg))
      {
        remote_control.axis_active[GM6020_AXIS_YAW] = true;
        GM6020_SetMultiTurnTargetPosition(
            GM6020_AXIS_YAW,
            remote_control.yaw_target_deg);
      }
    }
    else if (delta_ms > 0U)
    {
      yaw_input = normalize_channel(
          dbus_data->centered_channel[
              REMOTE_GIMBAL_YAW_CHANNEL]);
      remote_control.yaw_target_deg +=
          yaw_input
          * REMOTE_GIMBAL_YAW_DIRECTION
          * REMOTE_GIMBAL_YAW_MAX_RATE_DPS
          * (float)delta_ms
          / 1000.0f;
      remote_control.yaw_target_deg = clamp_float(
          remote_control.yaw_target_deg,
          REMOTE_GIMBAL_YAW_MIN_DEG,
          REMOTE_GIMBAL_YAW_MAX_DEG);
      GM6020_SetMultiTurnTargetPosition(
          GM6020_AXIS_YAW,
          remote_control.yaw_target_deg);
    }
  }

  if (pitch_available)
  {
    if (!remote_control.axis_active[GM6020_AXIS_PITCH])
    {
      if (GM6020_GetMultiTurnPosition(
              GM6020_AXIS_PITCH,
              &remote_control.pitch_target_deg))
      {
        remote_control.axis_active[GM6020_AXIS_PITCH] = true;
        GM6020_SetTargetPosition(
            GM6020_AXIS_PITCH,
            remote_control.pitch_target_deg);
      }
    }
    else if (delta_ms > 0U)
    {
      pitch_input = normalize_channel(
          dbus_data->centered_channel[
              REMOTE_GIMBAL_PITCH_CHANNEL]);
      remote_control.pitch_target_deg +=
          pitch_input
          * REMOTE_GIMBAL_PITCH_DIRECTION
          * REMOTE_GIMBAL_PITCH_MAX_RATE_DPS
          * (float)delta_ms
          / 1000.0f;
      remote_control.pitch_target_deg = clamp_float(
          remote_control.pitch_target_deg,
          REMOTE_GIMBAL_PITCH_MIN_DEG,
          REMOTE_GIMBAL_PITCH_MAX_DEG);
      GM6020_SetTargetPosition(
          GM6020_AXIS_PITCH,
          remote_control.pitch_target_deg);
    }
  }
}
