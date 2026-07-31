#include "motor_control.h"
#include "config/gimbal_params.h"
#include "config/pitch_fusion_config.h"
#include "pitch_fusion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * ============================================================================
 * 两轴 GM6020 云台电机控制模块
 * ============================================================================
 *
 * 【硬件拓扑】
 *   两个 DJI GM6020 分别连接到两条 CAN 总线，且电机 ID 都为 2：
 *     Yaw   → 上板 CAN1（经滑环到下板 CAN2），反馈帧 0x206
 *     Pitch → 上板 CAN2，反馈帧 0x206
 *
 * 【控制算法】
 *   串级 PID：外环角度环（P/PD）→ 内环速度环（PI/PD）
 *   每个轴拥有独立的编码器多圈累计、状态机和两套 PID 控制器。
 *
 * 【CAN 协议约束】
 *   两台电机物理总线不同，因此每条总线分别发送一帧 0x1FE。
 *   该总线只有一台 ID 2 电机，DATA[2:3] 放本轴电流，其余槽位置零。
 *
 * 【控制时序】
 *   GM6020_Process() 在主循环中高频调用，每轮耗尽 CAN RX FIFO 中
 *   所有待处理的反馈帧。GM6020 以约 1 kHz 的频率主动推送反馈，
 *   因此控制周期自然地锁定在 ~1 kHz，无需额外定时器。
 *
 * 【安全机制】
 *   - 反馈超过配置的超时时间 → FAULT 状态 → 输出零电流
 *   - PID 积分项 anti-windup（输出饱和时停止积分累积）
 *   - 速度调试模式目标转速硬限幅 ±200 RPM
 *   - 角度软限位框架（当前 Yaw ±180° 仍是待标定的占位范围）
 *   - 启动阶段尚未收到 CAN 反馈前保持零电流
 * ============================================================================
 */

/*
 * GM6020 CAN 协议常量：
 *   电流命令帧 ID 固定为 0x1FE（标准帧）
 *   编码器分辨率 8192 CPR（Counts Per Revolution），半圈 = 4096
 *   控制周期 0.001 s = 1 kHz（由电机反馈帧发送频率决定）
 */
#define GM6020_CURRENT_COMMAND_ID     0x1FEU
#define GM6020_ENCODER_HALF_CPR       ((int32_t)(GM6020_ENCODER_CPR / 2U))
#define GM6020_CONTROL_PERIOD_S       0.001f
#define GM6020_ZERO_SET_MAX_SPEED_RPM 2
#define GM6020_RPM_TO_DPS             6.0f
#define GM6020_DEG_TO_RAD             0.017453292519943295f

/*
 * 控制状态机枚举。
 *
 *   状态转移图：
 *     WAIT_FEEDBACK ──(首次反馈，未请求调试)──▶ POSITION_CONTROL
 *            │                                      │
 *            │                                      └──(超时)──▶ FAULT
 *            │
 *            └──(首次反馈，已请求调试)──▶ SPEED_DEBUG ──(超时)──▶ FAULT
 *
 *   POSITION_CONTROL: 正常位置伺服，角度环 PID → 速度环 PID → 电流输出
 *   SPEED_DEBUG:      绕过角度环，直接用目标 RPM 驱动速度环（调参用）
 *   WAIT_FEEDBACK:    上电/复位后等待电机送来第一帧反馈
 *   FAULT:            反馈超时，输出零电流，等待下一帧有效反馈自动恢复
 */
typedef enum
{
  GM6020_STATE_WAIT_FEEDBACK = 0,
  GM6020_STATE_POSITION_CONTROL,
  GM6020_STATE_SPEED_DEBUG,
  GM6020_STATE_FAULT,
  GM6020_STATE_COUNT
} GM6020_ControlState_t;

/*
 * 速度环 PID 状态。
 * 输出为转矩电流原始值（int16_t，±16384 ≈ ±3A），内部计算用 float。
 */
typedef struct
{
  float kp;              /* 比例增益 */
  float ki;              /* 积分增益 */
  float kd;              /* 微分增益 */
  float integral;        /* 积分累加项，受 anti-windup 限幅 */
  float previous_error;  /* 上一拍误差，用于微分项计算 */
  float output;          /* 最近一次 PID 输出值（电流命令） */
} SpeedPID_t;

/*
 * 角度环 PID 状态。
 * 输出为速度环的目标转速（rpm），内部计算用 float。
 */
typedef struct
{
  float kp;              /* 比例增益 */
  float ki;              /* 积分增益 */
  float kd;              /* 微分增益 */
  float integral;        /* 积分累加项，受 anti-windup 限幅 */
  float previous_error;  /* 上一拍误差，用于微分项计算 */
  float output;          /* 最近一次 PID 输出值（目标转速 rpm） */
} AnglePID_t;

/*
 * 单轴完整的控制器状态。
 * Yaw 和 Pitch 各持有一个独立的实例（controllers[2] 数组）。
 */
typedef struct
{
  /* ---- 电机反馈数据 ---- */
  GM6020_Feedback_t feedback;

  /* ---- 串级 PID ---- */
  SpeedPID_t speed_pid;   /* 内环：速度 → 电流 */
  AnglePID_t angle_pid;   /* 外环：角度 → 目标转速 */

  /* ---- PID 限幅 ---- */
  float speed_output_limit;     /* 速度环输出限幅（转矩电流，典型值 8192） */
  float angle_speed_limit_rpm;  /* 角度环输出限幅（目标转速上限 rpm） */

  /* ---- 机械标定 ---- */
  float zero_offset_deg;        /* 逻辑角度 0° 对应的编码器单圈角度偏移 */
  float minimum_angle_deg;      /* 逻辑软限位下限（度） */
  float maximum_angle_deg;      /* 逻辑软限位上限（度） */
  bool angle_limit_enabled;     /* 软限位是否启用 */

  /* ---- 速度调试 ---- */
  float target_speed_rpm;             /* 当前目标转速（位置控制时由角度环输出） */
  float debug_target_speed_rpm;       /* 速度调试模式下的人工目标转速 */
  float command_speed_feedforward_rpm;/* 位置轨迹目标速度前馈 */
  float command_accel_feedforward_rpm_s;/* 位置轨迹目标加速度前馈 */
  float applied_speed_feedforward_rpm;/* 本拍角度环实际叠加量 */
  float applied_current_feedforward;  /* 本拍速度环实际叠加量 */
  bool speed_debug_requested;         /* 是否正在/请求进入速度调试模式 */

  /* ---- 位置命令 ---- */
  float requested_angle_deg;           /* 单圈角度或累计多圈角度目标 */
  bool requested_angle_is_multi_turn;  /* true：累计多圈；false：单圈劣弧 */
  bool position_target_valid;          /* 是否有有效的位置目标 */
  int32_t target_total_angle_ecd;      /* 解析后的多圈编码器目标值 */
  bool pitch_fusion_feedback_active;   /* Pitch是否正在使用IMU融合反馈 */

  /* ---- 编码器多圈追踪 ---- */
  uint16_t previous_encoder;           /* 上一拍的原始编码器值，用于跨零点检测 */
  bool encoder_initialized;            /* 编码器累计是否已初始化 */
  int32_t multi_turn_origin_ecd;       /* 距离启动位置最近的编码器零点 */
  bool multi_turn_origin_valid;        /* 多圈位置零点是否有效 */

  /* ---- 输出 ---- */
  int16_t current_command;            /* 当前转矩电流命令（±16384） */

  /* ---- 状态机 ---- */
  GM6020_ControlState_t state;        /* 当前控制状态 */
} GM6020_Controller_t;

/*
 * 每个轴的 CAN 总线配置。
 * feedback_std_id: 电机反馈帧的标准 ID（两轴都是 0x206）
 * current_slot: 在 0x1FE 电流命令帧中的槽位编号（0=DATA[0:1], 1=DATA[2:3]）
 * filter_bank: bxCAN 共享过滤器组编号（CAN1 用 0，CAN2 用 14）
 */
typedef struct
{
  uint16_t feedback_std_id;
  uint8_t current_slot;
  uint8_t filter_bank;
} GM6020_AxisCanConfig_t;

/*
 * 两轴的 CAN 配置表，由 gimbal_params.h 宏填充。
 * 索引 0 = Yaw，索引 1 = Pitch。
 */
static const GM6020_AxisCanConfig_t axis_can_config[
    GM6020_AXIS_COUNT] =
{
  {
    YAW_FEEDBACK_STD_ID,
    YAW_CURRENT_COMMAND_SLOT,
    YAW_CAN_FILTER_BANK
  },
  {
    PITCH_FEEDBACK_STD_ID,
    PITCH_CURRENT_COMMAND_SLOT,
    PITCH_CAN_FILTER_BANK
  }
};

/*
 * 每个轴的 PID 参数和输出限幅配置。
 */
typedef struct
{
  float speed_kp;
  float speed_ki;
  float speed_kd;
  float speed_output_limit;
  float angle_kp;
  float angle_ki;
  float angle_kd;
  float angle_speed_limit_rpm;
} GM6020_AxisPidConfig_t;

/*
 * 两轴的 PID 配置表，由 gimbal_params.h 宏填充。
 */
static const GM6020_AxisPidConfig_t axis_pid_config[
    GM6020_AXIS_COUNT] =
{
  {
    YAW_SPEED_PID_KP,
    YAW_SPEED_PID_KI,
    YAW_SPEED_PID_KD,
    YAW_SPEED_PID_OUTPUT_LIMIT,
    YAW_ANGLE_PID_KP,
    YAW_ANGLE_PID_KI,
    YAW_ANGLE_PID_KD,
    YAW_ANGLE_SPEED_LIMIT_RPM
  },
  {
    PITCH_SPEED_PID_KP,
    PITCH_SPEED_PID_KI,
    PITCH_SPEED_PID_KD,
    PITCH_SPEED_PID_OUTPUT_LIMIT,
    PITCH_ANGLE_PID_KP,
    PITCH_ANGLE_PID_KI,
    PITCH_ANGLE_PID_KD,
    PITCH_ANGLE_SPEED_LIMIT_RPM
  }
};

/*
 * 每个轴的机械参数配置（零位偏移和软限位）。
 */
typedef struct
{
  float zero_offset_deg;
  float minimum_angle_deg;
  float maximum_angle_deg;
  bool limit_enabled;
} GM6020_AxisMechanicalConfig_t;

/*
 * 两轴的机械配置表，由 gimbal_params.h 宏填充。
 */
static const GM6020_AxisMechanicalConfig_t axis_mechanical_config[
    GM6020_AXIS_COUNT] =
{
  {
    YAW_ZERO_OFFSET_DEG,
    YAW_MIN_ANGLE_DEG,
    YAW_MAX_ANGLE_DEG,
    (YAW_ANGLE_LIMIT_ENABLE != 0U)
  },
  {
    PITCH_ZERO_OFFSET_DEG,
    PITCH_MIN_ANGLE_DEG,
    PITCH_MAX_ANGLE_DEG,
    (PITCH_ANGLE_LIMIT_ENABLE != 0U)
  }
};

typedef struct
{
  float target_rate_gain;
  float base_rate_gain;
  float gravity_sin_current;
  float gravity_cos_current;
  float gravity_bias_current;
  float static_friction_current;
  float velocity_current_per_rpm;
  float acceleration_current_per_rpm_s;
  float current_limit;
} GM6020_AxisFeedforwardConfig_t;

static const GM6020_AxisFeedforwardConfig_t axis_feedforward_config[
    GM6020_AXIS_COUNT] =
{
  {
    YAW_TARGET_RATE_FF_GAIN,
    YAW_BASE_RATE_FF_GAIN,
    YAW_GRAVITY_SIN_FF_CURRENT,
    YAW_GRAVITY_COS_FF_CURRENT,
    YAW_GRAVITY_BIAS_FF_CURRENT,
    YAW_STATIC_FRICTION_FF_CURRENT,
    YAW_VELOCITY_FF_CURRENT_PER_RPM,
    YAW_ACCEL_FF_CURRENT_PER_RPM_S,
    YAW_FEEDFORWARD_CURRENT_LIMIT
  },
  {
    PITCH_TARGET_RATE_FF_GAIN,
    PITCH_BASE_RATE_FF_GAIN,
    PITCH_GRAVITY_SIN_FF_CURRENT,
    PITCH_GRAVITY_COS_FF_CURRENT,
    PITCH_GRAVITY_BIAS_FF_CURRENT,
    PITCH_STATIC_FRICTION_FF_CURRENT,
    PITCH_VELOCITY_FF_CURRENT_PER_RPM,
    PITCH_ACCEL_FF_CURRENT_PER_RPM_S,
    PITCH_FEEDFORWARD_CURRENT_LIMIT
  }
};

/* ---- 模块级全局变量 ---- */

static CAN_HandleTypeDef *motor_can[GM6020_AXIS_COUNT];  /* 每轴独立 CAN 句柄 */
static GM6020_Controller_t controllers[GM6020_AXIS_COUNT]; /* 两轴控制器实例 */
static bool emergency_stop_latched;                     /* 串口锁存急停 */

/*====================================================================
 * 通用工具函数
 *====================================================================*/

/*
 * 检查轴枚举值是否有效（0 或 1）。
 * 返回 true 表示有效。
 */
static bool axis_is_valid(GM6020_Axis_t axis)
{
  return ((uint32_t)axis < (uint32_t)GM6020_AXIS_COUNT);
}

static bool controller_is_pitch(
    const GM6020_Controller_t *controller)
{
  return controller == &controllers[GM6020_AXIS_PITCH];
}

static GM6020_Axis_t controller_axis(
    const GM6020_Controller_t *controller)
{
  return controller_is_pitch(controller)
      ? GM6020_AXIS_PITCH
      : GM6020_AXIS_YAW;
}

static bool motor_can_is_initialized(void)
{
  return ((motor_can[GM6020_AXIS_YAW] != NULL)
          && (motor_can[GM6020_AXIS_PITCH] != NULL));
}

/*
 * 浮点数限幅：将 value 限制在 [minimum, maximum] 闭区间内。
 */
static float clamp_float(float value, float minimum, float maximum)
{
  if (value > maximum)
  {
    return maximum;
  }
  if (value < minimum)
  {
    return minimum;
  }
  return value;
}

/*
 * 校验单个轴的配置是否合法。
 * 检查项：CAN ID 是否为标准帧范围、电流槽位是否有效、
 * PID 限幅是否为正、零位偏移是否为有限浮点数、
 * 软限位开启时上下限是否合法。
 * 返回 true 表示配置有效。
 */
static bool axis_configuration_is_valid(GM6020_Axis_t axis)
{
  const GM6020_AxisCanConfig_t *can_config =
      &axis_can_config[axis];
  const GM6020_AxisPidConfig_t *pid_config =
      &axis_pid_config[axis];
  const GM6020_AxisMechanicalConfig_t *mechanical =
      &axis_mechanical_config[axis];
  const GM6020_AxisFeedforwardConfig_t *feedforward =
      &axis_feedforward_config[axis];

  if ((can_config->feedback_std_id > 0x7FFU)
      || (can_config->current_slot > 3U)
      || (can_config->filter_bank > 27U)
      || !(pid_config->speed_output_limit > 0.0f)
      || !(pid_config->angle_speed_limit_rpm > 0.0f)
      || !isfinite(mechanical->zero_offset_deg)
      || !isfinite(feedforward->target_rate_gain)
      || !isfinite(feedforward->base_rate_gain)
      || !isfinite(feedforward->gravity_sin_current)
      || !isfinite(feedforward->gravity_cos_current)
      || !isfinite(feedforward->gravity_bias_current)
      || !isfinite(feedforward->static_friction_current)
      || !isfinite(feedforward->velocity_current_per_rpm)
      || !isfinite(feedforward->acceleration_current_per_rpm_s)
      || !isfinite(feedforward->current_limit)
      || (feedforward->current_limit < 0.0f)
      || !(GM6020_POSITION_FF_ACCEL_LIMIT_RPM_S > 0.0f)
      || !(GM6020_FRICTION_TRANSITION_RPM > 0.0f))
  {
    return false;
  }

  if (mechanical->limit_enabled
      && (!isfinite(mechanical->minimum_angle_deg)
          || !isfinite(mechanical->maximum_angle_deg)
          || !(mechanical->minimum_angle_deg
               < mechanical->maximum_angle_deg)))
  {
    return false;
  }

  return true;
}

/*====================================================================
 * PID 重置函数
 *====================================================================*/

/*
 * 重置速度环 PID 状态（积分项、上一拍误差、输出均清零）。
 * 通常在状态切换或运行时修改 PID 增益后调用。
 */
static void speed_pid_reset(GM6020_Controller_t *controller)
{
  controller->speed_pid.integral = 0.0f;
  controller->speed_pid.previous_error = 0.0f;
  controller->speed_pid.output = 0.0f;
}

/*
 * 重置角度环 PID 状态。
 */
static void angle_pid_reset(GM6020_Controller_t *controller)
{
  controller->angle_pid.integral = 0.0f;
  controller->angle_pid.previous_error = 0.0f;
  controller->angle_pid.output = 0.0f;
}

static void position_feedforward_reset(
    GM6020_Controller_t *controller)
{
  controller->command_speed_feedforward_rpm = 0.0f;
  controller->command_accel_feedforward_rpm_s = 0.0f;
  controller->applied_speed_feedforward_rpm = 0.0f;
  controller->applied_current_feedforward = 0.0f;
}

/*====================================================================
 * 编码器角度转换函数
 *====================================================================*/

/*
 * 将任意浮点角度归一化到 [0, 360) 单圈范围内。
 * 例：-90° → 270°，450° → 90°。
 */
static float normalize_single_turn_degrees(float angle_deg)
/* ── 浮点角度归一化到 [0, 360°) ── */
{
  /* Normalize to single-turn range: subtract multiples of 360 */
  while (angle_deg >= 360.0f)
  {
    angle_deg -= 360.0f;
  }
  while (angle_deg < 0.0f)
  {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

/*
 * 将单圈角度（度）转换为编码器计数值。
 * 输入 0~360°，输出 0~8191，四舍五入取整。
 */
/* Convert degrees [0,360) → encoder counts [0,8191] with rounding */
static int32_t single_turn_degrees_to_encoder(float angle_deg)
{
  int32_t encoder;
  /* angle_deg × (8192/360) + 0.5 for round-to-nearest */

  angle_deg = normalize_single_turn_degrees(angle_deg);
  /* Edge case: rounding up to exactly 8192 → wrap to 0 */
  encoder = (int32_t)(angle_deg * (float)GM6020_ENCODER_CPR
                      / 360.0f + 0.5f);
  if (encoder >= GM6020_ENCODER_CPR)
  {
    encoder = 0;
  }
  return encoder;
}

/*====================================================================
 * 多圈编码器追踪 & 目标解析
 *====================================================================*/

/*
 * 以该轴当前多圈累计位置为基准，将单圈角度目标解析为多圈编码器目标。
 *
 * 【算法】
 *   目标单圈角度 → 目标编码器值 → 与当前编码器值做差 → 按劣弧方向选择
 *   最近路径（delta ∈ [-4096, 4096)）。将 delta 叠加到当前多圈累计值上，
 *   得到最终的多圈目标 total_angle_ecd。
 *
 * 【为什么需要多圈目标】
 *   角度环 PID 的输入是 total_angle_ecd 的差值。如果直接用单圈角度做差，
 *   在编码器零点（0 ↔ 8191）附近会出现 360° 跳变，导致电机全速转一整圈。
 *   多圈目标消除了这种歧义，电机始终沿最短路径转动。
 */
static void angle_resolve_single_turn_target(
  /* ── 步骤1: 目标角度 → 编码器值 ── */
    GM6020_Controller_t *controller,
    float angle_deg)
  /* ── 步骤2: 劣弧方向最短路径选择 ── */
{
  int32_t desired_encoder;
  int32_t delta;

  if (!controller->encoder_initialized)
  {
    return;
  }

  /* 将目标角度转为单圈编码器值 */
  desired_encoder = single_turn_degrees_to_encoder(angle_deg);

  /* 计算目标与当前编码器位置的差值 */
  delta = desired_encoder - (int32_t)controller->feedback.angle;

  /*
   * 劣弧方向选择：
   *   delta > +4096 → 目标在当前值的"上圈"，减一圈走更短路径
   *   delta < -4096 → 目标在当前值的"下圈"，加一圈走更短路径
   *   其他情况 → 同一个圈内，delta 即为最短路径
   */
  if (delta > GM6020_ENCODER_HALF_CPR)
  {
    delta -= GM6020_ENCODER_CPR;
  }
  else if (delta < -GM6020_ENCODER_HALF_CPR)
  {
  /* ── 步骤3: 叠加到当前多圈累计值 → 多圈目标 ── */
    delta += GM6020_ENCODER_CPR;
  }
  /* 叠加到当前多圈累计值，得到最终多圈目标 */
  controller->target_total_angle_ecd =
      controller->feedback.total_angle_ecd + delta;
}

/*
 * 将累计角度目标换算为相对于标定机械零点的多圈编码器目标。
 *
 * 与单圈目标不同，本函数不进行 0~360° 归一化，也不选择劣弧。
 * 例如 1080° 会固定解析为从标定零点正向转动 3 圈。
 */
static void angle_resolve_multi_turn_target(
    GM6020_Controller_t *controller,
    float accumulated_angle_deg)
{
  double target_encoder;

  if (!controller->encoder_initialized
      || !controller->multi_turn_origin_valid)
  {
    return;
  }

  target_encoder =
      (double)controller->multi_turn_origin_ecd
      + (double)accumulated_angle_deg
          * (double)GM6020_ENCODER_CPR / 360.0;

  if ((target_encoder > (double)INT32_MAX)
      || (target_encoder < (double)INT32_MIN))
  {
    return;
  }

  controller->target_total_angle_ecd =
      (int32_t)((target_encoder >= 0.0)
          ? (target_encoder + 0.5)
          : (target_encoder - 0.5));

}

/*
 * 读取当前编码器相对机械零位的多圈角度。
 * 该坐标与GM6020_SetTargetPosition()对外使用的逻辑角方向一致。
 */
static float controller_encoder_relative_position_deg(
    const GM6020_Controller_t *controller)
{
  if ((controller == NULL)
      || !controller->encoder_initialized
      || !controller->multi_turn_origin_valid)
  {
    return 0.0f;
  }

  return (float)(controller->feedback.total_angle_ecd
                 - controller->multi_turn_origin_ecd)
      * 360.0f / (float)GM6020_ENCODER_CPR;
}

/*
 * 把位置目标无冲击地重新锚定到指定的相对机械零位角。
 * 用于编码器/融合反馈切换和解除急停，防止继续执行旧目标。
 */
static void controller_hold_relative_position(
    GM6020_Controller_t *controller,
    float relative_position_deg)
{
  const float requested_encoder_angle_deg =
      normalize_single_turn_degrees(
          relative_position_deg + controller->zero_offset_deg);

  controller->requested_angle_deg =
      requested_encoder_angle_deg;
  controller->requested_angle_is_multi_turn = false;
  controller->position_target_valid = false;
  angle_resolve_single_turn_target(
      controller,
      requested_encoder_angle_deg);
  angle_pid_reset(controller);
}

/*
 * 返回与当前累计位置距离最近的“标定机械零点”累计编码器值。
 * zero_offset_deg 表示逻辑 0° 对应的单圈编码器位置。
 */
/*
 * 计算距离标定零位最近的编码器多圈原点。
 *
 * 标定后 zero_offset_deg 存的是云台中位对应的编码器单圈角度，
 * 此函数以当前多圈位置为基准，找到距离该标定零位最近的
 * 编码器多圈原点 (multi-turn origin ECD)，使 multi_turn_origin_ecd
 * 在物理上最接近标定后的机械零位。
 *
 * @return 距离标定零位最近的多圈编码器原点值
 */
static int32_t encoder_nearest_calibrated_zero(
    const GM6020_Controller_t *controller,
    int32_t current_total_ecd,
    uint16_t current_single_turn_ecd)
{
  const float normalized_zero =
      normalize_single_turn_degrees(controller->zero_offset_deg);
  uint32_t zero_ecd = (uint32_t)(
      normalized_zero * (float)GM6020_ENCODER_CPR / 360.0f
      + 0.5f);
  int32_t delta;

  if (zero_ecd >= GM6020_ENCODER_CPR)
  {
    zero_ecd = 0U;
  }

  delta = (int32_t)zero_ecd - (int32_t)current_single_turn_ecd;
  if (delta > (int32_t)GM6020_ENCODER_HALF_CPR)
  {
    delta -= (int32_t)GM6020_ENCODER_CPR;
  }
  else if (delta < -(int32_t)GM6020_ENCODER_HALF_CPR)
  {
    delta += (int32_t)GM6020_ENCODER_CPR;
  }

  return current_total_ecd + delta;
}

/*
 * 增量式编码器多圈累计更新。
 *
 * 【算法】
 *   比较当前编码器值与上一拍值，检测跨零点：
 *     delta > +4096 → 编码器由低值跳到高值，圈数 -1
 *     delta < -4096 → 编码器由高值跳到低值，圈数 +1
 *   多圈总位置 = turn_count × 8192 + 当前编码器值
 *
 * 【注意】
 *   此函数每个控制周期调用一次。只要控制周期内电机转动小于半圈，
 *   就能正确判断跨零方向。按 1 ms 周期计算，半圈对应
 *   4096 counts/ms = 0.5 rev/ms = 30000 RPM。
 *   就不会丢失圈数。GM6020 最大转速远低于此值，不存在溢出风险。
 */
static void encoder_update(GM6020_Controller_t *controller,
                           uint16_t encoder)
{
  int32_t delta;

  /*
   * 首次调用：建立累计角度，并把距离当前位置最近的标定机械零点
   * 作为多圈坐标原点。
   */
  if (!controller->encoder_initialized)
  {
    controller->previous_encoder = encoder;
    controller->feedback.turn_count = 0;
    controller->feedback.total_angle_ecd = encoder;
    controller->feedback.total_angle_deg =
        (float)encoder * 360.0f / (float)GM6020_ENCODER_CPR;
    controller->multi_turn_origin_ecd =
        encoder_nearest_calibrated_zero(
            controller,
            controller->feedback.total_angle_ecd,
            encoder);
    controller->multi_turn_origin_valid = true;
    controller->encoder_initialized = true;
    return;
  }

  /* 计算与上一拍的差值 */
  delta = (int32_t)encoder - (int32_t)controller->previous_encoder;

  /* 跨零点检测 */
  if (delta > GM6020_ENCODER_HALF_CPR)
  {
    /* 低值跳到高值：实际沿编码器负方向跨过零点 */
    controller->feedback.turn_count--;
  }
  else if (delta < -GM6020_ENCODER_HALF_CPR)
  {
    /* 高值跳到低值：实际沿编码器正方向跨过零点 */
    controller->feedback.turn_count++;
  }

  /* 保存当前值供下一拍比较 */
  controller->previous_encoder = encoder;

  /* 更新多圈累计值 */
  controller->feedback.total_angle_ecd =
      controller->feedback.turn_count * GM6020_ENCODER_CPR
      + (int32_t)encoder;
  controller->feedback.total_angle_deg =
      (float)controller->feedback.total_angle_ecd * 360.0f
      / (float)GM6020_ENCODER_CPR;
}

/*====================================================================
 * 前馈与PID更新函数（含总输出anti-windup）
 *====================================================================*/

static float position_speed_feedforward_rpm(
    GM6020_Controller_t *controller)
{
  const GM6020_Axis_t axis = controller_axis(controller);
  const GM6020_AxisFeedforwardConfig_t *config =
      &axis_feedforward_config[axis];
  float feedforward_rpm =
      config->target_rate_gain
      * controller->command_speed_feedforward_rpm;

  /*
   * 当前BMI088融合器只估计Pitch底座扰动：
   * base_rate = inertial_rate - relative_encoder_rate。
   * 为保持惯性角，电机相对速度前馈取其反向。
   */
  if ((axis == GM6020_AXIS_PITCH)
      && (config->base_rate_gain != 0.0f))
  {
    const PitchFusionData_t *fusion = PitchFusion_GetData();

    if ((fusion != NULL)
        && (fusion->status == PITCH_FUSION_RUNNING)
        && fusion->imu_valid
        && fusion->motor_valid
        && isfinite(fusion->base_disturbance_rate_dps))
    {
      feedforward_rpm -=
          config->base_rate_gain
          * fusion->base_disturbance_rate_dps
          / GM6020_RPM_TO_DPS;
    }
  }

  controller->applied_speed_feedforward_rpm =
      feedforward_rpm;
  return feedforward_rpm;
}

static float model_current_feedforward(
    GM6020_Controller_t *controller,
    float feedback_position_deg)
{
  const GM6020_Axis_t axis = controller_axis(controller);
  const GM6020_AxisFeedforwardConfig_t *config =
      &axis_feedforward_config[axis];
  const float motion_speed_rpm =
      controller->applied_speed_feedforward_rpm;
  const float position_rad =
      feedback_position_deg * GM6020_DEG_TO_RAD;
  const float friction_blend =
      motion_speed_rpm
      / (fabsf(motion_speed_rpm)
         + GM6020_FRICTION_TRANSITION_RPM);
  float current =
      config->gravity_sin_current * sinf(position_rad)
      + config->gravity_cos_current * cosf(position_rad)
      + config->gravity_bias_current
      + config->static_friction_current * friction_blend
      + config->velocity_current_per_rpm * motion_speed_rpm
      + config->acceleration_current_per_rpm_s
        * controller->command_accel_feedforward_rpm_s;

  current = clamp_float(
      current,
      -config->current_limit,
      config->current_limit);
  controller->applied_current_feedforward = current;
  return current;
}

/*
 * 角度环 PID 更新。
 *
 * 【输入】目标多圈编码器值 vs 当前多圈编码器值
 * 【输出】速度环目标转速（rpm），限幅在 ±angle_speed_limit_rpm
 *
 * 【Anti-windup 逻辑】
 *   传统 PID 在输出饱和时积分项会无限制累积（积分饱和/积分 windup），
 *   导致目标反向时需要长时间"退饱和"才能响应。
 *   本实现的 anti-windup：
 *     - 积分候选值（candidate_integral）本身先被限幅到 ±output_limit
 *     - 只有当 (candidate_output 未饱和) 或 (饱和但误差方向有助于退饱和)
 *       时才更新积分项
 *     - 换言之：输出已达上限且误差仍为正时，不再累积正向积分；
 *       输出已达下限且误差仍为负时，不再累积负向积分
 *
 * 【返回值】PID 输出值 = 比例项 + 微分项 + 积分项，限幅后的目标转速 rpm
 */
static float angle_pid_update(
    GM6020_Controller_t *controller,
    float feedback_position_deg,
    float speed_feedforward_rpm)
{
  AnglePID_t *pid = &controller->angle_pid;
  float output_limit = controller->angle_speed_limit_rpm;
  float target_position_deg;

  if (controller->pitch_fusion_feedback_active
      && (output_limit
          > PITCH_FUSION_CONTROL_MAX_SPEED_RPM))
  {
    output_limit = PITCH_FUSION_CONTROL_MAX_SPEED_RPM;
  }

  /*
   * ── 步骤1: 计算角度误差 (Position Error) ──
   * 误差 = 目标多圈ecd - 当前多圈ecd，转换为度。
   * error > 0 → 需要正向转动才能追上目标
   * error < 0 → 需要反向转动
   */
  target_position_deg =
      (float)(controller->target_total_angle_ecd
              - controller->multi_turn_origin_ecd)
      * 360.0f / (float)GM6020_ENCODER_CPR;
  const float error =
      target_position_deg - feedback_position_deg;

  /* ── 步骤2: 计算微分项 (Derivative Term) ── */
  /* D_term = d(error)/dt ≈ Δerror / Δt, Δt = 1/1000 s = 0.001 s */
  const float derivative =
      (error - pid->previous_error) / GM6020_CONTROL_PERIOD_S;  /* Δerror / 0.001s */

  /* ── 步骤3: 计算 P+D 项 (用于 anti-windup 判断) ── */
  const float proportional_and_derivative =
      pid->kp * error
      + pid->kd * derivative
      + speed_feedforward_rpm;

  /* ── 步骤4: 计算积分候选值 (Integral Candidate) ── */
  /* I_candidate = old_I + Ki × error × Δt, 预先限幅到 ±output_limit 防止绝对值过大 */
  const float candidate_integral = clamp_float(
      pid->integral + pid->ki * error * GM6020_CONTROL_PERIOD_S,  /* I += Ki × error × 0.001 */
      -output_limit,
      output_limit);

  /* ── 步骤5: 计算完整候选输出 (Candidate Output) ── */
  const float candidate_output =
      proportional_and_derivative + candidate_integral;  /* output = P + D + I_candidate */

  /* ── 步骤6: Anti-windup 积分更新决策 ── */
  /*
   * 条件A: candidate_output 在 (-limit, +limit) 内 → PID 未饱和 → 正常更新积分
   * 条件B: output ≥ +limit 且 error < 0 → 已达到正向饱和但误差已反向 → 退饱和 → 更新积分
   * 条件C: output ≤ -limit 且 error > 0 → 已达到负向饱和但误差已反向 → 退饱和 → 更新积分
   * 否则: 输出饱和且误差方向仍驱使继续饱和 → 冻结积分 (积分抗饱和/积分分离)
   */
  if (((candidate_output < output_limit)
       && (candidate_output > -output_limit))
      || ((candidate_output >= output_limit)
          && (error < 0.0f))
      || ((candidate_output <= -output_limit)
          && (error > 0.0f)))
  {
    pid->integral = candidate_integral;  /* 接受新的积分值 — accept new integral */
  }
  /* else: 积分保持旧值不变 — integral frozen (anti-windup active) */

  /* ── 步骤7: 计算最终输出并限幅 ── */
  pid->output = proportional_and_derivative + pid->integral;  /* P + D + I (maybe old I) */
  pid->output = clamp_float(pid->output,
                            -output_limit,
                            output_limit);  /* 硬限幅: output ∈ [−limit, +limit] */

  /* ── 步骤8: 保存本拍误差供下一拍微分 ── */
  pid->previous_error = error;  /* prev_error ← current_error for next derivative calc */
  return pid->output;           /* 返回目标转速 rpm → speed_pid_update() 的目标输入 */
}

/*
 * 速度环 PID 更新。
 *
 * 【输入】
 *   target:  目标转速（rpm），正值为正转
 *   feedback: 当前反馈转速（rpm）
 * 【输出】转矩电流命令（float，后续截断为 int16_t）
 *
 * 【Anti-windup 逻辑】与角度环相同，参见 angle_pid_update() 的注释。
 */
static float speed_pid_update(GM6020_Controller_t *controller,
                              float target,
                              float feedback,
                              float current_feedforward)
{
  SpeedPID_t *pid = &controller->speed_pid;
  float output_limit = controller->speed_output_limit;

  if (controller->pitch_fusion_feedback_active
      && (output_limit
          > PITCH_FUSION_CONTROL_MAX_CURRENT))
  {
    output_limit = PITCH_FUSION_CONTROL_MAX_CURRENT;
  }

  /* ── 步骤1: 速度误差 (Speed Error) ── */
  /* error > 0 → 实际转速低于目标 → 需要增加正向电流驱动 */
  const float error = target - feedback;  /* target_rpm - actual_rpm */

  /* ── 步骤2: 微分项 = Δerror / Δt ── */
  const float derivative =
      (error - pid->previous_error) / GM6020_CONTROL_PERIOD_S;

  /* ── 步骤3: P+D 项 ── */
  const float proportional_and_derivative =
      pid->kp * error
      + pid->kd * derivative
      + current_feedforward;

  /* ── 步骤4: 积分候选值 (先限幅) ── */
  const float candidate_integral = clamp_float(
      pid->integral + pid->ki * error * GM6020_CONTROL_PERIOD_S,  /* I += Ki×error×Δt */
      -output_limit,
      output_limit);

  /* ── 步骤5: 完整候选输出 ── */
  const float candidate_output =
      proportional_and_derivative + candidate_integral;

  /* ── 步骤6: Anti-windup ── */
  /* 与 angle_pid_update 相同的 anti-windup 逻辑（参见其详细注释） */
  if (((candidate_output < output_limit)
       && (candidate_output > -output_limit))
      || ((candidate_output >= output_limit)
          && (error < 0.0f))
      || ((candidate_output <= -output_limit)
          && (error > 0.0f)))
  {
    pid->integral = candidate_integral;
  }

  /* ── 步骤7: 最终输出 + 硬限幅 ── */
  pid->output = proportional_and_derivative + pid->integral;
  pid->output = clamp_float(pid->output,
                            -output_limit,
                            output_limit);  /* 最终电流值限幅到 ±8192 */

  /* ── 步骤8: 保存误差 ── */
  pid->previous_error = error;
  return pid->output;  /* 返回 float 电流值 → 调用方截断为 int16_t */
}

/*====================================================================
 * CAN 通信函数
 *====================================================================*/

/*
 * 在指定轴的 CAN 总线上发送电流命令帧 0x1FE。
 *
 * 【帧格式】
 *   StdId = 0x1FE, DLC = 8, 数据帧
 *   DATA[2:3] = 当前轴 ID 2 电流（大端，slot 1）
 *   其他字节 = 0
 *
 * 【返回值】HAL_CAN_AddTxMessage 的状态。
 */
static HAL_StatusTypeDef gm6020_send_axis_current(GM6020_Axis_t axis)
/*
 * 发送单轴转矩电流命令到 0x1FE 帧。
 * 每个轴可能挂在不同的 CAN 总线上 (motor_can[axis])，
 * 因此需要按轴独立发送。电流值按 Big-Endian int16 打包
 * 到对应电流槽位 (slot 0 → offset 0, slot 1 → offset 2)。
 */
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  uint32_t mailbox;
  int16_t current_command;
  uint16_t raw;
  uint32_t offset;

  if (!axis_is_valid(axis) || (motor_can[axis] == NULL))
  {
    return HAL_ERROR;
  }
  /* Construct CAN header: StdId=0x1FE, 8-byte data frame */

  tx_header.StdId = GM6020_CURRENT_COMMAND_ID;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  current_command = controllers[axis].current_command;

  /* Pack int16 current as Big-Endian into the correct slot offset */
#if GIMBAL_YAW_ONLY_TEST_MODE
  if (axis == GM6020_AXIS_PITCH)
  {
    current_command = 0;
  }
  /* Submit frame to the CAN TX mailbox of this axis's bus */
#endif

  raw = (uint16_t)current_command;
  offset = (uint32_t)axis_can_config[axis].current_slot * 2U;
  tx_data[offset] = (uint8_t)(raw >> 8);
  tx_data[offset + 1U] = (uint8_t)raw;

  return HAL_CAN_AddTxMessage(
      motor_can[axis], &tx_header, tx_data, &mailbox);
}

static HAL_StatusTypeDef gm6020_send_all_currents(void)
{
  HAL_StatusTypeDef result = HAL_OK;
  uint32_t axis_index;

  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    HAL_StatusTypeDef status = gm6020_send_axis_current(
        (GM6020_Axis_t)axis_index);
    if ((status != HAL_OK) && (result == HAL_OK))
    {
      result = status;
    }
  }

  return result;
}

/*====================================================================
 * 状态机：每个状态的反馈处理函数
 *====================================================================*/

/*
 * 状态机反馈处理函数指针类型。
 * 每个状态有自己的 on_feedback 处理函数，
 * 在收到有效 CAN 反馈帧后被 control_state_handle_feedback() 调用。
 */
typedef void (*ControlStateFeedbackHandler_t)(
    GM6020_Controller_t *controller);

/*
 * WAIT_FEEDBACK / FAULT 状态下的反馈处理：
 *   保持零电流输出。在收到首帧反馈时由
 *   control_state_handle_feedback() 负责转移到下一个状态。
 */
static void state_zero_current_on_feedback(
    GM6020_Controller_t *controller)
/* WAIT_FEEDBACK / FAULT: 保持零电流输出，等待状态机自动转移到下一状态 */
/* Keep zero current; state transition handled by control_state_handle_feedback() */
{
  controller->current_command = 0;
}

/*
 * POSITION_CONTROL 状态下的反馈处理：
 *   1. 角度环 PID：位置误差 → 速度环目标
 *   2. 速度环 PID：速度误差 → 转矩电流命令
 */
static void state_position_control_on_feedback(
/* POSITION_CONTROL: 外环角度PID → 目标转速 → 内环速度PID → 电流命令 */
/* Outer loop (angle) → target rpm → inner loop (speed) → torque current */
    GM6020_Controller_t *controller)
{
  const float encoder_position_deg =
      controller_encoder_relative_position_deg(controller);
  float feedback_position_deg = encoder_position_deg;
  float feedback_speed_rpm =
      (float)controller->feedback.speed_rpm;
  float speed_feedforward_rpm;
  float current_feedforward;
  bool fusion_available = false;

  if (controller_is_pitch(controller))
  {
    fusion_available = PitchFusion_GetControlFeedback(
        &feedback_position_deg,
        &feedback_speed_rpm);

    if (fusion_available
        != controller->pitch_fusion_feedback_active)
    {
      /*
       * 反馈源变化时丢弃旧运动目标，并锚定到新反馈的当前位置。
       * 同时清空两级PID，避免误差参考系变化造成电流阶跃。
       */
      controller->pitch_fusion_feedback_active =
          fusion_available;
      controller_hold_relative_position(
          controller,
          feedback_position_deg);
      position_feedforward_reset(controller);
      speed_pid_reset(controller);
    }
  }

  /*
   * 机械软限位必须以电机编码器为准，不能使用可能带有惯性补偿偏差的
   * 融合角判断端点。当前位置落在限位外（或恰好位于边界）时，临时
   * 使用编码器角度和电机转速闭环，使限位内的目标必然产生向内恢复
   * 的速度方向；回到正常范围后自动恢复融合反馈。
   */
  if (controller->angle_limit_enabled
      && ((encoder_position_deg
           <= controller->minimum_angle_deg)
          || (encoder_position_deg
              >= controller->maximum_angle_deg)))
  {
    feedback_position_deg = encoder_position_deg;
    feedback_speed_rpm =
        (float)controller->feedback.speed_rpm;
  }

  speed_feedforward_rpm =
      position_speed_feedforward_rpm(controller);

  /* 外环：位置 → 目标转速 */
  controller->target_speed_rpm = angle_pid_update(
      controller,
      feedback_position_deg,
      speed_feedforward_rpm);

  /*
   * 最后一层方向保护：越过下限只允许正转恢复，越过上限只允许反转
   * 恢复。这样即使目标速度前馈或PID积分残留异常，也不会继续顶向
   * 机械端点。被拦截时清除角度环积分，避免解除限位后出现反向冲击。
   */
  if (controller->angle_limit_enabled
      && (((encoder_position_deg
            <= controller->minimum_angle_deg)
           && (controller->target_speed_rpm < 0.0f))
          || ((encoder_position_deg
               >= controller->maximum_angle_deg)
              && (controller->target_speed_rpm > 0.0f))))
  {
    controller->target_speed_rpm = 0.0f;
    controller->angle_pid.integral = 0.0f;
    controller->angle_pid.output = 0.0f;
  }

  current_feedforward = model_current_feedforward(
      controller,
      feedback_position_deg);

  /* 内环：目标转速 vs 实际转速 → 电流命令 */
  controller->current_command = (int16_t)speed_pid_update(
      controller,
      controller->target_speed_rpm,
      feedback_speed_rpm,
      current_feedforward);

  /*
   * 电流命令也执行同样的单向保护。速度环在制动或保留积分时可能给出
   * 与目标速度不同号的瞬时电流，因此不能只检查角度环输出。
   */
  if (controller->angle_limit_enabled
      && (((encoder_position_deg
            <= controller->minimum_angle_deg)
           && (controller->current_command < 0))
          || ((encoder_position_deg
               >= controller->maximum_angle_deg)
              && (controller->current_command > 0))))
  {
    controller->current_command = 0;
    speed_pid_reset(controller);
  }
}

/*
 * SPEED_DEBUG 状态下的反馈处理：
 *   绕过角度环，直接用人工设定的 debug_target_speed_rpm 驱动速度环。
 *   用于整定速度环 PID 参数或测试电机响应。
 */
static void state_speed_debug_on_feedback(
/* SPEED_DEBUG: 绕过角度环，直接以人工设定的RPM驱动速度环 */
/* Bypass angle loop; drive speed loop directly with debug_target_speed_rpm */
    GM6020_Controller_t *controller)
{
  controller->target_speed_rpm =
      controller->debug_target_speed_rpm;
  controller->current_command = (int16_t)speed_pid_update(
      controller,
      controller->target_speed_rpm,
/* 状态→反馈处理函数映射表，索引与 GM6020_ControlState_t 枚举对齐 */
      (float)controller->feedback.speed_rpm,
      0.0f);
}

/* 状态 → 反馈处理函数映射表，索引与 GM6020_ControlState_t 枚举对齐 */
static const ControlStateFeedbackHandler_t state_handlers[
    GM6020_STATE_COUNT] =
{
  state_zero_current_on_feedback,       /* WAIT_FEEDBACK */
  state_position_control_on_feedback,   /* POSITION_CONTROL */
  state_speed_debug_on_feedback,        /* SPEED_DEBUG */
  state_zero_current_on_feedback        /* FAULT */
};

/*====================================================================
 * 状态机：状态转移 & 超时检测
 *====================================================================*/

/*
 * 执行状态转移。
 *
 * 【行为】
 *   1. 切换到目标状态
 *   2. 清零电流输出（新状态需要重新计算）
 *   3. 根据新状态重置 PID 和初始化相关变量：
 *      - WAIT_FEEDBACK: 等待电机响应前保持零电流
 *      - POSITION_CONTROL: 有位置目标则解析，否则锁定当前位置
 *      - SPEED_DEBUG: 应用人工设定的目标转速
 *      - FAULT: 标记离线，清除编码器初始化和旧位置目标，等待反馈恢复
 */
  /* Transition to new state: reset current, reset PIDs, init state-specific variables */
static void control_state_transition(
    GM6020_Controller_t *controller,
    GM6020_ControlState_t next_state)
{
  controller->state = next_state;
  controller->current_command = 0;
  position_feedforward_reset(controller);

  switch (controller->state)
  {
    case GM6020_STATE_WAIT_FEEDBACK:
      controller->target_speed_rpm = 0.0f;
      controller->pitch_fusion_feedback_active = false;
      speed_pid_reset(controller);
      angle_pid_reset(controller);
      break;

    case GM6020_STATE_POSITION_CONTROL:
      controller->target_speed_rpm = 0.0f;
      speed_pid_reset(controller);
      angle_pid_reset(controller);
      if (controller->position_target_valid)
      {
        if (controller->requested_angle_is_multi_turn)
        {
          angle_resolve_multi_turn_target(
              controller,
              controller->requested_angle_deg);
        }
        else
        {
          angle_resolve_single_turn_target(
              controller,
              controller->requested_angle_deg);
        }
      }
      else
      {
        float hold_position_deg =
            controller_encoder_relative_position_deg(controller);
        float hold_speed_rpm;

        /*
         * 没有外部位置命令时，锁定当前反馈位置。
         * Pitch融合有效时锁定当前惯性Pitch，其他情况锁定编码器位置。
         */
        controller->pitch_fusion_feedback_active =
            controller_is_pitch(controller)
            && PitchFusion_GetControlFeedback(
                &hold_position_deg,
                &hold_speed_rpm);
        controller_hold_relative_position(
            controller,
            hold_position_deg);
      }
      break;

    case GM6020_STATE_SPEED_DEBUG:
      controller->pitch_fusion_feedback_active = false;
      controller->target_speed_rpm =
          controller->debug_target_speed_rpm;
      speed_pid_reset(controller);
      angle_pid_reset(controller);
      break;

    case GM6020_STATE_FAULT:
    default:
      controller->state = GM6020_STATE_FAULT;
      controller->feedback.online = false;
      controller->encoder_initialized = false;
      controller->pitch_fusion_feedback_active = false;
      controller->position_target_valid = false;
      controller->target_speed_rpm = 0.0f;
      speed_pid_reset(controller);
      angle_pid_reset(controller);
      break;
  }
}

/*
 * 收到有效反馈帧后的统一入口。
 *
 * 【行为】
 *   - WAIT_FEEDBACK / FAULT: 自动转移到下一个状态
 *     （speed_debug_requested ? SPEED_DEBUG : POSITION_CONTROL）
 *   - POSITION_CONTROL / SPEED_DEBUG: 保持当前状态不变
 *   - 然后调用当前状态对应的反馈处理函数
 */
static void control_state_handle_feedback(
    GM6020_Controller_t *controller)
{
  switch (controller->state)
  {
    case GM6020_STATE_WAIT_FEEDBACK:
    case GM6020_STATE_FAULT:
      /*
       * 从 WAIT_FEEDBACK 或 FAULT 恢复：
       *   如果是上电就请求了速度调试 → 进入 SPEED_DEBUG
       *   否则 → 进入 POSITION_CONTROL
       */
      control_state_transition(
          controller,
          controller->speed_debug_requested
          ? GM6020_STATE_SPEED_DEBUG
          : GM6020_STATE_POSITION_CONTROL);
      break;

    case GM6020_STATE_POSITION_CONTROL:
    case GM6020_STATE_SPEED_DEBUG:
      /* 正常状态，无需转移 */
      break;

    default:
      /* 未知状态 → 进入 FAULT 保护 */
      control_state_transition(controller, GM6020_STATE_FAULT);
      return;
  }

  /* 调用当前状态的反馈处理函数（计算 PID、更新电流命令） */
  state_handlers[controller->state](controller);
}

/*
 * 超时检测。
 *
 * 在 POSITION_CONTROL 或 SPEED_DEBUG 状态下，
 * 如果距上一次收到反馈帧的时间超过 GM6020_FEEDBACK_TIMEOUT_MS，
 * 则认为电机离线，转入 FAULT 状态。
 *
 * 【返回值】true 表示发生了超时转移。
 */
static bool control_state_poll_timeout(
    GM6020_Controller_t *controller,
    uint32_t now)
{
  switch (controller->state)
  {
    case GM6020_STATE_POSITION_CONTROL:
    case GM6020_STATE_SPEED_DEBUG:
      if ((uint32_t)(now - controller->feedback.last_rx_ms)
          > GM6020_FEEDBACK_TIMEOUT_MS)
      {
        control_state_transition(controller, GM6020_STATE_FAULT);
        return true;
      }
      break;

    case GM6020_STATE_WAIT_FEEDBACK:
    case GM6020_STATE_FAULT:
    default:
      /* 等待状态和故障状态不检测超时 */
      break;
  }
  return false;
}

/*====================================================================
 * 控制器初始化
 *====================================================================*/

/*
 * 初始化单个轴的控制器。
 *
 * 从全局配置表（axis_pid_config、axis_mechanical_config）读取参数，
 * 清零所有运行时状态，设置为 WAIT_FEEDBACK 状态。
 */
static void controller_initialize(
    GM6020_Controller_t *controller,
    GM6020_Axis_t axis)
{
  const GM6020_AxisPidConfig_t *config = &axis_pid_config[axis];
  const GM6020_AxisMechanicalConfig_t *mechanical =
      &axis_mechanical_config[axis];

  /* 清零整个结构体 */
  memset(controller, 0, sizeof(*controller));

  /* 从配置表载入 PID 参数 */
  controller->speed_pid.kp = config->speed_kp;
  controller->speed_pid.ki = config->speed_ki;
  controller->speed_pid.kd = config->speed_kd;
  controller->speed_output_limit = config->speed_output_limit;
  controller->angle_pid.kp = config->angle_kp;
  controller->angle_pid.ki = config->angle_ki;
  controller->angle_pid.kd = config->angle_kd;
  controller->angle_speed_limit_rpm =
      config->angle_speed_limit_rpm;

  /* 从配置表载入机械参数 */
  controller->zero_offset_deg = mechanical->zero_offset_deg;
  controller->minimum_angle_deg = mechanical->minimum_angle_deg;
  controller->maximum_angle_deg = mechanical->maximum_angle_deg;
  controller->angle_limit_enabled = mechanical->limit_enabled;

  /*
   * 上电不主动运动。首次收到反馈并进入位置控制时，因为尚无外部
   * 位置目标，状态机会锁定反馈中的当前位置。这样装机标定前不会
   * 自动转向编码器原始零点。
   */
  controller->requested_angle_deg = 0.0f;
  controller->requested_angle_is_multi_turn = false;
  controller->position_target_valid = false;

  /* 初始状态：等待电机反馈 */
  controller->state = GM6020_STATE_WAIT_FEEDBACK;
}

/*
 * 配置 CAN 硬件滤波器。
 *
 * 【模式】ID 掩码模式（CAN_FILTERMODE_IDMASK），32 位尺度。
 *   CAN1 过滤器组 0 接收 Yaw 0x206，CAN2 过滤器组 14 接收 Pitch 0x206，
 *   两者均路由到各自的 FIFO0。
 *
 * 【参数】轴枚举决定 CAN 句柄、过滤器组和反馈 ID。
 */
static HAL_StatusTypeDef configure_feedback_filter(GM6020_Axis_t axis)
{
  CAN_FilterTypeDef filter = {0};
  const GM6020_AxisCanConfig_t *config;

  if (!axis_is_valid(axis) || (motor_can[axis] == NULL))
  {
    return HAL_ERROR;
  }

  config = &axis_can_config[axis];

  filter.FilterBank = config->filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  /*
   * 标准 ID 左移 5 位存入 FilterIdHigh（CAN 规范中标准 ID 在 32 位
   * 寄存器中的位偏移为 21，对应高 16 位的 [15:5]）。
   */
  filter.FilterIdHigh = (uint16_t)(config->feedback_std_id << 5);
  filter.FilterIdLow = 0U;
  /* 掩码匹配所有 11 位标准 ID */
  filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  return HAL_CAN_ConfigFilter(motor_can[axis], &filter);
}

static HAL_StatusTypeDef start_can_if_needed(CAN_HandleTypeDef *hcan)
{
  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  if (hcan->State == HAL_CAN_STATE_READY)
  {
    return HAL_CAN_Start(hcan);
  }

  return (hcan->State == HAL_CAN_STATE_LISTENING) ? HAL_OK : HAL_ERROR;
}

/*====================================================================
 * 公有 API
 *====================================================================*/

/*
 * 初始化两轴 GM6020 电机控制器。
 *
 * 【执行流程】
 *   1. 校验 Yaw=CAN1、Pitch=CAN2 的句柄映射
 *   2. 遍历两轴：校验配置、初始化控制器、配置独立 CAN 滤波器
 *   3. 启动 CAN1 和 CAN2
 *   4. 在两条总线上分别发送初始 0x1FE 零电流命令
 *
 * 【参数】
 *   yaw_can: CAN1 外设句柄；pitch_can: CAN2 外设句柄
 * 【返回值】HAL_OK 表示初始化成功。
 */
HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *yaw_can,
                              CAN_HandleTypeDef *pitch_can)
{
  HAL_StatusTypeDef status;
  uint32_t axis_index;

  if ((yaw_can == NULL)
      || (pitch_can == NULL)
      || (yaw_can == pitch_can)
      || (yaw_can->Instance != CAN1)
      || (pitch_can->Instance != CAN2))
  {
    return HAL_ERROR;
  }

  motor_can[GM6020_AXIS_YAW] = yaw_can;
  motor_can[GM6020_AXIS_PITCH] = pitch_can;
  emergency_stop_latched = false;

  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    if (!axis_configuration_is_valid(
            (GM6020_Axis_t)axis_index))
    {
      return HAL_ERROR;
    }

    controller_initialize(
        &controllers[axis_index], (GM6020_Axis_t)axis_index);

    status = configure_feedback_filter((GM6020_Axis_t)axis_index);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    status = start_can_if_needed(motor_can[axis_index]);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  /*
   * 两轴收到有效反馈前保持零电流。
   * GM6020 在收到第一个非零 0x1FE 帧之前不会发送反馈，
   * 但发送电流为零的帧是安全的（告知电机初始状态）。
   */
  return gm6020_send_all_currents();
}

HAL_StatusTypeDef GM6020_EmergencyStop(void)
{
  uint32_t axis_index;

  emergency_stop_latched = true;
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    GM6020_Controller_t *controller = &controllers[axis_index];

    controller->current_command = 0;
    controller->target_speed_rpm = 0.0f;
    controller->debug_target_speed_rpm = 0.0f;
    controller->speed_debug_requested = false;
    controller->position_target_valid = false;
    controller->requested_angle_is_multi_turn = false;
    position_feedforward_reset(controller);
    speed_pid_reset(controller);
    angle_pid_reset(controller);
  }

  if (!motor_can_is_initialized())
  {
    return HAL_ERROR;
  }

  return gm6020_send_all_currents();
}

void GM6020_ClearEmergencyStop(void)
{
  uint32_t axis_index;

  emergency_stop_latched = false;
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    GM6020_Controller_t *controller = &controllers[axis_index];

    controller->position_target_valid = false;
    controller->requested_angle_is_multi_turn = false;
    controller->speed_debug_requested = false;
    controller->debug_target_speed_rpm = 0.0f;

    if (controller->encoder_initialized && controller->feedback.online)
    {
      control_state_transition(
          controller, GM6020_STATE_POSITION_CONTROL);
    }
    else
    {
      control_state_transition(
          controller, GM6020_STATE_WAIT_FEEDBACK);
    }
  }
}

bool GM6020_IsEmergencyStopped(void)
{
  return emergency_stop_latched;
}

static bool zero_offset_can_be_applied(
    const GM6020_Controller_t *controller)
{
  if (controller == NULL)
  {
    return false;
  }

  return !controller->encoder_initialized
      || (controller->feedback.online
          && (controller->feedback.speed_rpm
              <= GM6020_ZERO_SET_MAX_SPEED_RPM)
          && (controller->feedback.speed_rpm
              >= -GM6020_ZERO_SET_MAX_SPEED_RPM));
}

static void apply_zero_offset_ecd(
    GM6020_Controller_t *controller,
    uint16_t zero_ecd)
{
  controller->zero_offset_deg =
      (float)zero_ecd * 360.0f
      / (float)GM6020_ENCODER_CPR;
  controller->position_target_valid = false;
  controller->requested_angle_deg = 0.0f;
  controller->requested_angle_is_multi_turn = false;
  controller->pitch_fusion_feedback_active = false;
  controller->target_speed_rpm = 0.0f;
  controller->current_command = 0;
  position_feedforward_reset(controller);
  speed_pid_reset(controller);
  angle_pid_reset(controller);

  if (controller->encoder_initialized)
  {
    controller->multi_turn_origin_ecd =
        encoder_nearest_calibrated_zero(
            controller,
            controller->feedback.total_angle_ecd,
            controller->feedback.angle);
    controller->multi_turn_origin_valid = true;
    controller->target_total_angle_ecd =
        controller->feedback.total_angle_ecd;
  }
  else
  {
    controller->multi_turn_origin_valid = false;
  }
}

bool GM6020_SetZeroOffsetsEcd(uint16_t yaw_zero_ecd,
                             uint16_t pitch_zero_ecd)
{
  const uint16_t zero_ecd[GM6020_AXIS_COUNT] =
  {
    yaw_zero_ecd,
    pitch_zero_ecd
  };
  uint32_t axis_index;

  if (!motor_can_is_initialized()
      || (yaw_zero_ecd >= GM6020_ENCODER_CPR)
      || (pitch_zero_ecd >= GM6020_ENCODER_CPR))
  {
    return false;
  }

  /* 已收到反馈时，只允许在两轴在线且静止的条件下更新坐标零点。 */
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    const GM6020_Controller_t *controller =
        &controllers[axis_index];

    if (!zero_offset_can_be_applied(controller))
    {
      return false;
    }
  }

  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    GM6020_Controller_t *controller = &controllers[axis_index];

    apply_zero_offset_ecd(controller, zero_ecd[axis_index]);
  }

  return true;
}

bool GM6020_SetAxisZeroOffsetEcd(GM6020_Axis_t axis,
                                uint16_t zero_ecd)
{
  GM6020_Controller_t *controller;

  if (!axis_is_valid(axis)
      || (motor_can[axis] == NULL)
      || (zero_ecd >= GM6020_ENCODER_CPR))
  {
    return false;
  }

  controller = &controllers[axis];
  if (!zero_offset_can_be_applied(controller))
  {
    return false;
  }

  apply_zero_offset_ecd(controller, zero_ecd);
  return true;
}

/*
 * 设置单轴位置目标。
 *
 * 【参数】
 *   axis:             目标轴（GM6020_AXIS_YAW 或 GM6020_AXIS_PITCH）
 *   target_angle_deg: 相对配置零位的逻辑角度（度）
 *
 * 【处理流程】
 *   1. 如果有软限位，正常位置将目标限幅到 [min, max]；若当前位置
 *      已在限位外，只允许保持当前位置或向正常范围内恢复
 *   2. 叠加零位偏置（标定时编码器零位与机械中位的偏差），归一化到 [0, 360)
 *   3. 如果当前已在 POSITION_CONTROL 状态，立即解析为多圈目标
 *      （否则等待状态转入 POSITION_CONTROL 时再解析）
 *
 * 【注意】
 *   此函数会修改控制器的多项共享状态，应在主循环上下文调用。
 *   中断只负责发布命令，主循环读取命令后再调用本函数。
 *   如果 axis 无效或 target_angle_deg 非有限，静默忽略。
 */
void GM6020_SetTargetPosition(GM6020_Axis_t axis,
                              float target_angle_deg)
{
  GM6020_Controller_t *controller;
  float limited_angle_deg;
  float minimum_angle_deg;
  float maximum_angle_deg;
  bool reset_angle_pid;

  if (emergency_stop_latched
      || !axis_is_valid(axis)
      || !isfinite(target_angle_deg))
  {
    return;
  }
  controller = &controllers[axis];
  reset_angle_pid =
      !controller->position_target_valid
      || controller->requested_angle_is_multi_turn;

  /* 步骤1：软限位 */
  limited_angle_deg = target_angle_deg;
  if (controller->angle_limit_enabled)
  {
    minimum_angle_deg = controller->minimum_angle_deg;
    maximum_angle_deg = controller->maximum_angle_deg;

    /*
     * 标定后的传感器零点可能使上电位置落在正常运行软限位之外。
     * 此时临时把“当前位置”作为越界侧边界：
     *   current < min → 允许 [current, max]
     *   current > max → 允许 [min, current]
     *
     * 因而可以原位保持或向安全区恢复，但任何继续向机械端点外侧
     * 运动的目标都会被挡住。进入正常范围后自动恢复固定软限位。
     */
    if (controller->encoder_initialized
        && controller->multi_turn_origin_valid)
    {
      const float current_angle_deg =
          controller_encoder_relative_position_deg(controller);

      if (isfinite(current_angle_deg))
      {
        if (current_angle_deg < minimum_angle_deg)
        {
          minimum_angle_deg = current_angle_deg;
        }
        else if (current_angle_deg > maximum_angle_deg)
        {
          maximum_angle_deg = current_angle_deg;
        }
      }
    }

    limited_angle_deg = clamp_float(
        limited_angle_deg,
        minimum_angle_deg,
        maximum_angle_deg);
  }

  /*
   * 步骤2：叠加零位偏置并归一化。
   * 对外使用以云台中位为 0° 的逻辑角度；
   * 内部使用电机编码器的单圈角度（0~360°）。
   * 因此：逻辑角度 + 零位偏置 = 电机编码器单圈角度。
   */
  controller->requested_angle_deg =
      normalize_single_turn_degrees(
          limited_angle_deg + controller->zero_offset_deg);
  controller->requested_angle_is_multi_turn = false;
  controller->position_target_valid = true;
  position_feedforward_reset(controller);
  if (reset_angle_pid)
  {
    angle_pid_reset(controller);
  }

  /* 步骤3：如果已在位置控制，立即解析目标 */
  if (controller->state == GM6020_STATE_POSITION_CONTROL)
  {
    angle_resolve_single_turn_target(
        controller,
        controller->requested_angle_deg);
  }
}

void GM6020_SetMultiTurnTargetPosition(
    GM6020_Axis_t axis,
    float target_angle_deg)
{
  GM6020_Controller_t *controller;
  bool reset_angle_pid;

  if (emergency_stop_latched
      || !axis_is_valid(axis)
      || !isfinite(target_angle_deg))
  {
    return;
  }

  controller = &controllers[axis];
  reset_angle_pid =
      !controller->position_target_valid
      || !controller->requested_angle_is_multi_turn;
  controller->requested_angle_deg = target_angle_deg;
  controller->requested_angle_is_multi_turn = true;
  controller->position_target_valid = true;
  position_feedforward_reset(controller);
  if (reset_angle_pid)
  {
    angle_pid_reset(controller);
  }

  if (controller->state == GM6020_STATE_POSITION_CONTROL)
  {
    angle_resolve_multi_turn_target(
        controller,
        controller->requested_angle_deg);
  }
}

/*
 * 同时设置 Yaw 和 Pitch 两轴的位置目标。
 * 等价于连续调用两次 GM6020_SetTargetPosition。
 *
 * 【参数】
 *   yaw_angle_deg:   Yaw 轴逻辑目标角度
 *   pitch_angle_deg: Pitch 轴逻辑目标角度
 */
void GM6020_SetGimbalPosition(float yaw_angle_deg,
                              float pitch_angle_deg)
{
  GM6020_SetTargetPosition(
      GM6020_AXIS_YAW, yaw_angle_deg);
  GM6020_SetTargetPosition(
      GM6020_AXIS_PITCH, pitch_angle_deg);
}

void GM6020_SetPositionFeedforward(
    GM6020_Axis_t axis,
    float target_speed_rpm,
    float target_acceleration_rpm_s)
{
  GM6020_Controller_t *controller;

  if (!axis_is_valid(axis))
  {
    return;
  }

  controller = &controllers[axis];
  if (emergency_stop_latched
      || !isfinite(target_speed_rpm)
      || !isfinite(target_acceleration_rpm_s)
      || !controller->position_target_valid)
  {
    position_feedforward_reset(controller);
    return;
  }

  controller->command_speed_feedforward_rpm = clamp_float(
      target_speed_rpm,
      -controller->angle_speed_limit_rpm,
      controller->angle_speed_limit_rpm);
  controller->command_accel_feedforward_rpm_s = clamp_float(
      target_acceleration_rpm_s,
      -GM6020_POSITION_FF_ACCEL_LIMIT_RPM_S,
      GM6020_POSITION_FF_ACCEL_LIMIT_RPM_S);
}

/*
 * 进入速度环调试模式。
 *
 * 绕过角度环，直接用指定的目标转速驱动速度环。
 * 常用于整定速度环 PID 参数或测试电机机械响应。
 *
 * 【参数】
 *   axis:             目标轴
 *   target_speed_rpm: 目标转速（rpm），自动限幅到 ±GM6020_DEBUG_SPEED_LIMIT_RPM
 *
 * 【注意】
 *   如果当前在 POSITION_CONTROL 状态，会立即转移到 SPEED_DEBUG。
 *   否则只是设置 speed_debug_requested 标志，等状态机自己转移。
 */
void GM6020_EnterSpeedDebug(GM6020_Axis_t axis,
                            float target_speed_rpm)
{
  GM6020_Controller_t *controller;

  if (emergency_stop_latched
      || !axis_is_valid(axis)
      || !isfinite(target_speed_rpm))
  {
    return;
  }

  controller = &controllers[axis];
  controller->debug_target_speed_rpm = clamp_float(
      target_speed_rpm,
      -GM6020_DEBUG_SPEED_LIMIT_RPM,
      GM6020_DEBUG_SPEED_LIMIT_RPM);
  controller->target_speed_rpm =
      controller->debug_target_speed_rpm;
  controller->speed_debug_requested = true;

  if (controller->state == GM6020_STATE_POSITION_CONTROL)
  {
    control_state_transition(
        controller, GM6020_STATE_SPEED_DEBUG);
  }
}

/*
 * 在速度调试模式下实时修改目标转速。
 *
 * 【参数】
 *   axis:             目标轴
 *   target_speed_rpm: 新目标转速（rpm），自动限幅
 *
 * 【注意】
 *   仅在 speed_debug_requested 为 true 时才会更新当前目标转速。
 *   如果已退出速度调试模式，此调用不会产生效果。
 */
void GM6020_SetSpeedDebugTarget(GM6020_Axis_t axis,
                                float target_speed_rpm)
{
  GM6020_Controller_t *controller;

  if (emergency_stop_latched
      || !axis_is_valid(axis)
      || !isfinite(target_speed_rpm))
  {
    return;
  }

  controller = &controllers[axis];
  controller->debug_target_speed_rpm = clamp_float(
      target_speed_rpm,
      -GM6020_DEBUG_SPEED_LIMIT_RPM,
      GM6020_DEBUG_SPEED_LIMIT_RPM);
  if (controller->speed_debug_requested)
  {
    controller->target_speed_rpm =
        controller->debug_target_speed_rpm;
  }
}

/*
 * 退出速度环调试模式，回到位置控制模式。
 *
 * 清零调试标志和调试目标，转移到 POSITION_CONTROL。
 * 进入位置控制后会锁定当前位置（因为没有有效的 position_target）。
 *
 * 【参数】
 *   axis: 目标轴
 */
void GM6020_ExitSpeedDebug(GM6020_Axis_t axis)
{
  GM6020_Controller_t *controller;

  if (!axis_is_valid(axis))
  {
    return;
  }

  controller = &controllers[axis];

  /* 如果根本不在调试模式或没有请求调试，忽略 */
  if (!controller->speed_debug_requested
      && (controller->state != GM6020_STATE_SPEED_DEBUG))
  {
    return;
  }

  controller->speed_debug_requested = false;
  controller->debug_target_speed_rpm = 0.0f;
  controller->target_speed_rpm = 0.0f;
  controller->position_target_valid = false;
  controller->requested_angle_is_multi_turn = false;

  if (controller->state == GM6020_STATE_SPEED_DEBUG)
  {
    control_state_transition(
        controller, GM6020_STATE_POSITION_CONTROL);
  }
}

/*
 * 运行时修改速度环 PID 增益。
 *
 * 【参数】
 *   kp, ki, kd: 新的 PID 增益值，必须为非负有限浮点数
 * 【返回值】true 表示设置成功。
 * 【副作用】修改增益后会重置速度环 PID（清零积分和误差），
 *           防止旧积分值在新参数下产生意外的输出跳变。
 */
bool GM6020_SetSpeedPidGains(GM6020_Axis_t axis,
                             float kp, float ki, float kd)
{
  GM6020_Controller_t *controller;

  if (!axis_is_valid(axis)
      || !isfinite(kp) || !isfinite(ki) || !isfinite(kd)
      || (kp < 0.0f) || (ki < 0.0f) || (kd < 0.0f))
  {
    return false;
  }

  controller = &controllers[axis];
  controller->speed_pid.kp = kp;
  controller->speed_pid.ki = ki;
  controller->speed_pid.kd = kd;
  speed_pid_reset(controller);
  return true;
}

/*
 * 运行时修改角度环 PID 增益。
 * 参数和行为同 GM6020_SetSpeedPidGains。
 */
bool GM6020_SetAnglePidGains(GM6020_Axis_t axis,
                             float kp, float ki, float kd)
{
  GM6020_Controller_t *controller;

  if (!axis_is_valid(axis)
      || !isfinite(kp) || !isfinite(ki) || !isfinite(kd)
      || (kp < 0.0f) || (ki < 0.0f) || (kd < 0.0f))
  {
    return false;
  }

  controller = &controllers[axis];
  controller->angle_pid.kp = kp;
  controller->angle_pid.ki = ki;
  controller->angle_pid.kd = kd;
  angle_pid_reset(controller);
  return true;
}

/*
 * 查询指定轴的当前控制模式。
 *
 * 【返回值】
 *   GM6020_MODE_POSITION    位置控制模式
 *   GM6020_MODE_SPEED_DEBUG 速度调试模式
 *   如果 axis 无效，默认返回 GM6020_MODE_POSITION。
 */
GM6020_ControlMode_t GM6020_GetControlMode(GM6020_Axis_t axis)
{
  if (!axis_is_valid(axis))
  {
    return GM6020_MODE_POSITION;
  }

  return controllers[axis].speed_debug_requested
       ? GM6020_MODE_SPEED_DEBUG
       : GM6020_MODE_POSITION;
}

/*
 * 获取指定轴的速度环调试数据。
 *
 * 返回数据结构包含目标/反馈转速、误差、输出电流、当前 PID 增益和控制模式，
 * 可用调试器观察或通过串口发送到上位机绘图。
 *
 * 【参数】
 *   axis: 目标轴
 * 【返回值】填充好的 GM6020_SpeedDebugData_t。
 *          如果 axis 无效，返回全零结构体。
 */
GM6020_SpeedDebugData_t GM6020_GetSpeedDebugData(
    GM6020_Axis_t axis)
{
  GM6020_SpeedDebugData_t data = {0};
  const GM6020_Controller_t *controller;

  if (!axis_is_valid(axis))
  {
    return data;
  }

  controller = &controllers[axis];
  data.target_speed_rpm = controller->target_speed_rpm;
  data.feedback_speed_rpm =
      (float)controller->feedback.speed_rpm;
  data.speed_error_rpm =
      data.target_speed_rpm - data.feedback_speed_rpm;
  data.output_current = controller->speed_pid.output;
  data.command_current = controller->current_command;
  data.speed_feedforward_rpm =
      controller->applied_speed_feedforward_rpm;
  data.current_feedforward =
      controller->applied_current_feedforward;
  data.kp = controller->speed_pid.kp;
  data.ki = controller->speed_pid.ki;
  data.kd = controller->speed_pid.kd;
  data.mode = GM6020_GetControlMode(axis);
  return data;
}

/*
 * 主循环核心调度函数。
 *
 * 【调用时机】在 while(1) 主循环中尽可能高频地调用。
 *            控制频率由 GM6020 反馈帧的到达速率自然决定（~1 kHz）。
 *
 * 【执行流程】
 *   阶段1 — 接收并分发：
 *     分别读取 CAN1/Yaw 与 CAN2/Pitch 的 RX FIFO0，
 *     根据物理总线确定轴并校验反馈 StdId，更新该轴反馈数据、
 *     编码器多圈累计，然后调用状态机处理（PID 计算 + 更新电流命令）。
 *
 *   阶段2 — 超时检测：
 *     遍历两轴，检查是否超过配置的超时时间未收到反馈，
 *     超时则转入 FAULT 状态（输出零电流）。
 *
 *   阶段3 — 发送命令：
 *     如果某轴电流命令发生变化，只在该轴对应总线上发送 0x1FE。
 *
 * 【并发说明】
 *   本函数在裸机主循环中运行（非中断上下文），不持有锁。
 *   与 CAN 接收中断（HAL_CAN_IRQHandler）之间通过 FIFO 解耦，
 *   不带竞态条件。与 USART6 中断之间通过 volatile 标志位解耦。
 */
void GM6020_Process(void)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint32_t now = HAL_GetTick();
  uint32_t axis_index;
  bool command_changed[GM6020_AXIS_COUNT] = {false, false};

  if (!motor_can_is_initialized())
  {
    return;
  }

  /*
   * --- 阶段1：接收反馈帧并分发到对应轴 ---
   *
   * 用 while 循环读取 FIFO 中所有积压的帧。
   * 一次 Process() 调用可能处理 0 帧（无新反馈）、1 帧（单轴反馈到达）
   * 或多帧（两轴反馈都到达或之前有积压）。
   */
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    CAN_HandleTypeDef *axis_can = motor_can[axis_index];
    GM6020_Controller_t *controller = &controllers[axis_index];

    while (HAL_CAN_GetRxFifoFillLevel(axis_can, CAN_RX_FIFO0) > 0U)
    {
      if (HAL_CAN_GetRxMessage(axis_can, CAN_RX_FIFO0,
                              &rx_header, rx_data) != HAL_OK)
      {
        break;
      }

      /* 过滤非标准数据帧（扩展帧、远程帧、DLC≠8 均丢弃） */
      if ((rx_header.IDE != CAN_ID_STD)
          || (rx_header.RTR != CAN_RTR_DATA)
          || (rx_header.DLC != 8U))
      {
        continue;
      }

      /*
       * 两台电机都是 ID 2（反馈 0x206），轴由物理 CAN 总线区分：
       * CAN1 固定对应 Yaw，CAN2 固定对应 Pitch。
       */
      if (rx_header.StdId
          != axis_can_config[axis_index].feedback_std_id)
      {
        continue;
      }

      /*
       * 解析 GM6020 反馈帧（8 字节大端）：
       *   DATA[0:1] = 编码器角度（0~8191）
       *   DATA[2:3] = 转速（rpm，有符号 int16）
       *   DATA[4:5] = 转矩电流（±16384 ≈ ±3A）
       *   DATA[6]   = 温度（℃）
       *   DATA[7]   = 保留
       */
      controller->feedback.angle =
          (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
      encoder_update(controller, controller->feedback.angle);
      controller->feedback.speed_rpm =
          (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
      controller->feedback.torque_current =
          (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
      controller->feedback.temperature = rx_data[6];
      controller->feedback.last_rx_ms = now;
      ++controller->feedback.rx_sequence;
      controller->feedback.online = true;

      /*
       * Pitch反馈更新后立即刷新融合器，使本帧PID同时使用最新编码器、
       * 电机转速和IMU快照。任务层仍会调用一次融合器，用于无CAN帧时
       * 的数据超时与诊断状态维护。
       */
      if (axis_index == (uint32_t)GM6020_AXIS_PITCH)
      {
        PitchFusion_Process();
      }

      if (emergency_stop_latched)
      {
        controller->current_command = 0;
        controller->target_speed_rpm = 0.0f;
      }
      else
      {
        control_state_handle_feedback(controller);
      }
      command_changed[axis_index] = true;
    }
  }

  /*
   * Phase 2: Timeout Detection — check each axis for feedback timeout
   * --- 阶段2：超时检测 ---
   *
   * 重新获取时间戳（阶段1可能耗时），依次检查两轴。
   * 注意：这里用的是同一个 now，两轴的超时判断基于同一个时间基准。
   */
  /* Re-fetch timestamp (phase 1 may have taken time); both axes share same time base */
  now = HAL_GetTick();
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    /* If timeout occurred → transition to FAULT → mark command_changed to re-send zero current */
    if (control_state_poll_timeout(
            &controllers[axis_index], now))
    {
      command_changed[axis_index] = true;
    }
  /* Phase 3: Send current command — only if that axis had a change (efficiency: skip idle transmits) */
  }

  /* 某轴电流变化时，只在该轴对应的 CAN 总线上发送。 */
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    if (command_changed[axis_index])
    {
      (void)gm6020_send_axis_current((GM6020_Axis_t)axis_index);
    }
  }
}

/*
 * 获取指定轴的只读反馈数据。
 *
 * 【返回值】
 *   指向 GM6020_Feedback_t 的只读指针（建议调用方仅读取，不要修改）。
 *   如果 axis 无效，返回 NULL。
 *
 * 【用途】
 *   调试器观察变量、串口上报、上位机监控等场景。
 */
const GM6020_Feedback_t *GM6020_GetFeedback(GM6020_Axis_t axis)
{
  if (!axis_is_valid(axis))
  {
    return NULL;
  }
  return &controllers[axis].feedback;
}

bool GM6020_GetMultiTurnPosition(
    GM6020_Axis_t axis,
    float *position_deg)
{
  const GM6020_Controller_t *controller;

  if (!axis_is_valid(axis) || (position_deg == NULL))
  {
    return false;
  }

  controller = &controllers[axis];
  if (!controller->encoder_initialized
      || !controller->multi_turn_origin_valid)
  {
    return false;
  }

  *position_deg =
      (float)(controller->feedback.total_angle_ecd
              - controller->multi_turn_origin_ecd)
      * 360.0f / (float)GM6020_ENCODER_CPR;
  return true;
}

bool GM6020_GetTargetPosition(
    GM6020_Axis_t axis,
    float *position_deg)
{
  const GM6020_Controller_t *controller;

  if (!axis_is_valid(axis) || (position_deg == NULL))
  {
    return false;
  }

  controller = &controllers[axis];
  if (emergency_stop_latched
      || (controller->state != GM6020_STATE_POSITION_CONTROL)
      || !controller->feedback.online
      || !controller->encoder_initialized
      || !controller->multi_turn_origin_valid)
  {
    return false;
  }

  *position_deg =
      (float)(controller->target_total_angle_ecd
              - controller->multi_turn_origin_ecd)
      * 360.0f / (float)GM6020_ENCODER_CPR;
  return isfinite(*position_deg);
}
