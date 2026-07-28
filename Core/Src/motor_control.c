#include "motor_control.h"
#include "config/gimbal_params.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * ============================================================================
 * 两轴 GM6020 云台电机控制模块
 * ============================================================================
 *
 * 【硬件拓扑】
 *   两个 DJI GM6020 无刷直流电机挂在同一条 CAN 总线上：
 *     Yaw   电机 ID=1 → 反馈帧 0x205，电流命令位于 0x1FE 的 DATA[0:1]
 *     Pitch 电机 ID=2 → 反馈帧 0x206，电流命令位于 0x1FE 的 DATA[2:3]
 *
 * 【控制算法】
 *   串级 PID：外环角度环（P/PD）→ 内环速度环（PI/PD）
 *   每个轴拥有独立的编码器多圈累计、状态机和两套 PID 控制器。
 *
 * 【CAN 协议约束】
 *   GM6020 的电流命令帧 0x1FE 必须同时包含两个电机的电流值，
 *   不能分别发送只带单个轴电流、另一轴置零的帧，否则另一方会失控。
 *   因此所有电流输出统一由 gm6020_send_group_current() 打包发送。
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
  bool speed_debug_requested;         /* 是否正在/请求进入速度调试模式 */

  /* ---- 位置命令 ---- */
  float requested_angle_deg;           /* 单圈角度或累计多圈角度目标 */
  bool requested_angle_is_multi_turn;  /* true：累计多圈；false：单圈劣弧 */
  bool position_target_valid;          /* 是否有有效的位置目标 */
  int32_t target_total_angle_ecd;      /* 解析后的多圈编码器目标值 */

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
 * feedback_std_id: 电机反馈帧的标准 ID（Yaw=0x205, Pitch=0x206）
 * current_slot: 在 0x1FE 电流命令帧中的槽位编号（0=DATA[0:1], 1=DATA[2:3]）
 */
typedef struct
{
  uint16_t feedback_std_id;
  uint8_t current_slot;
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
    YAW_CURRENT_COMMAND_SLOT
  },
  {
    PITCH_FEEDBACK_STD_ID,
    PITCH_CURRENT_COMMAND_SLOT
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

/* ---- 模块级全局变量 ---- */

static CAN_HandleTypeDef *motor_can;                     /* CAN1 句柄指针 */
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

  if ((can_config->feedback_std_id > 0x7FFU)
      || (can_config->current_slot > 3U)
      || !(pid_config->speed_output_limit > 0.0f)
      || !(pid_config->angle_speed_limit_rpm > 0.0f)
      || !isfinite(mechanical->zero_offset_deg))
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

/*====================================================================
 * 编码器角度转换函数
 *====================================================================*/

/*
 * 将任意浮点角度归一化到 [0, 360) 单圈范围内。
 * 例：-90° → 270°，450° → 90°。
 */
static float normalize_single_turn_degrees(float angle_deg)
{
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
static int32_t single_turn_degrees_to_encoder(float angle_deg)
{
  int32_t encoder;

  angle_deg = normalize_single_turn_degrees(angle_deg);
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
    GM6020_Controller_t *controller,
    float angle_deg)
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
    delta += GM6020_ENCODER_CPR;
  }

  /* 叠加到当前多圈累计值，得到最终多圈目标 */
  controller->target_total_angle_ecd =
      controller->feedback.total_angle_ecd + delta;

  /* 目标变更后重置角度环 PID，防止历史积分干扰新目标 */
  angle_pid_reset(controller);
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

  angle_pid_reset(controller);
}

/*
 * 返回与当前累计位置距离最近的“标定机械零点”累计编码器值。
 * zero_offset_deg 表示逻辑 0° 对应的单圈编码器位置。
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
 * PID 更新函数（含 anti-windup）
 *====================================================================*/

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
static float angle_pid_update(GM6020_Controller_t *controller)
{
  AnglePID_t *pid = &controller->angle_pid;
  const float output_limit = controller->angle_speed_limit_rpm;

  /*
   * 误差 = 目标 - 当前，单位为编码器计数，乘以转换因子得到角度误差（度）。
   * 正误差 = 需要正向转动才能追上目标。
   */
  const float error =
      (float)(controller->target_total_angle_ecd
              - controller->feedback.total_angle_ecd)
      * 360.0f / (float)GM6020_ENCODER_CPR;

  /* 微分项：误差变化率 / 控制周期（不直接用编码器反馈避免噪声放大） */
  const float derivative =
      (error - pid->previous_error) / GM6020_CONTROL_PERIOD_S;

  /* 比例 + 微分（不含积分，用于 anti-windup 判断） */
  const float proportional_and_derivative =
      pid->kp * error + pid->kd * derivative;

  /* 积分候选值：当前积分 + 本拍增量，先限幅防止积分绝对值过大 */
  const float candidate_integral = clamp_float(
      pid->integral + pid->ki * error * GM6020_CONTROL_PERIOD_S,
      -output_limit,
      output_limit);

  /* 完整的候选输出 */
  const float candidate_output =
      proportional_and_derivative + candidate_integral;

  /*
   * Anti-windup 条件判断：
   *   条件1：候选输出在限幅范围内 → 正常更新积分
   *   条件2：输出已达上限但误差 < 0 → 正在往回拉，可以更新积分（退饱和）
   *   条件3：输出已达下限但误差 > 0 → 正在往回拉，可以更新积分（退饱和）
   *   以上任一满足，则接受新的积分值；否则保持旧积分值不变
   */
  if (((candidate_output < output_limit)
       && (candidate_output > -output_limit))
      || ((candidate_output >= output_limit)
          && (error < 0.0f))
      || ((candidate_output <= -output_limit)
          && (error > 0.0f)))
  {
    pid->integral = candidate_integral;
  }

  /* 最终输出 = P + D + I（可能是旧的积分值），再限幅一次确保安全 */
  pid->output = proportional_and_derivative + pid->integral;
  pid->output = clamp_float(pid->output,
                            -output_limit,
                            output_limit);

  /* 保存本拍误差供下一拍微分计算 */
  pid->previous_error = error;
  return pid->output;
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
                              float feedback)
{
  SpeedPID_t *pid = &controller->speed_pid;
  const float output_limit = controller->speed_output_limit;

  /* 误差 = 目标转速 - 实际转速 */
  const float error = target - feedback;

  /* 微分项：误差变化率 */
  const float derivative =
      (error - pid->previous_error) / GM6020_CONTROL_PERIOD_S;

  const float proportional_and_derivative =
      pid->kp * error + pid->kd * derivative;

  const float candidate_integral = clamp_float(
      pid->integral + pid->ki * error * GM6020_CONTROL_PERIOD_S,
      -output_limit,
      output_limit);

  const float candidate_output =
      proportional_and_derivative + candidate_integral;

  /* Anti-windup：仅在输出未饱和或误差方向有利于退饱和时更新积分 */
  if (((candidate_output < output_limit)
       && (candidate_output > -output_limit))
      || ((candidate_output >= output_limit)
          && (error < 0.0f))
      || ((candidate_output <= -output_limit)
          && (error > 0.0f)))
  {
    pid->integral = candidate_integral;
  }

  pid->output = proportional_and_derivative + pid->integral;
  pid->output = clamp_float(pid->output,
                            -output_limit,
                            output_limit);
  pid->previous_error = error;
  return pid->output;
}

/*====================================================================
 * CAN 通信函数
 *====================================================================*/

/*
 * 发送合并电流命令帧 0x1FE。
 *
 * 【帧格式】
 *   StdId = 0x1FE, DLC = 8, 数据帧
 *   DATA[0:1] = Yaw 电流   (大端，slot 0)
 *   DATA[2:3] = Pitch 电流 (大端，slot 1)
 *   DATA[4:7] = 0（保留）
 *
 * 【重要】
 *   0x1FE 必须一次性包含两个轴的最新电流值。
 *   若分别发送含零值的帧会导致另一轴失控。
 *
 * 【返回值】HAL_CAN_AddTxMessage 的状态。
 */
static HAL_StatusTypeDef gm6020_send_group_current(void)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  uint32_t mailbox;
  uint32_t axis_index;

  tx_header.StdId = GM6020_CURRENT_COMMAND_ID;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  /*
   * 遍历两轴，将各自的 current_command 填入 tx_data 的正确偏移位置。
   * current_slot 决定偏移量：slot 0 → offset 0 (DATA[0:1])
   *                         slot 1 → offset 2 (DATA[2:3])
   */
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    int16_t current_command =
        controllers[axis_index].current_command;

#if GIMBAL_YAW_ONLY_TEST_MODE
    /*
     * 单轴测试期间，即使 Pitch 意外收到反馈并运行了内部状态机，
     * 发到 0x1FE 的 Pitch 电流槽也始终保持为 0。
     */
    if (axis_index == (uint32_t)GM6020_AXIS_PITCH)
    {
      current_command = 0;
    }
#endif

    const uint16_t raw = (uint16_t)current_command;
    const uint32_t offset =
        (uint32_t)axis_can_config[axis_index].current_slot * 2U;
    tx_data[offset] = (uint8_t)(raw >> 8);       /* 大端高字节 */
    tx_data[offset + 1U] = (uint8_t)raw;          /* 大端低字节 */
  }

  return HAL_CAN_AddTxMessage(
      motor_can, &tx_header, tx_data, &mailbox);
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
{
  controller->current_command = 0;
}

/*
 * POSITION_CONTROL 状态下的反馈处理：
 *   1. 角度环 PID：位置误差 → 速度环目标
 *   2. 速度环 PID：速度误差 → 转矩电流命令
 */
static void state_position_control_on_feedback(
    GM6020_Controller_t *controller)
{
  /* 外环：位置 → 目标转速 */
  controller->target_speed_rpm = angle_pid_update(controller);

  /* 内环：目标转速 vs 实际转速 → 电流命令 */
  controller->current_command = (int16_t)speed_pid_update(
      controller,
      controller->target_speed_rpm,
      (float)controller->feedback.speed_rpm);
}

/*
 * SPEED_DEBUG 状态下的反馈处理：
 *   绕过角度环，直接用人工设定的 debug_target_speed_rpm 驱动速度环。
 *   用于整定速度环 PID 参数或测试电机响应。
 */
static void state_speed_debug_on_feedback(
    GM6020_Controller_t *controller)
{
  controller->target_speed_rpm =
      controller->debug_target_speed_rpm;
  controller->current_command = (int16_t)speed_pid_update(
      controller,
      controller->target_speed_rpm,
      (float)controller->feedback.speed_rpm);
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
 *      - FAULT: 标记离线，清除编码器初始化标志，等待下次有效反馈
 */
static void control_state_transition(
    GM6020_Controller_t *controller,
    GM6020_ControlState_t next_state)
{
  controller->state = next_state;
  controller->current_command = 0;

  switch (controller->state)
  {
    case GM6020_STATE_WAIT_FEEDBACK:
      controller->target_speed_rpm = 0.0f;
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
        /*
         * 没有外部位置命令时，锁定当前位置。
         * 每个轴独立锁定，Yaw 锁 Yaw 的当前位置，Pitch 锁 Pitch 的当前位置。
         */
        controller->target_total_angle_ecd =
            controller->feedback.total_angle_ecd;
        controller->requested_angle_deg =
            normalize_single_turn_degrees(
                controller->feedback.total_angle_deg);
        controller->requested_angle_is_multi_turn = false;
      }
      break;

    case GM6020_STATE_SPEED_DEBUG:
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
 *   每个滤波器只接收一个特定的标准 ID 反馈帧。
 *   两个滤波器分别对应 Yaw (0x205) 和 Pitch (0x206)，
 *   均路由到 FIFO0。
 *
 * 【参数】
 *   filter_bank:   滤波器组编号（0 = Yaw, 1 = Pitch）
 *   feedback_std_id: 要接收的标准 ID
 */
static HAL_StatusTypeDef configure_feedback_filter(
    uint32_t filter_bank,
    uint16_t feedback_std_id)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  /*
   * 标准 ID 左移 5 位存入 FilterIdHigh（CAN 规范中标准 ID 在 32 位
   * 寄存器中的位偏移为 21，对应高 16 位的 [15:5]）。
   */
  filter.FilterIdHigh = (uint16_t)(feedback_std_id << 5);
  filter.FilterIdLow = 0U;
  /* 掩码匹配所有 11 位标准 ID */
  filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  return HAL_CAN_ConfigFilter(motor_can, &filter);
}

/*====================================================================
 * 公有 API
 *====================================================================*/

/*
 * 初始化两轴 GM6020 电机控制器。
 *
 * 【执行流程】
 *   1. 校验 CAN 句柄非空、两轴 CAN ID 和电流槽位不冲突
 *   2. 遍历两轴：校验配置、初始化控制器、配置 CAN 滤波器
 *   3. 启动 CAN 外设
 *   4. 发送初始零电流命令（0x1FE 帧，两轴电流均为 0）
 *
 * 【参数】
 *   hcan: CAN1 外设句柄指针
 * 【返回值】HAL_OK 表示初始化成功。
 */
HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *hcan)
{
  HAL_StatusTypeDef status;
  uint32_t axis_index;

  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  /*
   * 两轴的反馈 ID 和电流槽位必须不同，否则帧分发会出错。
   * 如果配置表中有冲突，编译阶段不会报错，运行时在此处拦截。
   */
  if ((axis_can_config[GM6020_AXIS_YAW].feedback_std_id
       == axis_can_config[GM6020_AXIS_PITCH].feedback_std_id)
      || (axis_can_config[GM6020_AXIS_YAW].current_slot
          == axis_can_config[GM6020_AXIS_PITCH].current_slot))
  {
    return HAL_ERROR;
  }

  motor_can = hcan;
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

    status = configure_feedback_filter(
        axis_index,
        axis_can_config[axis_index].feedback_std_id);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  status = HAL_CAN_Start(motor_can);
  if (status != HAL_OK)
  {
    return status;
  }

  /*
   * 两轴收到有效反馈前保持零电流。
   * GM6020 在收到第一个非零 0x1FE 帧之前不会发送反馈，
   * 但发送电流为零的帧是安全的（告知电机初始状态）。
   */
  return gm6020_send_group_current();
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
    speed_pid_reset(controller);
    angle_pid_reset(controller);
  }

  if (motor_can == NULL)
  {
    return HAL_ERROR;
  }

  return gm6020_send_group_current();
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
  controller->target_speed_rpm = 0.0f;
  controller->current_command = 0;
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

  if ((motor_can == NULL)
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

  if ((motor_can == NULL)
      || !axis_is_valid(axis)
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
 *   1. 如果有软限位，先将目标限幅到 [min, max]
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

  if (emergency_stop_latched
      || !axis_is_valid(axis)
      || !isfinite(target_angle_deg))
  {
    return;
  }
  controller = &controllers[axis];

  /* 步骤1：软限位 */
  limited_angle_deg = target_angle_deg;
  if (controller->angle_limit_enabled)
  {
    limited_angle_deg = clamp_float(
        limited_angle_deg,
        controller->minimum_angle_deg,
        controller->maximum_angle_deg);
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

  if (emergency_stop_latched
      || !axis_is_valid(axis)
      || !isfinite(target_angle_deg))
  {
    return;
  }

  controller = &controllers[axis];
  controller->requested_angle_deg = target_angle_deg;
  controller->requested_angle_is_multi_turn = true;
  controller->position_target_valid = true;

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
 *     循环读取 CAN RX FIFO0 中所有待处理的反馈帧，
 *     根据 StdId 匹配到对应的轴，更新该轴的反馈数据、
 *     编码器多圈累计，然后调用状态机处理（PID 计算 + 更新电流命令）。
 *
 *   阶段2 — 超时检测：
 *     遍历两轴，检查是否超过配置的超时时间未收到反馈，
 *     超时则转入 FAULT 状态（输出零电流）。
 *
 *   阶段3 — 发送命令：
 *     如果阶段1或阶段2中有任何电流命令发生变化，
 *     将两轴的最新电流值打包为 0x1FE 帧通过 CAN 发送。
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
  bool command_changed = false;

  /*
   * --- 阶段1：接收反馈帧并分发到对应轴 ---
   *
   * 用 while 循环读取 FIFO 中所有积压的帧。
   * 一次 Process() 调用可能处理 0 帧（无新反馈）、1 帧（单轴反馈到达）
   * 或多帧（两轴反馈都到达或之前有积压）。
   */
  while (HAL_CAN_GetRxFifoFillLevel(motor_can, CAN_RX_FIFO0) > 0U)
  {
    GM6020_Controller_t *controller = NULL;

    if (HAL_CAN_GetRxMessage(motor_can, CAN_RX_FIFO0,
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

    /* 根据 StdId 匹配到对应的轴 */
    for (axis_index = 0U;
         axis_index < (uint32_t)GM6020_AXIS_COUNT;
         ++axis_index)
    {
      if (rx_header.StdId
          == axis_can_config[axis_index].feedback_std_id)
      {
        controller = &controllers[axis_index];
        break;
      }
    }
    if (controller == NULL)
    {
      /* 未匹配到任何轴（可能是总线上的其他设备） → 丢弃 */
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
     * 急停锁存期间仍更新编码器和在线状态，但禁止运行 PID，
     * 并在每次反馈后继续发送零电流。
     */
    if (emergency_stop_latched)
    {
      controller->current_command = 0;
      controller->target_speed_rpm = 0.0f;
    }
    else
    {
      control_state_handle_feedback(controller);
    }
    command_changed = true;
  }

  /*
   * --- 阶段2：超时检测 ---
   *
   * 重新获取时间戳（阶段1可能耗时），依次检查两轴。
   * 注意：这里用的是同一个 now，两轴的超时判断基于同一个时间基准。
   */
  now = HAL_GetTick();
  for (axis_index = 0U;
       axis_index < (uint32_t)GM6020_AXIS_COUNT;
       ++axis_index)
  {
    if (control_state_poll_timeout(
            &controllers[axis_index], now))
    {
      command_changed = true;
    }
  }

  /*
   * --- 阶段3：发送合并电流命令 ---
   *
   * 只在有变化时才发送，避免无意义地占用 CAN 总线带宽。
   * 因为两个电机的电流打包在同一帧中，
   * 任意一个轴电流变化都会触发一次发送。
   */
  if (command_changed)
  {
    (void)gm6020_send_group_current();
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
