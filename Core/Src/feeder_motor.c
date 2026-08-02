/**
 * ===========================================================================
 * @file    feeder_motor.c
 * @brief   CAN1 C610 ID3 / M2006拨弹盘速度/单发位置控制模块
 * ===========================================================================
 *
 * CAN协议：
 *   反馈：StdId 0x203，CAN1 FIFO1
 *   控制：StdId 0x200，DATA[4:5]为ID3有符号大端电流命令
 *
 * 安全策略：
 *   - 上电、急停、遥控掉线和方向切换后必须在中挡保持100 ms重新解锁；
 *   - 重新解锁还要求反馈在线、C610错误码为0且电机接近静止；
 *   - 连发/退弹使用速度环；单发使用位置外环串接同一个速度内环；
 *   - 单发按36:1减速比和每圈10发计算固定步距，保持同一前向超出
 *     相位，相邻单发目标严格相差一个弹位且不主动反向找中心；
 *   - 反馈超时、C610错误码或软件堵转保护立即输出零电流；
 *   - 停止时不主动反向制动，避免C610回生电压抬升。
 * ===========================================================================
 */

#include "feeder_motor.h"

#include "config/feeder_params.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
  CAN_HandleTypeDef *can;
  FeederMotorDebugData_t debug;
      /* 对外调试快照，也是拨弹盘状态机的主要公开状态。 */

  float ramped_target_speed_rpm;
      /* 目标速度斜坡后的值，避免电流瞬间跳变。 */
  float speed_integral_raw;
      /* 速度环积分项。 */
  float previous_speed_error_rpm;
      /* 上一拍速度误差，用于D项。 */
  float filtered_derivative_rpm_s;
      /* 低通后的速度误差变化率。 */
  int64_t single_target_scaled_ecd;
      /* 单发目标，放大保存以保留每发步距的小数部分。 */
  int64_t single_completed_target_scaled_ecd;
      /* 最近一次已经完成的单发目标，下一发从这里加一个弹位。 */
  FeederRemoteCommand_t active_command;
      /* 当前真正执行中的命令，用于检测方向切换。 */
  uint32_t last_process_ms;
      /* 上次控制周期时间。 */
  uint32_t last_tx_ms;
      /* 上次成功发送0x200的时间。 */
  uint32_t neutral_start_ms;
      /* 中挡重新解锁计时起点。 */
  uint32_t stall_start_ms;
      /* 堵转条件开始持续满足的时刻。 */
  uint32_t single_settle_start_ms;
      /* 单发进入到位低速区间的时刻。 */
  int16_t last_sent_current_raw;
      /* 最近实际提交给 HAL CAN 的命令，用于避免重复发送。 */
  uint16_t previous_encoder;
      /* 上一帧单圈编码器值。 */
  bool neutral_timing;
      /* 是否正在累计中挡解锁时间。 */
  bool stall_timing;
      /* 是否正在累计堵转时间。 */
  bool single_request_pending;
      /* SINGLE 上升沿请求，消费一次后清零。 */
  bool single_settle_timing;
      /* 是否正在累计单发完成保持时间。 */
  bool encoder_initialized;
      /* 是否已经建立多圈编码器初值。 */
  bool speed_pid_initialized;
      /* 是否已有上一拍速度误差，可安全计算D项。 */
  bool initialized;
      /* Init 是否成功。 */
} FeederMotorContext_t;

static FeederMotorContext_t feeder;

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

static int16_t clamp_current(float value, int16_t limit_raw)
{
  if (!isfinite(value))
  {
    return 0;
  }
  if (value > (float)limit_raw)
  {
    return limit_raw;
  }
  if (value < (float)-limit_raw)
  {
    return (int16_t)-limit_raw;
  }
  return (int16_t)((value >= 0.0f)
      ? (value + 0.5f)
      : (value - 0.5f));
}

static int16_t slew_current(int16_t current, int16_t target,
                            uint32_t delta_ms)
{
  int32_t maximum_step =
      (int32_t)FEEDER_CURRENT_SLEW_RAW_PER_MS
      * (int32_t)delta_ms;
  int32_t delta = (int32_t)target - (int32_t)current;

  if (delta > maximum_step)
  {
    delta = maximum_step;
  }
  else if (delta < -maximum_step)
  {
    delta = -maximum_step;
  }
  return (int16_t)((int32_t)current + delta);
}

static float ramp_speed(float current, float target, uint32_t delta_ms)
{
  const float maximum_step =
      FEEDER_TARGET_RAMP_RPM_S
      * (float)delta_ms
      / 1000.0f;

  if (target > current + maximum_step)
  {
    return current + maximum_step;
  }
  if (target < current - maximum_step)
  {
    return current - maximum_step;
  }
  return target;
}

static void reset_speed_pid(void)
{
  feeder.speed_integral_raw = 0.0f;
  feeder.previous_speed_error_rpm = 0.0f;
  feeder.filtered_derivative_rpm_s = 0.0f;
  feeder.speed_pid_initialized = false;
  feeder.debug.speed_error_rpm = 0.0f;
  feeder.debug.pid_p_raw = 0.0f;
  feeder.debug.pid_i_raw = 0.0f;
  feeder.debug.pid_d_raw = 0.0f;
  feeder.debug.pid_output_raw = 0.0f;
}

static float update_speed_pid(float target_speed_rpm,
                               float feedback_speed_rpm,
                               uint32_t delta_ms,
                               int16_t current_limit_raw)
{
  const float delta_s = (float)delta_ms / 1000.0f;
  const float error = target_speed_rpm - feedback_speed_rpm;
  const float p_term = FEEDER_SPEED_KP * error;
  float raw_derivative = 0.0f;
  float d_term;
  float candidate_integral;
  float candidate_output;
  float output;

  if (feeder.speed_pid_initialized && (delta_s > 0.0f))
  {
    raw_derivative =
        (error - feeder.previous_speed_error_rpm) / delta_s;
  }
  else
  {
    feeder.speed_pid_initialized = true;
  }

  if (FEEDER_SPEED_D_FILTER_HZ > 0.0f)
  {
    const float filter_time_constant =
        1.0f / (2.0f * 3.14159265358979323846f
                * FEEDER_SPEED_D_FILTER_HZ);
    const float filter_alpha =
        delta_s / (filter_time_constant + delta_s);
    feeder.filtered_derivative_rpm_s +=
        filter_alpha
        * (raw_derivative
           - feeder.filtered_derivative_rpm_s);
  }
  else
  {
    feeder.filtered_derivative_rpm_s = raw_derivative;
  }
  d_term =
      FEEDER_SPEED_KD * feeder.filtered_derivative_rpm_s;

  candidate_integral = clamp_float(
      feeder.speed_integral_raw
      + FEEDER_SPEED_KI * error * delta_s,
      -FEEDER_SPEED_INTEGRAL_LIMIT_RAW,
      FEEDER_SPEED_INTEGRAL_LIMIT_RAW);
  candidate_output = p_term + candidate_integral + d_term;

  /*
   * 输出未饱和时正常积分；输出饱和时，仅允许积分向解除饱和的方向变化。
   * 这样堵转或启动大误差不会让积分项持续累积。
   */
  if (((candidate_output < (float)current_limit_raw)
       && (candidate_output
           > (float)-current_limit_raw))
      || ((candidate_output
           >= (float)current_limit_raw)
          && (error < 0.0f))
      || ((candidate_output
           <= (float)-current_limit_raw)
          && (error > 0.0f)))
  {
    feeder.speed_integral_raw = candidate_integral;
  }

  output = clamp_float(
      p_term + feeder.speed_integral_raw + d_term,
      (float)-current_limit_raw,
      (float)current_limit_raw);
  feeder.previous_speed_error_rpm = error;
  feeder.debug.speed_error_rpm = error;
  feeder.debug.pid_p_raw = p_term;
  feeder.debug.pid_i_raw = feeder.speed_integral_raw;
  feeder.debug.pid_d_raw = d_term;
  feeder.debug.pid_output_raw = output;
  return output;
}

static HAL_StatusTypeDef configure_feedback_filter(void)
{
  CAN_FilterTypeDef filter = {0};

  if (feeder.can == NULL)
  {
    return HAL_ERROR;
  }

  filter.FilterBank = FEEDER_CAN_FILTER_BANK;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh =
      (uint16_t)(FEEDER_FEEDBACK_STD_ID << 5U);
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5U);
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  return HAL_CAN_ConfigFilter(feeder.can, &filter);
}

static HAL_StatusTypeDef send_current(int16_t current_raw)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  uint32_t mailbox;
  const uint32_t offset = FEEDER_CURRENT_SLOT * 2U;
  const uint16_t raw = (uint16_t)current_raw;
  HAL_StatusTypeDef status;

  if ((feeder.can == NULL)
      || (feeder.can->State != HAL_CAN_STATE_LISTENING))
  {
    return HAL_ERROR;
  }

  tx_header.StdId = FEEDER_CONTROL_STD_ID;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;
  tx_data[offset] = (uint8_t)(raw >> 8U);
  tx_data[offset + 1U] = (uint8_t)raw;

  status = HAL_CAN_AddTxMessage(
      feeder.can, &tx_header, tx_data, &mailbox);
  if (status == HAL_OK)
  {
    feeder.last_sent_current_raw = current_raw;
    feeder.last_tx_ms = HAL_GetTick();
  }
  else
  {
    ++feeder.debug.tx_error_count;
  }
  return status;
}

static void invalidate_single_phase(void)
{
  feeder.single_completed_target_scaled_ecd = 0;
  feeder.debug.single_phase_valid = false;
  feeder.debug.single_holding = false;
}

static void force_zero_output(void)
{
  feeder.ramped_target_speed_rpm = 0.0f;
  feeder.debug.target_speed_rpm = 0.0f;
  feeder.debug.command_current_raw = 0;
  feeder.active_command = FEEDER_REMOTE_DISABLE;
  feeder.debug.single_shot_active = false;
  feeder.debug.single_holding = false;
  feeder.stall_timing = false;
  feeder.single_settle_timing = false;
  reset_speed_pid();
}

static void update_multi_turn_position(uint16_t encoder)
{
  int32_t delta;

  if (!feeder.encoder_initialized)
  {
    feeder.previous_encoder = encoder;
    feeder.debug.total_angle_ecd = encoder;
    feeder.encoder_initialized = true;
    return;
  }

  delta = (int32_t)encoder - (int32_t)feeder.previous_encoder;
  if (delta > (FEEDER_ENCODER_CPR / 2))
  {
    delta -= FEEDER_ENCODER_CPR;
  }
  else if (delta < -(FEEDER_ENCODER_CPR / 2))
  {
    delta += FEEDER_ENCODER_CPR;
  }

  feeder.debug.total_angle_ecd += delta;
  feeder.previous_encoder = encoder;
}

static void parse_feedback(
    const CAN_RxHeaderTypeDef *header,
    const uint8_t data[8],
    uint32_t now)
{
  if ((header == NULL)
      || (data == NULL)
      || (header->IDE != CAN_ID_STD)
      || (header->RTR != CAN_RTR_DATA)
      || (header->DLC != 8U)
      || (header->StdId != FEEDER_FEEDBACK_STD_ID))
  {
    return;
  }

  feeder.debug.angle =
      (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
  update_multi_turn_position(feeder.debug.angle);
  feeder.debug.speed_rpm =
      (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
  feeder.debug.actual_current_raw =
      (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
  feeder.debug.error_code = data[7];
  feeder.debug.last_rx_ms = now;
  ++feeder.debug.rx_sequence;
  feeder.debug.online = true;

  if (feeder.debug.error_code != 0U)
  {
    feeder.debug.fault_latched = true;
    feeder.debug.fault_reason = FEEDER_FAULT_ESC;
    feeder.debug.armed = false;
    invalidate_single_phase();
    force_zero_output();
  }
}

static void receive_feedback(uint32_t now)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  while ((feeder.can != NULL)
         && (HAL_CAN_GetRxFifoFillLevel(
                 feeder.can, CAN_RX_FIFO1) > 0U))
  {
    if (HAL_CAN_GetRxMessage(
            feeder.can,
            CAN_RX_FIFO1,
            &header,
            data) != HAL_OK)
    {
      break;
    }
    parse_feedback(&header, data, now);
  }
}

static bool feedback_is_safe_for_rearm(void)
{
  int32_t speed = feeder.debug.speed_rpm;

  if (speed < 0)
  {
    speed = -speed;
  }
  return feeder.debug.online
      && (feeder.debug.error_code == 0U)
      && (speed <= FEEDER_REARM_MAX_SPEED_RPM);
}

static void reset_neutral_timer(void)
{
  feeder.neutral_timing = false;
  feeder.neutral_start_ms = 0U;
}

static void disarm_for_neutral(void)
{
  feeder.debug.armed = false;
  reset_neutral_timer();
  force_zero_output();
}

static void update_rearm_state(uint32_t now)
{
  if (!feedback_is_safe_for_rearm())
  {
    reset_neutral_timer();
    return;
  }

  if (!feeder.neutral_timing)
  {
    feeder.neutral_start_ms = now;
    feeder.neutral_timing = true;
    return;
  }

  if ((uint32_t)(now - feeder.neutral_start_ms)
      >= FEEDER_NEUTRAL_REARM_MS)
  {
    feeder.debug.fault_latched = false;
    feeder.debug.fault_reason = FEEDER_FAULT_NONE;
    feeder.debug.armed = true;
    feeder.active_command = FEEDER_REMOTE_DISABLE;
    reset_neutral_timer();
  }
}

static int32_t clamp_int64_to_int32(int64_t value)
{
  if (value > INT32_MAX)
  {
    return INT32_MAX;
  }
  if (value < INT32_MIN)
  {
    return INT32_MIN;
  }
  return (int32_t)value;
}

static int64_t scaled_ecd_to_rounded_ecd(int64_t scaled_ecd)
{
  const int64_t divisor =
      (int64_t)FEEDER_PROJECTILES_PER_OUTPUT_REV;

  if (scaled_ecd >= 0)
  {
    return (scaled_ecd + divisor / 2) / divisor;
  }
  return (scaled_ecd - divisor / 2) / divisor;
}

static int64_t single_position_error_scaled(void)
{
  return feeder.single_target_scaled_ecd
      - feeder.debug.total_angle_ecd
        * (int64_t)FEEDER_PROJECTILES_PER_OUTPUT_REV;
}

static float single_position_error_output_deg(void)
{
  const int64_t error_scaled = single_position_error_scaled();
  const float error_ecd =
      (float)error_scaled
      / (float)FEEDER_PROJECTILES_PER_OUTPUT_REV;

  feeder.debug.position_error_ecd =
      clamp_int64_to_int32(
          scaled_ecd_to_rounded_ecd(error_scaled));
  return error_ecd * 360.0f
      / ((float)FEEDER_ENCODER_CPR
         * (float)FEEDER_GEAR_RATIO);
}

static void update_single_debug_target(void)
{
  feeder.debug.target_total_angle_ecd =
      scaled_ecd_to_rounded_ecd(
          feeder.single_target_scaled_ecd);
  feeder.debug.position_error_ecd =
      clamp_int64_to_int32(
          scaled_ecd_to_rounded_ecd(
              single_position_error_scaled()));
}

static bool start_single_shot(void)
{
  const int64_t one_projectile_step_scaled =
      (int64_t)FEEDER_ENCODER_CPR
      * (int64_t)FEEDER_GEAR_RATIO;
  const int64_t overshoot_scaled =
      (one_projectile_step_scaled
       * (int64_t)FEEDER_SINGLE_OVERSHOOT_PERCENT
       + 50)
      / 100;

  /*
   * scaled 量把一个弹位乘以“每圈发数”，等价于保留1位小数：
   *   普通编码器目标 = scaled_target / 10
   * 这样 29491.2 counts/shot 的 .2 不会在连续单发中丢失。
   */
  if (!feeder.encoder_initialized)
  {
    return false;
  }

  if (feeder.debug.single_phase_valid)
  {
    feeder.single_target_scaled_ecd =
        feeder.single_completed_target_scaled_ecd
        + one_projectile_step_scaled;
  }
  else
  {
    feeder.single_target_scaled_ecd =
        feeder.debug.total_angle_ecd
          * (int64_t)FEEDER_PROJECTILES_PER_OUTPUT_REV
        + one_projectile_step_scaled
        + overshoot_scaled;
  }
  update_single_debug_target();
  feeder.debug.single_shot_active = true;
  feeder.debug.single_holding = false;
  feeder.active_command = FEEDER_REMOTE_SINGLE;
  feeder.debug.state = FEEDER_STATE_RUNNING_SINGLE;
  feeder.single_settle_timing = false;
  feeder.ramped_target_speed_rpm = 0.0f;
  reset_speed_pid();
  return true;
}

static void finish_single_shot(void)
{
  feeder.single_completed_target_scaled_ecd =
      feeder.single_target_scaled_ecd;
  feeder.debug.single_phase_valid = true;
  ++feeder.debug.shot_count;
  feeder.ramped_target_speed_rpm = 0.0f;
  feeder.debug.target_speed_rpm = 0.0f;
  feeder.debug.command_current_raw = 0;
  feeder.debug.single_shot_active = false;
  feeder.debug.single_holding = true;
  feeder.single_settle_timing = false;
  feeder.stall_timing = false;
  reset_speed_pid();
  feeder.debug.position_error_ecd =
      clamp_int64_to_int32(
          scaled_ecd_to_rounded_ecd(
              single_position_error_scaled()));
  feeder.debug.state = FEEDER_STATE_HOLDING_SINGLE;
}

static void update_safety_state(uint32_t now)
{
  /*
   * 该函数只决定“能不能运行以及处于哪个状态”，不直接计算PID。
   * update_control() 会再次检查状态，只有运行状态才可能产生非零电流。
   */
  if (feeder.debug.online
      && ((uint32_t)(now - feeder.debug.last_rx_ms)
          > FEEDER_FEEDBACK_TIMEOUT_MS))
  {
    feeder.debug.online = false;
    feeder.encoder_initialized = false;
    feeder.single_request_pending = false;
    invalidate_single_phase();
    disarm_for_neutral();
  }

  if (feeder.debug.emergency_stop_latched)
  {
    disarm_for_neutral();
    feeder.debug.state = FEEDER_STATE_ESTOP;
    return;
  }

  if (feeder.debug.error_code != 0U)
  {
    feeder.debug.fault_latched = true;
    feeder.debug.fault_reason = FEEDER_FAULT_ESC;
  }

  if (feeder.debug.fault_latched)
  {
    feeder.debug.armed = false;
    invalidate_single_phase();
    force_zero_output();
    feeder.debug.state = FEEDER_STATE_FAULT;
    if (feeder.debug.remote_command == FEEDER_REMOTE_NEUTRAL)
    {
      update_rearm_state(now);
      if (feeder.debug.armed)
      {
        feeder.debug.state = FEEDER_STATE_ARMED_NEUTRAL;
      }
    }
    else
    {
      reset_neutral_timer();
    }
    return;
  }

  if (feeder.debug.remote_command == FEEDER_REMOTE_DISABLE)
  {
    feeder.single_request_pending = false;
    invalidate_single_phase();
    disarm_for_neutral();
    feeder.debug.state = FEEDER_STATE_DISABLED;
    return;
  }

  if (!feeder.debug.online)
  {
    feeder.single_request_pending = false;
    invalidate_single_phase();
    disarm_for_neutral();
    feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
    return;
  }

  if (!feeder.debug.armed)
  {
    force_zero_output();
    feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
    if (feeder.debug.remote_command == FEEDER_REMOTE_NEUTRAL)
    {
      update_rearm_state(now);
      if (feeder.debug.armed)
      {
        feeder.debug.state = FEEDER_STATE_ARMED_NEUTRAL;
      }
    }
    else
    {
      reset_neutral_timer();
    }
    return;
  }

  /*
   * 已完成单发后维持同一前向超出相位。拨轮回中只解除动作方向锁，
   * 不清除相位；下一次SINGLE边沿从已完成目标严格增加一个弹位。
   * 从SINGLE直接切换其他动作仍要求先经过NEUTRAL。
   */
  if (feeder.debug.state == FEEDER_STATE_HOLDING_SINGLE)
  {
    if (feeder.debug.remote_command == FEEDER_REMOTE_NEUTRAL)
    {
      feeder.active_command = FEEDER_REMOTE_DISABLE;
      return;
    }

    if (feeder.debug.remote_command == FEEDER_REMOTE_SINGLE)
    {
      if (feeder.single_request_pending)
      {
        feeder.single_request_pending = false;
        if (!start_single_shot())
        {
          invalidate_single_phase();
          disarm_for_neutral();
          feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
        }
      }
      return;
    }

    invalidate_single_phase();
    if (feeder.active_command != FEEDER_REMOTE_DISABLE)
    {
      disarm_for_neutral();
      feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
      return;
    }
    feeder.debug.state = FEEDER_STATE_ARMED_NEUTRAL;
  }

  /*
   * 单发一旦启动，即使自复位拨轮马上回中，也必须完成当前固定步距。
   * DISABLE已在前面处理；若中途请求连发或退弹，则立即停止并要求重新
   * 经过NEUTRAL，禁止直接切换方向。
   */
  if (feeder.debug.state == FEEDER_STATE_RUNNING_SINGLE)
  {
    if ((feeder.debug.remote_command
         == FEEDER_REMOTE_CONTINUOUS)
        || (feeder.debug.remote_command
            == FEEDER_REMOTE_REVERSE))
    {
      invalidate_single_phase();
      disarm_for_neutral();
      feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
    }
    return;
  }

  if (feeder.debug.remote_command == FEEDER_REMOTE_NEUTRAL)
  {
    if ((feeder.active_command
         == FEEDER_REMOTE_CONTINUOUS)
        || (feeder.active_command
            == FEEDER_REMOTE_REVERSE))
    {
      disarm_for_neutral();
      feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
    }
    else
    {
      force_zero_output();
      feeder.debug.state = FEEDER_STATE_ARMED_NEUTRAL;
    }
    return;
  }

  if (feeder.debug.remote_command == FEEDER_REMOTE_SINGLE)
  {
    if (feeder.single_request_pending)
    {
      feeder.single_request_pending = false;
      if (!start_single_shot())
      {
        disarm_for_neutral();
        feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
      }
    }
    else
    {
      force_zero_output();
      feeder.debug.state = FEEDER_STATE_ARMED_NEUTRAL;
    }
    return;
  }

  if ((feeder.debug.remote_command
       != FEEDER_REMOTE_CONTINUOUS)
      && (feeder.debug.remote_command
          != FEEDER_REMOTE_REVERSE))
  {
    disarm_for_neutral();
    feeder.debug.state = FEEDER_STATE_DISABLED;
    return;
  }

  if ((feeder.active_command
       != FEEDER_REMOTE_DISABLE)
      && (feeder.active_command
          != feeder.debug.remote_command))
  {
    disarm_for_neutral();
    feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
    return;
  }

  feeder.active_command = feeder.debug.remote_command;
  invalidate_single_phase();
  feeder.debug.state =
      (feeder.active_command == FEEDER_REMOTE_CONTINUOUS)
      ? FEEDER_STATE_RUNNING_CONTINUOUS
      : FEEDER_STATE_RUNNING_REVERSE;
}

static void update_stall_protection(uint32_t now)
{
  int32_t speed = feeder.debug.speed_rpm;
  int32_t current = feeder.debug.command_current_raw;
  const bool running =
      (feeder.debug.state == FEEDER_STATE_RUNNING_CONTINUOUS)
      || (feeder.debug.state == FEEDER_STATE_RUNNING_SINGLE)
      || (feeder.debug.state == FEEDER_STATE_RUNNING_REVERSE);
  const bool single_requires_forward_motion =
      (feeder.debug.state != FEEDER_STATE_RUNNING_SINGLE)
      || (feeder.debug.position_error_ecd
          > FEEDER_SINGLE_POSITION_TOLERANCE_ECD);

  if (speed < 0)
  {
    speed = -speed;
  }
  if (current < 0)
  {
    current = -current;
  }

  if (running
      && single_requires_forward_motion
      && (speed <= FEEDER_STALL_SPEED_THRESHOLD_RPM)
      && (current >= FEEDER_STALL_CURRENT_THRESHOLD_RAW))
  {
    if (!feeder.stall_timing)
    {
      feeder.stall_start_ms = now;
      feeder.stall_timing = true;
    }
    else if ((uint32_t)(now - feeder.stall_start_ms)
             >= FEEDER_STALL_TIMEOUT_MS)
    {
      feeder.debug.fault_latched = true;
      feeder.debug.fault_reason = FEEDER_FAULT_STALL;
      feeder.debug.armed = false;
      feeder.debug.state = FEEDER_STATE_FAULT;
      invalidate_single_phase();
      force_zero_output();
    }
  }
  else
  {
    feeder.stall_timing = false;
  }
}

static void update_control(uint32_t now)
{
  uint32_t delta_ms =
      (uint32_t)(now - feeder.last_process_ms);
  float desired_speed_rpm = 0.0f;
  float current_target;
  int16_t current_limit_raw;
  int16_t next_current_raw;
  bool single_motion = false;
  bool single_hold = false;

  /* dt 来自实际调度间隔；上限防止暂停后突然跳出很大的斜坡步长。 */
  feeder.last_process_ms = now;
  if (delta_ms == 0U)
  {
    delta_ms = 1U;
  }
  else if (delta_ms > FEEDER_MAX_CONTROL_DELTA_MS)
  {
    delta_ms = FEEDER_MAX_CONTROL_DELTA_MS;
  }

  if ((feeder.debug.state
       != FEEDER_STATE_RUNNING_CONTINUOUS)
      && (feeder.debug.state
          != FEEDER_STATE_RUNNING_SINGLE)
      && (feeder.debug.state
          != FEEDER_STATE_HOLDING_SINGLE)
      && (feeder.debug.state
          != FEEDER_STATE_RUNNING_REVERSE))
  {
    force_zero_output();
    return;
  }

  if (feeder.debug.state
      == FEEDER_STATE_RUNNING_CONTINUOUS)
  {
    desired_speed_rpm = FEEDER_CONTINUOUS_SPEED_RPM;
    current_limit_raw =
        FEEDER_CONTINUOUS_CURRENT_LIMIT_RAW;
    feeder.debug.target_total_angle_ecd =
        feeder.debug.total_angle_ecd;
    feeder.debug.position_error_ecd = 0;
  }
  else if (feeder.debug.state
           == FEEDER_STATE_RUNNING_REVERSE)
  {
    desired_speed_rpm = -FEEDER_REVERSE_SPEED_RPM;
    current_limit_raw =
        FEEDER_REVERSE_CURRENT_LIMIT_RAW;
    feeder.debug.target_total_angle_ecd =
        feeder.debug.total_angle_ecd;
    feeder.debug.position_error_ecd = 0;
  }
  else if (feeder.debug.state
           == FEEDER_STATE_HOLDING_SINGLE)
  {
    const float position_error_output_deg =
        single_position_error_output_deg();

    current_limit_raw =
        FEEDER_SINGLE_HOLD_CURRENT_LIMIT_RAW;
    single_hold = true;
    if (feeder.debug.position_error_ecd
        > FEEDER_SINGLE_HOLD_DEADBAND_ECD)
    {
      desired_speed_rpm = clamp_float(
          FEEDER_SINGLE_POSITION_KP_RPM_PER_DEG
            * position_error_output_deg,
          0.0f,
          FEEDER_SINGLE_HOLD_MAX_SPEED_RPM);
    }
    else
    {
      desired_speed_rpm = 0.0f;
      feeder.speed_integral_raw = 0.0f;
      feeder.debug.pid_i_raw = 0.0f;
    }
  }
  else
  {
    const float position_error_output_deg =
        single_position_error_output_deg();
    int32_t speed = feeder.debug.speed_rpm;
    const int32_t position_error =
        feeder.debug.position_error_ecd;

    current_limit_raw =
        FEEDER_SINGLE_CURRENT_LIMIT_RAW;
    single_motion = true;
    if (speed < 0)
    {
      speed = -speed;
    }

    /*
     * 允许速度环在仍正转时使用负电流制动，但位置目标永不要求反转。
     * 若惯性导致超过允许的前向窗口，则锁存故障而不是反向找中心。
     */
    if (position_error
        < -FEEDER_SINGLE_MAX_FORWARD_OVERRUN_ECD)
    {
      feeder.debug.fault_latched = true;
      feeder.debug.fault_reason =
          FEEDER_FAULT_SINGLE_OVERRUN;
      feeder.debug.armed = false;
      feeder.debug.state = FEEDER_STATE_FAULT;
      invalidate_single_phase();
      force_zero_output();
      return;
    }

    if ((position_error
         <= FEEDER_SINGLE_POSITION_TOLERANCE_ECD)
        && (position_error
            >= -FEEDER_SINGLE_MAX_FORWARD_OVERRUN_ECD)
        && (speed <= FEEDER_SINGLE_SETTLE_SPEED_RPM))
    {
      if (!feeder.single_settle_timing)
      {
        feeder.single_settle_start_ms = now;
        feeder.single_settle_timing = true;
      }
      else if ((uint32_t)(
                   now - feeder.single_settle_start_ms)
               >= FEEDER_SINGLE_SETTLE_TIME_MS)
      {
        finish_single_shot();
        return;
      }
    }
    else
    {
      feeder.single_settle_timing = false;
    }

    if (position_error
        <= FEEDER_SINGLE_POSITION_TOLERANCE_ECD)
    {
      desired_speed_rpm = 0.0f;
      feeder.speed_integral_raw = 0.0f;
      feeder.debug.pid_i_raw = 0.0f;
    }
    else
    {
      desired_speed_rpm = clamp_float(
          FEEDER_SINGLE_POSITION_KP_RPM_PER_DEG
            * position_error_output_deg,
          0.0f,
          FEEDER_SINGLE_MAX_SPEED_RPM);
    }
  }

  feeder.ramped_target_speed_rpm = ramp_speed(
      feeder.ramped_target_speed_rpm,
      desired_speed_rpm,
      delta_ms);
  feeder.debug.target_speed_rpm =
      feeder.ramped_target_speed_rpm;

  current_target = update_speed_pid(
      feeder.ramped_target_speed_rpm,
      (float)feeder.debug.speed_rpm,
      delta_ms,
      current_limit_raw);

  /*
   * HOLD阶段只允许正向恢复力。单发运动阶段允许负电流给仍在正转的
   * 电机减速，但电机转速到零或已经反向后立即禁止负向驱动。
   */
  if ((single_hold && (current_target < 0.0f))
      || (single_motion
          && (feeder.debug.speed_rpm <= 0)
          && (current_target < 0.0f)))
  {
    current_target = 0.0f;
    feeder.speed_integral_raw = 0.0f;
    feeder.debug.pid_i_raw = 0.0f;
  }

  next_current_raw = slew_current(
      feeder.debug.command_current_raw,
      clamp_current(current_target, current_limit_raw),
      delta_ms);
  if ((single_hold
       || (single_motion
           && (feeder.debug.speed_rpm <= 0)))
      && (next_current_raw < 0))
  {
    next_current_raw = 0;
  }
  feeder.debug.command_current_raw = next_current_raw;
}

HAL_StatusTypeDef FeederMotor_Init(CAN_HandleTypeDef *hcan)
{
  HAL_StatusTypeDef status;

  if ((hcan == NULL) || (hcan->Instance != CAN1))
  {
    return HAL_ERROR;
  }

  memset(&feeder, 0, sizeof(feeder));
  feeder.can = hcan;
  feeder.debug.remote_command = FEEDER_REMOTE_DISABLE;
  feeder.debug.state = FEEDER_STATE_DISABLED;
  feeder.debug.fault_reason = FEEDER_FAULT_NONE;
  feeder.active_command = FEEDER_REMOTE_DISABLE;
  feeder.last_sent_current_raw = INT16_MIN;
  feeder.last_process_ms = HAL_GetTick();
  feeder.last_tx_ms =
      feeder.last_process_ms - FEEDER_ZERO_KEEPALIVE_MS;

  status = configure_feedback_filter();
  if (status != HAL_OK)
  {
    return status;
  }

  if (hcan->State == HAL_CAN_STATE_READY)
  {
    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK)
    {
      return status;
    }
  }
  else if (hcan->State != HAL_CAN_STATE_LISTENING)
  {
    return HAL_ERROR;
  }

  feeder.initialized = true;
  (void)send_current(0);
  return HAL_OK;
}

void FeederMotor_SetRemoteCommand(FeederRemoteCommand_t command)
{
  if ((command < FEEDER_REMOTE_DISABLE)
      || (command > FEEDER_REMOTE_REVERSE))
  {
    command = FEEDER_REMOTE_DISABLE;
  }

  if ((command == FEEDER_REMOTE_SINGLE)
      && (feeder.debug.remote_command
          != FEEDER_REMOTE_SINGLE))
  {
    feeder.single_request_pending = true;
  }
  else if (command != FEEDER_REMOTE_SINGLE)
  {
    feeder.single_request_pending = false;
  }
  feeder.debug.remote_command = command;
}

HAL_StatusTypeDef FeederMotor_EmergencyStop(void)
{
  feeder.debug.emergency_stop_latched = true;
  feeder.debug.remote_command = FEEDER_REMOTE_DISABLE;
  feeder.debug.armed = false;
  feeder.debug.state = FEEDER_STATE_ESTOP;
  feeder.single_request_pending = false;
  invalidate_single_phase();
  reset_neutral_timer();
  force_zero_output();

  if (!feeder.initialized)
  {
    return HAL_ERROR;
  }
  return send_current(0);
}

void FeederMotor_ClearEmergencyStop(void)
{
  feeder.debug.emergency_stop_latched = false;
  feeder.debug.remote_command = FEEDER_REMOTE_DISABLE;
  feeder.debug.armed = false;
  feeder.debug.state = FEEDER_STATE_DISABLED;
  feeder.single_request_pending = false;
  invalidate_single_phase();
  reset_neutral_timer();
  force_zero_output();
}

bool FeederMotor_IsEmergencyStopped(void)
{
  return feeder.debug.emergency_stop_latched;
}

void FeederMotor_Process(void)
{
  const uint32_t now = HAL_GetTick();
  const bool keepalive_due =
      (uint32_t)(now - feeder.last_tx_ms)
      >= FEEDER_ZERO_KEEPALIVE_MS;

  if (!feeder.initialized)
  {
    return;
  }

  /* 先更新反馈，再运行安全状态机，最后才允许PID和CAN输出。 */
  receive_feedback(now);
  update_safety_state(now);
  update_control(now);
  update_stall_protection(now);

  if ((feeder.debug.command_current_raw
       != feeder.last_sent_current_raw)
      || ((feeder.debug.command_current_raw == 0)
          && keepalive_due))
  {
    (void)send_current(
        feeder.debug.command_current_raw);
  }
}

bool FeederMotor_GetDebugData(FeederMotorDebugData_t *data)
{
  uint32_t primask;

  if (data == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *data = feeder.debug;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return feeder.initialized;
}
