/**
 * ===========================================================================
 * @file    feeder_motor.c
 * @brief   CAN1 C610 ID3 / M2006拨弹盘速度PID控制模块
 * ===========================================================================
 *
 * CAN协议：
 *   反馈：StdId 0x203，CAN1 FIFO1
 *   控制：StdId 0x200，DATA[4:5]为ID3有符号大端电流命令
 *
 * 安全策略：
 *   - 上电、急停、遥控掉线和方向切换后必须在中挡保持100 ms重新解锁；
 *   - 重新解锁还要求反馈在线、C610错误码为0且电机接近静止；
 *   - 方向A已确认为拨弹方向，速度环使用带anti-windup的PID；
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

  float ramped_target_speed_rpm;
  float speed_integral_raw;
  float previous_speed_error_rpm;
  float filtered_derivative_rpm_s;
  FeederRemoteCommand_t active_direction;
  uint32_t last_process_ms;
  uint32_t last_tx_ms;
  uint32_t neutral_start_ms;
  uint32_t stall_start_ms;
  int16_t last_sent_current_raw;
  bool neutral_timing;
  bool stall_timing;
  bool speed_pid_initialized;
  bool initialized;
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

static int16_t clamp_current(float value)
{
  if (!isfinite(value))
  {
    return 0;
  }
  if (value > (float)FEEDER_CURRENT_LIMIT_RAW)
  {
    return FEEDER_CURRENT_LIMIT_RAW;
  }
  if (value < (float)-FEEDER_CURRENT_LIMIT_RAW)
  {
    return -FEEDER_CURRENT_LIMIT_RAW;
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
                              uint32_t delta_ms)
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
  if (((candidate_output < (float)FEEDER_CURRENT_LIMIT_RAW)
       && (candidate_output
           > (float)-FEEDER_CURRENT_LIMIT_RAW))
      || ((candidate_output
           >= (float)FEEDER_CURRENT_LIMIT_RAW)
          && (error < 0.0f))
      || ((candidate_output
           <= (float)-FEEDER_CURRENT_LIMIT_RAW)
          && (error > 0.0f)))
  {
    feeder.speed_integral_raw = candidate_integral;
  }

  output = clamp_float(
      p_term + feeder.speed_integral_raw + d_term,
      (float)-FEEDER_CURRENT_LIMIT_RAW,
      (float)FEEDER_CURRENT_LIMIT_RAW);
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

static void force_zero_output(void)
{
  feeder.ramped_target_speed_rpm = 0.0f;
  feeder.debug.target_speed_rpm = 0.0f;
  feeder.debug.command_current_raw = 0;
  feeder.active_direction = FEEDER_REMOTE_DISABLE;
  feeder.stall_timing = false;
  reset_speed_pid();
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
    feeder.active_direction = FEEDER_REMOTE_DISABLE;
    reset_neutral_timer();
  }
}

static void update_safety_state(uint32_t now)
{
  if (feeder.debug.online
      && ((uint32_t)(now - feeder.debug.last_rx_ms)
          > FEEDER_FEEDBACK_TIMEOUT_MS))
  {
    feeder.debug.online = false;
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
    disarm_for_neutral();
    feeder.debug.state = FEEDER_STATE_DISABLED;
    return;
  }

  if (!feeder.debug.online)
  {
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

  if (feeder.debug.remote_command == FEEDER_REMOTE_NEUTRAL)
  {
    if ((feeder.active_direction
         == FEEDER_REMOTE_DIRECTION_A)
        || (feeder.active_direction
            == FEEDER_REMOTE_DIRECTION_B))
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

  if ((feeder.debug.remote_command
       != FEEDER_REMOTE_DIRECTION_A)
      && (feeder.debug.remote_command
          != FEEDER_REMOTE_DIRECTION_B))
  {
    disarm_for_neutral();
    feeder.debug.state = FEEDER_STATE_DISABLED;
    return;
  }

  if ((feeder.active_direction
       != FEEDER_REMOTE_DISABLE)
      && (feeder.active_direction
          != feeder.debug.remote_command))
  {
    disarm_for_neutral();
    feeder.debug.state = FEEDER_STATE_WAIT_NEUTRAL;
    return;
  }

  feeder.active_direction = feeder.debug.remote_command;
  feeder.debug.state =
      (feeder.active_direction == FEEDER_REMOTE_DIRECTION_A)
      ? FEEDER_STATE_RUNNING_A
      : FEEDER_STATE_RUNNING_B;
}

static void update_stall_protection(uint32_t now)
{
  int32_t speed = feeder.debug.speed_rpm;
  int32_t current = feeder.debug.command_current_raw;
  const bool running =
      (feeder.debug.state == FEEDER_STATE_RUNNING_A)
      || (feeder.debug.state == FEEDER_STATE_RUNNING_B);

  if (speed < 0)
  {
    speed = -speed;
  }
  if (current < 0)
  {
    current = -current;
  }

  if (running
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
  float desired_speed_rpm;
  float current_target;

  feeder.last_process_ms = now;
  if (delta_ms == 0U)
  {
    delta_ms = 1U;
  }
  else if (delta_ms > FEEDER_MAX_CONTROL_DELTA_MS)
  {
    delta_ms = FEEDER_MAX_CONTROL_DELTA_MS;
  }

  if ((feeder.debug.state != FEEDER_STATE_RUNNING_A)
      && (feeder.debug.state != FEEDER_STATE_RUNNING_B))
  {
    force_zero_output();
    return;
  }

  desired_speed_rpm =
      (feeder.debug.state == FEEDER_STATE_RUNNING_A)
      ? FEEDER_TEST_TARGET_SPEED_RPM
      : -FEEDER_TEST_TARGET_SPEED_RPM;
  feeder.ramped_target_speed_rpm = ramp_speed(
      feeder.ramped_target_speed_rpm,
      desired_speed_rpm,
      delta_ms);
  feeder.debug.target_speed_rpm =
      feeder.ramped_target_speed_rpm;

  current_target = update_speed_pid(
      feeder.ramped_target_speed_rpm,
      (float)feeder.debug.speed_rpm,
      delta_ms);
  feeder.debug.command_current_raw = slew_current(
      feeder.debug.command_current_raw,
      clamp_current(current_target),
      delta_ms);
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
      || (command > FEEDER_REMOTE_DIRECTION_B))
  {
    command = FEEDER_REMOTE_DISABLE;
  }
  feeder.debug.remote_command = command;
}

HAL_StatusTypeDef FeederMotor_EmergencyStop(void)
{
  feeder.debug.emergency_stop_latched = true;
  feeder.debug.remote_command = FEEDER_REMOTE_DISABLE;
  feeder.debug.armed = false;
  feeder.debug.state = FEEDER_STATE_ESTOP;
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
