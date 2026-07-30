/**
 * ===========================================================================
 * @file    dual_m3508.c
 * @brief   CAN2双C620/M3508摩擦轮速度PID与联锁控制
 * ===========================================================================
 *
 * CAN协议：
 *   反馈：ID1=0x201，ID2=0x202，CAN2 FIFO1
 *   控制：0x200，ID1使用DATA[0:1]，ID2使用DATA[2:3]
 *
 * 逻辑方向：
 *   两个物理电机方向相反，但对上层都表现为正目标转速。
 *
 * 安全策略：
 *   - 两台电机必须同时在线才允许输出；
 *   - 任一反馈超时、过温或堵转都会锁存故障并停止两台电机；
 *   - 故障后必须先关闭请求，待两台在线且接近静止100 ms才可清除；
 *   - 急停/CLEAR不会恢复旧运行请求；
 *   - 双机连续到速200 ms后置位READY，供调试和后续联锁扩展使用。
 * ===========================================================================
 */

#include "dual_m3508.h"

#include "config/dual_m3508_params.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
  float integral_raw;
  float previous_error_rpm;
  float filtered_derivative_rpm_s;
  float logical_current_raw;
  uint32_t stall_start_ms;
  bool initialized;
  bool stall_timing;
} DualM3508PidState_t;

typedef struct
{
  CAN_HandleTypeDef *can;
  DualM3508DebugData_t debug;
  DualM3508PidState_t pid[DUAL_M3508_MOTOR_COUNT];
  float ramped_target_speed_rpm;
  uint32_t last_process_ms;
  uint32_t ready_start_ms;
  uint32_t rearm_start_ms;
  bool ready_timing;
  bool rearm_timing;
  bool enable_seen_off;
  bool initialized;
} DualM3508Context_t;

static DualM3508Context_t friction;

static const uint16_t feedback_ids[DUAL_M3508_MOTOR_COUNT] =
{
  DUAL_M3508_MOTOR1_FEEDBACK_STD_ID,
  DUAL_M3508_MOTOR2_FEEDBACK_STD_ID
};

static const int8_t motor_directions[DUAL_M3508_MOTOR_COUNT] =
{
  DUAL_M3508_MOTOR1_DIRECTION,
  DUAL_M3508_MOTOR2_DIRECTION
};

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

static int16_t round_to_int16(float value)
{
  value = clamp_float(
      value,
      (float)-DUAL_M3508_CURRENT_LIMIT_RAW,
      (float)DUAL_M3508_CURRENT_LIMIT_RAW);
  return (int16_t)((value >= 0.0f)
      ? (value + 0.5f)
      : (value - 0.5f));
}

static float ramp_float(float current, float target,
                        float rate_per_second, uint32_t delta_ms)
{
  const float maximum_step =
      rate_per_second * (float)delta_ms / 1000.0f;

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

static float slew_current(float current, float target,
                          uint32_t delta_ms)
{
  const float maximum_step =
      (float)DUAL_M3508_CURRENT_SLEW_RAW_PER_MS
      * (float)delta_ms;

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

static void reset_pid(uint32_t motor_index)
{
  DualM3508PidState_t *pid = &friction.pid[motor_index];
  DualM3508MotorDebugData_t *debug =
      &friction.debug.motor[motor_index];

  pid->integral_raw = 0.0f;
  pid->previous_error_rpm = 0.0f;
  pid->filtered_derivative_rpm_s = 0.0f;
  pid->logical_current_raw = 0.0f;
  pid->stall_start_ms = 0U;
  pid->initialized = false;
  pid->stall_timing = false;

  debug->command_current_raw = 0;
  debug->speed_error_rpm = 0.0f;
  debug->pid_p_raw = 0.0f;
  debug->pid_i_raw = 0.0f;
  debug->pid_d_raw = 0.0f;
  debug->pid_output_raw = 0.0f;
}

static void force_zero_output(void)
{
  uint32_t motor_index;

  friction.ramped_target_speed_rpm = 0.0f;
  friction.debug.target_speed_rpm = 0.0f;
  friction.debug.ready = false;
  friction.ready_timing = false;
  friction.ready_start_ms = 0U;

  for (motor_index = 0U;
       motor_index < DUAL_M3508_MOTOR_COUNT;
       ++motor_index)
  {
    reset_pid(motor_index);
  }
}

static HAL_StatusTypeDef configure_feedback_filter(void)
{
  CAN_FilterTypeDef filter = {0};

  if (friction.can == NULL)
  {
    return HAL_ERROR;
  }

  filter.FilterBank = DUAL_M3508_CAN_FILTER_BANK;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterIdHigh =
      (uint16_t)(DUAL_M3508_MOTOR1_FEEDBACK_STD_ID << 5U);
  filter.FilterIdLow =
      (uint16_t)(DUAL_M3508_MOTOR2_FEEDBACK_STD_ID << 5U);
  filter.FilterMaskIdHigh =
      (uint16_t)(DUAL_M3508_MOTOR1_FEEDBACK_STD_ID << 5U);
  filter.FilterMaskIdLow =
      (uint16_t)(DUAL_M3508_MOTOR2_FEEDBACK_STD_ID << 5U);
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;
  return HAL_CAN_ConfigFilter(friction.can, &filter);
}

static HAL_StatusTypeDef send_currents(void)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t data[8] = {0};
  uint32_t mailbox;
  uint32_t motor_index;
  HAL_StatusTypeDef status;

  if ((friction.can == NULL)
      || (friction.can->State != HAL_CAN_STATE_LISTENING))
  {
    return HAL_ERROR;
  }

  header.StdId = DUAL_M3508_CONTROL_STD_ID;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  header.TransmitGlobalTime = DISABLE;

  for (motor_index = 0U;
       motor_index < DUAL_M3508_MOTOR_COUNT;
       ++motor_index)
  {
    const uint16_t current =
        (uint16_t)friction.debug.motor[
            motor_index].command_current_raw;
    const uint32_t offset = motor_index * 2U;
    data[offset] = (uint8_t)(current >> 8U);
    data[offset + 1U] = (uint8_t)current;
  }

  status = HAL_CAN_AddTxMessage(
      friction.can, &header, data, &mailbox);
  if (status != HAL_OK)
  {
    ++friction.debug.tx_error_count;
  }
  return status;
}

static int32_t absolute_int32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static bool both_motors_online(void)
{
  return friction.debug.motor[0].online
      && friction.debug.motor[1].online;
}

static bool both_motors_stopped(void)
{
  return absolute_int32(
             friction.debug.motor[0].logical_speed_rpm)
             <= DUAL_M3508_REARM_MAX_SPEED_RPM
      && absolute_int32(
             friction.debug.motor[1].logical_speed_rpm)
             <= DUAL_M3508_REARM_MAX_SPEED_RPM;
}

static void parse_feedback(
    const CAN_RxHeaderTypeDef *header,
    const uint8_t data[8],
    uint32_t now)
{
  uint32_t motor_index;
  DualM3508MotorDebugData_t *motor;

  if ((header == NULL)
      || (data == NULL)
      || (header->IDE != CAN_ID_STD)
      || (header->RTR != CAN_RTR_DATA)
      || (header->DLC != 8U))
  {
    return;
  }

  if (header->StdId == feedback_ids[0])
  {
    motor_index = 0U;
  }
  else if (header->StdId == feedback_ids[1])
  {
    motor_index = 1U;
  }
  else
  {
    return;
  }

  motor = &friction.debug.motor[motor_index];
  motor->angle =
      (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
  motor->raw_speed_rpm =
      (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
  motor->logical_speed_rpm =
      (int16_t)(motor->raw_speed_rpm
                * motor_directions[motor_index]);
  motor->actual_current_raw =
      (int16_t)(((uint16_t)data[4] << 8U) | data[5]);
  motor->temperature_c = data[6];
  motor->last_rx_ms = now;
  ++motor->rx_sequence;
  motor->online = true;
}

static void receive_feedback(uint32_t now)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  while ((friction.can != NULL)
         && (HAL_CAN_GetRxFifoFillLevel(
                 friction.can, CAN_RX_FIFO1) > 0U))
  {
    if (HAL_CAN_GetRxMessage(
            friction.can,
            CAN_RX_FIFO1,
            &header,
            data) != HAL_OK)
    {
      break;
    }
    parse_feedback(&header, data, now);
  }
}

static void update_online_state(uint32_t now)
{
  uint32_t motor_index;

  for (motor_index = 0U;
       motor_index < DUAL_M3508_MOTOR_COUNT;
       ++motor_index)
  {
    DualM3508MotorDebugData_t *motor =
        &friction.debug.motor[motor_index];

    if (motor->online
        && ((uint32_t)(now - motor->last_rx_ms)
            > DUAL_M3508_FEEDBACK_TIMEOUT_MS))
    {
      motor->online = false;
    }
  }
}

static void latch_fault(DualM3508FaultReason_t reason)
{
  if (!friction.debug.fault_latched)
  {
    friction.debug.fault_reason = reason;
  }
  friction.debug.fault_latched = true;
  friction.debug.state = DUAL_M3508_STATE_FAULT;
  force_zero_output();
}

static void update_fault_rearm(uint32_t now)
{
  if (friction.debug.enable_requested
      || !both_motors_online()
      || !both_motors_stopped()
      || (friction.debug.motor[0].temperature_c
          >= DUAL_M3508_MAX_TEMPERATURE_C)
      || (friction.debug.motor[1].temperature_c
          >= DUAL_M3508_MAX_TEMPERATURE_C))
  {
    friction.rearm_timing = false;
    return;
  }

  if (!friction.rearm_timing)
  {
    friction.rearm_start_ms = now;
    friction.rearm_timing = true;
    return;
  }

  if ((uint32_t)(now - friction.rearm_start_ms)
      >= DUAL_M3508_REARM_HOLD_MS)
  {
    friction.debug.fault_latched = false;
    friction.debug.fault_reason = DUAL_M3508_FAULT_NONE;
    friction.debug.state = DUAL_M3508_STATE_DISABLED;
    friction.rearm_timing = false;
  }
}

static void update_state(uint32_t now)
{
  const bool was_running =
      (friction.debug.state == DUAL_M3508_STATE_RAMPING)
      || (friction.debug.state == DUAL_M3508_STATE_READY);

  if (friction.debug.emergency_stop_latched)
  {
    friction.debug.state = DUAL_M3508_STATE_ESTOP;
    force_zero_output();
    return;
  }

  if (friction.debug.fault_latched)
  {
    friction.debug.state = DUAL_M3508_STATE_FAULT;
    force_zero_output();
    update_fault_rearm(now);
    return;
  }

  if (!friction.debug.enable_requested)
  {
    friction.debug.state = DUAL_M3508_STATE_DISABLED;
    force_zero_output();
    return;
  }

  if (!both_motors_online())
  {
    if (was_running)
    {
      latch_fault(DUAL_M3508_FAULT_FEEDBACK);
    }
    else
    {
      friction.debug.state = DUAL_M3508_STATE_WAIT_FEEDBACK;
      force_zero_output();
    }
    return;
  }

  if ((friction.debug.motor[0].temperature_c
       >= DUAL_M3508_MAX_TEMPERATURE_C)
      || (friction.debug.motor[1].temperature_c
          >= DUAL_M3508_MAX_TEMPERATURE_C))
  {
    latch_fault(DUAL_M3508_FAULT_OVERTEMPERATURE);
    return;
  }

  friction.debug.state = friction.debug.ready
      ? DUAL_M3508_STATE_READY
      : DUAL_M3508_STATE_RAMPING;
}

static float update_speed_pid(uint32_t motor_index,
                              float target_speed_rpm,
                              float feedback_speed_rpm,
                              uint32_t delta_ms)
{
  DualM3508PidState_t *pid = &friction.pid[motor_index];
  DualM3508MotorDebugData_t *debug =
      &friction.debug.motor[motor_index];
  const float delta_s = (float)delta_ms / 1000.0f;
  const float error = target_speed_rpm - feedback_speed_rpm;
  const float p_term = DUAL_M3508_SPEED_KP * error;
  float raw_derivative = 0.0f;
  float d_term;
  float candidate_integral;
  float candidate_output;
  float output;

  if (pid->initialized && (delta_s > 0.0f))
  {
    raw_derivative =
        (error - pid->previous_error_rpm) / delta_s;
  }
  else
  {
    pid->initialized = true;
  }

  if (DUAL_M3508_D_FILTER_HZ > 0.0f)
  {
    const float time_constant =
        1.0f / (2.0f * 3.14159265358979323846f
                * DUAL_M3508_D_FILTER_HZ);
    const float alpha =
        delta_s / (time_constant + delta_s);
    pid->filtered_derivative_rpm_s +=
        alpha
        * (raw_derivative
           - pid->filtered_derivative_rpm_s);
  }
  else
  {
    pid->filtered_derivative_rpm_s = raw_derivative;
  }
  d_term =
      DUAL_M3508_SPEED_KD * pid->filtered_derivative_rpm_s;

  candidate_integral = clamp_float(
      pid->integral_raw
      + DUAL_M3508_SPEED_KI * error * delta_s,
      -DUAL_M3508_INTEGRAL_LIMIT_RAW,
      DUAL_M3508_INTEGRAL_LIMIT_RAW);
  candidate_output = p_term + candidate_integral + d_term;

  if (((candidate_output
        < (float)DUAL_M3508_CURRENT_LIMIT_RAW)
       && (candidate_output
           > (float)-DUAL_M3508_CURRENT_LIMIT_RAW))
      || ((candidate_output
           >= (float)DUAL_M3508_CURRENT_LIMIT_RAW)
          && (error < 0.0f))
      || ((candidate_output
           <= (float)-DUAL_M3508_CURRENT_LIMIT_RAW)
          && (error > 0.0f)))
  {
    pid->integral_raw = candidate_integral;
  }

  output = clamp_float(
      p_term + pid->integral_raw + d_term,
      (float)-DUAL_M3508_CURRENT_LIMIT_RAW,
      (float)DUAL_M3508_CURRENT_LIMIT_RAW);
  pid->previous_error_rpm = error;

  debug->speed_error_rpm = error;
  debug->pid_p_raw = p_term;
  debug->pid_i_raw = pid->integral_raw;
  debug->pid_d_raw = d_term;
  debug->pid_output_raw = output;
  return output;
}

static void update_ready_state(uint32_t now)
{
  const bool target_reached =
      friction.ramped_target_speed_rpm
      >= (DUAL_M3508_TARGET_SPEED_RPM
          - (float)DUAL_M3508_READY_TOLERANCE_RPM);
  const bool motors_at_speed =
      fabsf(friction.debug.motor[0].speed_error_rpm)
          <= (float)DUAL_M3508_READY_TOLERANCE_RPM
      && fabsf(friction.debug.motor[1].speed_error_rpm)
          <= (float)DUAL_M3508_READY_TOLERANCE_RPM;

  if (!target_reached || !motors_at_speed)
  {
    friction.debug.ready = false;
    friction.ready_timing = false;
    return;
  }

  if (!friction.ready_timing)
  {
    friction.ready_start_ms = now;
    friction.ready_timing = true;
    return;
  }

  if ((uint32_t)(now - friction.ready_start_ms)
      >= DUAL_M3508_READY_HOLD_MS)
  {
    friction.debug.ready = true;
    friction.debug.state = DUAL_M3508_STATE_READY;
  }
}

static void update_stall_protection(uint32_t now)
{
  uint32_t motor_index;

  for (motor_index = 0U;
       motor_index < DUAL_M3508_MOTOR_COUNT;
       ++motor_index)
  {
    DualM3508PidState_t *pid = &friction.pid[motor_index];
    const int32_t speed = absolute_int32(
        friction.debug.motor[motor_index].logical_speed_rpm);
    const int32_t current = absolute_int32(
        (int32_t)pid->logical_current_raw);

    if ((speed <= DUAL_M3508_STALL_SPEED_RPM)
        && (current >= DUAL_M3508_STALL_CURRENT_RAW))
    {
      if (!pid->stall_timing)
      {
        pid->stall_start_ms = now;
        pid->stall_timing = true;
      }
      else if ((uint32_t)(now - pid->stall_start_ms)
               >= DUAL_M3508_STALL_TIMEOUT_MS)
      {
        latch_fault(DUAL_M3508_FAULT_STALL);
        return;
      }
    }
    else
    {
      pid->stall_timing = false;
    }
  }
}

static void update_control(uint32_t now)
{
  uint32_t delta_ms =
      (uint32_t)(now - friction.last_process_ms);
  uint32_t motor_index;

  friction.last_process_ms = now;
  if (delta_ms == 0U)
  {
    delta_ms = 1U;
  }
  else if (delta_ms > DUAL_M3508_MAX_CONTROL_DELTA_MS)
  {
    delta_ms = DUAL_M3508_MAX_CONTROL_DELTA_MS;
  }

  if ((friction.debug.state != DUAL_M3508_STATE_RAMPING)
      && (friction.debug.state != DUAL_M3508_STATE_READY))
  {
    return;
  }

  friction.ramped_target_speed_rpm = ramp_float(
      friction.ramped_target_speed_rpm,
      DUAL_M3508_TARGET_SPEED_RPM,
      DUAL_M3508_TARGET_RAMP_RPM_S,
      delta_ms);
  friction.debug.target_speed_rpm =
      friction.ramped_target_speed_rpm;

  for (motor_index = 0U;
       motor_index < DUAL_M3508_MOTOR_COUNT;
       ++motor_index)
  {
    DualM3508PidState_t *pid = &friction.pid[motor_index];
    DualM3508MotorDebugData_t *motor =
        &friction.debug.motor[motor_index];
    const float pid_target = update_speed_pid(
        motor_index,
        friction.ramped_target_speed_rpm,
        (float)motor->logical_speed_rpm,
        delta_ms);

    pid->logical_current_raw = slew_current(
        pid->logical_current_raw,
        pid_target,
        delta_ms);
    motor->command_current_raw = round_to_int16(
        pid->logical_current_raw
        * (float)motor_directions[motor_index]);
  }

  update_ready_state(now);
  update_stall_protection(now);
}

HAL_StatusTypeDef DualM3508_Init(CAN_HandleTypeDef *hcan)
{
  HAL_StatusTypeDef status;

  if ((hcan == NULL) || (hcan->Instance != CAN2))
  {
    return HAL_ERROR;
  }

  memset(&friction, 0, sizeof(friction));
  friction.can = hcan;
  friction.debug.state = DUAL_M3508_STATE_DISABLED;
  friction.debug.fault_reason = DUAL_M3508_FAULT_NONE;
  friction.last_process_ms = HAL_GetTick();

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

  friction.initialized = true;
  (void)send_currents();
  return HAL_OK;
}

void DualM3508_SetEnabled(bool enabled)
{
  if (!enabled)
  {
    friction.debug.enable_requested = false;
    friction.enable_seen_off = true;
  }
  else if (friction.debug.enable_requested)
  {
    /* 已运行时允许持续保持请求。 */
  }
  else if (friction.enable_seen_off
           && !friction.debug.emergency_stop_latched)
  {
    friction.debug.enable_requested = true;
    friction.enable_seen_off = false;
  }
}

void DualM3508_DisableUntilOff(void)
{
  friction.debug.enable_requested = false;
  friction.enable_seen_off = false;
}

HAL_StatusTypeDef DualM3508_EmergencyStop(void)
{
  friction.debug.emergency_stop_latched = true;
  friction.debug.enable_requested = false;
  friction.enable_seen_off = false;
  friction.debug.state = DUAL_M3508_STATE_ESTOP;
  force_zero_output();

  if (!friction.initialized)
  {
    return HAL_ERROR;
  }
  return send_currents();
}

void DualM3508_ClearEmergencyStop(void)
{
  friction.debug.emergency_stop_latched = false;
  friction.debug.enable_requested = false;
  friction.enable_seen_off = false;
  friction.debug.state = friction.debug.fault_latched
      ? DUAL_M3508_STATE_FAULT
      : DUAL_M3508_STATE_DISABLED;
  force_zero_output();
}

bool DualM3508_IsEmergencyStopped(void)
{
  return friction.debug.emergency_stop_latched;
}

bool DualM3508_IsReady(void)
{
  return friction.initialized
      && friction.debug.ready
      && (friction.debug.state == DUAL_M3508_STATE_READY)
      && !friction.debug.emergency_stop_latched
      && !friction.debug.fault_latched;
}

void DualM3508_Process(void)
{
  const uint32_t now = HAL_GetTick();

  if (!friction.initialized)
  {
    return;
  }

  receive_feedback(now);
  update_online_state(now);
  update_state(now);
  update_control(now);
  (void)send_currents();
}

bool DualM3508_GetDebugData(DualM3508DebugData_t *data)
{
  uint32_t primask;

  if (data == NULL)
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *data = friction.debug;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return friction.initialized;
}
