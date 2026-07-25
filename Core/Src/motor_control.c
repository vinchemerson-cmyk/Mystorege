#include "motor_control.h"

#include <math.h>
#include <string.h>

/*
 * GM6020 电机 ID 为 1，使用转矩电流控制模式。
 *
 * 位置模式采用串级控制：
 *   角度 PID -> 目标速度 -> 速度 PID -> 目标转矩电流。
 *
 * GM6020 以 1 kHz 发送反馈，因此每收到一帧有效反馈就更新一次控制器，
 * 角度环和速度环的固定采样周期均为 1 ms。
 */
#define GM6020_FEEDBACK_STD_ID       0x205U
#define GM6020_CURRENT_COMMAND_ID    0x1FEU
#define GM6020_ENCODER_CPR           8192
#define GM6020_ENCODER_HALF_CPR      4096
#define GM6020_CONTROL_PERIOD_S      0.001f
#define GM6020_FEEDBACK_TIMEOUT_MS   100U

/* 速度环初始参数；应在安装实际负载后重新整定。 */
#define SPEED_PID_KP                 20.0f
#define SPEED_PID_KI                 25.0f
#define SPEED_PID_KD                 0.0f
/* 电流指令 8192 约对应 1.5 A，用作初始安全限幅。 */
#define SPEED_PID_OUTPUT_LIMIT       8192.0f

/* 角度外环参数；输出量为目标速度，单位 rpm。 */
#define ANGLE_PID_KP                 10.0f
#define ANGLE_PID_KI                 0.0f
#define ANGLE_PID_KD                 0.0f
#define ANGLE_PID_SPEED_LIMIT_RPM    100.0f

/* 内部状态机不对外暴露，只保留等待、位置控制和故障三个状态。 */
typedef enum
{
  GM6020_STATE_WAIT_FEEDBACK = 0,
  GM6020_STATE_POSITION_CONTROL,
  GM6020_STATE_FAULT
} GM6020_ControlState_t;

typedef struct
{
  float kp;             /* 比例系数 */
  float ki;             /* 积分系数 */
  float kd;             /* 微分系数 */
  float integral;       /* 积分项状态 */
  float previous_error; /* 上一次误差，用于计算微分项 */
  float output;         /* PID 输出：转矩电流原始指令 */
} SpeedPID_t;

typedef struct
{
  float kp;             /* 比例系数 */
  float ki;             /* 积分系数 */
  float kd;             /* 微分系数 */
  float integral;       /* 积分项状态 */
  float previous_error; /* 上一次角度误差 */
  float output;         /* PID 输出：目标速度，单位 rpm */
} AnglePID_t;

/* 外设句柄、反馈状态以及控制器运行状态。 */
static CAN_HandleTypeDef *motor_can;
static GM6020_Feedback_t motor_feedback;
static SpeedPID_t speed_pid = {
  .kp = SPEED_PID_KP,
  .ki = SPEED_PID_KI,
  .kd = SPEED_PID_KD
};
static AnglePID_t angle_pid = {
  .kp = ANGLE_PID_KP,
  .ki = ANGLE_PID_KI,
  .kd = ANGLE_PID_KD
};
static float target_speed_rpm;
static float requested_angle_deg;
static GM6020_ArcMode_t requested_arc_mode = GM6020_ARC_MINOR;
static bool position_target_valid;
static int32_t target_total_angle_ecd;
static uint16_t previous_encoder;
static bool encoder_initialized;
static GM6020_ControlState_t control_state =
    GM6020_STATE_WAIT_FEEDBACK;

/* 将浮点值限制在给定上下限之间。 */
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

/* 清空速度环状态，模式切换或电机掉线时调用。 */
static void speed_pid_reset(void)
{
  speed_pid.integral = 0.0f;
  speed_pid.previous_error = 0.0f;
  speed_pid.output = 0.0f;
}

/* 清空角度环状态，避免旧积分量影响新的控制目标。 */
static void angle_pid_reset(void)
{
  angle_pid.integral = 0.0f;
  angle_pid.previous_error = 0.0f;
  angle_pid.output = 0.0f;
}

/* 将任意角度归一化到 [0°, 360°) 范围。 */
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

/* 把单圈角度转换为 GM6020 的 0~8191 编码器值。 */
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

/*
 * 根据当前机械位置，将单圈目标解析成多圈目标。
 * 劣弧选择最短路径；优弧选择相反方向的长路径。
 */
static void angle_resolve_single_turn_target(float angle_deg,
                                              GM6020_ArcMode_t arc_mode)
{
  int32_t desired_encoder;
  int32_t delta;

  if (!encoder_initialized)
  {
    return;
  }

  desired_encoder = single_turn_degrees_to_encoder(angle_deg);

  delta = desired_encoder - (int32_t)motor_feedback.angle;
  if (delta > GM6020_ENCODER_HALF_CPR)
  {
    delta -= GM6020_ENCODER_CPR;
  }
  else if (delta < -GM6020_ENCODER_HALF_CPR)
  {
    delta += GM6020_ENCODER_CPR;
  }

  if (arc_mode == GM6020_ARC_MAJOR)
  {
    if (delta > 0)
    {
      delta -= GM6020_ENCODER_CPR;
    }
    else if (delta < 0)
    {
      delta += GM6020_ENCODER_CPR;
    }
    else
    {
      /* 当前角度与目标相同且要求走优弧时，正向旋转一整圈。 */
      delta = GM6020_ENCODER_CPR;
    }
  }

  target_total_angle_ecd = motor_feedback.total_angle_ecd + delta;
  angle_pid_reset();
}

/*
 * 更新编码器多圈角度。
 * 相邻采样差值超过半圈时，认为编码器跨越了 0/8191 边界。
 */
static void encoder_update(uint16_t encoder)
{
  int32_t delta;

  if (!encoder_initialized)
  {
    previous_encoder = encoder;
    motor_feedback.turn_count = 0;
    motor_feedback.total_angle_ecd = encoder;
    motor_feedback.total_angle_deg =
        (float)encoder * 360.0f / (float)GM6020_ENCODER_CPR;
    encoder_initialized = true;
    return;
  }

  delta = (int32_t)encoder - (int32_t)previous_encoder;
  if (delta > GM6020_ENCODER_HALF_CPR)
  {
    motor_feedback.turn_count--;
  }
  else if (delta < -GM6020_ENCODER_HALF_CPR)
  {
    motor_feedback.turn_count++;
  }

  previous_encoder = encoder;
  motor_feedback.total_angle_ecd =
      motor_feedback.turn_count * GM6020_ENCODER_CPR + (int32_t)encoder;
  motor_feedback.total_angle_deg =
      (float)motor_feedback.total_angle_ecd * 360.0f
      / (float)GM6020_ENCODER_CPR;
}

/* 执行角度外环，输入多圈角度误差，输出目标速度 rpm。 */
static float angle_pid_update(void)
{
  const float error =
      (float)(target_total_angle_ecd - motor_feedback.total_angle_ecd)
      * 360.0f / (float)GM6020_ENCODER_CPR;
  const float derivative =
      (error - angle_pid.previous_error) / GM6020_CONTROL_PERIOD_S;
  const float proportional_and_derivative =
      angle_pid.kp * error + angle_pid.kd * derivative;
  const float candidate_integral = clamp_float(
      angle_pid.integral + angle_pid.ki * error * GM6020_CONTROL_PERIOD_S,
      -ANGLE_PID_SPEED_LIMIT_RPM, ANGLE_PID_SPEED_LIMIT_RPM);
  const float candidate_output =
      proportional_and_derivative + candidate_integral;

  /* 角度环同样使用条件积分，防止目标速度达到限幅后继续积分。 */
  if (((candidate_output < ANGLE_PID_SPEED_LIMIT_RPM)
       && (candidate_output > -ANGLE_PID_SPEED_LIMIT_RPM))
      || ((candidate_output >= ANGLE_PID_SPEED_LIMIT_RPM) && (error < 0.0f))
      || ((candidate_output <= -ANGLE_PID_SPEED_LIMIT_RPM) && (error > 0.0f)))
  {
    angle_pid.integral = candidate_integral;
  }

  angle_pid.output = proportional_and_derivative + angle_pid.integral;
  angle_pid.output = clamp_float(angle_pid.output,
                                 -ANGLE_PID_SPEED_LIMIT_RPM,
                                 ANGLE_PID_SPEED_LIMIT_RPM);
  angle_pid.previous_error = error;

  return angle_pid.output;
}

/* 执行速度内环，输入速度误差，输出 GM6020 转矩电流指令。 */
static float speed_pid_update(float target, float feedback)
{
  const float error = target - feedback;
  const float derivative =
      (error - speed_pid.previous_error) / GM6020_CONTROL_PERIOD_S;
  const float proportional_and_derivative =
      speed_pid.kp * error + speed_pid.kd * derivative;
  const float candidate_integral = clamp_float(
      speed_pid.integral + speed_pid.ki * error * GM6020_CONTROL_PERIOD_S,
      -SPEED_PID_OUTPUT_LIMIT, SPEED_PID_OUTPUT_LIMIT);
  const float candidate_output =
      proportional_and_derivative + candidate_integral;

  /* 条件积分抗饱和：输出饱和时，只允许能让输出退出饱和的积分方向。 */
  if (((candidate_output < SPEED_PID_OUTPUT_LIMIT)
       && (candidate_output > -SPEED_PID_OUTPUT_LIMIT))
      || ((candidate_output >= SPEED_PID_OUTPUT_LIMIT) && (error < 0.0f))
      || ((candidate_output <= -SPEED_PID_OUTPUT_LIMIT) && (error > 0.0f)))
  {
    speed_pid.integral = candidate_integral;
  }

  speed_pid.output = proportional_and_derivative + speed_pid.integral;
  speed_pid.output = clamp_float(speed_pid.output,
                                 -SPEED_PID_OUTPUT_LIMIT,
                                 SPEED_PID_OUTPUT_LIMIT);
  speed_pid.previous_error = error;

  return speed_pid.output;
}

/* 通过 0x1FE 标准帧向 ID 1 电机发送转矩电流指令。 */
static HAL_StatusTypeDef gm6020_send_current(int16_t current)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  uint32_t mailbox;

  tx_header.StdId = GM6020_CURRENT_COMMAND_ID;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  /* ID 1 使用 DATA[0:1]，按高字节在前的顺序发送。 */
  tx_data[0] = (uint8_t)(((uint16_t)current) >> 8);
  tx_data[1] = (uint8_t)current;

  return HAL_CAN_AddTxMessage(motor_can, &tx_header, tx_data, &mailbox);
}

/*=========================== 控制状态机 ===========================*/

typedef void (*ControlStateFeedbackHandler_t)(void);

/* 等待态和故障态都禁止输出转矩。 */
static void state_zero_current_on_feedback(void)
{
  (void)gm6020_send_current(0);
}

/* 位置串级状态：角度 PID 的输出作为速度 PID 的目标速度。 */
static void state_position_control_on_feedback(void)
{
  target_speed_rpm = angle_pid_update();
  (void)gm6020_send_current((int16_t)speed_pid_update(
      target_speed_rpm, (float)motor_feedback.speed_rpm));
}

/*
 * 每个状态只绑定自己的反馈处理动作。
 * 状态转换集中在 control_state_transition()，业务处理中不再散落模式 if。
 */
static const ControlStateFeedbackHandler_t control_state_handlers[] =
{
  state_zero_current_on_feedback,      /* GM6020_STATE_WAIT_FEEDBACK */
  state_position_control_on_feedback,  /* GM6020_STATE_POSITION_CONTROL */
  state_zero_current_on_feedback       /* GM6020_STATE_FAULT */
};

/*
 * 状态入口动作统一放在这里：
 * 进入位置控制态时复位 PID；进入故障态时立即输出零电流。
 */
static void control_state_transition(GM6020_ControlState_t next_state)
{
  control_state = next_state;

  switch (control_state)
  {
    case GM6020_STATE_WAIT_FEEDBACK:
      target_speed_rpm = 0.0f;
      speed_pid_reset();
      angle_pid_reset();
      (void)gm6020_send_current(0);
      break;

    case GM6020_STATE_POSITION_CONTROL:
      target_speed_rpm = 0.0f;
      speed_pid_reset();
      angle_pid_reset();
      if (position_target_valid)
      {
        angle_resolve_single_turn_target(requested_angle_deg,
                                         requested_arc_mode);
      }
      else
      {
        /* 尚未收到位置命令时锁定上电位置，防止电机突然转动。 */
        target_total_angle_ecd = motor_feedback.total_angle_ecd;
        requested_angle_deg =
            normalize_single_turn_degrees(motor_feedback.total_angle_deg);
      }
      break;

    case GM6020_STATE_FAULT:
    default:
      control_state = GM6020_STATE_FAULT;
      motor_feedback.online = false;
      encoder_initialized = false;
      target_speed_rpm = 0.0f;
      speed_pid_reset();
      angle_pid_reset();
      (void)gm6020_send_current(0);
      break;
  }
}

/* 有效反馈事件：等待态/故障态进入位置控制态。 */
static void control_state_handle_feedback(void)
{
  switch (control_state)
  {
    case GM6020_STATE_WAIT_FEEDBACK:
    case GM6020_STATE_FAULT:
      control_state_transition(GM6020_STATE_POSITION_CONTROL);
      break;

    case GM6020_STATE_POSITION_CONTROL:
      break;

    default:
      control_state_transition(GM6020_STATE_FAULT);
      return;
  }

  control_state_handlers[control_state]();
}

/* 只有位置控制态需要检查反馈超时，其余状态不重复触发故障入口动作。 */
//看门狗机制
static void control_state_poll_timeout(uint32_t now)
{
  switch (control_state)
  {
    case GM6020_STATE_POSITION_CONTROL:
      if ((uint32_t)(now - motor_feedback.last_rx_ms)
          > GM6020_FEEDBACK_TIMEOUT_MS)
      {
        control_state_transition(GM6020_STATE_FAULT);
      }
      break;

    case GM6020_STATE_WAIT_FEEDBACK:
    case GM6020_STATE_FAULT:
    default:
      break;
  }
}

/*
 * 初始化 GM6020 控制模块：
 * 保存外设句柄、清空状态、配置 0x205 接收滤波器并启动 CAN。
 */
HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef filter = {0};
  HAL_StatusTypeDef status;

  if (hcan == NULL)
  {
    return HAL_ERROR;
  }

  motor_can = hcan;
  memset(&motor_feedback, 0, sizeof(motor_feedback));
  speed_pid_reset();
  angle_pid_reset();
  encoder_initialized = false;
  control_state = GM6020_STATE_WAIT_FEEDBACK;
  requested_angle_deg = 0.0f;
  requested_arc_mode = GM6020_ARC_MINOR;
  position_target_valid = false;
  target_speed_rpm = 0.0f;
  target_total_angle_ecd = 0;

  /* 仅允许 ID 1 电机的 0x205 标准反馈帧进入 FIFO0。 */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (uint16_t)(GM6020_FEEDBACK_STD_ID << 5);
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  status = HAL_CAN_ConfigFilter(motor_can, &filter);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_CAN_Start(motor_can);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 启动后先发送零电流，收到第一帧有效反馈前不驱动电机。 */
  return gm6020_send_current(0);
}

/*
 * 唯一的位置命令接口。
 * 未收到反馈时先缓存目标；进入位置控制态后自动解析为多圈目标。
 */
void GM6020_SetTargetPosition(float target_angle_deg,
                              GM6020_ArcMode_t arc_mode)
{
  if (!isfinite(target_angle_deg))
  {
    return;
  }

  switch (arc_mode)
  {
    case GM6020_ARC_MINOR:
    case GM6020_ARC_MAJOR:
      break;

    default:
      return;
  }

  requested_angle_deg = normalize_single_turn_degrees(target_angle_deg);
  requested_arc_mode = arc_mode;
  position_target_valid = true;

  switch (control_state)
  {
    case GM6020_STATE_POSITION_CONTROL:
      angle_resolve_single_turn_target(requested_angle_deg,
                                       requested_arc_mode);
      break;

    case GM6020_STATE_WAIT_FEEDBACK:
    case GM6020_STATE_FAULT:
    default:
      break;
  }
}

/*
 * 电机控制主处理函数，应在 while(1) 中尽可能高频调用。
 * 控制计算由 1 kHz CAN 反馈触发。
 */
void GM6020_Process(void)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint32_t now = HAL_GetTick();

  /* 一次处理 FIFO 中的全部反馈，避免串口发送期间积压报文。 */
  while (HAL_CAN_GetRxFifoFillLevel(motor_can, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(motor_can, CAN_RX_FIFO0,
                            &rx_header, rx_data) != HAL_OK)
    {
      break;
    }

    /* 严格检查标准数据帧、反馈 ID 和 8 字节数据长度。 */
    if ((rx_header.IDE != CAN_ID_STD)
        || (rx_header.RTR != CAN_RTR_DATA)
        || (rx_header.StdId != GM6020_FEEDBACK_STD_ID)
        || (rx_header.DLC != 8U))
    {
      continue;
    }

    /* GM6020 反馈采用大端字节序：角度、速度、电流各占 2 字节。 */
    motor_feedback.angle =
        (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
    encoder_update(motor_feedback.angle);
    motor_feedback.speed_rpm =
        (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
    motor_feedback.torque_current =
        (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
    motor_feedback.temperature = rx_data[6];
    motor_feedback.last_rx_ms = now;
    motor_feedback.online = true;

    /* 将有效反馈作为事件交给状态机。 */
    control_state_handle_feedback();
  }

  now = HAL_GetTick();
  /* 超时判断只产生状态迁移，故障入口动作由状态机统一执行。 */
  control_state_poll_timeout(now);
}

/* 返回反馈结构的只读指针，不允许外部模块直接修改内部状态。 */
const GM6020_Feedback_t *GM6020_GetFeedback(void)
{
  return &motor_feedback;
}
