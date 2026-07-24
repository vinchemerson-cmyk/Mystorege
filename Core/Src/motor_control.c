#include "motor_control.h"

#include <math.h>
#include <string.h>

/*
 * GM6020 ID 1, torque-current-control mode.
 *
 * Cascaded position control:
 *   angle PID -> target speed -> speed PID -> target torque current.
 *
 * The motor publishes feedback at 1 kHz.  Both loops are updated for every
 * received feedback frame, with a fixed 1 ms sample time.
 */
#define GM6020_FEEDBACK_STD_ID       0x205U
#define GM6020_CURRENT_COMMAND_ID    0x1FEU
#define GM6020_ENCODER_CPR           8192
#define GM6020_ENCODER_HALF_CPR      4096
#define GM6020_CONTROL_PERIOD_S      0.001f
#define GM6020_FEEDBACK_TIMEOUT_MS   100U
#define VOFA_SEND_PERIOD_MS          20U

/* Conservative initial settings; tune these values with the motor loaded. */
#define SPEED_PID_KP                 20.0f
#define SPEED_PID_KI                 25.0f
#define SPEED_PID_KD                 0.0f
/* 8192 raw units correspond to approximately 1.5 A. */
#define SPEED_PID_OUTPUT_LIMIT       8192.0f

/* Speed-only mode sine target: -100 to +100 rpm, 4 s period. */
#define SPEED_SINE_CENTER_RPM        0.0f
#define SPEED_SINE_AMPLITUDE_RPM     100.0f
#define SPEED_SINE_PERIOD_MS         4000U

/* Outer angle loop; its output is the target speed in rpm. */
#define ANGLE_PID_KP                 13.0f
#define ANGLE_PID_KI                 0.0f
#define ANGLE_PID_KD                 0.0f
#define ANGLE_PID_SPEED_LIMIT_RPM    100.0f

typedef enum
{
  ANGLE_ARC_MINOR = 0, /* Minor arc: shortest path, at most 180 degrees. */
  ANGLE_ARC_MAJOR      /* Major arc: longer path, at least 180 degrees. */
} AngleArcMode_t;

/*
 * Angle sine wave: 45 degree center, 45 degree amplitude, 4 s period.
 * The resulting target range is 0 to 90 degrees.
 * ANGLE_ARC_MODE selects the initial path from the mechanical position to
 * the continuous sine trajectory.
 */
#define ANGLE_SINE_CENTER_DEG        45.0f
#define ANGLE_SINE_AMPLITUDE_DEG     45.0f
#define ANGLE_SINE_PERIOD_MS         4000U
#define TWO_PI_F                     6.28318530718f
#define ANGLE_ARC_MODE               ANGLE_ARC_MINOR

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float output;
} SpeedPID_t;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float output;
} AnglePID_t;

static CAN_HandleTypeDef *motor_can;
static UART_HandleTypeDef *vofa_uart;
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
static float requested_angle_deg = ANGLE_SINE_CENTER_DEG;
static int32_t target_total_angle_ecd;
static int32_t angle_trajectory_turn_offset_ecd;
static uint16_t previous_encoder;
static bool encoder_initialized;
static GM6020_ControlMode_t control_mode = //GM6020_CONTROL_SPEED;//GM6020_CONTROL_POSITION/GM6020_CONTROL_SPEED
                                          GM6020_CONTROL_POSITION;
static uint32_t trajectory_start_ms;
static uint32_t last_vofa_ms;

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

static void speed_pid_reset(void)
{
  speed_pid.integral = 0.0f;
  speed_pid.previous_error = 0.0f;
  speed_pid.output = 0.0f;
}

static void angle_pid_reset(void)
{
  angle_pid.integral = 0.0f;
  angle_pid.previous_error = 0.0f;
  angle_pid.output = 0.0f;
}

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

static void angle_resolve_single_turn_target(float angle_deg,
                                              AngleArcMode_t arc_mode)
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

  if (arc_mode == ANGLE_ARC_MAJOR)
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
      /* Same single-turn angle: the major arc is one full positive turn. */
      delta = GM6020_ENCODER_CPR;
    }
  }

  target_total_angle_ecd = motor_feedback.total_angle_ecd + delta;
  angle_trajectory_turn_offset_ecd =
      target_total_angle_ecd - desired_encoder;
  angle_pid_reset();
}

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
    if (control_mode == GM6020_CONTROL_POSITION)
    {
      angle_resolve_single_turn_target(requested_angle_deg, ANGLE_ARC_MODE);
    }
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

static void control_trajectory_update(uint32_t now)
{
  const uint32_t elapsed = now - trajectory_start_ms;

  if (control_mode == GM6020_CONTROL_SPEED)
  {
    const uint32_t phase_ms = elapsed % SPEED_SINE_PERIOD_MS;
    const float phase_rad =
        TWO_PI_F * (float)phase_ms / (float)SPEED_SINE_PERIOD_MS;

    target_speed_rpm =
        SPEED_SINE_CENTER_RPM + SPEED_SINE_AMPLITUDE_RPM * sinf(phase_rad);
  }
  else
  {
    const uint32_t phase_ms = elapsed % ANGLE_SINE_PERIOD_MS;
    const float phase_rad =
        TWO_PI_F * (float)phase_ms / (float)ANGLE_SINE_PERIOD_MS;

    requested_angle_deg =
        ANGLE_SINE_CENTER_DEG
        + ANGLE_SINE_AMPLITUDE_DEG * sinf(phase_rad);

    if (encoder_initialized)
    {
      target_total_angle_ecd = angle_trajectory_turn_offset_ecd
                             + single_turn_degrees_to_encoder(
                                 requested_angle_deg);
    }
  }
}

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

  /* Conditional integration prevents windup while the command is saturated. */
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

  /* ID 1 occupies DATA[0:1], high byte first. */
  tx_data[0] = (uint8_t)(((uint16_t)current) >> 8);
  tx_data[1] = (uint8_t)current;

  return HAL_CAN_AddTxMessage(motor_can, &tx_header, tx_data, &mailbox);
}

static void vofa_send_justfloat(void)
{
  /*
   * VOFA+ JustFloat channels:
   *   0 resolved multi-turn target angle (degrees, position mode only)
   *   1 measured multi-turn angle (degrees)
   *   2 angle-PID target speed (rpm)
   *   3 measured speed (rpm)
   *   4 speed-PID target torque current (A)
   *   5 measured torque current (A)
   *
   * JustFloat packets are little-endian float data followed by
   * 00 00 80 7F.
   */
  float channels[6];
  uint8_t packet[sizeof(channels) + 4U];

  channels[0] = (control_mode == GM6020_CONTROL_POSITION)
              ? ((float)target_total_angle_ecd * 360.0f
                 / (float)GM6020_ENCODER_CPR)
              : 0.0f;
  channels[1] = motor_feedback.online
              ? motor_feedback.total_angle_deg : 0.0f;
  channels[2] = target_speed_rpm;
  channels[3] = motor_feedback.online
              ? (float)motor_feedback.speed_rpm : 0.0f;
  channels[4] = speed_pid.output * 3.0f / 16384.0f;
  channels[5] = motor_feedback.online
              ? ((float)motor_feedback.torque_current * 3.0f / 16384.0f)
              : 0.0f;

  memcpy(packet, channels, sizeof(channels));
  packet[sizeof(channels) + 0U] = 0x00U;
  packet[sizeof(channels) + 1U] = 0x00U;
  packet[sizeof(channels) + 2U] = 0x80U;
  packet[sizeof(channels) + 3U] = 0x7FU;

  (void)HAL_UART_Transmit(vofa_uart, packet, sizeof(packet), 5U);
}

HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *hcan,
                              UART_HandleTypeDef *huart)
{
  CAN_FilterTypeDef filter = {0};
  HAL_StatusTypeDef status;

  if ((hcan == NULL) || (huart == NULL))
  {
    return HAL_ERROR;
  }

  motor_can = hcan;
  vofa_uart = huart;
  memset(&motor_feedback, 0, sizeof(motor_feedback));
  speed_pid_reset();
  angle_pid_reset();
  encoder_initialized = false;
  control_mode = GM6020_CONTROL_POSITION;
  requested_angle_deg = ANGLE_SINE_CENTER_DEG;
  target_speed_rpm = 0.0f;
  target_total_angle_ecd = 0;
  angle_trajectory_turn_offset_ecd = 0;
  trajectory_start_ms = HAL_GetTick();

  /* Accept only the standard feedback ID 0x205 into FIFO0. */
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

  /* Keep the motor disabled until the first valid feedback frame arrives. */
  return gm6020_send_current(0);
}

void GM6020_SetControlMode(GM6020_ControlMode_t mode)
{
  if ((mode != GM6020_CONTROL_SPEED)
      && (mode != GM6020_CONTROL_POSITION))
  {
    return;
  }

  control_mode = mode;
  trajectory_start_ms = HAL_GetTick();
  target_speed_rpm = 0.0f;
  speed_pid_reset();
  angle_pid_reset();

  if (control_mode == GM6020_CONTROL_POSITION)
  {
    requested_angle_deg = ANGLE_SINE_CENTER_DEG;
    angle_resolve_single_turn_target(requested_angle_deg, ANGLE_ARC_MODE);
  }
}

GM6020_ControlMode_t GM6020_GetControlMode(void)
{
  return control_mode;
}

void GM6020_Process(void)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint32_t now = HAL_GetTick();

  control_trajectory_update(now);

  while (HAL_CAN_GetRxFifoFillLevel(motor_can, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(motor_can, CAN_RX_FIFO0,
                            &rx_header, rx_data) != HAL_OK)
    {
      break;
    }

    if ((rx_header.IDE != CAN_ID_STD)
        || (rx_header.RTR != CAN_RTR_DATA)
        || (rx_header.StdId != GM6020_FEEDBACK_STD_ID)
        || (rx_header.DLC != 8U))
    {
      continue;
    }

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

    if (control_mode == GM6020_CONTROL_POSITION)
    {
      target_speed_rpm = angle_pid_update();
    }
    (void)gm6020_send_current((int16_t)speed_pid_update(
        target_speed_rpm, (float)motor_feedback.speed_rpm));
  }

  now = HAL_GetTick();
  if (motor_feedback.online
      && ((uint32_t)(now - motor_feedback.last_rx_ms)
          > GM6020_FEEDBACK_TIMEOUT_MS))
  {
    motor_feedback.online = false;
    encoder_initialized = false;
    target_speed_rpm = 0.0f;
    angle_pid_reset();
    speed_pid_reset();
    (void)gm6020_send_current(0);
  }

  if ((uint32_t)(now - last_vofa_ms) >= VOFA_SEND_PERIOD_MS)
  {
    last_vofa_ms = now;
    vofa_send_justfloat();
  }
}

const GM6020_Feedback_t *GM6020_GetFeedback(void)
{
  return &motor_feedback;
}
