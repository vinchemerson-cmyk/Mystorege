#include "motor_control.h"

#include <string.h>

/*
 * GM6020 ID 1, torque-current-control mode.
 *
 * The motor publishes feedback at 1 kHz.  One PID update is therefore made
 * for every received feedback frame, with a fixed 1 ms sample time.
 */
#define GM6020_FEEDBACK_STD_ID       0x205U
#define GM6020_CURRENT_COMMAND_ID    0x1FEU
#define GM6020_CONTROL_PERIOD_S      0.001f
#define GM6020_FEEDBACK_TIMEOUT_MS   100U
#define VOFA_SEND_PERIOD_MS          10U

/* Conservative initial settings; tune these values with the motor loaded. */
#define SPEED_PID_KP                 30.0f
#define SPEED_PID_KI                 20.0f
#define SPEED_PID_KD                 0.0f
/* 8192 raw units correspond to approximately 1.5 A. */
#define SPEED_PID_OUTPUT_LIMIT       8192.0f
#define DEFAULT_TARGET_SPEED_RPM     100.0f

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float output;
} SpeedPID_t;

static CAN_HandleTypeDef *motor_can;
static UART_HandleTypeDef *vofa_uart;
static GM6020_Feedback_t motor_feedback;
static SpeedPID_t speed_pid = {
  .kp = SPEED_PID_KP,
  .ki = SPEED_PID_KI,
  .kd = SPEED_PID_KD
};
static float target_speed_rpm = DEFAULT_TARGET_SPEED_RPM;
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
   *   0 target speed (rpm)
   *   1 measured speed (rpm)
   *   2 PID target torque current (A)
   *   3 measured torque current (A)
   *
   * JustFloat packets are little-endian float data followed by
   * 00 00 80 7F.
   */
  float channels[4];
  uint8_t packet[sizeof(channels) + 4U];

  channels[0] = target_speed_rpm;
  channels[1] = motor_feedback.online
              ? (float)motor_feedback.speed_rpm : 0.0f;
  channels[2] = speed_pid.output * 3.0f / 16384.0f;
  channels[3] = motor_feedback.online
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

void GM6020_Process(void)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint32_t now = HAL_GetTick();

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
    motor_feedback.speed_rpm =
        (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
    motor_feedback.torque_current =
        (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
    motor_feedback.temperature = rx_data[6];
    motor_feedback.last_rx_ms = now;
    motor_feedback.online = true;

    (void)gm6020_send_current((int16_t)speed_pid_update(
        target_speed_rpm, (float)motor_feedback.speed_rpm));
  }

  now = HAL_GetTick();
  if (motor_feedback.online
      && ((uint32_t)(now - motor_feedback.last_rx_ms)
          > GM6020_FEEDBACK_TIMEOUT_MS))
  {
    motor_feedback.online = false;
    speed_pid_reset();
    (void)gm6020_send_current(0);
  }

  if ((uint32_t)(now - last_vofa_ms) >= VOFA_SEND_PERIOD_MS)
  {
    last_vofa_ms = now;
    vofa_send_justfloat();
  }
}

void GM6020_SetTargetSpeed(float speed_rpm)
{
  target_speed_rpm = clamp_float(speed_rpm, -320.0f, 320.0f);
  speed_pid_reset();
}

const GM6020_Feedback_t *GM6020_GetFeedback(void)
{
  return &motor_feedback;
}
