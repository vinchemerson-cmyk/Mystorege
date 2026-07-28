#include "control_input.h"

#include "motor_control.h"
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONTROL_IN_LINE_CAPACITY    48U
#define CONTROL_OUT_BUFFER_CAPACITY 96U
#define CONTROL_OUT_PERIOD_MS       1000U
#define CONTROL_IN_YAW_MIN_DEG    (-180.0f)
#define CONTROL_IN_YAW_MAX_DEG      180.0f
#define CONTROL_IN_PITCH_MIN_DEG  (-30.0f)
#define CONTROL_IN_PITCH_MAX_DEG    30.0f

typedef enum
{
  CONTROL_ACK_NONE = 0,
  CONTROL_ACK_OK,
  CONTROL_ACK_ERR
} ControlAck_t;

static char receive_line[CONTROL_IN_LINE_CAPACITY];
static uint32_t receive_length;
static bool receive_overflow;

static char pending_line[CONTROL_IN_LINE_CAPACITY];
static volatile bool pending_line_ready;
static volatile bool pending_line_invalid;
static volatile ControlAck_t pending_ack;

static uint8_t ok_reply[] = "OK\r\n";
static uint8_t err_reply[] = "ERR\r\n";
static uint8_t feedback_reply[CONTROL_OUT_BUFFER_CAPACITY];
static uint32_t last_feedback_tx_ms;

static void restore_interrupt_state(uint32_t previous_primask)
{
  if (previous_primask == 0U)
  {
    __enable_irq();
  }
}

static void publish_received_line(void)
{
  if (!pending_line_ready)
  {
    if (receive_overflow)
    {
      pending_line[0] = '\0';
      pending_line_invalid = true;
    }
    else
    {
      memcpy(pending_line, receive_line, receive_length);
      pending_line[receive_length] = '\0';
      pending_line_invalid = false;
    }
    pending_line_ready = true;
  }

  receive_length = 0U;
  receive_overflow = false;
}

void control_in_receive(const uint8_t *data, uint32_t length)
{
  uint32_t index;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    const uint8_t byte = data[index];

    if (byte == '\n')
    {
      publish_received_line();
    }
    else if (byte != '\r')
    {
      if (receive_length < (CONTROL_IN_LINE_CAPACITY - 1U))
      {
        receive_line[receive_length++] = (char)byte;
      }
      else
      {
        receive_overflow = true;
      }
    }
  }
}

static void skip_spaces(char **cursor)
{
  while (isspace((unsigned char)**cursor) != 0)
  {
    ++(*cursor);
  }
}

static bool parse_target_pair(char *line,
                              float *yaw_angle_deg,
                              float *pitch_angle_deg)
{
  char *cursor = line;
  char *end;
  float yaw;
  float pitch;

  skip_spaces(&cursor);
  yaw = strtof(cursor, &end);
  if (end == cursor)
  {
    return false;
  }

  cursor = end;
  skip_spaces(&cursor);
  if (*cursor != ',')
  {
    return false;
  }

  ++cursor;
  skip_spaces(&cursor);
  pitch = strtof(cursor, &end);
  if (end == cursor)
  {
    return false;
  }

  cursor = end;
  skip_spaces(&cursor);
  if (*cursor != '\0')
  {
    return false;
  }

  if (!isfinite(yaw) || !isfinite(pitch)
      || (yaw < CONTROL_IN_YAW_MIN_DEG)
      || (yaw > CONTROL_IN_YAW_MAX_DEG)
      || (pitch < CONTROL_IN_PITCH_MIN_DEG)
      || (pitch > CONTROL_IN_PITCH_MAX_DEG))
  {
    return false;
  }

  *yaw_angle_deg = yaw;
  *pitch_angle_deg = pitch;
  return true;
}

static void transmit_pending_ack(void)
{
  uint8_t result;

  if (pending_ack == CONTROL_ACK_OK)
  {
    result = CDC_Transmit_FS(ok_reply, sizeof(ok_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_ERR)
  {
    result = CDC_Transmit_FS(err_reply, sizeof(err_reply) - 1U);
  }
  else
  {
    return;
  }

  if (result == USBD_OK)
  {
    pending_ack = CONTROL_ACK_NONE;
  }
}

void control_in(void)
{
  char line[CONTROL_IN_LINE_CAPACITY];
  bool line_invalid;
  bool line_available = false;
  uint32_t previous_primask;
  float yaw_angle_deg;
  float pitch_angle_deg;

  transmit_pending_ack();

  previous_primask = __get_PRIMASK();
  __disable_irq();
  if (pending_line_ready)
  {
    memcpy(line, pending_line, sizeof(line));
    line_invalid = pending_line_invalid;
    pending_line_ready = false;
    line_available = true;
  }
  restore_interrupt_state(previous_primask);

  if (!line_available)
  {
    return;
  }

  if (!line_invalid
      && parse_target_pair(line, &yaw_angle_deg, &pitch_angle_deg))
  {
    GM6020_SetGimbalPosition(yaw_angle_deg, pitch_angle_deg);
    pending_ack = CONTROL_ACK_OK;
  }
  else
  {
    pending_ack = CONTROL_ACK_ERR;
  }

  transmit_pending_ack();
}

static int32_t angle_to_centidegrees(float angle_deg)
{
  const float scaled_angle = angle_deg * 100.0f;

  if (!isfinite(scaled_angle))
  {
    return 0;
  }
  if (scaled_angle >= (float)INT32_MAX)
  {
    return INT32_MAX;
  }
  if (scaled_angle <= (float)INT32_MIN)
  {
    return INT32_MIN;
  }

  return (int32_t)((scaled_angle >= 0.0f)
      ? (scaled_angle + 0.5f)
      : (scaled_angle - 0.5f));
}

static void format_angle(char *buffer, size_t capacity,
                         float angle_deg)
{
  const int32_t centidegrees = angle_to_centidegrees(angle_deg);
  const uint32_t magnitude = (centidegrees < 0)
      ? (uint32_t)(-(int64_t)centidegrees)
      : (uint32_t)centidegrees;

  (void)snprintf(
      buffer,
      capacity,
      "%s%lu.%02lu",
      (centidegrees < 0) ? "-" : "",
      (unsigned long)(magnitude / 100U),
      (unsigned long)(magnitude % 100U));
}

void control_out(void)
{
  const uint32_t now = HAL_GetTick();
  const GM6020_Feedback_t *yaw_feedback;
  const GM6020_Feedback_t *pitch_feedback;
  char yaw_angle[20];
  char pitch_angle[20];
  int length;

  if ((uint32_t)(now - last_feedback_tx_ms)
      < CONTROL_OUT_PERIOD_MS)
  {
    return;
  }
  last_feedback_tx_ms = now;

  yaw_feedback = GM6020_GetFeedback(GM6020_AXIS_YAW);
  pitch_feedback = GM6020_GetFeedback(GM6020_AXIS_PITCH);
  if ((yaw_feedback == NULL) || (pitch_feedback == NULL))
  {
    return;
  }

  format_angle(yaw_angle, sizeof(yaw_angle),
               yaw_feedback->total_angle_deg);
  format_angle(pitch_angle, sizeof(pitch_angle),
               pitch_feedback->total_angle_deg);

  length = snprintf(
      (char *)feedback_reply,
      sizeof(feedback_reply),
      "FB,%s,%s,%d,%d,%u,%u\r\n",
      yaw_angle,
      pitch_angle,
      (int)yaw_feedback->speed_rpm,
      (int)pitch_feedback->speed_rpm,
      yaw_feedback->online ? 1U : 0U,
      pitch_feedback->online ? 1U : 0U);

  if ((length > 0)
      && ((uint32_t)length < sizeof(feedback_reply)))
  {
    (void)CDC_Transmit_FS(feedback_reply, (uint16_t)length);
  }
}
