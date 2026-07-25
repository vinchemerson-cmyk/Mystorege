#include "mouse_position_rx.h"

#include "motor_control.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * PC 端固定发送 8 字节小端帧：
 *   [0]    0xAA
 *   [1]    0x55
 *   [2:5]  float 目标角度（度）
 *   [6]    0=劣弧，1=优弧
 *   [7]    字节 0~6 的 XOR 校验
 */
#define MOUSE_FRAME_HEAD0  0xAAU
#define MOUSE_FRAME_HEAD1  0x55U
#define MOUSE_FRAME_SIZE   8U

typedef struct
{
  uint8_t bytes[MOUSE_FRAME_SIZE];
  uint8_t index;
} MouseFrameParser_t;

static UART_HandleTypeDef *mouse_uart;
static uint8_t mouse_rx_byte;
static MouseFrameParser_t mouse_parser;

/*
 * 中断只发布最新命令，真正调用电机控制接口放在主循环中，
 * 避免 USART 中断与 1 kHz CAN 控制计算同时修改 PID 状态。
 */
static volatile uint32_t pending_angle_bits;
static volatile uint8_t pending_arc_mode;
static volatile bool command_pending;
static volatile bool receive_restart_required;

static bool mouse_frame_checksum_valid(const uint8_t *frame)
{
  uint8_t checksum = 0U;
  uint8_t index;

  for (index = 0U; index < MOUSE_FRAME_SIZE - 1U; ++index)
  {
    checksum ^= frame[index];
  }
  return checksum == frame[MOUSE_FRAME_SIZE - 1U];
}

static void mouse_frame_publish(const uint8_t *frame)
{
  uint32_t angle_bits;

  if (!mouse_frame_checksum_valid(frame) || (frame[6] > 1U))
  {
    return;
  }

  memcpy(&angle_bits, &frame[2], sizeof(angle_bits));
  pending_angle_bits = angle_bits;
  pending_arc_mode = frame[6];
  command_pending = true;
}

static void mouse_frame_push_byte(uint8_t byte)
{
  switch (mouse_parser.index)
  {
    case 0U:
      if (byte == MOUSE_FRAME_HEAD0)
      {
        mouse_parser.bytes[0] = byte;
        mouse_parser.index = 1U;
      }
      break;

    case 1U:
      if (byte == MOUSE_FRAME_HEAD1)
      {
        mouse_parser.bytes[1] = byte;
        mouse_parser.index = 2U;
      }
      else
      {
        mouse_parser.index =
            (byte == MOUSE_FRAME_HEAD0) ? 1U : 0U;
      }
      break;

    default:
      mouse_parser.bytes[mouse_parser.index++] = byte;
      if (mouse_parser.index == MOUSE_FRAME_SIZE)
      {
        mouse_frame_publish(mouse_parser.bytes);
        mouse_parser.index = 0U;
      }
      break;
  }
}

static HAL_StatusTypeDef mouse_uart_start_receive(void)
{
  const HAL_StatusTypeDef status =
      HAL_UART_Receive_IT(mouse_uart, &mouse_rx_byte, 1U);

  receive_restart_required = (status != HAL_OK);
  return status;
}

HAL_StatusTypeDef MousePositionRx_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  mouse_uart = huart;
  mouse_rx_byte = 0U;
  memset(&mouse_parser, 0, sizeof(mouse_parser));
  pending_angle_bits = 0U;
  pending_arc_mode = 0U;
  command_pending = false;
  receive_restart_required = false;

  return mouse_uart_start_receive();
}

void MousePositionRx_Process(void)
{
  uint32_t angle_bits;
  uint8_t arc_value;
  uint32_t previous_primask;
  float target_angle_deg;
  GM6020_ArcMode_t arc_mode;

  if (receive_restart_required)
  {
    (void)mouse_uart_start_receive();
  }

  if (!command_pending)
  {
    return;
  }

  /*
   * Cortex-M4 的 32 位读写本身是原子的；这里仍短暂关中断，
   * 保证角度、路径和 pending 标志属于同一帧。
   */
  previous_primask = __get_PRIMASK();
  __disable_irq();
  angle_bits = pending_angle_bits;
  arc_value = pending_arc_mode;
  command_pending = false;
  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  memcpy(&target_angle_deg, &angle_bits, sizeof(target_angle_deg));
  if (!isfinite(target_angle_deg) || (arc_value > 1U))
  {
    return;
  }

  arc_mode = (arc_value == 0U)
           ? GM6020_ARC_MINOR
           : GM6020_ARC_MAJOR;
  GM6020_SetTargetPosition(target_angle_deg, arc_mode);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((mouse_uart != NULL) && (huart->Instance == mouse_uart->Instance))
  {
    mouse_frame_push_byte(mouse_rx_byte);
    (void)mouse_uart_start_receive();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((mouse_uart != NULL) && (huart->Instance == mouse_uart->Instance))
  {
    memset(&mouse_parser, 0, sizeof(mouse_parser));
    (void)HAL_UART_AbortReceive(huart);
    (void)mouse_uart_start_receive();
  }
}
