#include "chassis_can.h"

#include "config/chassis_can_config.h"

#include <math.h>
#include <stdint.h>

#if (CHASSIS_CAN_CTRL_STD_ID > 0x7FFU)
#error "CHASSIS_CAN_CTRL_STD_ID must not exceed 0x7FF"
#endif

#if (CHASSIS_CAN_MODE_STD_ID > 0x7FFU)
#error "CHASSIS_CAN_MODE_STD_ID must not exceed 0x7FF"
#endif

#if (CHASSIS_CAN_CTRL_STD_ID == CHASSIS_CAN_MODE_STD_ID)
#error "The chassis control and mode CAN IDs must be different"
#endif

#if (CHASSIS_CAN_TX_PERIOD_MS == 0U)
#error "CHASSIS_CAN_TX_PERIOD_MS must be greater than zero"
#endif

static CAN_HandleTypeDef *chassis_can;
static Chassis_Ctrl_Cmd_s chassis_command =
{
  .vx = 0.0f,
  .vy = 0.0f,
  .wz = 0.0f,
  .offset_angle_rad = 0.0f,
  .chassis_mode = CHASSIS_MODE_FOLLOW
};
static uint32_t last_tx_ms;

static bool q10_value_is_valid(float value)
{
  const float scaled = value * CHASSIS_Q10_SCALE;

  return isfinite(value)
      && (scaled >= -32768.0f)
      && (scaled <= 32767.0f);
}

static int16_t q10_encode(float value)
{
  return (int16_t)lroundf(value * CHASSIS_Q10_SCALE);
}

static void pack_int16_be(uint8_t *destination, int16_t value)
{
  const uint16_t raw = (uint16_t)value;

  destination[0] = (uint8_t)(raw >> 8);
  destination[1] = (uint8_t)raw;
}

static HAL_StatusTypeDef chassis_can_send(void)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t control_data[8] = {0};
  uint8_t mode_data[1] = {0};
  uint32_t mailbox;
  HAL_StatusTypeDef status;

  if ((chassis_can == NULL)
      || (HAL_CAN_GetTxMailboxesFreeLevel(chassis_can) < 2U))
  {
    return HAL_BUSY;
  }

  pack_int16_be(&control_data[0], q10_encode(chassis_command.vx));
  pack_int16_be(&control_data[2], q10_encode(chassis_command.vy));
  pack_int16_be(&control_data[4], q10_encode(chassis_command.wz));
  pack_int16_be(
      &control_data[6],
      q10_encode(chassis_command.offset_angle_rad));

  tx_header.StdId = CHASSIS_CAN_CTRL_STD_ID;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  status = HAL_CAN_AddTxMessage(
      chassis_can, &tx_header, control_data, &mailbox);
  if (status != HAL_OK)
  {
    return status;
  }

  tx_header.StdId = CHASSIS_CAN_MODE_STD_ID;
  tx_header.DLC = 1U;
  mode_data[0] = (uint8_t)chassis_command.chassis_mode;

  return HAL_CAN_AddTxMessage(
      chassis_can, &tx_header, mode_data, &mailbox);
}

HAL_StatusTypeDef ChassisCAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_StatusTypeDef status;

  if ((hcan == NULL) || (hcan->Instance != CAN2))
  {
    return HAL_ERROR;
  }

  status = HAL_CAN_Start(hcan);
  if (status != HAL_OK)
  {
    return status;
  }

  chassis_can = hcan;
  last_tx_ms = HAL_GetTick();
  return HAL_OK;
}

bool ChassisCAN_SetCommand(const Chassis_Ctrl_Cmd_s *command)
{
  if ((command == NULL)
      || !q10_value_is_valid(command->vx)
      || !q10_value_is_valid(command->vy)
      || !q10_value_is_valid(command->wz)
      || !q10_value_is_valid(command->offset_angle_rad)
      || (command->chassis_mode < CHASSIS_MODE_SOFTWARE_OFF)
      || (command->chassis_mode > CHASSIS_MODE_SPIN))
  {
    return false;
  }

  chassis_command = *command;
  return true;
}

void ChassisCAN_Process(void)
{
  const uint32_t now = HAL_GetTick();

  if ((chassis_can == NULL)
      || ((uint32_t)(now - last_tx_ms) < CHASSIS_CAN_TX_PERIOD_MS))
  {
    return;
  }

  if (chassis_can_send() == HAL_OK)
  {
    last_tx_ms = now;
  }
}
