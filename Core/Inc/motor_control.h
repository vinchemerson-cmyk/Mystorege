#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  GM6020_CONTROL_SPEED = 0,
  GM6020_CONTROL_POSITION
} GM6020_ControlMode_t;

typedef struct
{
  uint16_t angle;
  int32_t turn_count;
  int32_t total_angle_ecd;
  float total_angle_deg;
  int16_t speed_rpm;
  int16_t torque_current;
  uint8_t temperature;
  uint32_t last_rx_ms;
  bool online;
} GM6020_Feedback_t;

HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *hcan,
                              UART_HandleTypeDef *huart);
void GM6020_SetControlMode(GM6020_ControlMode_t mode);
GM6020_ControlMode_t GM6020_GetControlMode(void);
void GM6020_Process(void);
const GM6020_Feedback_t *GM6020_GetFeedback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
