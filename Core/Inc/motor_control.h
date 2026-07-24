#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t angle;
  int16_t speed_rpm;
  int16_t torque_current;
  uint8_t temperature;
  uint32_t last_rx_ms;
  bool online;
} GM6020_Feedback_t;

HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *hcan,
                              UART_HandleTypeDef *huart);
void GM6020_Process(void);
void GM6020_SetTargetSpeed(float speed_rpm);
const GM6020_Feedback_t *GM6020_GetFeedback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
