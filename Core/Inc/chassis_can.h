#ifndef CHASSIS_CAN_H
#define CHASSIS_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>

typedef enum
{
  CHASSIS_MODE_SOFTWARE_OFF = 0,
  CHASSIS_MODE_FOLLOW       = 1,
  CHASSIS_MODE_NO_FOLLOW    = 2,
  CHASSIS_MODE_SPIN         = 3
} chassis_mode_e;

/*
 * 底盘控制命令。
 * 四个物理量在发送时自动乘以1024并编码为int16 Q10。
 */
typedef struct
{
  float vx;
  float vy;
  float wz;
  float offset_angle_rad;
  chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;

/* 启动CAN2底盘发送。 */
HAL_StatusTypeDef ChassisCAN_Init(CAN_HandleTypeDef *hcan);

/* 更新下一周期要发送的底盘命令。 */
bool ChassisCAN_SetCommand(const Chassis_Ctrl_Cmd_s *command);

/* 主循环周期调用，默认每10 ms发送一次。 */
void ChassisCAN_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CAN_H */
