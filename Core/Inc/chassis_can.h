/**
 * ===========================================================================
 * @file    chassis_can.h
 * @brief   底盘 CAN 控制命令发送模块 — 对外 API 声明与类型定义
 * ===========================================================================
 *
 * 【模块职责】
 *   通过 CAN1 总线向底盘控制器周期发送运动指令（控制量 + 模式）。
 *   与Yaw GM6020共享CAN1物理总线，但使用不同标准帧ID。
 *
 * 【数据类型】
 *   chassis_mode_e     底盘控制模式枚举（断电/跟随/非跟随/自旋）
 *   Chassis_Ctrl_Cmd_s 底盘控制命令结构体（vx/vy/wz/offset_angle_rad + 模式）
 *
 * 【对外 API】
 *   ChassisCAN_Init()            启动 CAN1 发送通道
 *   ChassisCAN_SetCommand()      更新待发送命令
 *   ChassisCAN_EmergencyStop()   锁存式急停
 *   ChassisCAN_ClearEmergencyStop() 解除急停
 *   ChassisCAN_Process()         主循环周期调度 (~100 Hz)
 *
 * 【依赖】
 *   config/chassis_can_config.h  CAN ID、Q10 缩放因子、发送周期
 * ===========================================================================
 */

#ifndef CHASSIS_CAN_H
#define CHASSIS_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>

/*
 * 底盘控制模式枚举 (Chassis Control Mode Enum)。
 * 每个值的含义参见 chassis_can.c 文件头注释。
 */
typedef enum
{
  CHASSIS_MODE_SOFTWARE_OFF = 0,  /* 软件断电 — software power-off, motors disabled */
  CHASSIS_MODE_FOLLOW       = 1,  /* 跟随模式 — follow mode, chassis orientation tracks gimbal */
  CHASSIS_MODE_NO_FOLLOW    = 2,  /* 非跟随模式 — no-follow mode, independent velocity control */
  CHASSIS_MODE_SPIN         = 3   /* 自旋模式 — spin mode, rotate in place */
} chassis_mode_e;

/*
 * 底盘控制命令结构体 (Chassis Control Command)。
 * 四个物理量在发送时自动乘以 Q10_SCALE=1024 并编码为 int16 通过 CAN 发送。
 */
typedef struct
{
  float vx;               /* 前进速度 (m/s) — forward velocity */
  float vy;               /* 横移速度 (m/s) — lateral / sideways velocity */
  float wz;               /* 旋转角速度 (rad/s) — angular velocity about Z-axis */
  float offset_angle_rad; /* 偏移角 (rad) — gimbal-to-chassis offset angle in radians */
  chassis_mode_e chassis_mode; /* 底盘控制模式 — chassis operation mode */
} Chassis_Ctrl_Cmd_s;

/* 启动CAN1底盘发送。 */
HAL_StatusTypeDef ChassisCAN_Init(CAN_HandleTypeDef *hcan);

/* 更新下一周期要发送的底盘命令。 */
bool ChassisCAN_SetCommand(const Chassis_Ctrl_Cmd_s *command);

/*
 * 锁存式急停：立即请求发送零速度和软件断电模式。
 * 急停锁存期间 ChassisCAN_SetCommand() 拒绝普通控制命令。
 */
HAL_StatusTypeDef ChassisCAN_EmergencyStop(void);

/*
 * 解除急停锁存。解除后仍保持零速度和软件断电，
 * 必须再调用 ChassisCAN_SetCommand() 才能恢复底盘动作。
 */
void ChassisCAN_ClearEmergencyStop(void);

/* 主循环周期调用，默认每10 ms发送一次。 */
void ChassisCAN_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CAN_H */
