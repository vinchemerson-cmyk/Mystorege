/**
 * ===========================================================================
 * @file    feeder_motor.h
 * @brief   CAN1 C610 ID3 / M2006拨弹盘安全低速测试模块
 * ===========================================================================
 */

#ifndef FEEDER_MOTOR_H
#define FEEDER_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 遥控器命令枚举 (Remote Command Enum)。
 * 由遥控器摇杆/拨杆状态映射而来。
 */
typedef enum
{
  FEEDER_REMOTE_DISABLE = 0,  /* 禁用 — no operation, zero current */
  FEEDER_REMOTE_NEUTRAL,      /* 中挡 — neutral: 重新解锁条件 */
  FEEDER_REMOTE_CONTINUOUS,   /* 连发 — continuous feeding (正转) */
  FEEDER_REMOTE_SINGLE,       /* 单发 — single shot (固定步距) */
  FEEDER_REMOTE_REVERSE       /* 退弹 — reverse feeding (反转) */
} FeederRemoteCommand_t;

/*
 * 拨弹盘状态机枚举 (Feeder State Machine Enum)。
 */
typedef enum
{
  FEEDER_STATE_DISABLED = 0,        /* 禁用 — disabled, zero current */
  FEEDER_STATE_WAIT_NEUTRAL,        /* 等待中挡重新解锁 — waiting for neutral rearm */
  FEEDER_STATE_ARMED_NEUTRAL,       /* 已解锁中挡 — armed and in neutral */
  FEEDER_STATE_RUNNING_CONTINUOUS,  /* 连发运行 — continuous feeding */
  FEEDER_STATE_RUNNING_SINGLE,      /* 单发运动 — single-shot moving */
  FEEDER_STATE_HOLDING_SINGLE,      /* 单发保持 — holding at single-shot target */
  FEEDER_STATE_RUNNING_REVERSE,     /* 退弹运行 — reverse feeding */
  FEEDER_STATE_ESTOP,               /* 急停 — emergency stop */
  FEEDER_STATE_FAULT                /* 故障 — latched fault */
} FeederMotorState_t;

/*
 * 故障原因枚举 (Fault Reason Enum)。
 */
typedef enum
{
  FEEDER_FAULT_NONE = 0,         /* 无故障 — no fault */
  FEEDER_FAULT_ESC,              /* C610 电调错误码 — ESC error code set */
  FEEDER_FAULT_STALL,            /* 堵转保护 — motor stalled (high current, low speed) */
  FEEDER_FAULT_SINGLE_OVERRUN    /* 单发前向过冲 — single shot forward overrun */
} FeederFaultReason_t;

/*
 * 拨弹盘调试数据 (Feeder Debug Data) — 供 USART6 调试任务读取的一致快照。
 */
typedef struct
{
  /* ---- C610 反馈字段 (ESC Feedback) ---- */
  uint16_t angle;              /* 单圈编码器角度 — single-turn encoder angle (8192 CPR) */
  int16_t speed_rpm;           /* 电机转速 — motor speed (rpm) */
  int16_t actual_current_raw;  /* 实际转矩电流 — actual torque current */
  uint8_t error_code;          /* C610 电调错误码 — ESC error code (0 = no error) */
  uint32_t last_rx_ms;         /* 最近反馈时间戳 — last feedback timestamp */
  uint32_t rx_sequence;        /* 反馈序列号 — feedback sequence number */
  bool online;                 /* 在线状态 — online flag (feedback within timeout) */

  /* ---- 速度环 PID 状态 (Speed PID) ---- */
  float target_speed_rpm;      /* 目标转速 — target speed (rpm) */
  float speed_error_rpm;       /* 转速误差 — speed error = target - feedback */
  float pid_p_raw;             /* 比例项输出 — PID P term */
  float pid_i_raw;             /* 积分项输出 — PID I term */
  float pid_d_raw;             /* 微分项输出 — PID D term */
  float pid_output_raw;        /* PID 总输出 — total PID output (current) */
  int16_t command_current_raw; /* 发送的电流命令 — commanded current */

  /* ---- 单发位置控制字段 (Single-shot Position) ---- */
  int64_t total_angle_ecd;        /* 多圈累计编码器角度 — multi-turn total angle */
  int64_t target_total_angle_ecd; /* 目标多圈角度 — target multi-turn angle */
  int32_t position_error_ecd;     /* 位置误差 (弹位步距单位) — position error in projectile steps */
  uint32_t shot_count;            /* 已发射弹数 — completed shot counter */

  /* ---- 状态字段 (State Flags) ---- */
  FeederRemoteCommand_t remote_command;  /* 当前遥控命令 — current remote command */
  FeederMotorState_t state;              /* 当前状态机状态 — current state */
  FeederFaultReason_t fault_reason;      /* 故障原因 — fault reason */
  bool armed;                            /* 是否已解锁 — armed flag */
  bool single_shot_active;               /* 单发运动中 — single-shot in progress */
  bool single_phase_valid;               /* 单发相位有效 — single-shot phase valid */
  bool single_holding;                   /* 单发保持中 — holding at shot target */
  bool emergency_stop_latched;           /* 急停锁存 — emergency stop latched */
  bool fault_latched;                    /* 故障锁存 — fault latched */

  uint32_t tx_error_count;   /* CAN 发送错误计数 — CAN TX error counter */
} FeederMotorDebugData_t;

/*
 * 复用已经由GM6020模块启动的CAN1，配置过滤器组1精确接收0x203到FIFO1，
 * 并尝试发送一帧0x200零电流。
 */
HAL_StatusTypeDef FeederMotor_Init(CAN_HandleTypeDef *hcan);

/*
 * 遥控命令入口。SINGLE只在非SINGLE→SINGLE边沿产生一次单发请求；
 * 上电、急停、掉线或故障后必须持续提交NEUTRAL才能重新解锁。
 */
void FeederMotor_SetRemoteCommand(FeederRemoteCommand_t command);

/* 锁存急停并立即请求发送零电流。 */
HAL_StatusTypeDef FeederMotor_EmergencyStop(void);

/* 只解除急停锁存，仍保持未解锁和零电流。 */
void FeederMotor_ClearEmergencyStop(void);
bool FeederMotor_IsEmergencyStopped(void);

/* 1 ms任务调用：读取CAN1 FIFO1、更新安全状态机和发送0x200电流帧。 */
void FeederMotor_Process(void);

/* 获取供USART6调试任务使用的一致快照。 */
bool FeederMotor_GetDebugData(FeederMotorDebugData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* FEEDER_MOTOR_H */
