/**
 * ===========================================================================
 * @file    dual_m3508.h
 * @brief   CAN2双C620/M3508摩擦轮控制接口
 * ===========================================================================
 */

#ifndef DUAL_M3508_H
#define DUAL_M3508_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define DUAL_M3508_DEBUG_MOTOR_COUNT  2U

/*
 * 摩擦轮状态机枚举 (Friction State Machine Enum)。
 */
typedef enum
{
  DUAL_M3508_STATE_DISABLED = 0,    /* 禁用 — disabled, zero current */
  DUAL_M3508_STATE_WAIT_FEEDBACK,   /* 等待双机反馈 — waiting for both motors online */
  DUAL_M3508_STATE_RAMPING,         /* 转速爬升 — ramping to target speed */
  DUAL_M3508_STATE_READY,           /* 就绪 — both at target speed, ready to fire */
  DUAL_M3508_STATE_ESTOP,           /* 急停 — emergency stop */
  DUAL_M3508_STATE_FAULT            /* 故障 — latched fault (stall/overtemp/feedback loss) */
} DualM3508State_t;

/*
 * 故障原因枚举 (Fault Reason Enum)。
 */
typedef enum
{
  DUAL_M3508_FAULT_NONE = 0,        /* 无故障 — no fault */
  DUAL_M3508_FAULT_FEEDBACK,        /* 运行中反馈丢失 — feedback loss while running */
  DUAL_M3508_FAULT_OVERTEMPERATURE, /* 过温保护 — motor over-temperature */
  DUAL_M3508_FAULT_STALL            /* 堵转保护 — motor stalled (high current, low speed) */
} DualM3508FaultReason_t;

/*
 * 单台摩擦轮电机调试数据 (Per-Motor Debug Data)。
 */
typedef struct
{
  uint16_t angle;               /* 单圈编码器角度 — single-turn encoder angle (8192 CPR) */
  int16_t raw_speed_rpm;        /* 原始转速 — raw speed (rpm) */
  int16_t logical_speed_rpm;    /* 逻辑转速 (含方向修正) — logical speed after direction mapping */
  int16_t actual_current_raw;   /* 实际转矩电流 — actual torque current */
  int16_t command_current_raw;  /* 发送的电流命令 — commanded current */
  uint8_t temperature_c;        /* 电机温度 — motor temperature (℃) */
  uint32_t last_rx_ms;          /* 最近反馈时间戳 — last feedback timestamp */
  uint32_t rx_sequence;         /* 反馈序列号 — feedback sequence number */
  bool online;                  /* 在线状态 — online flag */

  /* ---- 速度环 PID 状态 ---- */
  float speed_error_rpm;        /* 转速误差 — speed error (rpm) */
  float pid_p_raw;              /* 比例项 — PID P term */
  float pid_i_raw;              /* 积分项 — PID I term */
  float pid_d_raw;              /* 微分项 — PID D term */
  float pid_output_raw;         /* PID 总输出 — total PID output */
} DualM3508MotorDebugData_t;

/*
 * 摩擦轮整体调试数据 (Dual-Motor Friction Debug Data) — 供 USART6 调试任务读取。
 */
typedef struct
{
  DualM3508MotorDebugData_t motor[DUAL_M3508_DEBUG_MOTOR_COUNT]; /* 两台电机的独立数据 */
  float target_speed_rpm;       /* 爬升目标转速 — ramped target speed (rpm) */
  DualM3508State_t state;       /* 当前状态 — current state */
  DualM3508FaultReason_t fault_reason; /* 故障原因 — fault reason */
  bool enable_requested;        /* 运行请求 — run requested */
  bool ready;                   /* 双机就绪 — both motors ready */
  bool emergency_stop_latched;  /* 急停锁存 — emergency stop latched */
  bool fault_latched;           /* 故障锁存 — fault latched */
  uint32_t tx_error_count;      /* CAN 发送错误计数 — CAN TX error counter */
} DualM3508DebugData_t;

/*
 * 复用已由GM6020模块启动的CAN2，配置bank15将0x201/0x202路由到FIFO1。
 */
HAL_StatusTypeDef DualM3508_Init(CAN_HandleTypeDef *hcan);

/* true请求双摩擦轮运行；false立即请求零电流。 */
void DualM3508_SetEnabled(bool enabled);

/*
 * 通信无效时关闭输出并取消重新启动资格；恢复通信后必须先提交false。
 */
void DualM3508_DisableUntilOff(void);

/* 锁存急停并立即发送双零电流。 */
HAL_StatusTypeDef DualM3508_EmergencyStop(void);

/*
 * 只解除急停锁存，不自动恢复旧运行命令。遥控器必须再次提交enable。
 */
void DualM3508_ClearEmergencyStop(void);
bool DualM3508_IsEmergencyStopped(void);

/* 两台电机均在线且连续到达目标速度后返回true。 */
bool DualM3508_IsReady(void);

/* 1 ms控制任务调用。 */
void DualM3508_Process(void);

/* 获取UART6调试快照。 */
bool DualM3508_GetDebugData(DualM3508DebugData_t *data);

#ifdef __cplusplus
}
#endif

#endif /* DUAL_M3508_H */
