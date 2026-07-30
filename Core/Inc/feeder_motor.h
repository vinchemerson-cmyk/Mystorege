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

typedef enum
{
  FEEDER_REMOTE_DISABLE = 0,
  FEEDER_REMOTE_NEUTRAL,
  FEEDER_REMOTE_DIRECTION_A,
  FEEDER_REMOTE_DIRECTION_B
} FeederRemoteCommand_t;

typedef enum
{
  FEEDER_STATE_DISABLED = 0,
  FEEDER_STATE_WAIT_NEUTRAL,
  FEEDER_STATE_ARMED_NEUTRAL,
  FEEDER_STATE_RUNNING_A,
  FEEDER_STATE_RUNNING_B,
  FEEDER_STATE_ESTOP,
  FEEDER_STATE_FAULT
} FeederMotorState_t;

typedef enum
{
  FEEDER_FAULT_NONE = 0,
  FEEDER_FAULT_ESC,
  FEEDER_FAULT_STALL
} FeederFaultReason_t;

typedef struct
{
  uint16_t angle;
  int16_t speed_rpm;
  int16_t actual_current_raw;
  uint8_t error_code;
  uint32_t last_rx_ms;
  uint32_t rx_sequence;
  bool online;

  float target_speed_rpm;
  float speed_error_rpm;
  float pid_p_raw;
  float pid_i_raw;
  float pid_d_raw;
  float pid_output_raw;
  int16_t command_current_raw;
  FeederRemoteCommand_t remote_command;
  FeederMotorState_t state;
  FeederFaultReason_t fault_reason;
  bool armed;
  bool emergency_stop_latched;
  bool fault_latched;

  uint32_t tx_error_count;
} FeederMotorDebugData_t;

/*
 * 复用已经由GM6020模块启动的CAN1，配置过滤器组1精确接收0x203到FIFO1，
 * 并尝试发送一帧0x200零电流。
 */
HAL_StatusTypeDef FeederMotor_Init(CAN_HandleTypeDef *hcan);

/*
 * 临时遥控命令入口。上电、急停、掉线或故障后，必须先持续提交NEUTRAL，
 * 模块才会重新解锁；方向切换也必须经过NEUTRAL。
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
