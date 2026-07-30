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

typedef enum
{
  DUAL_M3508_STATE_DISABLED = 0,
  DUAL_M3508_STATE_WAIT_FEEDBACK,
  DUAL_M3508_STATE_RAMPING,
  DUAL_M3508_STATE_READY,
  DUAL_M3508_STATE_ESTOP,
  DUAL_M3508_STATE_FAULT
} DualM3508State_t;

typedef enum
{
  DUAL_M3508_FAULT_NONE = 0,
  DUAL_M3508_FAULT_FEEDBACK,
  DUAL_M3508_FAULT_OVERTEMPERATURE,
  DUAL_M3508_FAULT_STALL
} DualM3508FaultReason_t;

typedef struct
{
  uint16_t angle;
  int16_t raw_speed_rpm;
  int16_t logical_speed_rpm;
  int16_t actual_current_raw;
  int16_t command_current_raw;
  uint8_t temperature_c;
  uint32_t last_rx_ms;
  uint32_t rx_sequence;
  bool online;

  float speed_error_rpm;
  float pid_p_raw;
  float pid_i_raw;
  float pid_d_raw;
  float pid_output_raw;
} DualM3508MotorDebugData_t;

typedef struct
{
  DualM3508MotorDebugData_t motor[DUAL_M3508_DEBUG_MOTOR_COUNT];
  float target_speed_rpm;
  DualM3508State_t state;
  DualM3508FaultReason_t fault_reason;
  bool enable_requested;
  bool ready;
  bool emergency_stop_latched;
  bool fault_latched;
  uint32_t tx_error_count;
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
