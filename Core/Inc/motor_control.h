#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* 单圈目标角度的路径选择。 */
typedef enum
{
  GM6020_ARC_MINOR = 0, /* 劣弧：不超过 180° 的最短路径 */
  GM6020_ARC_MAJOR      /* 优弧：反方向、不小于 180° 的长路径 */
} GM6020_ArcMode_t;

/* GM6020 反馈数据及多圈编码器状态。 */
typedef struct
{
  uint16_t angle;           /* 单圈机械角度原始值，范围 0~8191 */
  int32_t turn_count;       /* 跨越编码器零点后累计的圈数，可正可负 */
  int32_t total_angle_ecd;  /* 多圈角度，单位为编码器计数 */
  float total_angle_deg;    /* 多圈角度，单位为度 */
  int16_t speed_rpm;        /* 电机反馈转速，单位 rpm */
  int16_t torque_current;   /* 转矩电流原始值，±16384 对应约 ±3 A */
  uint8_t temperature;      /* 电机温度，单位摄氏度 */
  uint32_t last_rx_ms;      /* 最近一次收到有效 CAN 反馈的系统时间 */
  bool online;              /* true 表示反馈未超时，电机在线 */
} GM6020_Feedback_t;

/* 配置 CAN 滤波器、启动 CAN，并初始化位置控制状态机。 */
HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *hcan);

/*
 * 唯一的位置控制命令接口。
 * target_angle_deg 为 0°~360° 单圈机械角度，函数内部会自动归一化；
 * arc_mode 用于选择优弧或劣弧，多圈目标由当前累计角度自动解析。
 */
void GM6020_SetTargetPosition(float target_angle_deg,
                              GM6020_ArcMode_t arc_mode);

/* 主循环周期调用：接收反馈、执行位置/速度串级 PID 并发送电流。 */
void GM6020_Process(void);

/* 获取只读反馈数据指针，供其他模块查询电机状态。 */
const GM6020_Feedback_t *GM6020_GetFeedback(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
