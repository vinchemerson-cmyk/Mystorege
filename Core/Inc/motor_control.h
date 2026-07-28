#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * 轴枚举。
 *
 * 默认电机 ID 映射：
 *   Yaw   (偏航) → GM6020 ID 1，CAN 反馈帧 StdId = 0x205
 *   Pitch (俯仰) → GM6020 ID 2，CAN 反馈帧 StdId = 0x206
 *
 * GM6020_AXIS_COUNT 用于数组大小声明和循环边界。
 */
typedef enum
{
  GM6020_AXIS_YAW = 0,
  GM6020_AXIS_PITCH,
  GM6020_AXIS_COUNT
} GM6020_Axis_t;

/*
 * 对外可查询的控制模式。
 */
typedef enum
{
  GM6020_MODE_POSITION = 0, /* 角度环 + 速度环串级位置控制 */
  GM6020_MODE_SPEED_DEBUG   /* 绕过角度环，目标 RPM 直接进入速度环 */
} GM6020_ControlMode_t;

/*
 * 单个 GM6020 电机的完整反馈数据及多圈编码器状态。
 *
 * 【编码器说明】
 *   GM6020 使用 8192 CPR 单圈绝对值编码器。每次上电后编码器值
 *   在 0~8191 之间，与电机转子绝对位置对应。通过跨零点检测，
 *   可以在运行过程中累计多圈角度，实现无限制旋转。
 */
typedef struct
{
  uint16_t angle;           /* 单圈机械角度原始值，范围 0~8191 (8192 CPR) */
  int32_t turn_count;       /* 累计圈数：正向跨零 +1，反向跨零 -1，上电从 0 开始 */
  int32_t total_angle_ecd;  /* 多圈累计角度，单位：编码器计数 (counts) */
  float total_angle_deg;    /* 多圈累计角度，单位：度 (degrees) */
  int16_t speed_rpm;        /* 电机反馈转速，单位：rpm；正方向由安装和标定定义 */
  int16_t torque_current;   /* 转矩电流原始值，±16384 对应约 ±3 A（GM6020 手册） */
  uint8_t temperature;      /* 电机内部温度传感器读数，单位：摄氏度 (℃) */
  uint32_t last_rx_ms;      /* 最近一次收到有效 CAN 反馈时的 HAL_GetTick() 值 */
  bool online;              /* true：反馈时间未超过配置的离线超时阈值 */
} GM6020_Feedback_t;

/*
 * 单轴速度环调试数据。
 *
 * 将速度环的关键变量打包为结构体，方便调试器观察或串口发送到上位机。
 * 可配合 GM6020_GetSpeedDebugData() 在运行中实时获取。
 */
typedef struct
{
  float target_speed_rpm;     /* 速度环目标转速 (rpm) */
  float feedback_speed_rpm;   /* 电机反馈转速 (rpm) */
  float speed_error_rpm;      /* 转速误差 = target - feedback */
  float output_current;       /* 速度环 PID 输出的转矩电流值 */
  float kp;                   /* 当前使用的比例增益 (Kp) */
  float ki;                   /* 当前使用的积分增益 (Ki) */
  float kd;                   /* 当前使用的微分增益 (Kd) */
  GM6020_ControlMode_t mode;  /* 当前控制模式 */
} GM6020_SpeedDebugData_t;

/*
 * 初始化 Yaw/Pitch 两轴控制器。
 *
 * 内部流程：
 *   1. 校验两轴 CAN ID 和电流槽位不冲突
 *   2. 遍历两轴 → 配置校验 → 控制器初始化 → CAN 硬件滤波器配置
 *   3. 启动 CAN 外设
 *   4. 发送初始 0x1FE 零电流帧
 *
 * 必须在 MX_CAN1_Init() 之后调用。
 * 返回 HAL_OK 表示成功，否则应调用 Error_Handler()。
 */
HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *hcan);

/*
 * 设置单轴位置目标。
 *
 * target_angle_deg 为相对配置零位的逻辑角度（度）。
 * 函数内部执行顺序：
 *   1. 若有软限位（angle_limit_enabled），将目标限幅到 [min, max]
 *   2. 叠加零位偏置 zero_offset_deg，归一化到 [0, 360)
 *   3. 若已处于 POSITION_CONTROL 状态，按劣弧方向解析为多圈目标；
 *      否则等状态转移时再解析
 *
 * 只能在主循环控制上下文调用。中断应只发布命令，避免与
 * GM6020_Process() 并发修改目标和 PID 状态。
 */
void GM6020_SetTargetPosition(GM6020_Axis_t axis,
                              float target_angle_deg);

/*
 * 设置累计多圈位置目标。
 *
 * target_angle_deg 相对于该轴本次启动时的初始位置：
 *   360°  = 正向 1 圈
 *   1080° = 正向 3 圈
 *   -720° = 反向 2 圈
 *
 * 该接口不做单圈归一化，也不按劣弧选择路径。
 */
void GM6020_SetMultiTurnTargetPosition(
    GM6020_Axis_t axis,
    float target_angle_deg);

/*
 * 在同一次业务调用中同时更新 Yaw 和 Pitch 两轴的位置目标。
 *
 * 等价于：
 *   GM6020_SetTargetPosition(GM6020_AXIS_YAW, yaw_angle_deg);
 *   GM6020_SetTargetPosition(GM6020_AXIS_PITCH, pitch_angle_deg);
 *
 * 用于有两个独立位置指令来源的场景（如两轴摇杆、双角度串口协议）。
 */
void GM6020_SetGimbalPosition(float yaw_angle_deg,
                              float pitch_angle_deg);

/*
 * ======================== 速度环调试接口 ========================
 *
 * 速度调试模式绕过角度环，直接用指定 RPM 驱动速度环。
 * 典型用法：
 *   1. GM6020_EnterSpeedDebug(GM6020_AXIS_YAW, 100) — 以 100 RPM 开始调试
 *   2. 观察 GM6020_GetSpeedDebugData() 返回值，调整 PID 增益
 *   3. GM6020_SetSpeedDebugTarget(GM6020_AXIS_YAW, -50) — 阶跃响应测试
 *   4. GM6020_ExitSpeedDebug(GM6020_AXIS_YAW) — 回到位置控制
 *
 * 目标转速会自动限幅到 ±GM6020_DEBUG_SPEED_LIMIT_RPM（默认 200 RPM）。
 */

/* 进入速度调试模式。目标转速自动限幅到 ±200 RPM。 */
void GM6020_EnterSpeedDebug(GM6020_Axis_t axis,
                            float target_speed_rpm);

/* 在速度调试模式下实时修改目标转速。仅在 speed_debug_requested 时生效。 */
void GM6020_SetSpeedDebugTarget(GM6020_Axis_t axis,
                                float target_speed_rpm);

/* 退出速度调试模式，回到位置控制（会锁定当前位置）。 */
void GM6020_ExitSpeedDebug(GM6020_Axis_t axis);

/*
 * 运行时修改速度环 PID 增益。
 *
 * 每个轴可使用不同的参数（Yaw/Pitch 负载惯量不同）。
 * 修改后会自动重置积分项和上一拍误差，防止旧积分在新参数下跳变。
 *
 * 返回 false 表示参数无效（axis 越界、负增益、非有限值等）。
 */
bool GM6020_SetSpeedPidGains(GM6020_Axis_t axis,
                             float kp, float ki, float kd);

/*
 * 运行时修改角度环 PID 增益。
 * 行为与 GM6020_SetSpeedPidGains 相同（参数校验 + 重置积分）。
 */
bool GM6020_SetAnglePidGains(GM6020_Axis_t axis,
                             float kp, float ki, float kd);

/* 查询指定轴的当前控制模式（位置控制 / 速度调试）。 */
GM6020_ControlMode_t GM6020_GetControlMode(GM6020_Axis_t axis);

/* 获取指定轴的实时速度环调试数据（转速、误差、输出、PID 增益）。 */
GM6020_SpeedDebugData_t GM6020_GetSpeedDebugData(
    GM6020_Axis_t axis);

/*
 * 主循环调度函数 —— 应尽可能高频调用（~1 kHz）。
 *
 * 每轮调用执行：
 *   1. 接收 CAN RX FIFO0 中所有待处理反馈帧
 *   2. 按 StdId 匹配到对应轴 → 更新反馈 → 编码器累计 → 状态机 → PID
 *   3. 按配置的反馈超时时间检查两轴在线状态
 *   4. 若有电流变化，发送合并 0x1FE 帧
 *
 * 控制频率自然锁定在电机反馈帧发送频率（约 1 kHz），无需定时器。
 */
void GM6020_Process(void);

/*
 * 返回指定轴的只读反馈数据指针。
 *
 * 返回的指针指向内部静态数组元素，调用方应视为只读。
 * axis 无效时返回 NULL。
 *
 * 用途：调试观察、串口上报、上位机监控。
 */
const GM6020_Feedback_t *GM6020_GetFeedback(GM6020_Axis_t axis);

/*
 * 获取相对于本次启动初始位置的累计多圈角度。
 * 编码器尚未初始化或参数无效时返回 false。
 */
bool GM6020_GetMultiTurnPosition(
    GM6020_Axis_t axis,
    float *position_deg);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
