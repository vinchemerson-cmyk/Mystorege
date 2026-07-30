/**
 * ===========================================================================
 * @file    motor_control.h
 * @brief   双轴 GM6020 云台电机控制模块 — 对外 API 声明、类型定义与数据接口
 * ===========================================================================
 *
 * 【模块职责】
 *   管理两个 DJI GM6020 无刷直流电机的完整伺服控制：
 *     - CAN 反馈接收（编码器角度、转速、转矩电流、温度）
 *     - 多圈编码器累计追踪（跨零点检测）
 *     - 串级 PID 控制（外环角度环 → 内环速度环 → 转矩电流输出）
 *     - 锁存式急停保护
 *     - 速度环调试模式
 *
 * 【数据类型】
 *   GM6020_Axis_t          轴枚举 (Yaw=0, Pitch=1)
 *   GM6020_ControlMode_t   控制模式 (位置控制 / 速度调试)
 *   GM6020_Feedback_t      完整反馈数据（角度/转速/电流/温度/在线状态）
 *   GM6020_SpeedDebugData_t 速度环调试数据（用于上位机绘图）
 *
 * 【对外 API 分组】
 *   初始化:    GM6020_Init()
 *   位置控制:  GM6020_SetTargetPosition(), GM6020_SetMultiTurnTargetPosition(),
 *              GM6020_SetGimbalPosition()
 *   速度调试:  GM6020_EnterSpeedDebug(), GM6020_SetSpeedDebugTarget(),
 *              GM6020_ExitSpeedDebug()
 *   PID 调参:  GM6020_SetSpeedPidGains(), GM6020_SetAnglePidGains()
 *   安全:      GM6020_EmergencyStop(), GM6020_ClearEmergencyStop(),
 *              GM6020_IsEmergencyStopped()
 *   查询:      GM6020_GetFeedback(), GM6020_GetMultiTurnPosition(),
 *              GM6020_GetTargetPosition(), GM6020_GetControlMode(),
 *              GM6020_GetSpeedDebugData()
 *   主循环:    GM6020_Process()
 *
 * 【依赖】
 *   config/gimbal_params.h  CAN ID、PID 增益、零位偏置、软限位
 *   CAN1 硬件 (PD0/PD1)     Yaw 电机总线（经滑环到下板 CAN2）
 *   CAN2 硬件 (PB5/PB6)     Pitch 电机总线
 *
 * 【控制架构】参见 motor_control.c 文件头注释。
 * ===========================================================================
 */

#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define GM6020_ENCODER_CPR 8192U

/*
 * 轴枚举 (Axis Enumeration)。
 *
 * 当前电机映射：
 *   Yaw   (偏航 / Yaw axis)    → 上板 CAN1，GM6020 ID 2，反馈 StdId = 0x206
 *   Pitch (俯仰 / Pitch axis)  → 上板 CAN2，GM6020 ID 2，反馈 StdId = 0x206
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
  GM6020_MODE_POSITION = 0, /* 位置控制模式 — Position Control: 角度环 + 速度环串级 */
  GM6020_MODE_SPEED_DEBUG   /* 速度调试模式 — Speed Debug: 绕过角度环，目标RPM直接进速度环 */
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
  uint16_t angle;           /* 单圈机械角度原始值 — raw single-turn mechanical angle (0~8191, 8192 CPR) */
  int32_t turn_count;       /* 累计圈数 — accumulated turn count: 正向跨零+1, 反向跨零-1, starts at 0 */
  int32_t total_angle_ecd;  /* 多圈累计角度 — multi-turn total angle in encoder counts */
  float total_angle_deg;    /* 多圈累计角度 — multi-turn total angle in degrees */
  int16_t speed_rpm;        /* 电机反馈转速 — feedback speed (rpm), 正方向由安装和标定定义 */
  int16_t torque_current;   /* 转矩电流原始值 — torque current raw value (±16384 ≈ ±3A per GM6020 datasheet) */
  uint8_t temperature;      /* 电机内部温度 — internal temperature sensor reading (℃) */
  uint32_t last_rx_ms;      /* 最近有效反馈时间戳 — last valid CAN feedback timestamp (HAL_GetTick) */
  uint32_t rx_sequence;     /* 有效反馈帧序号 — increments once for every newly decoded CAN frame */
  bool online;              /* 在线状态 — true: 距上次反馈未超过超时阈值 */
} GM6020_Feedback_t;

/*
 * 单轴速度环调试数据。
 *
 * 将速度环的关键变量打包为结构体，方便调试器观察或串口发送到上位机。
 * 可配合 GM6020_GetSpeedDebugData() 在运行中实时获取。
 */
typedef struct
{
  float target_speed_rpm;     /* 速度环目标转速 — target speed (rpm) */
  float feedback_speed_rpm;   /* 电机反馈转速 — feedback speed (rpm) */
  float speed_error_rpm;      /* 转速误差 — speed error = target - feedback */
  float output_current;       /* 速度环 PID 输出的转矩电流值 — output torque current */
  int16_t command_current;     /* 限幅后实际写入CAN控制帧的电流命令 */
  float speed_feedforward_rpm;/* 角度环叠加的目标速度前馈 */
  float current_feedforward;  /* 速度环叠加的模型电流前馈 */
  float kp;                   /* 当前使用的比例增益 — current proportional gain (Kp) */
  float ki;                   /* 当前使用的积分增益 — current integral gain (Ki) */
  float kd;                   /* 当前使用的微分增益 — current derivative gain (Kd) */
  GM6020_ControlMode_t mode;  /* 当前控制模式 — current control mode */
} GM6020_SpeedDebugData_t;

/*
 * 初始化 Yaw/Pitch 两轴控制器。
 *
 * 内部流程：
 *   1. 校验 Yaw/Pitch CAN 句柄和轴配置
 *   2. 配置 CAN1/Yaw 与 CAN2/Pitch 的独立硬件滤波器
 *   3. 启动两路 CAN 外设
 *   4. 在两条总线上分别发送初始 0x1FE 零电流帧
 *
 * 必须在 MX_CAN1_Init() 和 MX_CAN2_Init() 之后调用。
 * 返回 HAL_OK 表示成功，否则应调用 Error_Handler()。
 */
HAL_StatusTypeDef GM6020_Init(CAN_HandleTypeDef *yaw_can,
                              CAN_HandleTypeDef *pitch_can);

/*
 * 锁存式急停。
 *
 * 调用后立即在两条 CAN 总线上分别发送 0x1FE 零电流命令，
 * 并阻止位置/速度命令生效。
 * 急停状态会保持到 GM6020_ClearEmergencyStop() 被显式调用。
 */
HAL_StatusTypeDef GM6020_EmergencyStop(void);

/*
 * 解除急停并锁定两轴当前位置，不恢复急停前的旧运动目标。
 */
void GM6020_ClearEmergencyStop(void);

/* 查询当前是否处于锁存急停状态。 */
bool GM6020_IsEmergencyStopped(void);

/*
 * 应用两轴编码器机械零位。
 *
 * yaw_zero_ecd/pitch_zero_ecd 为云台处在逻辑 0° 时对应的 GM6020
 * 单圈编码器值。只能在主循环中、两轴静止时调用；函数会把位置
 * 目标同步到当前反馈，更新零点时不会产生位置跳变。
 */
bool GM6020_SetZeroOffsetsEcd(uint16_t yaw_zero_ecd,
                             uint16_t pitch_zero_ecd);

/*
 * 仅设置指定轴的编码器机械零位。
 *
 * 用于单轴装机测试；只检查并更新所选轴，不要求另一轴在线。
 */
bool GM6020_SetAxisZeroOffsetEcd(GM6020_Axis_t axis,
                                uint16_t zero_ecd);

/*
 * 设置单轴位置目标。
 *
 * target_angle_deg 为相对配置零位的逻辑角度（度）。
 * 函数内部执行顺序：
 *   1. 若有软限位（angle_limit_enabled），正常位置将目标限幅到
 *      [min, max]；若当前位置已在限位外，只允许保持当前位置或向
 *      正常范围内恢复，不允许命令继续向外运动
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
 * target_angle_deg 相对于标定后的机械零点：
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
 * 为位置控制提交轨迹前馈。
 *
 * target_speed_rpm是位置目标的一阶导数，target_acceleration_rpm_s是
 * 二阶导数。接口只缓存并限幅前馈，不改变位置目标；急停、故障、反馈源
 * 切换和速度调试模式会自动清零。没有可靠加速度轨迹时传0。
 */
void GM6020_SetPositionFeedforward(
    GM6020_Axis_t axis,
    float target_speed_rpm,
    float target_acceleration_rpm_s);

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
 *   2. 按 CAN1/Yaw、CAN2/Pitch 路由反馈 → 编码器累计 → 状态机 → PID
 *   3. 按配置的反馈超时时间检查两轴在线状态
 *   4. 若某轴电流变化，在该轴对应 CAN 总线上发送 0x1FE 帧
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
 * 获取相对于标定机械零点的累计多圈角度。
 * 编码器尚未初始化或参数无效时返回 false。
 */
bool GM6020_GetMultiTurnPosition(
    GM6020_Axis_t axis,
    float *position_deg);

/*
 * 获取位置环当前实际使用的理论位置目标。
 *
 * 只有该轴反馈在线、坐标已初始化、未急停且正在位置控制时返回 true。
 * 纯速度环没有位置目标，因此返回 false。
 */
bool GM6020_GetTargetPosition(
    GM6020_Axis_t axis,
    float *position_deg);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
