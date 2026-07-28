/**
 * ===========================================================================
 * @file    gimbal_params.h
 * @brief   双轴云台 GM6020 电机控制参数配置 — CAN ID、PID 增益、机械标定、安全限值
 * ===========================================================================
 *
 * 【本文件是项目的核心配置文件】
 *   所有与电机控制相关的参数均集中在此文件中以 #define 形式定义，
 *   编译后由 motor_control.c 中的配置表 (axis_pid_config[],
 *   axis_mechanical_config[]) 读取并填充到运行时控制器结构体中。
 *   修改参数后重新编译即可生效，无需修改任何 C 代码。
 *
 * 【调参流程】
 *   1. 确认电机 CAN ID 和电流槽位 (YAW_FEEDBACK_STD_ID 等)
 *   2. 标定机械中位对应的编码器角度 (YAW_ZERO_OFFSET_DEG 等)
 *   3. 设置机械限位 (YAW_ANGLE_LIMIT_ENABLE 等)
 *   4. 先调速度环 (SPEED_PID_KP/KD)，再调角度环 (ANGLE_PID_KP)
 *
 * 【参数分组】
 *   - 电机 CAN ID 配置 (第1节)
 *   - Yaw 轴 PID 参数 (第2节)
 *   - Pitch 轴 PID 参数 (第3节)
 *   - Yaw 轴机械参数 (第4节)
 *   - Pitch 轴机械参数 (第5节)
 *   - 公共安全参数 (第6节)
 * ===========================================================================
 */

#ifndef GIMBAL_PARAMS_H
#define GIMBAL_PARAMS_H

/*
 * 【调参总体流程】
 *   1. 确认电机 CAN ID 和电流槽位
 *   2. 标定机械中位对应的编码器角度
 *   3. 设置机械限位
 *   4. 先调速度环，再调角度环
 */

/*==================== 电机 CAN ID 配置 (Motor CAN ID Configuration) ====================*/
/* Yaw (偏航) 电机 */
#define YAW_FEEDBACK_STD_ID             0x205U   /* Yaw 反馈帧标准ID — Yaw feedback StdID */
#define YAW_CURRENT_COMMAND_SLOT        0U       /* Yaw 电流命令在0x1FE中的槽位 (DATA[0:1]) */

/* Pitch (俯仰) 电机 */
#define PITCH_FEEDBACK_STD_ID           0x206U   /* Pitch 反馈帧标准ID — Pitch feedback StdID */
#define PITCH_CURRENT_COMMAND_SLOT      1U       /* Pitch 电流命令在0x1FE中的槽位 (DATA[2:3]) */

/*======================= Yaw 轴 PID 参数 (Yaw Axis PID Parameters) =======================*/
/* ---- 速度环 (Speed Loop / Inner Loop) ---- */
#define YAW_SPEED_PID_KP                20.0f    /* 比例增益 — proportional gain */
#define YAW_SPEED_PID_KI                0.0f     /* 积分增益 — integral gain */
#define YAW_SPEED_PID_KD                0.0f     /* 微分增益 — derivative gain */
#define YAW_SPEED_PID_OUTPUT_LIMIT      8192.0f  /* 速度环输出限幅 (转矩电流, ±8192 ≈ ±1.5A) */

/* ---- 角度环 (Angle Loop / Outer Loop) ---- */
#define YAW_ANGLE_PID_KP                10.0f    /* 比例增益 — proportional gain */
#define YAW_ANGLE_PID_KI                0.0f     /* 积分增益 — integral gain */
#define YAW_ANGLE_PID_KD                0.0f     /* 微分增益 — derivative gain */
#define YAW_ANGLE_SPEED_LIMIT_RPM       200.0f   /* 角度环输出限幅 (目标转速上限 rpm) */

/*===================== Pitch 轴 PID 参数 (Pitch Axis PID Parameters) =====================*/
/* ---- 速度环 (Speed Loop) ---- */
#define PITCH_SPEED_PID_KP              20.0f    /* 比例增益 — 俯仰惯量小，增益可比Yaw高 */
#define PITCH_SPEED_PID_KI              0.0f     /* 积分增益 — integral gain */
#define PITCH_SPEED_PID_KD              0.0f     /* 微分增益 — derivative gain */
#define PITCH_SPEED_PID_OUTPUT_LIMIT    8192.0f  /* 速度环输出限幅 (转矩电流) */

/* ---- 角度环 (Angle Loop) ---- */
#define PITCH_ANGLE_PID_KP              10.0f    /* 比例增益 — proportional gain */
#define PITCH_ANGLE_PID_KI              0.0f     /* 积分增益 — integral gain */
#define PITCH_ANGLE_PID_KD              0.0f     /* 微分增益 — derivative gain */
#define PITCH_ANGLE_SPEED_LIMIT_RPM     100.0f   /* 角度环输出限幅 — 俯仰轴需更保守 */

/*===================== Yaw 轴机械参数 (Yaw Axis Mechanical Parameters) ====================*/
/*
 * 逻辑角度 0° 对应的电机单圈编码器角度 (Encoder angle at mechanical zero position)。
 * 安装完成后读取中位反馈角度，并填入 YAW_ZERO_OFFSET_DEG。
 * 例：若云台中位时编码器读数为 2048，则设 YAW_ZERO_OFFSET_DEG = 2048 × 360 / 8192 = 90°。
 * 上电自动标定完成后，运行时会用开机姿态覆盖这里的默认值。
 */
#define YAW_ZERO_OFFSET_DEG             0.0f     /* 零位偏置 — zero position offset (degrees) */

/*
 * Yaw 逻辑角度软限位 (Soft angle limits)。当前 ±180° 是占位值 (placeholder)，
 * 必须根据实际线材 (cable routing) 和机械干涉范围 (mechanical interference)
 * 缩小，例如 -120°~120°。
 * 启用后角度命令和 PID 输出会自动限幅。
 */
#define YAW_ANGLE_LIMIT_ENABLE          0U       /* 软限位使能 — 0=关闭, 1=开启 */
#define YAW_MIN_ANGLE_DEG              (-180.0f) /* 最小逻辑角度 — minimum logical angle */
#define YAW_MAX_ANGLE_DEG               180.0f   /* 最大逻辑角度 — maximum logical angle */

/*=================== Pitch 轴机械参数 (Pitch Axis Mechanical Parameters) ==================*/
#define PITCH_ZERO_OFFSET_DEG           0.0f     /* 零位偏置 — zero position offset (degrees) */

/*
 * 当前需求只指定 Yaw 限位，因此 Pitch 默认关闭。
 * 确定俯仰机械范围后可改为 1U，并填写上下限。
 */
#define PITCH_ANGLE_LIMIT_ENABLE        0U       /* 软限位使能 — 0=关闭, 1=开启 */
#define PITCH_MIN_ANGLE_DEG            (-30.0f)  /* 最小俯仰角度 — 低头 (look down) */
#define PITCH_MAX_ANGLE_DEG             30.0f    /* 最大俯仰角度 — 抬头 (look up) */

/*===================== 公共安全参数 (Common Safety Parameters) =====================*/
/*
 * Yaw 单轴装机测试模式：
 *   1 = 只等待/标定/遥控 Yaw，发送 0x1FE 时强制 Pitch 电流槽为 0；
 *   0 = 恢复 Yaw + Pitch 双轴标定和遥控控制。
 */
#define GIMBAL_YAW_ONLY_TEST_MODE       1U

/* 反馈超时阈值：100ms 内未收到有效 CAN 反馈 → FAULT 状态 → 输出零电流 */
#define GM6020_FEEDBACK_TIMEOUT_MS      100U     /* 反馈超时 — feedback timeout (ms) */

/* 速度调试模式转速硬限幅：±200 RPM (GM6020 额定转速约 300 RPM) */
#define GM6020_DEBUG_SPEED_LIMIT_RPM    200.0f   /* 调试转速上限 — debug speed limit (rpm) */

#endif /* GIMBAL_PARAMS_H */
