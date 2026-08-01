/**
 * ===========================================================================
 * @file    dual_m3508_params.h
 * @brief   CAN2双C620/M3508摩擦轮速度环与安全参数
 * ===========================================================================
 */

#ifndef DUAL_M3508_PARAMS_H
#define DUAL_M3508_PARAMS_H

#define DUAL_M3508_MOTOR_COUNT                 2U

/* C620 ID1/ID2：反馈0x201/0x202，控制0x200的前两个电流槽位。 */
#define DUAL_M3508_MOTOR1_ESC_ID               1U
#define DUAL_M3508_MOTOR2_ESC_ID               2U
#define DUAL_M3508_CONTROL_STD_ID           0x200U
#define DUAL_M3508_MOTOR1_FEEDBACK_STD_ID   0x201U
#define DUAL_M3508_MOTOR2_FEEDBACK_STD_ID   0x202U

/*
 * 两个摩擦轮机械方向相反。逻辑正转统一定义为发射方向：
 *   logical_speed = direction * raw_feedback_speed
 *   raw_current   = direction * logical_pid_current
 */
#define DUAL_M3508_MOTOR1_DIRECTION             1
#define DUAL_M3508_MOTOR2_DIRECTION            -1

/*
 * STM32F407的CAN1/CAN2共享过滤器组。CAN2使用14~27：
 *   bank14/FIFO0 已由Pitch GM6020 0x206占用；
 *   bank15/FIFO1 使用16位列表模式接收0x201和0x202。
 */
#define DUAL_M3508_CAN_FILTER_BANK              15U

/* 首次双机方向与闭环验证参数。 */
#define DUAL_M3508_TARGET_SPEED_RPM               5000.0f//5000.0f//乘上减速比
#define DUAL_M3508_TARGET_RAMP_RPM_S              800.0f
#define DUAL_M3508_SPEED_KP                       10.0f
#define DUAL_M3508_SPEED_KI                       0.0f
#define DUAL_M3508_SPEED_KD                       0.0f
#define DUAL_M3508_INTEGRAL_LIMIT_RAW          1000.0f
#define DUAL_M3508_D_FILTER_HZ                   50.0f

/*
 * C620手册：±16384 raw对应±20 A转矩电流。
 * 首次测试限制±3000 raw，约为±3.66 A转矩电流。
 */
#define DUAL_M3508_CURRENT_LIMIT_RAW           10000
#define DUAL_M3508_CURRENT_SLEW_RAW_PER_MS       50

/* 在线、到速、复位和保护参数。 */
#define DUAL_M3508_FEEDBACK_TIMEOUT_MS           50U
#define DUAL_M3508_READY_TOLERANCE_RPM            15
#define DUAL_M3508_READY_HOLD_MS                 200U
#define DUAL_M3508_REARM_HOLD_MS                 100U
#define DUAL_M3508_REARM_MAX_SPEED_RPM            20
#define DUAL_M3508_MAX_TEMPERATURE_C              80U
#define DUAL_M3508_STALL_SPEED_RPM                  5
#define DUAL_M3508_STALL_CURRENT_RAW              400
#define DUAL_M3508_STALL_TIMEOUT_MS              1000U
#define DUAL_M3508_MAX_CONTROL_DELTA_MS            10U

#if DUAL_M3508_CURRENT_LIMIT_RAW > 16384
#error "C620 current command must stay within +/-16384"
#endif

#if (DUAL_M3508_MOTOR1_ESC_ID != 1U) \
    || (DUAL_M3508_MOTOR2_ESC_ID != 2U)
#error "This 0x200 frame layout requires C620 IDs 1 and 2"
#endif

#if ((DUAL_M3508_MOTOR1_DIRECTION != 1) \
     && (DUAL_M3508_MOTOR1_DIRECTION != -1))
#error "M3508 motor1 direction must be +1 or -1"
#endif

#if ((DUAL_M3508_MOTOR2_DIRECTION != 1) \
     && (DUAL_M3508_MOTOR2_DIRECTION != -1))
#error "M3508 motor2 direction must be +1 or -1"
#endif

#if DUAL_M3508_MOTOR1_DIRECTION == DUAL_M3508_MOTOR2_DIRECTION
#error "The two friction motors must use opposite direction signs"
#endif

#endif /* DUAL_M3508_PARAMS_H */
