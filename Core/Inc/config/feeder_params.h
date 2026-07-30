/**
 * ===========================================================================
 * @file    feeder_params.h
 * @brief   C610/M2006拨弹盘速度环PID与安全参数
 * ===========================================================================
 *
 * 实机已经确认方向A为拨弹方向。PID参数仍需根据空载和带弹日志逐步整定。
 */

#ifndef FEEDER_PARAMS_H
#define FEEDER_PARAMS_H

/* C610电调ID 3：反馈0x203，控制0x200的DATA[4:5]。 */
#define FEEDER_ESC_ID                         3U
#define FEEDER_CONTROL_STD_ID             0x200U
#define FEEDER_FEEDBACK_STD_ID            0x203U
#define FEEDER_CURRENT_SLOT                   2U

/* STM32F407 CAN1使用过滤器组0~13；组1专用于0x203并路由到FIFO1。 */
#define FEEDER_CAN_FILTER_BANK                1U

/*
 * 拨弹速度环：
 *   - Kp直接按转速误差产生电流；
 *   - Ki按秒积分，并带独立积分限幅和输出饱和anti-windup；
 *   - Kd按误差变化率计算，并使用一阶低通滤波；首次调试保持为0。
 */
#define FEEDER_TEST_TARGET_SPEED_RPM         3000.0f
#define FEEDER_TARGET_RAMP_RPM_S            500.0f
#define FEEDER_SPEED_KP                        7.0f
#define FEEDER_SPEED_KI                         8.0f
#define FEEDER_SPEED_KD                         0.0f
#define FEEDER_SPEED_INTEGRAL_LIMIT_RAW      1000.0f
#define FEEDER_SPEED_D_FILTER_HZ               50.0f
#define FEEDER_CURRENT_LIMIT_RAW             6000
#define FEEDER_CURRENT_SLEW_RAW_PER_MS        200

/* 反馈、重新解锁和堵转保护。 */
#define FEEDER_FEEDBACK_TIMEOUT_MS            50U
#define FEEDER_NEUTRAL_REARM_MS              100U
#define FEEDER_REARM_MAX_SPEED_RPM             5
#define FEEDER_STALL_SPEED_THRESHOLD_RPM       5
#define FEEDER_STALL_CURRENT_THRESHOLD_RAW   400
#define FEEDER_STALL_TIMEOUT_MS              500U

/* 停止时仍每10 ms重发一次零电流，运行时控制量变化会立即发送。 */
#define FEEDER_ZERO_KEEPALIVE_MS              10U
#define FEEDER_MAX_CONTROL_DELTA_MS            10U

#if (FEEDER_ESC_ID < 1U) || (FEEDER_ESC_ID > 4U)
#error "FEEDER_ESC_ID must be in the 0x200 control-frame range 1..4"
#endif

#if FEEDER_CURRENT_SLOT != (FEEDER_ESC_ID - 1U)
#error "FEEDER_CURRENT_SLOT must equal FEEDER_ESC_ID - 1"
#endif

#if FEEDER_CURRENT_LIMIT_RAW > 10000
#error "C610 current command must stay within +/-10000"
#endif

#endif /* FEEDER_PARAMS_H */
