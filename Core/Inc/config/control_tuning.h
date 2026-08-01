/**
 * ===========================================================================
 * @file    control_tuning.h
 * @brief   拨轮、拨弹盘和Pitch水平控制的集中调参入口
 * ===========================================================================
 *
 * 只把需要反复上机调整的量放在这里。CAN ID、过滤器组和机械限位等
 * 硬件固定配置仍保留在各模块原有配置文件中。
 *
 * 建议调参顺序：
 *   1. 确认拨轮上下方向和死区；
 *   2. 先调拨弹盘速度环，再调单发位置外环；
 *   3. Pitch先调速度环，再调角度环，最后逐步放宽融合安全限制；
 *   4. 每次只修改一组参数，并保存USART6日志。
 * ===========================================================================
 */

#ifndef CONTROL_TUNING_H
#define CONTROL_TUNING_H

/*=========================== DBUS拨轮 ===========================*/

/*
 * 拨轮去中心值约为-660~+660。
 * DOWN_DIRECTION=+1表示原始正值是向下；若实机上下相反，只改成-1。
 */
#define TUNE_DBUS_DIAL_DOWN_DIRECTION                  1
#define TUNE_DBUS_DIAL_TRIGGER_THRESHOLD             200
#define TUNE_DBUS_DIAL_RELEASE_THRESHOLD             100

/*======================= M2006拨弹盘机械参数 =====================*/

#define TUNE_FEEDER_ENCODER_CPR                      8192
#define TUNE_FEEDER_GEAR_RATIO                         36
#define TUNE_FEEDER_PROJECTILES_PER_OUTPUT_REV         10

/*======================= M2006拨弹盘动作参数 =====================*/

/* 连发使用方向A；退弹使用较低的方向B速度。单位均为电机转子rpm。 */
#define TUNE_FEEDER_CONTINUOUS_SPEED_RPM             3000.0f
#define TUNE_FEEDER_REVERSE_SPEED_RPM                300.0f
#define TUNE_FEEDER_SPEED_RAMP_RPM_S                8000.0f

/*
 * 单发位置外环：
 *   输出轴每发转过360/10=36°，对应电机3.6圈。
 *   第一次在当前相位基础上增加一发步距和前向冗余；以后从上一发已经
 *   完成的超出相位严格增加一发步距，不反向返回理论弹位。
 *   OVERSHOOT_PERCENT必须明显小于50%，防止接近下一发落点。
 */
#define TUNE_FEEDER_SINGLE_POSITION_KP_RPM_PER_DEG     12.0f//20.0f
#define TUNE_FEEDER_SINGLE_MAX_SPEED_RPM              1000.0f
#define TUNE_FEEDER_SINGLE_OVERSHOOT_PERCENT             8
#define TUNE_FEEDER_SINGLE_POSITION_TOLERANCE_ECD     256
#define TUNE_FEEDER_SINGLE_MAX_FORWARD_OVERRUN_ECD   1024
#define TUNE_FEEDER_SINGLE_SETTLE_SPEED_RPM            10
#define TUNE_FEEDER_SINGLE_SETTLE_TIME_MS              50U

/*
 * 单发完成后的单向保持：只在弹丸压力把拨盘向后推时提供较小正向恢复力，
 * 目标位置或目标前方不施加反向电流。保持限流必须显著低于单发运动限流。
 */
#define TUNE_FEEDER_SINGLE_HOLD_DEADBAND_ECD           256
#define TUNE_FEEDER_SINGLE_HOLD_MAX_SPEED_RPM           60.0f
#define TUNE_FEEDER_SINGLE_HOLD_CURRENT_LIMIT_RAW      800

/*======================= M2006拨弹盘速度PID ======================*/

#define TUNE_FEEDER_SPEED_KP                            15.0f
#define TUNE_FEEDER_SPEED_KI                            5.0f
#define TUNE_FEEDER_SPEED_KD                            0.0f
#define TUNE_FEEDER_SPEED_INTEGRAL_LIMIT_RAW         1000.0f
#define TUNE_FEEDER_SPEED_D_FILTER_HZ                   50.0f

/* 不同动作分别限流；最终仍受C610协议±10000限制。 */
#define TUNE_FEEDER_CONTINUOUS_CURRENT_LIMIT_RAW      6000
#define TUNE_FEEDER_SINGLE_CURRENT_LIMIT_RAW          6000//3500
#define TUNE_FEEDER_REVERSE_CURRENT_LIMIT_RAW         2500
#define TUNE_FEEDER_CURRENT_SLEW_RAW_PER_MS            120

/* 反馈、重新解锁和堵转保护。 */
#define TUNE_FEEDER_FEEDBACK_TIMEOUT_MS                 50U
#define TUNE_FEEDER_NEUTRAL_REARM_MS                   100U
#define TUNE_FEEDER_REARM_MAX_SPEED_RPM                  5
#define TUNE_FEEDER_STALL_SPEED_THRESHOLD_RPM            5
#define TUNE_FEEDER_STALL_CURRENT_THRESHOLD_RAW        3500
#define TUNE_FEEDER_STALL_TIMEOUT_MS                   500U

/*====================== Pitch遥控与自动水平 ======================*/

#define TUNE_PITCH_REMOTE_DEADZONE                       30
#define TUNE_PITCH_REMOTE_FULL_SCALE                    660
#define TUNE_PITCH_REMOTE_MAX_RATE_DPS                   60.0f

/*
 * 摇杆回中后，目标以RETURN_RATE平滑回到传感器水平0°。
 * 车辆颠簸时目标保持0°，陀螺仪反馈仍以1 kHz立即参与控制，不受该斜坡限制。
 */
#define TUNE_PITCH_LEVEL_TARGET_DEG                       0.0f
#define TUNE_PITCH_LEVEL_RETURN_RATE_DPS                 60.0f

/*=========================== Pitch PID ===========================*/

#define TUNE_PITCH_SPEED_PID_KP                         170.0f
#define TUNE_PITCH_SPEED_PID_KI                          0.0f
#define TUNE_PITCH_SPEED_PID_KD                           0.0f
#define TUNE_PITCH_SPEED_PID_OUTPUT_LIMIT              8192.0f

#define TUNE_PITCH_ANGLE_PID_KP                          8.0f
#define TUNE_PITCH_ANGLE_PID_KI                           0.0f
#define TUNE_PITCH_ANGLE_PID_KD                           0.0f
#define TUNE_PITCH_ANGLE_SPEED_LIMIT_RPM                150.0f

#define TUNE_PITCH_TARGET_RATE_FF_GAIN                    0.25f
#define TUNE_PITCH_BASE_RATE_FF_GAIN                      0.0f
#define TUNE_PITCH_GRAVITY_SIN_FF_CURRENT                 1.0f
#define TUNE_PITCH_GRAVITY_COS_FF_CURRENT                 0.0f
#define TUNE_PITCH_GRAVITY_BIAS_FF_CURRENT                0.0f
#define TUNE_PITCH_STATIC_FRICTION_FF_CURRENT             0.0f
#define TUNE_PITCH_VELOCITY_FF_CURRENT_PER_RPM            0.0f
#define TUNE_PITCH_ACCEL_FF_CURRENT_PER_RPM_S             0.0f
#define TUNE_PITCH_FEEDFORWARD_CURRENT_LIMIT           2000.0f

/*====================== Pitch BMI088/Kalman ======================*/

#define TUNE_PITCH_SENSOR_LEVEL_OFFSET_DEG                 0.0f
#define TUNE_PITCH_FUSION_CALIBRATION_SAMPLE_COUNT       500U
#define TUNE_PITCH_FUSION_STILL_GYRO_LIMIT_DPS             2.0f
#define TUNE_PITCH_FUSION_STILL_ENCODER_RATE_LIMIT_DPS     6.0f
#define TUNE_PITCH_FUSION_ACCEL_NORM_MIN_G                  0.85f
#define TUNE_PITCH_FUSION_ACCEL_NORM_MAX_G                  1.15f
#define TUNE_PITCH_FUSION_ACCEL_INNOVATION_MAX_DEG         12.0f
#define TUNE_PITCH_FUSION_RECOVERY_HEALTHY_SAMPLE_COUNT    20U

#define TUNE_PITCH_KALMAN_PROCESS_ANGLE_NOISE_DEG2    0.000004f
#define TUNE_PITCH_KALMAN_PROCESS_BIAS_NOISE_DPS2    0.00000001f
#define TUNE_PITCH_KALMAN_ACCEL_MEASUREMENT_NOISE_DEG2    5.0f
#define TUNE_PITCH_KALMAN_INITIAL_ANGLE_VARIANCE_DEG2     0.01f
#define TUNE_PITCH_KALMAN_INITIAL_BIAS_VARIANCE_DPS2      0.01f

/*
 * 融合闭环仍保留编码器安全带。为允许车体较大俯仰时保持世界系水平，
 * 控制用融合角相对编码器角的最大偏差放宽到±10°；机械端点保护仍由
 * 编码器软限位独立执行。
 */
#define TUNE_PITCH_FUSION_CONTROL_ENABLE                  1U
#define TUNE_PITCH_FUSION_MAX_ANGLE_DELTA_DEG            45.0f
#define TUNE_PITCH_FUSION_MAX_RATE_DELTA_DPS            180.0f
#define TUNE_PITCH_FUSION_MAX_SPEED_RPM                  45.0f
#define TUNE_PITCH_FUSION_MAX_CURRENT_RAW              4000.0f

/* Pitch上电传感器标定后的慢速回正，独立于运行中的水平稳定。 */
#define TUNE_PITCH_STARTUP_RETURN_RATE_DPS                10.0f
#define TUNE_PITCH_STARTUP_MAX_TARGET_LEAD_DEG             2.0f
#define TUNE_PITCH_STARTUP_RELEASE_DELAY_MS              1000U
#define TUNE_PITCH_STARTUP_RETURN_TIMEOUT_MS             8000U
#define TUNE_PITCH_STARTUP_TOLERANCE_DEG                    0.5f
#define TUNE_PITCH_STARTUP_SETTLE_SPEED_RPM                 2
#define TUNE_PITCH_STARTUP_SETTLE_TIME_MS                 100U

#if (TUNE_DBUS_DIAL_DOWN_DIRECTION != 1) \
    && (TUNE_DBUS_DIAL_DOWN_DIRECTION != -1)
#error "TUNE_DBUS_DIAL_DOWN_DIRECTION must be +1 or -1"
#endif

#if TUNE_DBUS_DIAL_RELEASE_THRESHOLD \
    >= TUNE_DBUS_DIAL_TRIGGER_THRESHOLD
#error "Dial release threshold must be smaller than trigger threshold"
#endif

#if (TUNE_FEEDER_GEAR_RATIO <= 0) \
    || (TUNE_FEEDER_PROJECTILES_PER_OUTPUT_REV <= 0)
#error "Feeder gear ratio and projectile count must be positive"
#endif

#if (TUNE_FEEDER_SINGLE_OVERSHOOT_PERCENT <= 0) \
    || (TUNE_FEEDER_SINGLE_OVERSHOOT_PERCENT >= 50)
#error "Single-shot overshoot must be between 1 and 49 percent"
#endif

#if TUNE_FEEDER_SINGLE_MAX_FORWARD_OVERRUN_ECD \
    < TUNE_FEEDER_SINGLE_POSITION_TOLERANCE_ECD
#error "Single-shot forward overrun window must include position tolerance"
#endif

#if TUNE_FEEDER_SINGLE_HOLD_DEADBAND_ECD \
    < TUNE_FEEDER_SINGLE_POSITION_TOLERANCE_ECD
#error "Single-shot hold deadband must include position tolerance"
#endif

#endif /* CONTROL_TUNING_H */
