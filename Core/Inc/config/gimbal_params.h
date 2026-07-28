#ifndef GIMBAL_PARAMS_H
#define GIMBAL_PARAMS_H

/*
 *   1. 确认电机 CAN ID 和电流槽位
 *   2. 标定机械中位对应的编码器角度
 *   3. 设置机械限位
 *   4. 先调速度环，再调角度环
 */

/*======================== 电机 CAN ID 配置 ========================*/
#define YAW_FEEDBACK_STD_ID             0x205U
#define YAW_CURRENT_COMMAND_SLOT        0U

#define PITCH_FEEDBACK_STD_ID           0x206U
#define PITCH_CURRENT_COMMAND_SLOT      1U

/*========================== Yaw 轴 PID 参数 ==========================*/
#define YAW_SPEED_PID_KP                10.0f
#define YAW_SPEED_PID_KI                0.0f
#define YAW_SPEED_PID_KD                0.0f
#define YAW_SPEED_PID_OUTPUT_LIMIT      8192.0f

#define YAW_ANGLE_PID_KP                10.0f
#define YAW_ANGLE_PID_KI                0.0f
#define YAW_ANGLE_PID_KD                0.0f
#define YAW_ANGLE_SPEED_LIMIT_RPM       200.0f

/*========================= Pitch 轴 PID 参数 =========================*/
#define PITCH_SPEED_PID_KP              20.0f
#define PITCH_SPEED_PID_KI              0.0f
#define PITCH_SPEED_PID_KD              0.0f
#define PITCH_SPEED_PID_OUTPUT_LIMIT    8192.0f

#define PITCH_ANGLE_PID_KP              10.0f
#define PITCH_ANGLE_PID_KI              0.0f
#define PITCH_ANGLE_PID_KD              0.0f
#define PITCH_ANGLE_SPEED_LIMIT_RPM     100.0f

/*========================== Yaw 轴机械参数 ==========================*/
/*
 * 逻辑角度 0° 对应的电机单圈编码器角度。
 * 安装完成后读取中位反馈角度，并填入 YAW_ZERO_OFFSET_DEG。
 */
#define YAW_ZERO_OFFSET_DEG             0.0f

/*
 * Yaw 逻辑角度软限位。当前 ±180° 是占位值，必须根据实际线材和机械
 * 干涉范围缩小，例如 -120°~120°。
 */
#define YAW_ANGLE_LIMIT_ENABLE          0U
#define YAW_MIN_ANGLE_DEG              (-180.0f)
#define YAW_MAX_ANGLE_DEG               180.0f

/*========================= Pitch 轴机械参数 =========================*/
#define PITCH_ZERO_OFFSET_DEG           0.0f

/*
 * 当前需求只指定 Yaw 限位，因此 Pitch 默认关闭。
 * 确定俯仰机械范围后可改为 1U，并填写上下限。
 */
#define PITCH_ANGLE_LIMIT_ENABLE        0U
#define PITCH_MIN_ANGLE_DEG            (-30.0f)
#define PITCH_MAX_ANGLE_DEG             30.0f

/*=========================== 公共安全参数 ===========================*/
#define GM6020_FEEDBACK_TIMEOUT_MS      100U
#define GM6020_DEBUG_SPEED_LIMIT_RPM    200.0f

#endif /* GIMBAL_PARAMS_H */
