/**
 * ===========================================================================
 * @file    pitch_fusion_config.h
 * @brief   Pitch 轴 BMI088 / GM6020 融合参数
 * ===========================================================================
 *
 * 默认坐标假设：
 *   - BMI088 X 轴指向云台前方
 *   - BMI088 Z 轴指向云台上方
 *   - 绕 BMI088 Y 轴旋转为 Pitch
 *
 * 实机首次验证必须观察 PITCH_FUSION 诊断输出：
 *   1. 抬头时 GYR、ENC、FUS 应同号；
 *   2. 静止缓慢抬头时 AR（加速度计角度）应与 ENC 同号；
 *   3. 若方向相反，只修改本文件中的轴索引或符号，不修改融合算法。
 * ===========================================================================
 */

#ifndef PITCH_FUSION_CONFIG_H
#define PITCH_FUSION_CONFIG_H

#include "config/control_tuning.h"

/* BMI088 量程与驱动配置一致：Accel ±6 g，Gyro ±1000 dps。 */
#define PITCH_FUSION_ACCEL_LSB_PER_G              5460.0f
#define PITCH_FUSION_GYRO_LSB_PER_DPS           32.768f

/* BMI088 原始数组索引：X=0，Y=1，Z=2。 */
#define PITCH_FUSION_GYRO_AXIS_INDEX                 1U
#define PITCH_FUSION_GYRO_DIRECTION                  1.0f

/*
 * 加速度计 Pitch 角：
 *   atan2(FORWARD_DIRECTION * accel[FORWARD_AXIS],
 *         sqrt(up_axis² + lateral_axis²))
 */
#define PITCH_FUSION_ACCEL_FORWARD_AXIS_INDEX        0U
#define PITCH_FUSION_ACCEL_UP_AXIS_INDEX             2U
#define PITCH_FUSION_ACCEL_FORWARD_DIRECTION        -1.0f
#define PITCH_FUSION_ACCEL_UP_DIRECTION              1.0f

/*
 * 传感器安装零偏：实际云台水平时，accel_pitch_raw_deg应等于该值。
 * Pitch上电零位标定会先减去此值，再把重力水平定义为逻辑0°。
 */
#define PITCH_SENSOR_LEVEL_OFFSET_DEG \
    TUNE_PITCH_SENSOR_LEVEL_OFFSET_DEG

/* GM6020 Pitch 编码器逻辑方向。若抬头时 ENC 变为负数，改成 -1.0f。 */
#define PITCH_FUSION_ENCODER_DIRECTION                1.0f

/* 静止标定：连续满足条件 500 个新 IMU 样本（约 500 ms）。 */
#define PITCH_FUSION_CALIBRATION_SAMPLE_COUNT \
    TUNE_PITCH_FUSION_CALIBRATION_SAMPLE_COUNT
#define PITCH_FUSION_STILL_GYRO_LIMIT_DPS \
    TUNE_PITCH_FUSION_STILL_GYRO_LIMIT_DPS
#define PITCH_FUSION_STILL_ENCODER_RATE_LIMIT_DPS \
    TUNE_PITCH_FUSION_STILL_ENCODER_RATE_LIMIT_DPS

/*
 * 加速度重力角需要同时满足：
 *   1. 加速度模长接近 1 g；
 *   2. 与陀螺预测角的差值不过大。
 *
 * 第二个条件用于拒绝模长仍接近1 g、但方向已经被平动加速度扰乱的样本。
 */
#define PITCH_FUSION_ACCEL_NORM_MIN_G \
    TUNE_PITCH_FUSION_ACCEL_NORM_MIN_G
#define PITCH_FUSION_ACCEL_NORM_MAX_G \
    TUNE_PITCH_FUSION_ACCEL_NORM_MAX_G
#define PITCH_FUSION_ACCEL_INNOVATION_MAX_DEG \
    TUNE_PITCH_FUSION_ACCEL_INNOVATION_MAX_DEG

/*
 * 二维Kalman状态：[惯性Pitch角度(deg), 陀螺零偏(deg/s)]。
 *
 * PROCESS_ANGLE_NOISE_DEG2:
 *   每次预测附加到角度协方差的离散过程噪声。当前4e-6与1deg²
 *   加速度观测噪声组合后，1 kHz稳态角度修正增益约为0.002，
 *   与原0.5 s互补滤波的低频修正速度接近。
 *
 * PROCESS_BIAS_NOISE_DPS2:
 *   每次预测附加到陀螺零偏协方差的随机游走噪声。
 *
 * ACCEL_MEASUREMENT_NOISE_DEG2:
 *   可信加速度重力角的观测噪声方差。实机应使用静止日志方差替换。
 */
#define PITCH_KALMAN_PROCESS_ANGLE_NOISE_DEG2 \
    TUNE_PITCH_KALMAN_PROCESS_ANGLE_NOISE_DEG2
#define PITCH_KALMAN_PROCESS_BIAS_NOISE_DPS2 \
    TUNE_PITCH_KALMAN_PROCESS_BIAS_NOISE_DPS2
#define PITCH_KALMAN_ACCEL_MEASUREMENT_NOISE_DEG2 \
    TUNE_PITCH_KALMAN_ACCEL_MEASUREMENT_NOISE_DEG2
#define PITCH_KALMAN_INITIAL_ANGLE_VARIANCE_DEG2 \
    TUNE_PITCH_KALMAN_INITIAL_ANGLE_VARIANCE_DEG2
#define PITCH_KALMAN_INITIAL_BIAS_VARIANCE_DPS2 \
    TUNE_PITCH_KALMAN_INITIAL_BIAS_VARIANCE_DPS2

/* 数据新鲜度与积分步长保护。 */
#define PITCH_FUSION_IMU_TIMEOUT_MS                   20U
#define PITCH_FUSION_MAX_DELTA_TIME_S                  0.020f
#define PITCH_FUSION_RECOVERY_HEALTHY_SAMPLE_COUNT \
    TUNE_PITCH_FUSION_RECOVERY_HEALTHY_SAMPLE_COUNT

/*
 * Pitch融合闭环启用与首次装机保护。
 *
 * CONTROL_MAX_ANGLE_DELTA_DEG:
 *   闭环使用的融合角相对编码器角最多偏离配置的安全角。融合器内部仍
 *   保留完整惯性角供诊断；增大该值前必须确认方向、机械范围和稳定性。
 *
 * CONTROL_MAX_RATE_DELTA_DPS:
 *   陀螺角速度相对电机反馈角速度的最大修正量。
 *
 * CONTROL_MAX_SPEED_RPM / CONTROL_MAX_CURRENT:
 *   仅在Pitch融合反馈生效时使用的调试期限制，不影响Yaw和Pitch编码器
 *   回退控制。
 */
/*
 * 当前由control_tuning.h启用受限融合闭环。首次上机仍必须确认传感器
 * 方向、创新量和掉线恢复；异常时控制层自动平滑回退到编码器。
 */
#define PITCH_FUSION_CONTROL_ENABLE \
    TUNE_PITCH_FUSION_CONTROL_ENABLE
#define PITCH_FUSION_CONTROL_MAX_ANGLE_DELTA_DEG \
    TUNE_PITCH_FUSION_MAX_ANGLE_DELTA_DEG
#define PITCH_FUSION_CONTROL_MAX_RATE_DELTA_DPS \
    TUNE_PITCH_FUSION_MAX_RATE_DELTA_DPS
#define PITCH_FUSION_CONTROL_MAX_SPEED_RPM \
    TUNE_PITCH_FUSION_MAX_SPEED_RPM
#define PITCH_FUSION_CONTROL_MAX_CURRENT \
    TUNE_PITCH_FUSION_MAX_CURRENT_RAW

/*
 * Pitch传感器零点标定后的自动回零。
 *
 * 标定成功后先从当前位置建立目标，再以固定角速度向传感器定义的0°移动。
 * 目标相对实际位置的领先量受限，避免电机受阻时累积过大的位置误差。
 */
#define PITCH_AUTO_ZERO_RETURN_ENABLE                   1U
#define PITCH_AUTO_ZERO_RETURN_RATE_DPS \
    TUNE_PITCH_STARTUP_RETURN_RATE_DPS
#define PITCH_AUTO_ZERO_MAX_TARGET_LEAD_DEG \
    TUNE_PITCH_STARTUP_MAX_TARGET_LEAD_DEG
/*
 * 实测机械端点约为-34.23°（抬头）和+21.88°（低头）。
 * 启动包络只增加少量测量容差，不等同于正常运行软限位。
 */
#define PITCH_AUTO_ZERO_MIN_START_ANGLE_DEG           (-35.0f)
#define PITCH_AUTO_ZERO_MAX_START_ANGLE_DEG            23.0f
#define PITCH_AUTO_ZERO_RELEASE_DELAY_MS \
    TUNE_PITCH_STARTUP_RELEASE_DELAY_MS
#define PITCH_AUTO_ZERO_RETURN_TIMEOUT_MS \
    TUNE_PITCH_STARTUP_RETURN_TIMEOUT_MS
#define PITCH_AUTO_ZERO_TOLERANCE_DEG \
    TUNE_PITCH_STARTUP_TOLERANCE_DEG
#define PITCH_AUTO_ZERO_SETTLE_SPEED_RPM \
    TUNE_PITCH_STARTUP_SETTLE_SPEED_RPM
#define PITCH_AUTO_ZERO_SETTLE_TIME_MS \
    TUNE_PITCH_STARTUP_SETTLE_TIME_MS
#define PITCH_AUTO_ZERO_MAX_DELTA_MS                   20U

#if (PITCH_FUSION_GYRO_AXIS_INDEX > 2U) \
    || (PITCH_FUSION_ACCEL_FORWARD_AXIS_INDEX > 2U) \
    || (PITCH_FUSION_ACCEL_UP_AXIS_INDEX > 2U)
#error "Pitch fusion BMI088 axis index must be 0, 1, or 2."
#endif

#if PITCH_FUSION_ACCEL_FORWARD_AXIS_INDEX \
    == PITCH_FUSION_ACCEL_UP_AXIS_INDEX
#error "Pitch fusion forward and up accelerometer axes must be different."
#endif

#endif /* PITCH_FUSION_CONFIG_H */
