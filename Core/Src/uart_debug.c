/**
 * ===========================================================================
 * @file    uart_debug.c
 * @brief   通过STM32 USART6分时输出全部关键调试数据
 * ===========================================================================
 *
 * 串口：460800-8-N-1，ASCII，CRLF。
 * 开发板C型连接：STM32 USART6对应外壳丝印“UART1”的3-pin接口。
 * 只需连接板端TXD到USB-TTL的RX，并连接GND；不要连接接口5V。
 *
 * 低优先级通信任务每10 ms调用一次，十四个时隙循环：
 *   0/2/4/6/8/10/12: POSITION   Yaw实际位置和理论位置
 *   1/7:          GIMBAL       两轴反馈、速度环、PID、标定与急停
 *   3:            IMU_RAW      BMI088六轴原始值与错误计数
 *   5:            PITCH_FUSION Pitch融合、健康状态和控制电流
 *   9:            RC_CHASSIS   DBUS通道、拨杆和底盘发送命令
 *   11:           FEEDER       C610 ID3反馈、状态机和电流命令
 *   13:           FRICTION     双C620/M3508反馈、联锁和电流命令
 *
 * POSITION约50 Hz，GIMBAL约14.3 Hz，其余数据各约7.1 Hz。格式化只
 * 发生在低优先级任务，数据搬运由USART6 TX DMA完成，不会阻塞1 kHz
 * 控制和IMU采集任务。
 * ===========================================================================
 */

#include "uart_debug.h"

#include "bmi088.h"
#include "chassis_can.h"
#include "dbus.h"
#include "dual_m3508.h"
#include "feeder_motor.h"
#include "gimbal_calibration.h"
#include "motor_control.h"
#include "pitch_fusion.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define UART_DEBUG_BUFFER_CAPACITY  512U
#define UART_DEBUG_PHASE_COUNT       14U

#define POSITION_REFERENCE_NONE             0U
#define POSITION_REFERENCE_CONTROLLER       1U
#define POSITION_REFERENCE_SPEED_INTEGRAL   2U
#define POSITION_ACTUAL_ENCODER              1U
#define POSITION_ACTUAL_FUSION               2U
#define POSITION_TRACKER_MAX_DELTA_MS      100U

static UART_HandleTypeDef *debug_uart;
static uint8_t debug_buffer[UART_DEBUG_BUFFER_CAPACITY];
static uint8_t debug_phase;
static uint32_t debug_tx_error_count;

typedef struct
{
  float position_deg;
  uint32_t last_update_ms;
  GM6020_ControlMode_t previous_mode;
  bool initialized;
} PositionReferenceTracker_t;

static PositionReferenceTracker_t
    position_reference_tracker[GM6020_AXIS_COUNT];

static int32_t debug_float_to_scaled(float value, float scale)
{
  float scaled;

  if (!isfinite(value) || !isfinite(scale))
  {
    return 0;
  }

  scaled = value * scale;
  if (scaled >= (float)INT32_MAX)
  {
    return INT32_MAX;
  }
  if (scaled <= (float)INT32_MIN)
  {
    return INT32_MIN;
  }
  return (int32_t)((scaled >= 0.0f)
      ? (scaled + 0.5f)
      : (scaled - 0.5f));
}

static bool get_actual_position(
    GM6020_Axis_t axis,
    float *position_deg,
    uint8_t *source)
{
  const GM6020_Feedback_t *feedback = GM6020_GetFeedback(axis);
  bool position_available;

  if ((position_deg == NULL) || (source == NULL))
  {
    return false;
  }

  *source = POSITION_ACTUAL_ENCODER;
  if (axis == GM6020_AXIS_PITCH)
  {
    float unused_speed_rpm;

    if (PitchFusion_GetControlFeedback(
            position_deg, &unused_speed_rpm))
    {
      *source = POSITION_ACTUAL_FUSION;
      return true;
    }
  }

  position_available =
      GM6020_GetMultiTurnPosition(axis, position_deg);
  return position_available
      && (feedback != NULL)
      && feedback->online;
}

static bool get_reference_position(
    GM6020_Axis_t axis,
    uint32_t now,
    float actual_position_deg,
    bool actual_valid,
    float *reference_position_deg,
    uint8_t *source)
{
  PositionReferenceTracker_t *tracker;
  const GM6020_SpeedDebugData_t control =
      GM6020_GetSpeedDebugData(axis);
  uint32_t delta_ms;

  if ((reference_position_deg == NULL)
      || (source == NULL)
      || ((uint32_t)axis >= (uint32_t)GM6020_AXIS_COUNT))
  {
    return false;
  }

  tracker = &position_reference_tracker[axis];
  *source = POSITION_REFERENCE_NONE;
  if (!actual_valid)
  {
    tracker->initialized = false;
    *reference_position_deg = actual_position_deg;
    return false;
  }

  if (control.mode == GM6020_MODE_POSITION)
  {
    tracker->position_deg = actual_position_deg;
    tracker->last_update_ms = now;
    tracker->previous_mode = control.mode;
    tracker->initialized = true;

    if (GM6020_GetTargetPosition(
            axis, reference_position_deg))
    {
      tracker->position_deg = *reference_position_deg;
      *source = POSITION_REFERENCE_CONTROLLER;
      return true;
    }

    *reference_position_deg = actual_position_deg;
    return false;
  }

  delta_ms = (uint32_t)(now - tracker->last_update_ms);
  if (!tracker->initialized
      || (tracker->previous_mode != control.mode)
      || (delta_ms > POSITION_TRACKER_MAX_DELTA_MS))
  {
    tracker->position_deg = actual_position_deg;
  }
  else
  {
    /*
     * 纯速度环没有位置目标。这里只为串口绘图积分一条理论轨迹：
     * rpm * 6 = deg/s。它不参与电机控制，也不会改变任何位置目标。
     */
    tracker->position_deg +=
        control.target_speed_rpm
        * 6.0f
        * (float)delta_ms
        / 1000.0f;
  }

  tracker->last_update_ms = now;
  tracker->previous_mode = control.mode;
  tracker->initialized = true;
  *reference_position_deg = tracker->position_deg;
  *source = POSITION_REFERENCE_SPEED_INTEGRAL;
  return true;
}

static int format_position_line(void)
{
  const uint32_t now = HAL_GetTick();
  float yaw_actual_deg = 0.0f;
  float yaw_reference_deg = 0.0f;
  uint8_t yaw_actual_source = 0U;
  uint8_t yaw_reference_source = 0U;
  const bool yaw_actual_valid = get_actual_position(
      GM6020_AXIS_YAW,
      &yaw_actual_deg,
      &yaw_actual_source);
  const bool yaw_reference_valid = get_reference_position(
      GM6020_AXIS_YAW,
      now,
      yaw_actual_deg,
      yaw_actual_valid,
      &yaw_reference_deg,
      &yaw_reference_source);
  const GM6020_ControlMode_t yaw_mode =
      GM6020_GetControlMode(GM6020_AXIS_YAW);

#if UART_DEBUG_PITCH_POSITION_ENABLE
  float pitch_actual_deg = 0.0f;
  float pitch_reference_deg = 0.0f;
  uint8_t pitch_actual_source = 0U;
  uint8_t pitch_reference_source = 0U;
  const bool pitch_actual_valid = get_actual_position(
      GM6020_AXIS_PITCH,
      &pitch_actual_deg,
      &pitch_actual_source);
  const bool pitch_reference_valid = get_reference_position(
      GM6020_AXIS_PITCH,
      now,
      pitch_actual_deg,
      pitch_actual_valid,
      &pitch_reference_deg,
      &pitch_reference_source);
  const GM6020_ControlMode_t pitch_mode =
      GM6020_GetControlMode(GM6020_AXIS_PITCH);

  return snprintf(
      (char *)debug_buffer,
      sizeof(debug_buffer),
      "POSITION,T=%lu,Y_ACT=%ld,Y_REF=%ld,Y_AOK=%u,Y_ROK=%u,"
      "Y_ASRC=%u,Y_RSRC=%u,Y_MODE=%u,"
      "P_ACT=%ld,P_REF=%ld,P_AOK=%u,P_ROK=%u,"
      "P_ASRC=%u,P_RSRC=%u,P_MODE=%u,TXE=%lu\r\n",
      (unsigned long)now,
      (long)debug_float_to_scaled(yaw_actual_deg, 100.0f),
      (long)debug_float_to_scaled(yaw_reference_deg, 100.0f),
      yaw_actual_valid ? 1U : 0U,
      yaw_reference_valid ? 1U : 0U,
      (unsigned int)yaw_actual_source,
      (unsigned int)yaw_reference_source,
      (unsigned int)yaw_mode,
      (long)debug_float_to_scaled(pitch_actual_deg, 100.0f),
      (long)debug_float_to_scaled(pitch_reference_deg, 100.0f),
      pitch_actual_valid ? 1U : 0U,
      pitch_reference_valid ? 1U : 0U,
      (unsigned int)pitch_actual_source,
      (unsigned int)pitch_reference_source,
      (unsigned int)pitch_mode,
      (unsigned long)debug_tx_error_count);
#else
  return snprintf(
      (char *)debug_buffer,
      sizeof(debug_buffer),
      "POSITION,T=%lu,Y_ACT=%ld,Y_REF=%ld,Y_AOK=%u,Y_ROK=%u,"
      "Y_ASRC=%u,Y_RSRC=%u,Y_MODE=%u,TXE=%lu\r\n",
      (unsigned long)now,
      (long)debug_float_to_scaled(yaw_actual_deg, 100.0f),
      (long)debug_float_to_scaled(yaw_reference_deg, 100.0f),
      yaw_actual_valid ? 1U : 0U,
      yaw_reference_valid ? 1U : 0U,
      (unsigned int)yaw_actual_source,
      (unsigned int)yaw_reference_source,
      (unsigned int)yaw_mode,
      (unsigned long)debug_tx_error_count);
#endif
}

static int format_gimbal_line(void)
{
  const GM6020_Feedback_t *yaw_feedback_ptr =
      GM6020_GetFeedback(GM6020_AXIS_YAW);
  const GM6020_Feedback_t *pitch_feedback_ptr =
      GM6020_GetFeedback(GM6020_AXIS_PITCH);
  GM6020_Feedback_t yaw_feedback = {0};
  GM6020_Feedback_t pitch_feedback = {0};
  const GM6020_SpeedDebugData_t yaw_control =
      GM6020_GetSpeedDebugData(GM6020_AXIS_YAW);
  const GM6020_SpeedDebugData_t pitch_control =
      GM6020_GetSpeedDebugData(GM6020_AXIS_PITCH);
  float yaw_position_deg = 0.0f;
  float pitch_position_deg = 0.0f;

  if (yaw_feedback_ptr != NULL)
  {
    yaw_feedback = *yaw_feedback_ptr;
  }
  if (pitch_feedback_ptr != NULL)
  {
    pitch_feedback = *pitch_feedback_ptr;
  }
  (void)GM6020_GetMultiTurnPosition(
      GM6020_AXIS_YAW, &yaw_position_deg);
  (void)GM6020_GetMultiTurnPosition(
      GM6020_AXIS_PITCH, &pitch_position_deg);

  return snprintf(
      (char *)debug_buffer,
      sizeof(debug_buffer),
      "GIMBAL,T=%lu,E=%u,CY=%u,CP=%u,"
      "YON=%u,YE=%u,YP=%ld,YR=%d,YTC=%d,YC=%d,"
      "YT=%ld,YER=%ld,YO=%ld,YVFF=%ld,YIFF=%ld,"
      "YKP=%ld,YKI=%ld,YKD=%ld,YM=%u,"
      "PON=%u,PE=%u,PP=%ld,PR=%d,PTC=%d,PC=%d,"
      "PT=%ld,PER=%ld,PO=%ld,PVFF=%ld,PIFF=%ld,"
      "PKP=%ld,PKI=%ld,PKD=%ld,PM=%u,TXE=%lu\r\n",
      (unsigned long)HAL_GetTick(),
      GM6020_IsEmergencyStopped() ? 1U : 0U,
      (unsigned int)GimbalCalibration_GetAxisStatus(
          GM6020_AXIS_YAW),
      (unsigned int)GimbalCalibration_GetAxisStatus(
          GM6020_AXIS_PITCH),
      yaw_feedback.online ? 1U : 0U,
      (unsigned int)yaw_feedback.angle,
      (long)debug_float_to_scaled(yaw_position_deg, 100.0f),
      (int)yaw_feedback.speed_rpm,
      (int)yaw_feedback.torque_current,
      (int)yaw_control.command_current,
      (long)debug_float_to_scaled(
          yaw_control.target_speed_rpm, 100.0f),
      (long)debug_float_to_scaled(
          yaw_control.speed_error_rpm, 100.0f),
      (long)debug_float_to_scaled(
          yaw_control.output_current, 1.0f),
      (long)debug_float_to_scaled(
          yaw_control.speed_feedforward_rpm, 100.0f),
      (long)debug_float_to_scaled(
          yaw_control.current_feedforward, 1.0f),
      (long)debug_float_to_scaled(yaw_control.kp, 100.0f),
      (long)debug_float_to_scaled(yaw_control.ki, 100.0f),
      (long)debug_float_to_scaled(yaw_control.kd, 100.0f),
      (unsigned int)yaw_control.mode,
      pitch_feedback.online ? 1U : 0U,
      (unsigned int)pitch_feedback.angle,
      (long)debug_float_to_scaled(pitch_position_deg, 100.0f),
      (int)pitch_feedback.speed_rpm,
      (int)pitch_feedback.torque_current,
      (int)pitch_control.command_current,
      (long)debug_float_to_scaled(
          pitch_control.target_speed_rpm, 100.0f),
      (long)debug_float_to_scaled(
          pitch_control.speed_error_rpm, 100.0f),
      (long)debug_float_to_scaled(
          pitch_control.output_current, 1.0f),
      (long)debug_float_to_scaled(
          pitch_control.speed_feedforward_rpm, 100.0f),
      (long)debug_float_to_scaled(
          pitch_control.current_feedforward, 1.0f),
      (long)debug_float_to_scaled(pitch_control.kp, 100.0f),
      (long)debug_float_to_scaled(pitch_control.ki, 100.0f),
      (long)debug_float_to_scaled(pitch_control.kd, 100.0f),
      (unsigned int)pitch_control.mode,
      (unsigned long)debug_tx_error_count);
}

static int format_rc_chassis_line(void)
{
  const DBUS_Data_t *dbus_ptr = DBUS_GetData();
  DBUS_Data_t dbus = {0};
  Chassis_Ctrl_Cmd_s chassis = {0};

  if (dbus_ptr != NULL)
  {
    dbus = *dbus_ptr;
  }
  (void)ChassisCAN_GetCommand(&chassis);

  return snprintf(
      (char *)debug_buffer,
      sizeof(debug_buffer),
      "RC_CHASSIS,T=%lu,F=%lu,VF=%lu,IF=%lu,ON=%u,LV=%u,"
      "R0=%u,R1=%u,R2=%u,R3=%u,"
      "C0=%d,C1=%d,C2=%d,C3=%d,S1=%u,S2=%u,"
      "DIAL=%u,DC=%d,DV=%u,"
      "VX=%ld,VY=%ld,WZ=%ld,OFF=%ld,M=%u,CE=%u,TXE=%lu\r\n",
      (unsigned long)HAL_GetTick(),
      (unsigned long)dbus.frame_count,
      (unsigned long)dbus.valid_frame_count,
      (unsigned long)dbus.invalid_frame_count,
      dbus.online ? 1U : 0U,
      dbus.last_frame_valid ? 1U : 0U,
      (unsigned int)dbus.channel[0],
      (unsigned int)dbus.channel[1],
      (unsigned int)dbus.channel[2],
      (unsigned int)dbus.channel[3],
      (int)dbus.centered_channel[0],
      (int)dbus.centered_channel[1],
      (int)dbus.centered_channel[2],
      (int)dbus.centered_channel[3],
      (unsigned int)dbus.switch_value[0],
      (unsigned int)dbus.switch_value[1],
      (unsigned int)dbus.dial,
      (int)dbus.centered_dial,
      dbus.dial_valid ? 1U : 0U,
      (long)debug_float_to_scaled(chassis.vx, 1000.0f),
      (long)debug_float_to_scaled(chassis.vy, 1000.0f),
      (long)debug_float_to_scaled(chassis.wz, 1000.0f),
      (long)debug_float_to_scaled(
          chassis.offset_angle_rad, 1000.0f),
      (unsigned int)chassis.chassis_mode,
      ChassisCAN_IsEmergencyStopped() ? 1U : 0U,
      (unsigned long)debug_tx_error_count);
}

static int format_feeder_line(void)
{
  FeederMotorDebugData_t feeder_data = {0};
  const uint32_t now = HAL_GetTick();
  const bool initialized =
      FeederMotor_GetDebugData(&feeder_data);
  const uint32_t feedback_age_ms =
      (feeder_data.rx_sequence > 0U)
      ? (uint32_t)(now - feeder_data.last_rx_ms)
      : UINT32_MAX;

  return snprintf(
      (char *)debug_buffer,
      sizeof(debug_buffer),
      "FEEDER,T=%lu,INIT=%u,ON=%u,SEQ=%lu,ST=%u,RC=%u,"
      "ARM=%u,EST=%u,FLT=%u,FR=%u,ANG=%u,RPM=%d,TGT=%ld,"
      "POS=%lld,REF=%lld,PE=%ld,SHOT=%lu,SA=%u,PV=%u,HOLD=%u,"
      "E=%ld,P=%ld,I=%ld,D=%ld,OUT=%ld,CMD=%d,ACT=%d,"
      "ERR=%u,AGE=%lu,TXE=%lu,UTE=%lu\r\n",
      (unsigned long)now,
      initialized ? 1U : 0U,
      feeder_data.online ? 1U : 0U,
      (unsigned long)feeder_data.rx_sequence,
      (unsigned int)feeder_data.state,
      (unsigned int)feeder_data.remote_command,
      feeder_data.armed ? 1U : 0U,
      feeder_data.emergency_stop_latched ? 1U : 0U,
      feeder_data.fault_latched ? 1U : 0U,
      (unsigned int)feeder_data.fault_reason,
      (unsigned int)feeder_data.angle,
      (int)feeder_data.speed_rpm,
      (long)debug_float_to_scaled(
          feeder_data.target_speed_rpm, 100.0f),
      (long long)feeder_data.total_angle_ecd,
      (long long)feeder_data.target_total_angle_ecd,
      (long)feeder_data.position_error_ecd,
      (unsigned long)feeder_data.shot_count,
      feeder_data.single_shot_active ? 1U : 0U,
      feeder_data.single_phase_valid ? 1U : 0U,
      feeder_data.single_holding ? 1U : 0U,
      (long)debug_float_to_scaled(
          feeder_data.speed_error_rpm, 1.0f),
      (long)debug_float_to_scaled(
          feeder_data.pid_p_raw, 1.0f),
      (long)debug_float_to_scaled(
          feeder_data.pid_i_raw, 1.0f),
      (long)debug_float_to_scaled(
          feeder_data.pid_d_raw, 1.0f),
      (long)debug_float_to_scaled(
          feeder_data.pid_output_raw, 1.0f),
      (int)feeder_data.command_current_raw,
      (int)feeder_data.actual_current_raw,
      (unsigned int)feeder_data.error_code,
      (unsigned long)feedback_age_ms,
      (unsigned long)feeder_data.tx_error_count,
      (unsigned long)debug_tx_error_count);
}

static int format_friction_line(void)
{
  DualM3508DebugData_t data = {0};
  const uint32_t now = HAL_GetTick();
  const bool initialized = DualM3508_GetDebugData(&data);
  const uint32_t motor1_age_ms =
      (data.motor[0].rx_sequence > 0U)
      ? (uint32_t)(now - data.motor[0].last_rx_ms)
      : UINT32_MAX;
  const uint32_t motor2_age_ms =
      (data.motor[1].rx_sequence > 0U)
      ? (uint32_t)(now - data.motor[1].last_rx_ms)
      : UINT32_MAX;

  return snprintf(
      (char *)debug_buffer,
      sizeof(debug_buffer),
      "FRICTION,T=%lu,INIT=%u,EN=%u,ST=%u,RDY=%u,EST=%u,"
      "FLT=%u,FR=%u,TGT=%ld,"
      "M1ON=%u,M1ANG=%u,M1RAW=%d,M1RPM=%d,M1E=%ld,"
      "M1CMD=%d,M1ACT=%d,M1TMP=%u,M1AGE=%lu,"
      "M2ON=%u,M2ANG=%u,M2RAW=%d,M2RPM=%d,M2E=%ld,"
      "M2CMD=%d,M2ACT=%d,M2TMP=%u,M2AGE=%lu,"
      "TXE=%lu,UTE=%lu\r\n",
      (unsigned long)now,
      initialized ? 1U : 0U,
      data.enable_requested ? 1U : 0U,
      (unsigned int)data.state,
      data.ready ? 1U : 0U,
      data.emergency_stop_latched ? 1U : 0U,
      data.fault_latched ? 1U : 0U,
      (unsigned int)data.fault_reason,
      (long)debug_float_to_scaled(
          data.target_speed_rpm, 100.0f),
      data.motor[0].online ? 1U : 0U,
      (unsigned int)data.motor[0].angle,
      (int)data.motor[0].raw_speed_rpm,
      (int)data.motor[0].logical_speed_rpm,
      (long)debug_float_to_scaled(
          data.motor[0].speed_error_rpm, 1.0f),
      (int)data.motor[0].command_current_raw,
      (int)data.motor[0].actual_current_raw,
      (unsigned int)data.motor[0].temperature_c,
      (unsigned long)motor1_age_ms,
      data.motor[1].online ? 1U : 0U,
      (unsigned int)data.motor[1].angle,
      (int)data.motor[1].raw_speed_rpm,
      (int)data.motor[1].logical_speed_rpm,
      (long)debug_float_to_scaled(
          data.motor[1].speed_error_rpm, 1.0f),
      (int)data.motor[1].command_current_raw,
      (int)data.motor[1].actual_current_raw,
      (unsigned int)data.motor[1].temperature_c,
      (unsigned long)motor2_age_ms,
      (unsigned long)data.tx_error_count,
      (unsigned long)debug_tx_error_count);
}

static const char *imu_status_text(BMI088_Status_t status)
{
  switch (status)
  {
    case BMI088_STATUS_OK:
      return "WAITING_SAMPLE";
    case BMI088_STATUS_SPI_ERROR:
      return "SPI_ERROR";
    case BMI088_STATUS_ID_MISMATCH:
      return "ID_ERROR";
    case BMI088_STATUS_CONFIG_ERROR:
      return "CONFIG_ERROR";
    case BMI088_STATUS_UNINITIALIZED:
    default:
      return "UNINITIALIZED";
  }
}

static int format_imu_line(void)
{
  const BMI088_Diagnostic_t *diagnostic = BMI088_GetDiagnostic();
  BMI088_Sample_t sample;

  if ((diagnostic != NULL)
      && (diagnostic->status == BMI088_STATUS_OK)
      && BMI088_GetLatestSample(&sample))
  {
    return snprintf(
        (char *)debug_buffer,
        sizeof(debug_buffer),
        "IMU_RAW,T=%lu,N=%lu,AX=%d,AY=%d,AZ=%d,"
        "GX=%d,GY=%d,GZ=%d,TEMP=%ld,ERR=%lu,TXE=%lu\r\n",
        (unsigned long)HAL_GetTick(),
        (unsigned long)sample.sample_count,
        (int)sample.accel_raw[0],
        (int)sample.accel_raw[1],
        (int)sample.accel_raw[2],
        (int)sample.gyro_raw[0],
        (int)sample.gyro_raw[1],
        (int)sample.gyro_raw[2],
        (long)sample.temperature_milli_c,
        (unsigned long)diagnostic->read_error_count,
        (unsigned long)debug_tx_error_count);
  }

  if (diagnostic == NULL)
  {
    return snprintf(
        (char *)debug_buffer,
        sizeof(debug_buffer),
        "IMU,NO_DIAGNOSTIC,TXE=%lu\r\n",
        (unsigned long)debug_tx_error_count);
  }

  if (diagnostic->status == BMI088_STATUS_CONFIG_ERROR)
  {
    const char *device =
        (diagnostic->config_failed_device
         == BMI088_CONFIG_DEVICE_GYRO)
        ? "GYRO"
        : "ACC";

    return snprintf(
        (char *)debug_buffer,
        sizeof(debug_buffer),
        "IMU,%s,ACC=0x%02X,GYRO=0x%02X,"
        "CFG=%s:0x%02X,W=0x%02X,R=0x%02X,ERR=%lu,TXE=%lu\r\n",
        imu_status_text(diagnostic->status),
        (unsigned int)diagnostic->accel_chip_id,
        (unsigned int)diagnostic->gyro_chip_id,
        device,
        (unsigned int)diagnostic->config_failed_register,
        (unsigned int)diagnostic->config_written_value,
        (unsigned int)diagnostic->config_readback_value,
        (unsigned long)diagnostic->read_error_count,
        (unsigned long)debug_tx_error_count);
  }

  return snprintf(
      (char *)debug_buffer,
      sizeof(debug_buffer),
      "IMU,%s,ACC=0x%02X,GYRO=0x%02X,ERR=%lu,TXE=%lu\r\n",
      imu_status_text(diagnostic->status),
      (unsigned int)diagnostic->accel_chip_id,
      (unsigned int)diagnostic->gyro_chip_id,
      (unsigned long)diagnostic->read_error_count,
      (unsigned long)debug_tx_error_count);
}

static int format_pitch_fusion_line(void)
{
  const PitchFusionData_t *fusion_ptr = PitchFusion_GetData();
  PitchFusionData_t fusion = {0};
  const GM6020_Feedback_t *pitch_feedback =
      GM6020_GetFeedback(GM6020_AXIS_PITCH);
  const GM6020_SpeedDebugData_t pitch_control =
      GM6020_GetSpeedDebugData(GM6020_AXIS_PITCH);

  if (fusion_ptr != NULL)
  {
    fusion = *fusion_ptr;
  }

  return snprintf(
      (char *)debug_buffer,
      sizeof(debug_buffer),
      "PITCH_FUSION,T=%lu,S=%u,SC=%lu,FC=%lu,CAL=%u,"
      "AR=%ld,AA=%ld,G=%ld,GB=%ld,N=%ld,"
      "E=%ld,ER=%ld,F=%ld,FR=%ld,B=%ld,I=%ld,"
      "TC=%d,CMD=%d,PID=%ld,FF=%ld,R=%u,"
      "H=%u%u%u,RP=%u,TXE=%lu\r\n",
      (unsigned long)HAL_GetTick(),
      (unsigned int)fusion.status,
      (unsigned long)fusion.imu_sample_count,
      (unsigned long)fusion.fusion_update_count,
      (unsigned int)fusion.calibration_sample_count,
      (long)debug_float_to_scaled(
          fusion.accel_pitch_raw_deg, 100.0f),
      (long)debug_float_to_scaled(
          fusion.accel_pitch_aligned_deg, 100.0f),
      (long)debug_float_to_scaled(
          fusion.gyro_pitch_rate_dps, 100.0f),
      (long)debug_float_to_scaled(
          fusion.gyro_bias_dps, 100.0f),
      (long)debug_float_to_scaled(
          fusion.accel_norm_g, 1000.0f),
      (long)debug_float_to_scaled(
          fusion.encoder_pitch_deg, 100.0f),
      (long)debug_float_to_scaled(
          fusion.encoder_pitch_rate_dps, 100.0f),
      (long)debug_float_to_scaled(
          fusion.fused_pitch_deg, 100.0f),
      (long)debug_float_to_scaled(
          fusion.fused_pitch_rate_dps, 100.0f),
      (long)debug_float_to_scaled(
          fusion.base_disturbance_rate_dps, 100.0f),
      (long)debug_float_to_scaled(
          fusion.accel_innovation_deg, 100.0f),
      (pitch_feedback != NULL)
          ? (int)pitch_feedback->torque_current
          : 0,
      (int)pitch_control.command_current,
      (long)debug_float_to_scaled(
          pitch_control.output_current, 1.0f),
      (long)debug_float_to_scaled(
          pitch_control.current_feedforward, 1.0f),
      (unsigned int)fusion.recovery_sample_count,
      fusion.imu_valid ? 1U : 0U,
      fusion.motor_valid ? 1U : 0U,
      fusion.accel_trusted ? 1U : 0U,
      fusion.recovery_pending ? 1U : 0U,
      (unsigned long)debug_tx_error_count);
}

HAL_StatusTypeDef UART_Debug_Init(UART_HandleTypeDef *uart)
{
  if ((uart == NULL)
      || (uart->Instance != USART6)
      || (uart->hdmatx == NULL)
      || (uart->gState == HAL_UART_STATE_RESET))
  {
    return HAL_ERROR;
  }

  debug_uart = uart;
  debug_phase = 0U;
  debug_tx_error_count = 0U;
  for (uint32_t axis = 0U;
       axis < (uint32_t)GM6020_AXIS_COUNT;
       ++axis)
  {
    position_reference_tracker[axis].position_deg = 0.0f;
    position_reference_tracker[axis].last_update_ms = 0U;
    position_reference_tracker[axis].previous_mode =
        GM6020_MODE_POSITION;
    position_reference_tracker[axis].initialized = false;
  }
  return HAL_OK;
}

void UART_Debug_Process(void)
{
  int length;

  if (debug_uart == NULL)
  {
    return;
  }

  /*
   * DMA仍在使用debug_buffer时绝不重新格式化。HAL在DMA搬运结束后还会
   * 经USART6 TC中断把gState恢复为READY，因此这里只需检查发送状态。
   */
  if (debug_uart->gState != HAL_UART_STATE_READY)
  {
    return;
  }

  switch (debug_phase)
  {
    case 0U:
    case 2U:
    case 4U:
    case 6U:
    case 8U:
    case 10U:
    case 12U:
      length = format_position_line();
      break;
    case 1U:
    case 7U:
      length = format_gimbal_line();
      break;
    case 3U:
      length = format_imu_line();
      break;
    case 5U:
      length = format_pitch_fusion_line();
      break;
    case 9U:
      length = format_rc_chassis_line();
      break;
    case 11U:
      length = format_feeder_line();
      break;
    case 13U:
      length = format_friction_line();
      break;
    default:
      length = format_position_line();
      break;
  }
  if ((length <= 0)
      || ((uint32_t)length >= sizeof(debug_buffer)))
  {
    ++debug_tx_error_count;
    return;
  }

  if (HAL_UART_Transmit_DMA(
          debug_uart,
          debug_buffer,
          (uint16_t)length) != HAL_OK)
  {
    ++debug_tx_error_count;
    return;
  }

  debug_phase = (uint8_t)(
      (debug_phase + 1U) % UART_DEBUG_PHASE_COUNT);
}
