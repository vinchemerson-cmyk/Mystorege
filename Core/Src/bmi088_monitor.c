/**
 * ===========================================================================
 * @file    bmi088_monitor.c
 * @brief   BMI088 IMU 数据 USB CDC 监视输出
 * ===========================================================================
 *
 * 通过 USB CDC 虚拟串口周期输出 BMI088 6 轴原始数据或错误诊断信息。
 *
 * 【输出格式】
 *   正常模式 (BMI088_STATUS_OK)，两种行交替输出:
 *     IMU_RAW,<sample_count>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp_milli_c>,<read_errors>\r\n
 *     PITCH_FUSION,S=<state>,CAL=<count>,AR=<cdeg>,AA=<cdeg>,G=<cdps>,
 *       E=<cdeg>,ER=<cdps>,F=<cdeg>,FR=<cdps>,B=<cdps>,I=<cdeg>,
 *       R=<count>,H=<imu><motor><accel>\r\n
 *     - 总发送周期: 20 ms；两种行各约 25 Hz
 *     - accel_raw: 16-bit 原始值, gyro_raw: 16-bit 原始值
 *     - temperature_milli_c: 毫摄氏度 (°C × 1000)
 *     - PITCH_FUSION 角度/角速度均放大100倍后以整数输出
 *     - I 为加速度角相对陀螺预测角的创新量，R 为掉线恢复健康样本数
 *     - H 三位依次表示 IMU有效、电机有效、加速度可信
 *
 *   错误模式 (非 OK 状态):
 *     IMU,<status>,ACC=0x<id>,GYRO=0x<id>,ERR=<count>\r\n
 *     或含配置失败详情: IMU,<status>,ACC=0x<id>,GYRO=0x<id>,CFG=<device>:0x<reg>,W=0x<val>,R=0x<val>,ERR=<count>\r\n
 *     - 周期: 1 Hz (BMI088_ERROR_MONITOR_PERIOD_MS = 1000 ms)
 *     - device: "ACC" 或 "GYRO"
 *     - 显示哪个寄存器写入失败、写入值和回读值
 *
 * 【异步发送 (Asynchronous Send)】
 *   CDC_Transmit_FS() 异步使用传入缓冲区。使用 monitor_pending 标志
 *   跟踪发送状态：USBD_BUSY 时保留本帧，下一轮重试直到发送成功。
 * ===========================================================================
 */

#include "bmi088_monitor.h"

#include "bmi088.h"
#include "pitch_fusion.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#include <stdbool.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>

#define BMI088_RAW_MONITOR_PERIOD_MS       20U   /* 正常数据上报周期 — raw data report period (20 Hz) */
#define BMI088_ERROR_MONITOR_PERIOD_MS   1000U   /* 错误诊断上报周期 — error report period (1 Hz) */
#define BMI088_MONITOR_BUFFER_CAPACITY    256U   /* 输出缓冲区容量 — output buffer capacity */

/* USB CDC 异步发送缓冲区 (Asynchronous send buffer) */
static uint8_t monitor_buffer[BMI088_MONITOR_BUFFER_CAPACITY];  /* 格式化输出缓冲区 */
static uint16_t monitor_length;          /* 当前待发送数据长度 — pending transmit length */
static uint32_t last_monitor_tx_ms;      /* 上次发送时间戳 — last transmit timestamp */
static bool monitor_pending;             /* 有待发送数据 — data pending (CDC busy retry) */
static bool report_fusion_next;          /* 正常模式下交替输出原始数据和融合数据 */

static int32_t monitor_float_to_centi(float value)
{
  const float scaled = value * 100.0f;

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

/* ─── 公有 API ─── */

/*
 * 初始化监视器：重置时间戳，使第一次 Process 调用立即上报。
 */
void BMI088_Monitor_Init(void)
{
  monitor_length = 0U;
  last_monitor_tx_ms =
      HAL_GetTick() - BMI088_ERROR_MONITOR_PERIOD_MS;
  monitor_pending = false;
  report_fusion_next = false;
}

void BMI088_Monitor_Process(void)
{
  const uint32_t now = HAL_GetTick();
  const BMI088_Diagnostic_t *diagnostic;
  BMI088_Sample_t sample;
  const char *status_text;
  const char *config_device_text;
  uint32_t period_ms;
  int length;

  if (monitor_pending)
  {
    if (CDC_Transmit_FS(
            monitor_buffer,
            monitor_length) == USBD_OK)
    {
      monitor_pending = false;
      last_monitor_tx_ms = now;
    }
    return;
  }

  diagnostic = BMI088_GetDiagnostic();
  if (diagnostic == NULL)
  {
    return;
  }

  period_ms = (diagnostic->status == BMI088_STATUS_OK)
      ? BMI088_RAW_MONITOR_PERIOD_MS
      : BMI088_ERROR_MONITOR_PERIOD_MS;
  if ((uint32_t)(now - last_monitor_tx_ms) < period_ms)
  {
    return;
  }

  if ((diagnostic->status == BMI088_STATUS_OK)
      && BMI088_GetLatestSample(&sample))
  {
    if (report_fusion_next)
    {
      const PitchFusionData_t *fusion =
          PitchFusion_GetData();

      length = snprintf(
          (char *)monitor_buffer,
          sizeof(monitor_buffer),
          "PITCH_FUSION,S=%u,CAL=%u,"
          "AR=%ld,AA=%ld,G=%ld,E=%ld,ER=%ld,"
          "F=%ld,FR=%ld,B=%ld,I=%ld,R=%u,H=%u%u%u\r\n",
          (unsigned int)fusion->status,
          (unsigned int)fusion->calibration_sample_count,
          (long)monitor_float_to_centi(
              fusion->accel_pitch_raw_deg),
          (long)monitor_float_to_centi(
              fusion->accel_pitch_aligned_deg),
          (long)monitor_float_to_centi(
              fusion->gyro_pitch_rate_dps),
          (long)monitor_float_to_centi(
              fusion->encoder_pitch_deg),
          (long)monitor_float_to_centi(
              fusion->encoder_pitch_rate_dps),
          (long)monitor_float_to_centi(
              fusion->fused_pitch_deg),
          (long)monitor_float_to_centi(
              fusion->fused_pitch_rate_dps),
          (long)monitor_float_to_centi(
              fusion->base_disturbance_rate_dps),
          (long)monitor_float_to_centi(
              fusion->accel_innovation_deg),
          (unsigned int)fusion->recovery_sample_count,
          fusion->imu_valid ? 1U : 0U,
          fusion->motor_valid ? 1U : 0U,
          fusion->accel_trusted ? 1U : 0U);
    }
    else
    {
      length = snprintf(
          (char *)monitor_buffer,
          sizeof(monitor_buffer),
          "IMU_RAW,%lu,%d,%d,%d,%d,%d,%d,%ld,%lu\r\n",
          (unsigned long)sample.sample_count,
          (int)sample.accel_raw[0],
          (int)sample.accel_raw[1],
          (int)sample.accel_raw[2],
          (int)sample.gyro_raw[0],
          (int)sample.gyro_raw[1],
          (int)sample.gyro_raw[2],
          (long)sample.temperature_milli_c,
          (unsigned long)diagnostic->read_error_count);
    }
    report_fusion_next = !report_fusion_next;
  }
  else
  {
    switch (diagnostic->status)
    {
      case BMI088_STATUS_OK:
        status_text = "WAITING_SAMPLE";
        break;
      case BMI088_STATUS_SPI_ERROR:
        status_text = "SPI_ERROR";
        break;
      case BMI088_STATUS_ID_MISMATCH:
        status_text = "ID_ERROR";
        break;
      case BMI088_STATUS_CONFIG_ERROR:
        status_text = "CONFIG_ERROR";
        break;
      case BMI088_STATUS_UNINITIALIZED:
      default:
        status_text = "UNINITIALIZED";
        break;
    }

    if (diagnostic->status == BMI088_STATUS_CONFIG_ERROR)
    {
      config_device_text =
          (diagnostic->config_failed_device
           == BMI088_CONFIG_DEVICE_GYRO)
          ? "GYRO"
          : "ACC";
      length = snprintf(
          (char *)monitor_buffer,
          sizeof(monitor_buffer),
          "IMU,%s,ACC=0x%02X,GYRO=0x%02X,"
          "CFG=%s:0x%02X,W=0x%02X,R=0x%02X,ERR=%lu\r\n",
          status_text,
          (unsigned int)diagnostic->accel_chip_id,
          (unsigned int)diagnostic->gyro_chip_id,
          config_device_text,
          (unsigned int)diagnostic->config_failed_register,
          (unsigned int)diagnostic->config_written_value,
          (unsigned int)diagnostic->config_readback_value,
          (unsigned long)diagnostic->read_error_count);
    }
    else
    {
      length = snprintf(
          (char *)monitor_buffer,
          sizeof(monitor_buffer),
          "IMU,%s,ACC=0x%02X,GYRO=0x%02X,ERR=%lu\r\n",
          status_text,
          (unsigned int)diagnostic->accel_chip_id,
          (unsigned int)diagnostic->gyro_chip_id,
          (unsigned long)diagnostic->read_error_count);
    }
  }
  if ((length <= 0)
      || ((uint32_t)length >= sizeof(monitor_buffer)))
  {
    return;
  }

  monitor_length = (uint16_t)length;
  monitor_pending = true;
  if (CDC_Transmit_FS(
          monitor_buffer,
          monitor_length) == USBD_OK)
  {
    monitor_pending = false;
    last_monitor_tx_ms = now;
  }
}
