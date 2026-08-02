/**
 * ===========================================================================
 * @file    bmi088.c
 * @brief   BMI088 6轴 IMU SPI 驱动实现 — 初始化、寄存器配置、突发读取、序列锁发布
 * ===========================================================================
 *
 * 【硬件连接 (Hardware Connection)】
 *   SPI1 共享总线 + 两个独立 CS 引脚:
 *     BMI088 Accel CS  → CubeMX GPIO 标签 BMI088_ACCEL_CS
 *     BMI088 Gyro  CS  → CubeMX GPIO 标签 BMI088_GYRO_CS
 *     SPI1_SCK  = PB3 (AF5)
 *     SPI1_MISO = PB4 (AF5)
 *     SPI1_MOSI = PA7 (AF5)
 *
 * 【SPI 协议要点】
 *   - 加速度计: 读寄存器 = 发送 (reg | 0x80), 返回 dummy + 寄存器值 (3字节)
 *              写寄存器 = 发送 (reg & 0x7F) + value (2字节)
 *   - 陀螺仪:   读寄存器 = 发送 (reg | 0x80), 返回 寄存器值 (2字节)
 *              写寄存器 = 发送 (reg & 0x7F) + value (2字节)
 *   - 加速度计上电默认 I2C 模式，需一次假读 (dummy read) 利用 CS↑ 沿切换到 SPI
 *   - BMI088 陀螺仪从上电到可通信需 ≥30 ms
 *
 * 【初始化流程 (Initialization Sequence)】
 *   1. 拉高双 CS → 等待 30ms (陀螺仪启动)
 *   2. 假读加速度计 → 等待 1ms → I2C→SPI 模式切换完成
 *   3. 读取双 CHIP_ID (Accel=0x1E, Gyro=0x0F) 校验
 *   4. 加速度计: 上电配置 → 使能 → ODR/BWP → 量程 (每步写+回读验证)
 *   5. 陀螺仪:   量程 → 带宽 (每步写+回读验证)
 *
 * 【数据采集流程 (Sampling Sequence)】
 *   1. 突发读加速度计 8 字节 (DATA[0x12~0x17])
 *   2. 突发读陀螺仪 7 字节 (DATA[0x02~0x08])
 *   3. 每 1000 次采样读取一次温度 (0.125°C/LSB, 23°C 偏置)
 *   4. 序列锁发布到 latest_sample (写者递增奇数→写→递增偶数)
 *
 * 【序列锁 (SeqLock / Sequence Lock)】
 *   写者 (BMI088_ReadSample):
 *     ++sequence (→奇数) → __DMB() → 写 latest_sample → __DMB() → ++sequence (→偶数)
 *   读者 (BMI088_GetLatestSample):
 *     begin=sequence → __DMB() → 读 latest_sample → __DMB() → end=sequence
 *     条件: begin==end 且均为偶数 → 快照有效 (未被写者撕裂)
 *   最多重试 3 次。适合单写者多读者场景，无需临界区/互斥锁。
 *
 * 【寄存器映射 (Register Map)】
 *   加速度计:
 *     0x00: CHIP_ID (R)     0x12: ACC_X_LSB~ACC_Z_MSB (R)
 *     0x22: TEMP_MSB~LSB (R) 0x40: ACC_CONF (R/W) — ODR/BWP
 *     0x41: ACC_RANGE (R/W)  0x7C: ACC_PWR_CONF (R/W)
 *     0x7D: ACC_PWR_CTRL (R/W)
 *   陀螺仪:
 *     0x00: CHIP_ID (R)      0x02: RATE_X_LSB~RATE_Z_MSB (R)
 *     0x0F: RANGE (R/W)      0x10: BANDWIDTH (R/W)
 *
 * 【依赖】
 *   bmi088.h  — 对外 API、状态/诊断/采样数据类型
 *   spi.h     — SPI1 硬件初始化 (CubeMX)
 *   HAL_SPI   — STM32 HAL SPI 驱动
 * ===========================================================================
 */

#include "bmi088.h"

#include "main.h"

#include <string.h>

/* ─── BMI088 寄存器地址 (Register Addresses) ─── */
#define BMI088_CHIP_ID_REG            0x00U   /* CHIP_ID 寄存器 (加速度计和陀螺仪共用地址) */
#define BMI088_SPI_READ_BIT           0x80U   /* SPI 读标志位 — set bit 7 for read */
#define BMI088_SPI_TIMEOUT_MS         10U     /* SPI 传输超时 (ms) — SPI transfer timeout */
#define BMI088_GYRO_STARTUP_DELAY_MS  30U     /* 陀螺仪上电稳定时间 — gyro power-up delay */
#define BMI088_SPI_SWITCH_DELAY_MS     1U     /* I2C→SPI 切换后等待 — accel I2C→SPI switch delay */
#define BMI088_REGISTER_WRITE_DELAY_MS 1U     /* 寄存器写入后稳定时间 — post-write settling delay */
#define BMI088_POWER_CONFIG_DELAY_MS   5U     /* 电源模式切换等待 — power mode change delay */

/* ─── 加速度计寄存器 (Accelerometer Registers) ─── */
#define BMI088_ACCEL_DATA_REG          0x12U  /* 加速度数据起始 (ACC_X_LSB) — data burst start */
#define BMI088_ACCEL_TEMP_REG          0x22U  /* 温度数据 — temperature MSB+LSB */
#define BMI088_ACCEL_CONF_REG          0x40U  /* 配置: ODR + Bandwidth */
#define BMI088_ACCEL_RANGE_REG         0x41U  /* 配置: 量程 ±3/6/12/24 g */
#define BMI088_ACCEL_PWR_CONF_REG      0x7CU  /* 电源配置: active/suspend mode */
#define BMI088_ACCEL_PWR_CTRL_REG      0x7DU  /* 电源控制: accel enable/disable */

/* ─── 陀螺仪寄存器 (Gyroscope Registers) ─── */
#define BMI088_GYRO_DATA_REG           0x02U  /* 角速率数据起始 (RATE_X_LSB) — data burst start */
#define BMI088_GYRO_RANGE_REG          0x0FU  /* 配置: 量程 ±125/250/500/1000/2000 dps */
#define BMI088_GYRO_BANDWIDTH_REG      0x10U  /* 配置: ODR + Filter Bandwidth */

/* ─── 寄存器预设值 (Register Preset Values) ─── */
/* 加速度计: ±6g, 800Hz ODR, Normal bandwidth */
#define BMI088_ACCEL_NORMAL_800HZ      0xABU  /* ODR=800Hz, BWP=Normal */
#define BMI088_ACCEL_RANGE_6G          0x01U  /* ±6g, 16-bit 分辨率 */
#define BMI088_ACCEL_ACTIVE_MODE       0x00U  /* Active mode (非 Suspend) */
#define BMI088_ACCEL_ENABLE            0x04U  /* 使能加速度计 — accel enable bit */

/* 陀螺仪: ±1000dps, 1000Hz ODR / 116Hz filter bandwidth */
#define BMI088_GYRO_RANGE_1000_DPS     0x01U  /* ±1000 dps */
#define BMI088_GYRO_1000HZ_116HZ       0x02U  /* ODR=1000Hz, Filter BW=116Hz */

/* ─── 数据采集参数 ─── */
/* 加速度计返回 dummy+6字节数据=8字节帧，陀螺仪返回 1+6=7字节帧 */
#define BMI088_ACCEL_BURST_LENGTH      8U     /* 加速度计突发读取总长度 (1 addr + 1 dummy + 6 data) */
#define BMI088_GYRO_BURST_LENGTH       7U     /* 陀螺仪突发读取总长度 (1 addr + 6 data) */
#define BMI088_TEMP_READ_DIVIDER    1000U    /* 温度读取分频: 每 1000 次采样读 1 次温度 */

/*
 * ─── 模块级静态变量 (Module Static Variables) ───
 */
static SPI_HandleTypeDef *bmi088_spi;          /* SPI1 句柄 — SPI handle for BMI088 */
static BMI088_Diagnostic_t diagnostic;          /* 诊断数据 — diagnostic info (read by monitor) */
static BMI088_Sample_t latest_sample;           /* 最新采样数据 — latest sample (seqlock protected) */
static volatile uint32_t sample_sequence;       /* 序列锁序列号 — seqlock sequence (odd=writing, even=stable) */
static int32_t latest_temperature_milli_c;     /* 最新温度 (毫°C) — latest temperature in millidegrees C */
static bool temperature_valid;                  /* 温度有效标志 — temperature has been read at least once */

/* ─── 内部工具函数 (Internal Utility Functions) ─── */

/*
 * 记录首次寄存器配置失败的信息到诊断结构体。
 * 只在尚未记录过失败 (config_failed_device == NONE) 时才写入，
 * 确保诊断输出反映的是初始化流程中的第一个失败点。
 */
static void bmi088_record_config_failure(
    BMI088_ConfigDevice_t device,
    uint8_t reg,
    uint8_t written_value,
    uint8_t readback_value)
{
  if (diagnostic.config_failed_device
      == BMI088_CONFIG_DEVICE_NONE)
  {
    diagnostic.config_failed_device = device;
    diagnostic.config_failed_register = reg;
    diagnostic.config_written_value = written_value;
    diagnostic.config_readback_value = readback_value;
  }
}

static HAL_StatusTypeDef bmi088_accel_read_register(
    uint8_t reg,
    uint8_t *value)
{
  uint8_t tx[3] = {
      (uint8_t)(reg | BMI088_SPI_READ_BIT),
      0U,
      0U
  };
  uint8_t rx[3] = {0U, 0U, 0U};
      /* 加速度计读操作的第2个字节是dummy，真正数据在rx[2]。 */
  HAL_StatusTypeDef status;

  if ((bmi088_spi == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(
      bmi088_spi,
      tx,
      rx,
      (uint16_t)sizeof(tx),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    /*
     * BMI088加速度计读操作会先返回一个dummy byte，
     * 实际寄存器值位于地址字节之后的第二个接收字节。
     */
    *value = rx[2];
  }
  return status;
}

static HAL_StatusTypeDef bmi088_gyro_read_register(
    uint8_t reg,
    uint8_t *value)
{
  uint8_t tx[2] = {
      (uint8_t)(reg | BMI088_SPI_READ_BIT),
      0U
  };
  uint8_t rx[2] = {0U, 0U};
      /* 陀螺仪没有加速度计那样的额外dummy字节，数据在rx[1]。 */
  HAL_StatusTypeDef status;

  if ((bmi088_spi == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(
      bmi088_spi,
      tx,
      rx,
      (uint16_t)sizeof(tx),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    *value = rx[1];
  }
  return status;
}

static HAL_StatusTypeDef bmi088_accel_write_register(
    uint8_t reg,
    uint8_t value)
{
  uint8_t tx[2] = {
      (uint8_t)(reg & (uint8_t)~BMI088_SPI_READ_BIT),
      value
  };
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(
      bmi088_spi,
      tx,
      (uint16_t)sizeof(tx),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_SET);
  HAL_Delay(BMI088_REGISTER_WRITE_DELAY_MS);
  return status;
}

static HAL_StatusTypeDef bmi088_gyro_write_register(
    uint8_t reg,
    uint8_t value)
{
  uint8_t tx[2] = {
      (uint8_t)(reg & (uint8_t)~BMI088_SPI_READ_BIT),
      value
  };
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(
      bmi088_spi,
      tx,
      (uint16_t)sizeof(tx),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_SET);
  HAL_Delay(BMI088_REGISTER_WRITE_DELAY_MS);
  return status;
}

static bool bmi088_accel_write_and_verify(
/* SPI Write → Read-back → Verify (masked compare). Records failure on mismatch. */
    uint8_t reg,
    uint8_t value,
    uint8_t verify_mask)
{
  uint8_t readback = 0U;
  HAL_StatusTypeDef write_status;
  HAL_StatusTypeDef read_status = HAL_ERROR;

  write_status = bmi088_accel_write_register(reg, value);
  if (write_status == HAL_OK)
  {
    read_status = bmi088_accel_read_register(reg, &readback);
  }

  if ((write_status == HAL_OK)
      && (read_status == HAL_OK)
      && ((readback & verify_mask) == (value & verify_mask)))
  {
    return true;
  }

  bmi088_record_config_failure(
      BMI088_CONFIG_DEVICE_ACCEL,
      reg,
      value,
      readback);
  return false;
}

/* Same as accel version; records failure to diagnostic on mismatch. */
static bool bmi088_gyro_write_and_verify(
    uint8_t reg,
    uint8_t value,
    uint8_t verify_mask)
{
  uint8_t readback = 0U;
  HAL_StatusTypeDef write_status;
  HAL_StatusTypeDef read_status = HAL_ERROR;

  write_status = bmi088_gyro_write_register(reg, value);
  if (write_status == HAL_OK)
  {
    read_status = bmi088_gyro_read_register(reg, &readback);
  }

  if ((write_status == HAL_OK)
      && (read_status == HAL_OK)
      && ((readback & verify_mask) == (value & verify_mask)))
  {
    return true;
  }

  bmi088_record_config_failure(
      BMI088_CONFIG_DEVICE_GYRO,
      reg,
      value,
      readback);
  return false;
}
/* Burst-read from accelerometer: addr|0x80 → receive (dummy + data), skip first 2 rx bytes */

static HAL_StatusTypeDef bmi088_accel_read_burst(
    uint8_t start_reg,
    uint8_t *data,
    uint16_t length)
{
  uint8_t tx[BMI088_ACCEL_BURST_LENGTH] = {0U};
  uint8_t rx[BMI088_ACCEL_BURST_LENGTH] = {0U};
      /* 本项目最大读取6字节数据，前面保留地址和dummy两个字节。 */
  HAL_StatusTypeDef status;

  if ((data == NULL)
      || (length > (BMI088_ACCEL_BURST_LENGTH - 2U)))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(start_reg | BMI088_SPI_READ_BIT);
  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(
      bmi088_spi,
      tx,
      rx,
      (uint16_t)(length + 2U),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    memcpy(data, &rx[2], length);
  }
  return status;
/* Burst-read from gyroscope: addr|0x80 → receive data, skip first 1 rx byte (no dummy) */
}

static HAL_StatusTypeDef bmi088_gyro_read_burst(
    uint8_t start_reg,
    uint8_t *data,
    uint16_t length)
{
  uint8_t tx[BMI088_GYRO_BURST_LENGTH] = {0U};
  uint8_t rx[BMI088_GYRO_BURST_LENGTH] = {0U};
      /* 陀螺仪突发读只保留地址字节，后面是6字节XYZ数据。 */
  HAL_StatusTypeDef status;

  if ((data == NULL)
      || (length > (BMI088_GYRO_BURST_LENGTH - 1U)))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(start_reg | BMI088_SPI_READ_BIT);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(
      bmi088_spi,
      tx,
      rx,
      (uint16_t)(length + 1U),
      BMI088_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    memcpy(data, &rx[1], length);
  }
  return status;
}

static int16_t bmi088_decode_le_i16(
/* Decode two bytes (LSB first) as signed 16-bit integer — Little-Endian int16_t */
    uint8_t lsb,
    uint8_t msb)
{
  return (int16_t)(
      (uint16_t)lsb | ((uint16_t)msb << 8U));
}

/* Read temperature via accelerometer burst (0x22 TEMP_MSB+LSB). Resolution: 0.125°C/LSB, offset: 23°C */
static HAL_StatusTypeDef bmi088_read_temperature(void)
{
  uint8_t data[2];
  uint16_t raw_unsigned;
  int16_t raw_signed;
  HAL_StatusTypeDef status;

  status = bmi088_accel_read_burst(
      BMI088_ACCEL_TEMP_REG,
      data,
      (uint16_t)sizeof(data));
  if (status != HAL_OK)
  {
    return status;
  }

  raw_unsigned =
      ((uint16_t)data[0] << 3U)
      | ((uint16_t)data[1] >> 5U);
  raw_signed = (raw_unsigned > 1023U)
      ? (int16_t)((int32_t)raw_unsigned - 2048)
      : (int16_t)raw_unsigned;

  /* 分辨率0.125°C/LSB，零点偏置23°C。 */
  latest_temperature_milli_c =
      23000 + ((int32_t)raw_signed * 125);
  temperature_valid = true;
  return HAL_OK;
}
/* ── BMI088 初始化主函数 ── */

BMI088_Status_t BMI088_Init(SPI_HandleTypeDef *spi)
{
  uint8_t dummy = 0U;
  HAL_StatusTypeDef accel_status;
  HAL_StatusTypeDef gyro_status;

  memset(&diagnostic, 0, sizeof(diagnostic));
  memset(&latest_sample, 0, sizeof(latest_sample));
  sample_sequence = 0U;
  latest_temperature_milli_c = 0;
  temperature_valid = false;
  bmi088_spi = spi;

  if ((bmi088_spi == NULL) || (bmi088_spi->Instance != SPI1))
  {
    diagnostic.status = BMI088_STATUS_SPI_ERROR;
    return diagnostic.status;
  }

  HAL_GPIO_WritePin(
      BMI088_ACCEL_CS_GPIO_Port,
      BMI088_ACCEL_CS_Pin,
      GPIO_PIN_SET);
  HAL_GPIO_WritePin(
      BMI088_GYRO_CS_GPIO_Port,
      BMI088_GYRO_CS_Pin,
      GPIO_PIN_SET);
  /*
   * 陀螺仪从上电到可通信最长需要30 ms。这里从进入初始化函数开始
   * 完整等待一次，不依赖MCU此前外设初始化已经消耗的时间。
   */
  HAL_Delay(BMI088_GYRO_STARTUP_DELAY_MS);

  /*
   * 加速度计上电默认进入I2C接口状态。第一次无效SPI读取提供
   * CS上升沿，使其切换到SPI模式；该次返回值按数据手册要求丢弃。
   */
  (void)bmi088_accel_read_register(BMI088_CHIP_ID_REG, &dummy);
  HAL_Delay(BMI088_SPI_SWITCH_DELAY_MS);

  accel_status = bmi088_accel_read_register(
      BMI088_CHIP_ID_REG,
      &diagnostic.accel_chip_id);
  gyro_status = bmi088_gyro_read_register(
      BMI088_CHIP_ID_REG,
      &diagnostic.gyro_chip_id);

  diagnostic.accel_transfer_ok = (accel_status == HAL_OK);
  diagnostic.gyro_transfer_ok = (gyro_status == HAL_OK);

  if (!diagnostic.accel_transfer_ok
      || !diagnostic.gyro_transfer_ok)
  {
    diagnostic.status = BMI088_STATUS_SPI_ERROR;
  }
  else if ((diagnostic.accel_chip_id
            != BMI088_ACCEL_EXPECTED_CHIP_ID)
           || (diagnostic.gyro_chip_id
               != BMI088_GYRO_EXPECTED_CHIP_ID))
  {
    diagnostic.status = BMI088_STATUS_ID_MISMATCH;
  }
  else
  {
    diagnostic.configuration_ok =
        bmi088_accel_write_and_verify(
            BMI088_ACCEL_PWR_CONF_REG,
            BMI088_ACCEL_ACTIVE_MODE,
            0xFFU);
    HAL_Delay(BMI088_POWER_CONFIG_DELAY_MS);
    diagnostic.configuration_ok =
        diagnostic.configuration_ok
        && bmi088_accel_write_and_verify(
            BMI088_ACCEL_PWR_CTRL_REG,
            BMI088_ACCEL_ENABLE,
            0xFFU);
    HAL_Delay(BMI088_POWER_CONFIG_DELAY_MS);
    diagnostic.configuration_ok =
        diagnostic.configuration_ok
        && bmi088_accel_write_and_verify(
            BMI088_ACCEL_CONF_REG,
            BMI088_ACCEL_NORMAL_800HZ,
            0xFFU)
        && bmi088_accel_write_and_verify(
            BMI088_ACCEL_RANGE_REG,
            BMI088_ACCEL_RANGE_6G,
            0x03U)
        && bmi088_gyro_write_and_verify(
            BMI088_GYRO_RANGE_REG,
            BMI088_GYRO_RANGE_1000_DPS,
            0x07U)
        && bmi088_gyro_write_and_verify(
            BMI088_GYRO_BANDWIDTH_REG,
            BMI088_GYRO_1000HZ_116HZ,
            0x07U);

    diagnostic.status = diagnostic.configuration_ok
        ? BMI088_STATUS_OK
        : BMI088_STATUS_CONFIG_ERROR;
  }

  return diagnostic.status;
/* ── 读取一次完整采样（6轴 + 温度），序列锁发布到 latest_sample ── */
}

bool BMI088_ReadSample(void)
{
  uint8_t accel_data[6];
      /* 加速度 X/Y/Z，各2字节，小端序。 */
  uint8_t gyro_data[6];
      /* 陀螺仪 X/Y/Z，各2字节，小端序。 */
  BMI088_Sample_t sample;
      /* 先在局部组装完整样本，最后一次性发布，避免读者看到半成品。 */

  if (diagnostic.status != BMI088_STATUS_OK)
  {
    return false;
  }

  if ((bmi088_accel_read_burst(
           BMI088_ACCEL_DATA_REG,
           accel_data,
           (uint16_t)sizeof(accel_data)) != HAL_OK)
      || (bmi088_gyro_read_burst(
              BMI088_GYRO_DATA_REG,
              gyro_data,
              (uint16_t)sizeof(gyro_data)) != HAL_OK))
  {
    ++diagnostic.read_error_count;
    return false;
  }

  memset(&sample, 0, sizeof(sample));
  sample.accel_raw[0] =
      bmi088_decode_le_i16(accel_data[0], accel_data[1]);
  sample.accel_raw[1] =
      bmi088_decode_le_i16(accel_data[2], accel_data[3]);
  sample.accel_raw[2] =
      bmi088_decode_le_i16(accel_data[4], accel_data[5]);
  sample.gyro_raw[0] =
      bmi088_decode_le_i16(gyro_data[0], gyro_data[1]);
  sample.gyro_raw[1] =
      bmi088_decode_le_i16(gyro_data[2], gyro_data[3]);
  sample.gyro_raw[2] =
      bmi088_decode_le_i16(gyro_data[4], gyro_data[5]);

  /* 采样序号由驱动维护，不依赖HAL tick，便于判断是否有新样本。 */
  sample.sample_count = latest_sample.sample_count + 1U;
  if (!temperature_valid
      || ((sample.sample_count % BMI088_TEMP_READ_DIVIDER) == 0U))
  {
    if (bmi088_read_temperature() != HAL_OK)
    {
      ++diagnostic.read_error_count;
    }
  }

  sample.temperature_milli_c =
      latest_temperature_milli_c;
  sample.timestamp_ms = HAL_GetTick();
  sample.valid = true;

  /*
   * 序列锁：写入时为奇数，完整发布后变为偶数。
   * USB监视任务可据此获得不撕裂的一致快照，无需互斥锁。
   */
  ++sample_sequence;
  __DMB();
  latest_sample = sample;
  __DMB();
  ++sample_sequence;
/* ── 序列锁读取：获取无撕裂的最新样本快照（最多重试3次）── */
  return true;
}

bool BMI088_GetLatestSample(BMI088_Sample_t *sample)
{
  uint32_t begin_sequence;
  uint32_t end_sequence;
  uint32_t attempt;

  if (sample == NULL)
  {
    return false;
  }

  for (attempt = 0U; attempt < 3U; ++attempt)
  {
    begin_sequence = sample_sequence;
    if ((begin_sequence & 1U) != 0U)
    {
      continue;
    }

    __DMB();
    *sample = latest_sample;
    __DMB();
    end_sequence = sample_sequence;

    if ((begin_sequence == end_sequence)
        && ((end_sequence & 1U) == 0U))
    {
      return sample->valid;
    }
  }

  return false;
}

const BMI088_Diagnostic_t *BMI088_GetDiagnostic(void)
{
  return &diagnostic;
}
