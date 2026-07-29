/**
 * ===========================================================================
 * @file    bmi088.h
 * @brief   BMI088 6轴 IMU (加速度计+陀螺仪) SPI 驱动 — 对外 API 与数据类型
 * ===========================================================================
 *
 * 【BMI088 简介】
 *   Bosch BMI088 是一款高性能 6 轴惯性测量单元 (IMU)：
 *   - 加速度计 (Accelerometer): ±3/±6/±12/±24 g, 16-bit
 *   - 陀螺仪 (Gyroscope): ±125/±250/±500/±1000/±2000 dps, 16-bit
 *   - 片上温度传感器 (On-chip temperature sensor)
 *   - SPI 接口（也支持 I2C，本项目使用 SPI 以获更高带宽）
 *
 * 【SPI 双设备架构 (Dual-Device SPI)】
 *   BMI088 内部加速度计和陀螺仪是两个独立的 SPI 从设备，
 *   共享 SPI 总线但使用不同的片选 (CS) 引脚：
 *     - Accel CS: BMI088_ACCEL_CS (由 CubeMX GPIO 宏定义)
 *     - Gyro  CS: BMI088_GYRO_CS  (由 CubeMX GPIO 宏定义)
 *     - SPI MOSI/MISO/SCK: 共享 SPI1
 *   加速度计上电默认 I2C 模式，需要一次假读操作使其切换到 SPI 模式。
 *
 * 【本驱动配置】
 *   - 加速度计: ±6 g, 800 Hz ODR, Normal bandwidth
 *   - 陀螺仪:   ±1000 dps, 1000 Hz ODR / 116 Hz bandwidth
 *   - 温度:     每 1000 次采样读取一次温度 (节约 SPI 带宽)
 *
 * 【数据一致性 (Data Consistency)】
 *   使用序列锁 (seqlock / sequence lock) 机制：
 *   写入时 sequence 先增为奇数 → 写数据 → 再增为偶数
 *   读取时检查 sequence 奇偶和前后一致性，获取不撕裂的快照 (tear-free snapshot)
 *   无需互斥锁，适合裸机 / RTOS 单写多读场景。
 *
 * 【依赖】
 *   SPI1 (spi.h/spi.c) — STM32CubeMX 生成的 SPI 硬件初始化
 *   GPIO 片选引脚 — 由 CubeMX 标签自动生成 BMI088_ACCEL_CS / BMI088_GYRO_CS 宏
 * ===========================================================================
 */

#ifndef BMI088_H
#define BMI088_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 芯片 ID (Chip ID) */
#define BMI088_ACCEL_EXPECTED_CHIP_ID  0x1EU  /* 加速度计预期芯片ID — expected accelerometer chip ID */
#define BMI088_GYRO_EXPECTED_CHIP_ID   0x0FU  /* 陀螺仪预期芯片ID — expected gyroscope chip ID */

/*
 * BMI088 驱动状态枚举 (Driver Status Enum)。
 */
typedef enum
{
  BMI088_STATUS_UNINITIALIZED = 0,  /* 未初始化 — not initialized */
  BMI088_STATUS_OK,                 /* 正常 — OK, fully operational */
  BMI088_STATUS_SPI_ERROR,          /* SPI 通信错误 — SPI transfer failed */
  BMI088_STATUS_ID_MISMATCH,        /* 芯片 ID 不匹配 — wrong chip ID read */
  BMI088_STATUS_CONFIG_ERROR        /* 寄存器配置验证失败 — config write+readback mismatch */
} BMI088_Status_t;

/*
 * 配置失败的设备类型 (Config Failed Device Type)。
 * 用于诊断哪个子设备 (加速度计/陀螺仪) 的寄存器配置失败。
 */
typedef enum
{
  BMI088_CONFIG_DEVICE_NONE = 0,  /* 无失败 — no failure */
  BMI088_CONFIG_DEVICE_ACCEL,     /* 加速度计 — accelerometer */
  BMI088_CONFIG_DEVICE_GYRO       /* 陀螺仪 — gyroscope */
} BMI088_ConfigDevice_t;

/*
 * BMI088 诊断数据结构 (Diagnostic Data)。
 * 保存初始化过程中的所有自检结果，可供 USB 监视模块上报。
 */
typedef struct
{
  BMI088_Status_t status;                    /* 当前驱动状态 — current driver status */
  uint8_t accel_chip_id;                     /* 加速度计实际读到的芯片ID — actual accel chip ID */
  uint8_t gyro_chip_id;                      /* 陀螺仪实际读到的芯片ID — actual gyro chip ID */
  bool accel_transfer_ok;                    /* 加速度计 SPI 通信成功 — accel SPI transfer OK */
  bool gyro_transfer_ok;                     /* 陀螺仪 SPI 通信成功 — gyro SPI transfer OK */
  bool configuration_ok;                     /* 寄存器配置验证全部通过 — all config verified */
  BMI088_ConfigDevice_t config_failed_device; /* 配置失败设备类型 — which device failed config */
  uint8_t config_failed_register;            /* 配置失败寄存器地址 — failed register address */
  uint8_t config_written_value;              /* 配置写入值 — value written */
  uint8_t config_readback_value;             /* 配置回读值 — value read back */
  uint32_t read_error_count;                 /* 运行时读取错误累计计数 — cumulative read errors */
} BMI088_Diagnostic_t;

/*
 * BMI088 采样数据结构 (Sample Data)。
 * 一次完整的 6 轴 + 温度采样。
 */
typedef struct
{
  int16_t accel_raw[3];        /* 加速度计原始值 X/Y/Z (16-bit 有符号) — raw accelerometer */
  int16_t gyro_raw[3];         /* 陀螺仪原始值 X/Y/Z (16-bit 有符号) — raw gyroscope */
  int32_t temperature_milli_c;  /* 温度 (毫摄氏度) — temperature in millidegrees Celsius */
  uint32_t sample_count;       /* 采样序号 (单调递增) — monotonic sample counter */
  uint32_t timestamp_ms;       /* 采样时间戳 (HAL_GetTick) — sample timestamp (ms) */
  bool valid;                  /* 数据有效标志 — sample data is valid */
} BMI088_Sample_t;

/*
 * 初始化 SPI 通信、校验 CHIP_ID 并配置寄存器 (Initialize BMI088)。
 *
 * 执行流程：
 *   1. 清零诊断和采样数据结构
 *   2. 校验 SPI 句柄为 SPI1
 *   3. 拉高两个 CS 引脚
 *   4. 等待陀螺仪上电稳定 (30 ms)
 *   5. 发送假读 (dummy read) 将加速度计从 I2C 切换到 SPI 模式
 *   6. 读取两个子设备的 CHIP_ID 并校验
 *   7. 按顺序配置加速度计电源→使能→ODR/带宽→量程
 *   8. 配置陀螺仪量程→带宽
 *   每次配置后立即回读验证 (write-and-verify)
 *
 * 配置参数：
 *   Accel: +/-6 g, 800 Hz ODR, Normal bandwidth
 *   Gyro : +/-1000 dps, 1000 Hz ODR / 116 Hz bandwidth
 */
BMI088_Status_t BMI088_Init(SPI_HandleTypeDef *spi);

/*
 * 读取一次完整的 6 轴 + 温度采样 (Read one full IMU sample)。
 * 由唯一的 IMU 采集任务每 1 ms 调用一次，与 BMI088 的 ~1 kHz ODR 匹配。
 * 使用序列锁发布到 latest_sample，消费者通过 BMI088_GetLatestSample() 读取。
 * @return true 采样成功并已发布
 */
bool BMI088_ReadSample(void);

/*
 * 获取一致的最新数据快照 (Get consistent latest sample snapshot)。
 * 使用序列锁机制确保读取过程中数据不被写者撕裂。
 * 最多重试 3 次，3 次都失败则返回 false。
 * @param sample 输出参数，接收样本副本
 * @return true  成功获取有效快照
 * @return false 尚无有效样本或序列锁一直冲突
 */
bool BMI088_GetLatestSample(BMI088_Sample_t *sample);

/* 获取只读诊断数据指针 — get read-only diagnostic data */
const BMI088_Diagnostic_t *BMI088_GetDiagnostic(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
