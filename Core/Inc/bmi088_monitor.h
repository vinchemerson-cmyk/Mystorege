/**
 * ===========================================================================
 * @file    bmi088_monitor.h
 * @brief   BMI088 IMU 数据 USB CDC 监视输出 — 对外 API 声明
 * ===========================================================================
 *
 * 通过 USB CDC 虚拟串口周期输出 BMI088 6 轴原始数据或错误诊断信息。
 * 正常模式 (IMU_RAW) 输出: 20 Hz (每 50 ms)
 * 错误模式 (IMU)    输出: 1 Hz  (每 1000 ms)
 *
 * 格式详见 bmi088_monitor.c 文件头注释。
 * ===========================================================================
 */

#ifndef BMI088_MONITOR_H
#define BMI088_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化监视器 — initialize monitor (reset timestamp) */
void BMI088_Monitor_Init(void);

/* 主循环周期调用 — main loop periodic call (内部自节流) */
void BMI088_Monitor_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_MONITOR_H */
