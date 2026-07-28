/**
 * ===========================================================================
 * @file    dbus_monitor.h
 * @brief   DBUS 数据 USB CDC 监视输出
 * ===========================================================================
 */

#ifndef DBUS_MONITOR_H
#define DBUS_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 主循环周期调用。以 20 Hz 通过 USB CDC 输出最近的 DBUS 解析结果；
 * USB 忙时不阻塞主循环，等待下一轮重试。
 */
void DBUS_Monitor_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* DBUS_MONITOR_H */
