/**
 * ===========================================================================
 * @file    dbus.h
 * @brief   DBUS (DJI 遥控器接收机协议) 接收与解析模块 — 对外 API 声明
 * ===========================================================================
 *
 * 【DBUS 协议简介】
 *   DBUS 是 DJI 遥控器接收机（如 DR16）使用的串口协议：
 *   - 波特率: 100 kbps, 8E1 (8 数据位, 偶校验, 1 停止位)
 *   - 帧长度: 18 字节固定
 *   - 周期:   约 14 ms
 *   - 内容:   4 通道摇杆 + 2 开关 + 鼠标/键盘值
 *
 * 【接收机制】
 *   使用 USART3 + DMA (Receive-to-IDLE) 异步接收。
 *   DMA 将串口数据自动搬运到缓冲区，IDLE 事件触发 HAL 回调，
 *   回调中校验帧长 (18 字节) 后复制到 latest_frame。
 *   主循环通过 DBUS_Process() 解析最新帧，通过 DBUS_GetData()
 *   读取解析结果。
 *
 * 【依赖】
 *   USART3, DMA1_Stream1 (由 CubeMX 生成在 dma.c/usart.c 中)
 * ===========================================================================
 */

#ifndef DBUS_H
#define DBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/* DBUS 帧长度：18 字节固定 (DBUS frame length: fixed 18 bytes) */
#define DBUS_FRAME_LENGTH 18U

/* DBUS 摇杆通道数量与正常数据范围 */
#define DBUS_CHANNEL_COUNT       4U
#define DBUS_CHANNEL_MIN         364U
#define DBUS_CHANNEL_CENTER      1024U
#define DBUS_CHANNEL_MAX         1684U

/* 三挡开关编码：上=1、下=2、中=3 */
#define DBUS_SWITCH_UP           1U
#define DBUS_SWITCH_DOWN         2U
#define DBUS_SWITCH_MIDDLE       3U

/* 超过 100 ms 没有收到有效帧，认为遥控器离线 */
#define DBUS_OFFLINE_TIMEOUT_MS  100U

/*
 * DBUS 解析结果。
 *
 * channel[]          原始 11 bit 通道值，正常范围 364~1684，中值约 1024
 * centered_channel[] 去中心值，正常范围约 -660~+660
 * switch_value[]     三挡开关值，0号=S1，1号=S2
 * frame_count        主循环取到的完整 18 字节帧总数
 * valid_frame_count  通过通道和开关范围检查的帧数
 * invalid_frame_count 未通过范围检查的帧数
 * last_frame_valid   最近取到的一帧是否合法
 * online             最近 100 ms 内是否收到过合法帧
 */
typedef struct
{
  uint16_t channel[DBUS_CHANNEL_COUNT];
  int16_t centered_channel[DBUS_CHANNEL_COUNT];
  uint8_t switch_value[2];
  uint32_t frame_count;
  uint32_t valid_frame_count;
  uint32_t invalid_frame_count;
  uint32_t last_valid_frame_ms;
  bool last_frame_valid;
  bool online;
} DBUS_Data_t;

/*
 * 初始化 DBUS 原始数据接收 (Initialize DBUS raw data reception)。
 *
 * USART3、GPIO 和 DMA 必须已经由 CubeMX 完成初始化。该函数启动
 * USART3 RX 的 Receive-to-IDLE DMA，并持续接收 18 字节 DBUS 帧。
 */
HAL_StatusTypeDef DBUS_Init(UART_HandleTypeDef *huart);

/*
 * 取出最近收到的一帧原始 DBUS 数据 (Read the latest raw DBUS frame)。
 *
 * 返回 true  表示 frame 已写入一帧新数据；
 * 返回 false 表示当前没有尚未读取的新帧。
 * 协议字段解析（摇杆值、开关状态等）将在后续步骤中完成。
 */
bool DBUS_ReadRawFrame(uint8_t frame[DBUS_FRAME_LENGTH]);

/*
 * 主循环 DBUS 处理入口。
 *
 * 非阻塞取出最新原始帧，解析 4 个摇杆通道与 2 个开关，并检查数据范围。
 * 即使当前没有新帧，也会更新 online 超时状态。
 *
 * 返回 true 表示本次调用处理了一帧完整数据（该帧可能合法或不合法）；
 * 返回 false 表示本次没有新帧。
 */
bool DBUS_Process(void);

/* 获取最近一次 DBUS 解析状态；返回的只读指针始终有效。 */
const DBUS_Data_t *DBUS_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* DBUS_H */
