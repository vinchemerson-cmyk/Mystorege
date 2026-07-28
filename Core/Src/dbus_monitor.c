/**
 * ===========================================================================
 * @file    dbus_monitor.c
 * @brief   通过 USB CDC 虚拟串口周期输出 DBUS 摇杆数据
 * ===========================================================================
 *
 * 输出格式（ASCII、CRLF 结尾）：
 * RC,frame,online,valid,ch0,ch1,ch2,ch3,s1,s2,c0,c1,c2,c3,invalid\r\n
 *
 * ch0~ch3 为原始值（正常约 364~1684，中值约 1024）；
 * c0~c3 为减去 1024 后的去中心值（正常约 -660~+660）；
 * s1/s2 的三挡位置编码为上=1、下=2、中=3。
 * ===========================================================================
 */

#include "dbus_monitor.h"

#include "dbus.h"
#include "usbd_cdc_if.h"

#include <stdbool.h>
#include <stdio.h>

#define DBUS_MONITOR_PERIOD_MS       50U
#define DBUS_MONITOR_BUFFER_CAPACITY 160U

/*
 * CDC_Transmit_FS() 异步使用传入缓冲区，因此发送完成前不能修改内容。
 * 使用静态缓冲区和 pending 标志，在 USBD_BUSY 时保留本帧并稍后重试。
 */
static uint8_t monitor_buffer[DBUS_MONITOR_BUFFER_CAPACITY];
static uint16_t monitor_length;
static uint32_t last_monitor_tx_ms;
static bool monitor_pending;

void DBUS_Monitor_Process(void)
{
  const uint32_t now = HAL_GetTick();
  const DBUS_Data_t *data;
  int length;

  if (monitor_pending)
  {
    if (CDC_Transmit_FS(monitor_buffer, monitor_length) == USBD_OK)
    {
      monitor_pending = false;
      last_monitor_tx_ms = now;
    }
    return;
  }

  if ((uint32_t)(now - last_monitor_tx_ms) < DBUS_MONITOR_PERIOD_MS)
  {
    return;
  }

  data = DBUS_GetData();
  if (data == NULL)
  {
    return;
  }

  length = snprintf(
      (char *)monitor_buffer,
      sizeof(monitor_buffer),
      "RC,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%lu\r\n",
      (unsigned long)data->frame_count,
      data->online ? 1U : 0U,
      data->last_frame_valid ? 1U : 0U,
      (unsigned int)data->channel[0],
      (unsigned int)data->channel[1],
      (unsigned int)data->channel[2],
      (unsigned int)data->channel[3],
      (unsigned int)data->switch_value[0],
      (unsigned int)data->switch_value[1],
      (int)data->centered_channel[0],
      (int)data->centered_channel[1],
      (int)data->centered_channel[2],
      (int)data->centered_channel[3],
      (unsigned long)data->invalid_frame_count);

  if ((length <= 0) || ((uint32_t)length >= sizeof(monitor_buffer)))
  {
    return;
  }

  monitor_length = (uint16_t)length;
  monitor_pending = true;

  if (CDC_Transmit_FS(monitor_buffer, monitor_length) == USBD_OK)
  {
    monitor_pending = false;
    last_monitor_tx_ms = now;
  }
}
