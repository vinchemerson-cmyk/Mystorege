/**
 * ===========================================================================
 * @file    dbus.c
 * @brief   DBUS (DJI 遥控器接收机协议) 接收与解析模块
 * ===========================================================================
 *
 * 【硬件连接】
 *   USART3 RX → 遥控器接收机 DBUS 输出
 *   波特率 100 kbps, 8E1 (8 data bits, Even parity, 1 Stop bit)
 *   DMA1_Stream1 负责将 USART3 RX 数据自动搬运到内存缓冲区
 *
 * 【接收流程】
 *   1. DBUS_Init() → 启动 HAL_UARTEx_ReceiveToIdle_DMA()
 *      DMA 持续将串口数据写入 dbus_dma_buffer[18]
 *   2. 收到完整 18 字节或 IDLE 空闲 → HAL 触发 HAL_UARTEx_RxEventCallback()
 *   3. 回调中校验帧长 == 18 → memcpy 到 dbus_latest_frame → 置 ready 标志
 *   4. 主循环调用 DBUS_Process() 非阻塞解析最新帧
 *
 * 【DMA 模式】
 *   Normal 模式（非 Circular）：每次接收完成后需要手动重新启动。
 *   回调中调用 DBUS_StartReceive() 重启 DMA，确保连续接收。
 *   禁用 Half-Transfer 中断 (DMA_IT_HT)：18 字节帧在 9 字节处
 *   无需要处理的事件，减少不必要的中断开销。
 *
 * 【去重机制】
 *   接收机约每 14 ms 发送一帧。dbus_frame_ready
 *   标志保证每个新帧只被消费一次：ReadRawFrame 读取后清零标志，
 *   直到下一个 HAL 回调再次置位。
 *
 * 【依赖】
 *   USART3 + DMA1_Stream1 (CubeMX 初始化)
 *   HAL_UARTEx_ReceiveToIdle_DMA (STM32 HAL 扩展 API)
 * ===========================================================================
 */

#include "dbus.h"

#include <string.h>

/* ─── 模块静态变量 (Module Static Variables) ─── */

/* USART3 句柄指针，由 DBUS_Init() 赋值 */
static UART_HandleTypeDef *dbus_uart;                  /* UART handle for USART3 */

/* DMA 接收缓冲区：DMA 直接写入，中断回调中复制到 latest_frame */
static uint8_t dbus_dma_buffer[DBUS_FRAME_LENGTH];     /* DMA reception buffer — DMA 接收缓冲 */

/* 最新完整帧缓存：中断写入 → 主循环读取（临界区保护） */
static uint8_t dbus_latest_frame[DBUS_FRAME_LENGTH];   /* latest complete frame — 最新完整帧 */

/* 新帧就绪标志：volatile 确保中断写入对主循环可见 */
static volatile bool dbus_frame_ready;                 /* new frame ready flag — 新帧就绪标志 */

/* 主循环更新的最近一次合法解析结果与诊断计数 */
static DBUS_Data_t dbus_data;

/* ─── 内部函数 (Internal Functions) ─── */

/*
 * 启动 / 重新启动 USART3 DMA 接收。
 * 在初始化时首次调用，以及在每次 HAL 接收完成回调中重新启动。
 * 禁用 DMA Half-Transfer 中断以减少不必要的开销。
 */
static HAL_StatusTypeDef DBUS_StartReceive(void)
{
  HAL_StatusTypeDef status;

  if ((dbus_uart == NULL) || (dbus_uart->hdmarx == NULL))
  {
    return HAL_ERROR;
  }

  status = HAL_UARTEx_ReceiveToIdle_DMA(
      dbus_uart, dbus_dma_buffer, DBUS_FRAME_LENGTH);
  if (status != HAL_OK)
  {
    return status;
  }

  /*
   * DBUS 只在完整帧或串口空闲事件时处理数据，不需要 9 字节处的
   * DMA half-transfer 回调。
   */
  __HAL_DMA_DISABLE_IT(dbus_uart->hdmarx, DMA_IT_HT);

  return HAL_OK;
}

/*
 * 检查解包后的摇杆和开关是否落在 DBUS 正常范围内。
 * 该协议没有独立 CRC，因此使用字段范围排除错位帧和线路噪声。
 */
static bool DBUS_FieldsAreValid(
    const uint16_t channel[DBUS_CHANNEL_COUNT],
    const uint8_t switch_value[2])
{
  uint32_t index;

  for (index = 0U; index < DBUS_CHANNEL_COUNT; ++index)
  {
    if ((channel[index] < DBUS_CHANNEL_MIN)
        || (channel[index] > DBUS_CHANNEL_MAX))
    {
      return false;
    }
  }

  for (index = 0U; index < 2U; ++index)
  {
    if ((switch_value[index] < DBUS_SWITCH_UP)
        || (switch_value[index] > DBUS_SWITCH_MIDDLE))
    {
      return false;
    }
  }

  return true;
}

/* ─── 公有 API (Public API) ─── */

/*
 * 初始化 DBUS 接收：校验 USART3 和 DMA → 清零缓冲区 → 启动 DMA 接收。
 * @return HAL_OK 成功，HAL_ERROR 参数无效或缺少 DMA
 */
HAL_StatusTypeDef DBUS_Init(UART_HandleTypeDef *huart)
{
  if ((huart == NULL)
      || (huart->Instance != USART3)
      || (huart->hdmarx == NULL))
  {
    return HAL_ERROR;
  }

  dbus_uart = huart;
  dbus_frame_ready = false;
  memset(dbus_dma_buffer, 0, sizeof(dbus_dma_buffer));
  memset(dbus_latest_frame, 0, sizeof(dbus_latest_frame));
  memset(&dbus_data, 0, sizeof(dbus_data));

  return DBUS_StartReceive();
}

/*
 * 非阻塞读取最新原始帧 (Non-blocking read of latest raw frame)。
 * 使用临界区保护 dbus_latest_frame 的并发访问。
 * @return true 有新帧可用并已复制到 frame，false 无新帧
 */
bool DBUS_ReadRawFrame(uint8_t frame[DBUS_FRAME_LENGTH])
{
  uint32_t previous_primask;
  bool frame_available = false;

  if (frame == NULL)
  {
    return false;
  }

  previous_primask = __get_PRIMASK();
  __disable_irq();

  if (dbus_frame_ready)
  {
    memcpy(frame, dbus_latest_frame, DBUS_FRAME_LENGTH);
    dbus_frame_ready = false;
    frame_available = true;
  }

  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  return frame_available;
}

/*
 * 解析最新 DBUS 帧并维护有效帧计数与在线状态。
 *
 * 4 个摇杆通道均为跨字节打包的 11 bit 无符号整数：
 *   CH0 = byte0[7:0] + byte1[2:0]
 *   CH1 = byte1[7:3] + byte2[5:0]
 *   CH2 = byte2[7:6] + byte3[7:0] + byte4[0]
 *   CH3 = byte4[7:1] + byte5[3:0]
 * S1/S2 分别位于 byte5 的 bit[7:6] / bit[5:4]。
 */
bool DBUS_Process(void)
{
  uint8_t frame[DBUS_FRAME_LENGTH];
  uint16_t channel[DBUS_CHANNEL_COUNT];
  uint8_t switch_value[2];
  uint32_t index;
  const uint32_t now = HAL_GetTick();
  const bool frame_available = DBUS_ReadRawFrame(frame);

  if (frame_available)
  {
    channel[0] =
        ((uint16_t)frame[0]
         | ((uint16_t)frame[1] << 8U))
        & 0x07FFU;
    channel[1] =
        (((uint16_t)frame[1] >> 3U)
         | ((uint16_t)frame[2] << 5U))
        & 0x07FFU;
    channel[2] =
        (((uint16_t)frame[2] >> 6U)
         | ((uint16_t)frame[3] << 2U)
         | ((uint16_t)frame[4] << 10U))
        & 0x07FFU;
    channel[3] =
        (((uint16_t)frame[4] >> 1U)
         | ((uint16_t)frame[5] << 7U))
        & 0x07FFU;

    switch_value[0] = (frame[5] >> 6U) & 0x03U;
    switch_value[1] = (frame[5] >> 4U) & 0x03U;

    ++dbus_data.frame_count;
    if (DBUS_FieldsAreValid(channel, switch_value))
    {
      for (index = 0U; index < DBUS_CHANNEL_COUNT; ++index)
      {
        dbus_data.channel[index] = channel[index];
        dbus_data.centered_channel[index] =
            (int16_t)channel[index] - (int16_t)DBUS_CHANNEL_CENTER;
      }
      dbus_data.switch_value[0] = switch_value[0];
      dbus_data.switch_value[1] = switch_value[1];
      ++dbus_data.valid_frame_count;
      dbus_data.last_valid_frame_ms = now;
      dbus_data.last_frame_valid = true;
      dbus_data.online = true;
    }
    else
    {
      ++dbus_data.invalid_frame_count;
      dbus_data.last_frame_valid = false;
    }
  }

  if (dbus_data.online
      && ((uint32_t)(now - dbus_data.last_valid_frame_ms)
          > DBUS_OFFLINE_TIMEOUT_MS))
  {
    dbus_data.online = false;
  }

  return frame_available;
}

const DBUS_Data_t *DBUS_GetData(void)
{
  return &dbus_data;
}

/*
 * HAL 接收事件回调 (HAL UART Extended Rx Event Callback)。
 * 由 HAL 在 DMA 接收完成（满 18 字节）或 IDLE 事件时在中断上下文调用。
 * 校验帧长后复制到 latest_frame 并置 ready 标志，然后重启 DMA 接收。
 */
void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart, uint16_t size)
{
  if ((huart == dbus_uart) && (dbus_uart != NULL))
  {
    if (size == DBUS_FRAME_LENGTH)
    {
      memcpy(dbus_latest_frame,
             dbus_dma_buffer,
             DBUS_FRAME_LENGTH);
      dbus_frame_ready = true;
    }

    /*
     * DMA 使用 Normal 模式，因此每次完整帧或 IDLE 事件后都需要
     * 重新启动接收。长度异常的片段会被丢弃，下一帧重新对齐。
     */
    (void)DBUS_StartReceive();
  }
}
