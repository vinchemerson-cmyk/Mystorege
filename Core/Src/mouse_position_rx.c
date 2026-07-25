#include "mouse_position_rx.h"

#include "motor_control.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * ============================================================================
 * 鼠标位置串口接收模块
 * ============================================================================
 *
 * 【功能】
 *   通过 USART6 以 115200-8-N-1 从 PC 端接收 8 字节的鼠标目标角度帧，
 *   校验帧有效性后，将角度值安全地传递给 GM6020 Yaw 轴位置环。
 *
 * 【串口帧格式】（小端字节序）
 *   Byte[0]    = 0xAA     帧头 0（同步字节 1）
 *   Byte[1]    = 0x55     帧头 1（同步字节 2）
 *   Byte[2:5]  = float    目标逻辑角度（度），IEEE 754 单精度，小端
 *   Byte[6]    = 0x00     保留字节，必须为 0（用于帧格式校验）
 *   Byte[7]    = uint8_t  XOR 校验和，等于 Byte[0] ^ Byte[1] ^ ... ^ Byte[6]
 *
 * 【线程安全设计】
 *   USART6 接收在中断上下文（HAL_UART_RxCpltCallback）中完成逐字节解析，
 *   PID 控制计算在主循环的 GM6020_Process() 中完成。
 *
 *   中断与主循环之间的数据传递：
 *     中断侧  → 将角度值写入 pending_angle_bits（volatile uint32_t）
 *             并置 command_pending = true
 *     主循环 → 在 MousePositionRx_Process() 中读取 pending_angle_bits
 *             并通过 GM6020_SetTargetPosition() 传递给电机控制模块
 *
 *   这种"中断发布 → 主循环消费"的设计避免了在 USART 中断中直接调用
 *   PID 控制函数，防止中断打断正在运行的 PID 计算导致数据不一致。
 *
 * 【错误恢复】
 *   帧头失同步 → 状态机自动丢弃字节，等待下一帧的 0xAA
 *   XOR 校验失败 / 保留字节非零 → 丢弃当前帧，不发布命令
 *   UART 硬件错误（噪声、帧错误等）→ 中止接收 → 重置解析器 → 重新启动
 * ============================================================================
 */

/*
 * 帧协议常量。
 * 帧头采用双字节 AA 55，降低与数据字节的误匹配概率。
 */
#define MOUSE_FRAME_HEAD0  0xAAU
#define MOUSE_FRAME_HEAD1  0x55U
#define MOUSE_FRAME_SIZE   8U

/*
 * 帧解析器状态。
 * 使用 index 跟踪当前正在收集的字节位置（0~7），
 * 而非显式状态枚举。index 的特殊含义：
 *   0 → 等待 HEAD0 (0xAA)
 *   1 → 等待 HEAD1 (0x55)
 *   2~7 → 收集数据字节
 *   到达 8 → 满一帧，触发发布
 */
typedef struct
{
  uint8_t bytes[MOUSE_FRAME_SIZE]; /* 帧缓冲区（8 字节） */
  uint8_t index;                   /* 当前收集位置（0~7），到达 8 时发布并归零 */
} MouseFrameParser_t;

/* ---- 模块级全局变量 ---- */

static UART_HandleTypeDef *mouse_uart;  /* USART6 句柄指针 */
static uint8_t mouse_rx_byte;           /* 中断接收的单字节缓冲区 */
static MouseFrameParser_t mouse_parser; /* 帧解析器状态 */

/*
 * 中断与主循环之间的数据通道。
 *
 * pending_angle_bits 存储 IEEE 754 单精度浮点数的原始 32 位位模式。
 * 使用 uint32_t 保存位模式，便于在中断与主循环之间进行一次对齐的
 * 32 位传递；主循环再用 memcpy 转回 float，避免使用指针强转进行
 * 类型重解释。
 *
 * command_pending 为 true 表示中断接收到了新的有效帧，等待主循环消费。
 *
 * receive_restart_required 为 true 表示最近一次
 * HAL_UART_Receive_IT 启动失败，需要主循环继续重试。
 */
static volatile uint32_t pending_angle_bits;
static volatile bool command_pending;
static volatile bool receive_restart_required;

/*
 * 校验帧的 XOR 校验和。
 *
 * 计算方式：Byte[0] ~ Byte[6] 的 XOR 值应与 Byte[7] 相等。
 * 返回 true 表示校验通过。
 */
static bool mouse_frame_checksum_valid(const uint8_t *frame)
{
  uint8_t checksum = 0U;
  uint8_t index;

  for (index = 0U; index < MOUSE_FRAME_SIZE - 1U; ++index)
  {
    checksum ^= frame[index];
  }
  return checksum == frame[MOUSE_FRAME_SIZE - 1U];
}

/*
 * 发布一帧有效的鼠标位置命令。
 *
 * 校验条件：
 *   1. XOR 校验通过（防止传输比特错误）
 *   2. Byte[6] == 0（保留字节，用于帧格式二次确认）
 *
 * 校验通过后，提取 Byte[2:5] 的 float 角度值，将其 32 位模式
 * 写入 pending_angle_bits，并置 command_pending = true。
 *
 * 【注意】本函数在 USART6 中断上下文中被调用。
 */
static void mouse_frame_publish(const uint8_t *frame)
{
  uint32_t angle_bits;

  /* 双重校验：XOR + 保留字节 */
  if (!mouse_frame_checksum_valid(frame) || (frame[6] != 0U))
  {
    return;
  }

  /*
   * 将 4 字节小端 float 的位模式复制到 uint32_t。
   * 使用 memcpy 而非指针强转以避免严格别名违规。
   */
  memcpy(&angle_bits, &frame[2], sizeof(angle_bits));
  pending_angle_bits = angle_bits;
  command_pending = true;
}

/*
 * 逐字节帧解析状态机。
 *
 * 【状态机行为】
 *   index = 0（等待 HEAD0）:
 *     收到 0xAA → 存入 bytes[0]，index 前进到 1
 *     收到其他 → 保持 index=0，等待下一字节（丢弃噪声字节）
 *
 *   index = 1（等待 HEAD1）:
 *     收到 0x55 → 存入 bytes[1]，index 前进到 2（开始收集数据）
 *     收到 0xAA → 重新作为 HEAD0，index 回到 1
 *                  （处理 AA AA 55 ... 这种帧头重复的场景）
 *     收到其他 → 帧头匹配失败，index 回到 0，等待下一个 0xAA
 *
 *   index >= 2（收集数据字节）:
 *     存入 bytes[index]，index++
 *     index == 8 → 满一帧，调用 mouse_frame_publish() 处理，
 *                   然后 index 归零，开始等待下一帧
 *
 * 【为什么用双字节帧头】
 *   单字节帧头（如只用 0xAA）在浮点数据中出现的概率约 1/256，
 *   双字节将误匹配概率降至 1/65536，大幅减少误同步的可能。
 *
 * 【注意】本函数在 USART6 中断上下文中被调用。
 */
static void mouse_frame_push_byte(uint8_t byte)
{
  switch (mouse_parser.index)
  {
    case 0U:
      /*
       * 等待帧头第一个字节 0xAA。
       * 任何非 0xAA 的字节都被当作噪声丢弃。
       */
      if (byte == MOUSE_FRAME_HEAD0)
      {
        mouse_parser.bytes[0] = byte;
        mouse_parser.index = 1U;
      }
      break;

    case 1U:
      /*
       * 等待帧头第二个字节 0x55。
       * 如果收到 0xAA（而非 0x55），说明可能是上一帧的 0xAA
       * 被误当成帧头后紧跟着又一个 0xAA —— 重置为 index=1，
       * 利用这个新的 0xAA 作为帧头重新开始。
       */
      if (byte == MOUSE_FRAME_HEAD1)
      {
        mouse_parser.bytes[1] = byte;
        mouse_parser.index = 2U;
      }
      else
      {
        mouse_parser.index =
            (byte == MOUSE_FRAME_HEAD0) ? 1U : 0U;
      }
      break;

    default:
      /*
       * 收集第 2~7 字节（数据 + 校验）。
       * 到达 MOUSE_FRAME_SIZE (8) 时发布帧并重置。
       */
      mouse_parser.bytes[mouse_parser.index++] = byte;
      if (mouse_parser.index == MOUSE_FRAME_SIZE)
      {
        mouse_frame_publish(mouse_parser.bytes);
        mouse_parser.index = 0U;
      }
      break;
  }
}

/*
 * 启动单字节 UART 中断接收。
 *
 * 使用 HAL_UART_Receive_IT 以 1 字节为单位的循环接收模式：
 * 每收到 1 字节触发一次 HAL_UART_RxCpltCallback，
 * 处理完成后再次调用本函数重新使能下一字节的接收。
 *
 * 如果 HAL_UART_Receive_IT 返回错误（如 UART 未就绪），
 * 设置 receive_restart_required 标志，由主循环重试。
 */
static HAL_StatusTypeDef mouse_uart_start_receive(void)
{
  const HAL_StatusTypeDef status =
      HAL_UART_Receive_IT(mouse_uart, &mouse_rx_byte, 1U);

  receive_restart_required = (status != HAL_OK);
  return status;
}

/*
 * 初始化鼠标位置串口接收器。
 *
 * 清零所有内部状态，并启动第一次 UART 中断接收。
 * 必须在 MX_USART6_UART_Init() 之后调用。
 *
 * 【参数】
 *   huart: USART6 外设句柄指针
 * 【返回值】HAL_OK 表示初始化成功并已启动接收。
 */
HAL_StatusTypeDef MousePositionRx_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return HAL_ERROR;
  }

  mouse_uart = huart;
  mouse_rx_byte = 0U;
  memset(&mouse_parser, 0, sizeof(mouse_parser));
  pending_angle_bits = 0U;
  command_pending = false;
  receive_restart_required = false;

  return mouse_uart_start_receive();
}

/*
 * 主循环中的鼠标位置处理函数。
 *
 * 【调用时机】在 while(1) 主循环中调用，排在 GM6020_Process() 之前。
 *            确保最新的位置目标先被设置，再运行 PID 计算。
 *
 * 【处理流程】
 *   1. 检查是否需要重启中断接收（上一次出错后的恢复）
 *   2. 检查是否有待处理的命令（command_pending）
 *   3. 关中断临界区：安全读取 pending_angle_bits 并清零 command_pending
 *   4. 将 uint32_t 位模式恢复为 float 角度值
 *   5. 校验 float 是否为有限值（防止 NaN/Inf 传入电机控制）
 *   6. 调用 GM6020_SetTargetPosition() 将角度映射到 Yaw 轴
 *
 * 【关于关中断保护】
 *   Cortex-M4 对自然对齐的 uint32_t 及单字节 bool 可用单次指令访问，
 *   但"读取 angle_bits + 清零 command_pending"这两步需要作为一个
 *   不可分割的整体来执行。如果在这两步之间发生新一帧中断：
 *     - pending_angle_bits 被中断更新为新值
 *     - 但主循环已经读走了旧值
 *     - command_pending 在主循环中被清零（丢失了新帧的标志）
 *   因此需要短暂关闭中断保护这段临界区。
 */
void MousePositionRx_Process(void)
{
  uint32_t angle_bits;
  uint32_t previous_primask;
  float target_angle_deg;

  /*
   * 错误恢复：中断回调会先立即尝试恢复接收；如果启动失败，
   * mouse_uart_start_receive() 会置位标志，由主循环在此继续重试。
   */
  if (receive_restart_required)
  {
    (void)mouse_uart_start_receive();
  }

  /* 无待处理命令 → 直接返回，不浪费 CPU 时间 */
  if (!command_pending)
  {
    return;
  }

  /*
   * 临界区：关中断 → 读取角度值 + 清除 pending 标志 → 恢复中断。
   *
   * 保存 PRIMASK 并仅在原本开中断的情况下恢复，避免：
   *   - 在原本就关中断的上下文中错误地开中断
   *   - 嵌套关中断导致的状态丢失
   */
  previous_primask = __get_PRIMASK();
  __disable_irq();
  angle_bits = pending_angle_bits;
  command_pending = false;
  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  /* 将 32 位模式安全地转为 float（无严格别名违规） */
  memcpy(&target_angle_deg, &angle_bits, sizeof(target_angle_deg));

  /*
   * 防御性检查：如果 PC 端发送了非法的浮点值（NaN、Inf 等），
   * 直接丢弃，不传入电机控制。电机收到 NaN 会导致不可预期的行为。
   */
  if (!isfinite(target_angle_deg))
  {
    return;
  }

  /*
   * 当前 8 字节鼠标协议只有一个角度值，因此固定映射到 Yaw 轴。
   * Pitch 轴：
   *   - 可通过 GM6020_SetGimbalPosition() 两轴接口控制
   *   - 或等待后续扩展的双角度串口协议（如 12 字节帧携带两个角度）
   */
  GM6020_SetTargetPosition(
      GM6020_AXIS_YAW, target_angle_deg);
}

/*
 * UART 接收完成中断回调（由 HAL_UART_IRQHandler 内部调用）。
 *
 * 【行为】
 *   1. 确认中断来源是 mouse_uart（USART6）
 *   2. 将收到的字节馈入帧解析器
 *   3. 启动下一次单字节接收
 *
 * 【注意】
 *   此函数在 USART6 中断上下文中执行，应保持短小快速。
 *   不做浮点运算，不做电机控制调用，不做长时间阻塞操作。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((mouse_uart != NULL) && (huart->Instance == mouse_uart->Instance))
  {
    mouse_frame_push_byte(mouse_rx_byte);
    (void)mouse_uart_start_receive();
  }
}

/*
 * UART 错误中断回调（由 HAL_UART_IRQHandler 内部调用）。
 *
 * 【触发条件】
 *   帧错误（噪声导致起始位/停止位不匹配）
 *   噪声检测、溢出错误等硬件层错误
 *
 * 【恢复策略】
 *   1. 重置帧解析器（丢弃可能已损坏的半帧）
 *   2. 中止当前 UART 接收（HAL_UART_AbortReceive）
 *   3. 立即尝试重新启动单字节接收
 *   4. 如果启动失败，置位 receive_restart_required，由主循环重试
 *
 * 【恢复位置】
 *   当前实现会在错误回调中执行 Abort 并立即调用
 *   mouse_uart_start_receive()；主循环仅负责失败后的后续重试。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((mouse_uart != NULL) && (huart->Instance == mouse_uart->Instance))
  {
    memset(&mouse_parser, 0, sizeof(mouse_parser));
    (void)HAL_UART_AbortReceive(huart);
    (void)mouse_uart_start_receive();
  }
}
