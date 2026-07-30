/**
 * ===========================================================================
 * @file    chassis_can.c
 * @brief   底盘 CAN 控制命令发送模块 (Chassis CAN Control Command Module)
 * ===========================================================================
 *
 * 【模块功能】
 *   本模块负责通过 CAN1 总线向底盘发送运动控制指令。底盘（例如麦克纳姆轮
 *   底盘或差速底盘）通过 CAN 接收控制量帧和模式帧来执行运动。
 *
 * 【系统拓扑】
 *   CAN1 → Yaw GM6020 + 本模块发送帧（共享物理总线，帧 ID 不同）
 *   CAN2 → Pitch GM6020
 *
 * 【发送协议】
 *   每个控制周期（默认 10 ms）发送两帧标准 CAN 数据帧：
 *
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ 帧1 — 控制量帧 (StdId = 0x300, DLC = 8)                │
 *   │   DATA[0:1] = vx  前进速度 (Q10 定点数, int16)         │
 *   │   DATA[2:3] = vy  横移速度 (Q10 定点数, int16)         │
 *   │   DATA[4:5] = wz  旋转速度 (Q10 定点数, int16)         │
 *   │   DATA[6:7] = offset_angle_rad 偏移角 (Q10, int16)     │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 帧2 — 模式帧 (StdId = 0x301, DLC = 1)                  │
 *   │   DATA[0]   = chassis_mode (uint8 枚举)                │
 *   └─────────────────────────────────────────────────────────┘
 *
 *   Q10 编码：实际值 = 原始值 / 1024，发送时乘以 1024 取整。
 *   例如 vx = 1.5 m/s → 编码为 1536。
 *
 * 【底盘模式 (chassis_mode_e)】
 *   CHASSIS_MODE_SOFTWARE_OFF (0) — 软件断电：电机不输出力矩
 *   CHASSIS_MODE_FOLLOW       (1) — 跟随模式：云台角度偏移用于底盘定向
 *   CHASSIS_MODE_NO_FOLLOW    (2) — 非跟随模式：独立速度控制
 *   CHASSIS_MODE_SPIN         (3) — 自旋模式：原地旋转
 *
 * 【安全机制】
 *   - 急停锁存：调用 ChassisCAN_EmergencyStop() 立即发送零速度+断电模式，
 *     并拒绝后续普通命令直到 ChassisCAN_ClearEmergencyStop() 被调用
 *   - 参数校验：每个 Q10 值在编码前检查是否为有限浮点数且不超出 int16 范围
 *   - 编译期检查：#error 确保 CAN ID 为有效标准帧 ID 且控制帧与模式帧 ID 不同
 *
 * 【数据流】
 *   外部调用 ChassisCAN_SetCommand() → 更新 chassis_command →
 *   ChassisCAN_Process() 按周期发送 → CAN1 总线上底盘接收
 *
 * 【依赖】
 *   - config/chassis_can_config.h   CAN ID、Q10 缩放因子、发送周期等配置
 *   - chassis_can.h                 对外 API 声明与类型定义
 *   - HAL CAN 驱动                  CAN 帧的硬件收发
 * ===========================================================================
 */

#include "chassis_can.h"

#include "config/chassis_can_config.h"

#include <math.h>
#include <stdint.h>

#if (CHASSIS_CAN_CTRL_STD_ID > 0x7FFU)
#error "CHASSIS_CAN_CTRL_STD_ID must not exceed 0x7FF"
#endif

#if (CHASSIS_CAN_MODE_STD_ID > 0x7FFU)
#error "CHASSIS_CAN_MODE_STD_ID must not exceed 0x7FF"
#endif

#if (CHASSIS_CAN_CTRL_STD_ID == CHASSIS_CAN_MODE_STD_ID)
#error "The chassis control and mode CAN IDs must be different"
#endif

#if (CHASSIS_CAN_TX_PERIOD_MS == 0U)
#error "CHASSIS_CAN_TX_PERIOD_MS must be greater than zero"
#endif

/* ---- 模块级静态变量 ---- */

/* CAN1 句柄，由 ChassisCAN_Init() 赋值，ChassisCAN_Process() 使用 */
static CAN_HandleTypeDef *chassis_can;

/*
 * 下一周期待发送的底盘控制命令缓存。
 * 初始值：零速度、软件断电。收到合法DBUS命令后才切换到跟随模式。
 * 通过 ChassisCAN_SetCommand() 更新。
 */
static Chassis_Ctrl_Cmd_s chassis_command =
{
  .vx                = 0.0f,               /* 前进速度 (m/s) —— forward velocity */
  .vy                = 0.0f,               /* 横移速度 (m/s) —— lateral velocity  */
  .wz                = 0.0f,               /* 旋转速度 (rad/s) —— angular velocity */
  .offset_angle_rad  = 0.0f,               /* 偏移角 (rad) —— offset angle in radians */
  .chassis_mode      = CHASSIS_MODE_SOFTWARE_OFF /* 无遥控数据时保持软件断电 */
};

/* 上一次成功发送的时间戳 (ms)，用于周期节流 */
static uint32_t last_tx_ms;                 /* last transmit timestamp —— 上次发送时刻 */

/* 急停锁存标志：true 时拒绝 ChassisCAN_SetCommand 并维持零速度断电 */
static bool emergency_stop_latched;         /* emergency stop latch —— 急停锁存状态 */

/* ===================================================================
 * 内部工具函数 (Internal Helper Functions)
 * ===================================================================*/

/*
 * Q10 编码前校验：检查浮点数是否能安全编码为 int16 Q10 定点数。
 *
 * Q10 格式：实际值 = 编码值 / 1024
 * 合法编码范围：[-32768, 32767]
 * 对应实际值范围：[-32.0, 32.0)（分辨率 1/1024 ≈ 0.001）
 *
 * 返回 true 表示值可以安全编码。
 */
static bool q10_value_is_valid(float value)
{
  const float scaled = value * CHASSIS_Q10_SCALE;

  return isfinite(value)
      && (scaled >= -32768.0f)
      && (scaled <= 32767.0f);
}

/*
 * Q10 编码：float → int16。
 * 调用前应先通过 q10_value_is_valid() 校验，否则溢出行为未定义。
 *
 * @param value  物理量（m/s、rad/s 或 rad）
 * @return       Q10 定点数编码值，四舍五入取整
 */
static int16_t q10_encode(float value)
{
  /* lroundf: 四舍五入到最近整数（round to nearest integer） */
  return (int16_t)lroundf(value * CHASSIS_Q10_SCALE);
}

/*
 * 将 int16_t 以大端字节序（Big-Endian）打包到目标缓冲区的连续两个字节。
 * CAN 总线上 GM6020 和底盘控制都使用大端序。
 *
 * @param destination  目标缓冲区（至少 2 字节）
 * @param value        要打包的 int16_t 值
 */
static void pack_int16_be(uint8_t *destination, int16_t value)
{
  const uint16_t raw = (uint16_t)value;

  destination[0] = (uint8_t)(raw >> 8);
  destination[1] = (uint8_t)raw;
}

/*
 * 构造并发送两帧 CAN 消息：控制量帧 (0x300) + 模式帧 (0x301)。
 *
 * 【发送前检查】
 *   - CAN 句柄是否已初始化
 *   - 发送邮箱是否有至少 2 个空闲槽位（两帧各需要一个）
 *
 * 【返回值】
 *   HAL_OK    — 两帧均成功提交到硬件发送邮箱
 *   HAL_BUSY  — CAN 未就绪或邮箱不足（调用方下一周期重试即可）
 *   其他值    — HAL CAN 驱动错误码
 */
static HAL_StatusTypeDef chassis_can_send(void)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t control_data[8] = {0};   /* 控制量帧的 8 字节数据载荷 */
  uint8_t mode_data[1] = {0};      /* 模式帧的 1 字节数据载荷 */
  uint32_t mailbox;                /* 硬件分配的发送邮箱编号（本模块不使用） */
  HAL_StatusTypeDef status;

  /* 检查前置条件：CAN 已初始化且邮箱足够 */
  if ((chassis_can == NULL)
      || (HAL_CAN_GetTxMailboxesFreeLevel(chassis_can) < 2U))
  {
    return HAL_BUSY;
  }

  /* --- 构造控制量帧 (StdId=0x300, DLC=8) --- */
  /* 将四个 float 物理量编码为 Q10 int16 并打包为大端字节序 */
  pack_int16_be(&control_data[0], q10_encode(chassis_command.vx));             /* DATA[0:1] */
  pack_int16_be(&control_data[2], q10_encode(chassis_command.vy));             /* DATA[2:3] */
  pack_int16_be(&control_data[4], q10_encode(chassis_command.wz));             /* DATA[4:5] */
  pack_int16_be(
      &control_data[6],
      q10_encode(chassis_command.offset_angle_rad));                           /* DATA[6:7] */

  /* 填充 CAN 帧头：标准帧、数据帧、不附带硬件时间戳 */
  tx_header.StdId = CHASSIS_CAN_CTRL_STD_ID;          /* 0x300 */
  tx_header.IDE = CAN_ID_STD;                         /* 标准 ID (11-bit) */
  tx_header.RTR = CAN_RTR_DATA;                       /* 数据帧（非远程帧） */
  tx_header.DLC = 8U;                                 /* 数据长度 8 字节 */
  tx_header.TransmitGlobalTime = DISABLE;              /* 不附加硬件时间戳 */

  /* 提交控制量帧到 CAN 发送邮箱 */
  status = HAL_CAN_AddTxMessage(
      chassis_can, &tx_header, control_data, &mailbox);
  if (status != HAL_OK)
  {
    return status;  /* 发送失败，不继续发送模式帧 */
  }

  /* --- 构造模式帧 (StdId=0x301, DLC=1) --- */
  tx_header.StdId = CHASSIS_CAN_MODE_STD_ID;          /* 0x301 */
  tx_header.DLC = 1U;                                 /* 数据长度 1 字节 */
  mode_data[0] = (uint8_t)chassis_command.chassis_mode;

  /* 提交模式帧到 CAN 发送邮箱 */
  return HAL_CAN_AddTxMessage(
      chassis_can, &tx_header, mode_data, &mailbox);
}

/*
 * 初始化 CAN1 底盘发送通道。
 *
 * 【执行流程】
 *   1. 校验 hcan 非空且为 CAN1 外设
 *   2. CAN1 未启动时启动；若 Yaw 模块已启动则直接复用
 *   3. 记录句柄和初始时间戳，清除急停锁存
 *
 * 【注意】必须在 MX_CAN1_Init() 之后调用。
 */
HAL_StatusTypeDef ChassisCAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_StatusTypeDef status;

  if ((hcan == NULL) || (hcan->Instance != CAN1))
  {
    return HAL_ERROR;
  }

  if (hcan->State == HAL_CAN_STATE_READY)
  {
    status = HAL_CAN_Start(hcan);
    if (status != HAL_OK)
    {
      return status;
    }
  }
  else if (hcan->State != HAL_CAN_STATE_LISTENING)
  {
    return HAL_ERROR;
  }

  chassis_can = hcan;
  last_tx_ms = HAL_GetTick();
  emergency_stop_latched = false;
  return HAL_OK;
}

/*
 * 更新下一周期要发送的底盘命令。
 *
 * 【拒绝条件】（任一满足即返回 false）
 *   - 急停锁存中（emergency_stop_latched）
 *   - command 为空指针
 *   - 任意物理量 Q10 编码溢出（非有限值或超出 ±32 范围）
 *   - chassis_mode 枚举值非法
 *
 * 【返回值】
 *   true  — 命令已更新，将在下一个发送周期生效
 *   false — 命令被拒绝
 */
bool ChassisCAN_SetCommand(const Chassis_Ctrl_Cmd_s *command)
{
  if (emergency_stop_latched
      || (command == NULL)
      || !q10_value_is_valid(command->vx)
      || !q10_value_is_valid(command->vy)
      || !q10_value_is_valid(command->wz)
      || !q10_value_is_valid(command->offset_angle_rad)
      || (command->chassis_mode < CHASSIS_MODE_SOFTWARE_OFF)
      || (command->chassis_mode > CHASSIS_MODE_SPIN))
  {
    return false;
  }

  chassis_command = *command;
  return true;
}

/*
 * 锁存式急停：立即发送零速度 + 软件断电模式，并阻止后续普通命令。
 *
 * 【行为】
 *   1. 置位 emergency_stop_latched 锁存标志
 *   2. 将所有速度清零、模式设为 SOFTWARE_OFF
 *   3. 立即触发一次 CAN 发送（不等周期定时器）
 *
 * 【注意】
 *   急停后 ChassisCAN_Process() 仍会按周期发送零速度帧，
 *   确保底盘持续收到停止指令（防止通信中断导致的失控）。
 */
HAL_StatusTypeDef ChassisCAN_EmergencyStop(void)
{
  emergency_stop_latched = true;
  chassis_command.vx = 0.0f;
  chassis_command.vy = 0.0f;
  chassis_command.wz = 0.0f;
  chassis_command.offset_angle_rad = 0.0f;
  chassis_command.chassis_mode = CHASSIS_MODE_SOFTWARE_OFF;

  if (chassis_can == NULL)
  {
    return HAL_ERROR;
  }

  last_tx_ms = HAL_GetTick() - CHASSIS_CAN_TX_PERIOD_MS;
  return chassis_can_send();
}

/*
 * 解除急停锁存。
 * 解除后仍保持零速度和软件断电模式（因为 chassis_command 未被恢复）。
 * 外部需再调用 ChassisCAN_SetCommand() 才能恢复底盘运动。
 */
void ChassisCAN_ClearEmergencyStop(void)
{
  emergency_stop_latched = false;
}

/*
 * 主循环周期调度：按 CHASSIS_CAN_TX_PERIOD_MS (10 ms) 周期发送底盘命令。
 *
 * 【发送节流】
 *   使用 last_tx_ms 记录上次成功发送时间，避免每轮主循环都发送
 *   （主循环频率 ~1 kHz → 周期 1 ms，而底盘只需 ~100 Hz → 周期 10 ms）。
 *
 * 【发送失败处理】
 *   HAL_CAN_AddTxMessage 失败时（如 CAN 总线拥堵），不更新时间戳，
 *   下一轮主循环会立即重试，直到发送成功。
 *
 * 【主循环调用】while(1) 中每轮调用一次。
 */
void ChassisCAN_Process(void)
{
  const uint32_t now = HAL_GetTick();

  if ((chassis_can == NULL)
      || ((uint32_t)(now - last_tx_ms) < CHASSIS_CAN_TX_PERIOD_MS))
  {
    return;
  }

  if (chassis_can_send() == HAL_OK)
  {
    last_tx_ms = now;
  }
}
