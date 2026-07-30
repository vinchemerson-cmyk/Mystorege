/**
 * ===========================================================================
 * @file    control_input.c
 * @brief   USB CDC 虚拟串口命令解析与反馈上报模块
 * ===========================================================================
 *
 * 【模块定位】
 *   本模块是上位机与 MCU 之间的串口协议层，负责：
 *     输入 (control_in):  解析上位机下发的角度目标命令或急停命令，
 *                          并分发给 motor_control.c 和 chassis_can.c
 *     输出 (control_out): 周期上报双轴角度、转速、在线状态和急停状态
 *
 * 【串口拓扑】
 *   USB OTG FS (PA11/PA12) → CDC 虚拟串口 → 上位机 (PuTTY/串口助手/自定义上位机)
 *   波特率由 USB FS 协议协商，与 USART6 硬件串口独立。
 *
 * 【接收机制】
 *   USB CDC 中断回调 CDC_Receive_FS() → control_in_receive() 缓冲字节 →
 *   检测到 '\n' 行尾 → 发布一行待处理命令 →
 *   主循环 control_in() 在处理行 → 解析 → 执行 → ACK 回复
 *
 *   采用双缓冲设计：中断中只做字节接收和行组装，不做任何解析或
 *   外部函数调用，避免在中断上下文中占用过长 CPU 时间和可能的竞态。
 *
 * 【协议格式】
 *   输入（上位机→MCU）:
 *     "<Yaw角度>,<Pitch角度>\r\n"  — 设置双轴目标角度
 *       例: "123.45,-15.30\r\n"    — Yaw 123.45°, Pitch -15.30°
 *         Yaw 范围:  ±36000° (累计多圈)
 *         Pitch 范围: -31.0°~+18.5° (单圈位置)
 *     "ESTOP\r\n"                  — 锁存式急停（不区分大小写）
 *     "CLEAR\r\n"                  — 解除急停（严格区分大小写）
 *     "CALSTATUS\r\n"              — 查询上电自动标定状态（可选）
 *
 *   输出（MCU→上位机）:
 *     "OK\r\n"       — 命令执行成功
 *     "ERR\r\n"      — 命令格式错误
 *     "ESTOPPED\r\n" — 已执行急停
 *     "CLEARED\r\n"  — 已解除急停
 *     "LOCKED\r\n"   — 急停锁存中，拒绝角度命令
 *     "CALIBRATING\r\n" / "CALHOMING\r\n" / "CALIBRATED\r\n"
 *     / "CALWAIT\r\n" / "CALMOVING\r\n" / "CALERROR\r\n"
 *                    — 零位标定状态
 *     "FB,<yaw_deg>,<pitch_deg>,<yaw_rpm>,<pitch_rpm>,<yaw_online>,<pitch_online>,<estop>\r\n"
 *                    — 周期反馈帧 (~1 Hz)
 *
 * 【并发安全】
 *   中断上下文 (control_in_receive): 只写 volatile 标志和缓冲区
 *   主循环上下文 (control_in/control_out): 读写标志，访问 motor/chassis API
 *   通过 __disable_irq()/__enable_irq() 临界区保护共享变量访问。
 * ===========================================================================
 */

#include "control_input.h"

#include "chassis_can.h"
#include "config/gimbal_params.h"
#include "gimbal_calibration.h"
#include "motor_control.h"
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * 模块常量定义
 */

/* 输入行缓冲区容量：允许接收最长 47 字符的命令行（不含 '\0'） */
#define CONTROL_IN_LINE_CAPACITY    48U    /* input line buffer capacity — 输入行缓冲区容量 */

/* 输出反馈缓冲区容量：足够容纳 "FB,xxx.xx,xxx.xx,xxxx,xxxx,x,x,x\r\n" + '\0' */
#define CONTROL_OUT_BUFFER_CAPACITY 96U    /* output feedback buffer capacity — 输出反馈缓冲区容量 */

/* 反馈上报周期：1000 ms = 1 Hz */
#define CONTROL_OUT_PERIOD_MS       1000U  /* feedback transmit period — 反馈上报周期 */

/*
 * Yaw 多圈角度范围：±36000° = ±100 圈。
 * Yaw 轴可以无限制旋转（每圈跨零累计），这里只限制单次命令的目标范围。
 */
#define CONTROL_IN_YAW_MIN_DEG    (-36000.0f)  /* Yaw 最小角度目标 — minimum yaw angle */
#define CONTROL_IN_YAW_MAX_DEG      36000.0f   /* Yaw 最大角度目标 — maximum yaw angle */

/* Pitch命令与电机层共用同一组实测运行软限位，避免配置漂移。 */
#define CONTROL_IN_PITCH_MIN_DEG PITCH_MIN_ANGLE_DEG
#define CONTROL_IN_PITCH_MAX_DEG PITCH_MAX_ANGLE_DEG

/*
 * 命令响应确认枚举 (Control Acknowledgment Enum)。
 * 每个串口命令处理后通过 USB CDC 回复一个简短的 ACK 字符串。
 */
typedef enum
{
  CONTROL_ACK_NONE     = 0,  /* 无待发送 ACK — no pending acknowledgment */
  CONTROL_ACK_OK,            /* 命令执行成功 "OK\r\n" */
  CONTROL_ACK_ERR,           /* 命令格式错误 "ERR\r\n" */
  CONTROL_ACK_ESTOPPED,      /* 急停已执行 "ESTOPPED\r\n" */
  CONTROL_ACK_CLEARED,       /* 急停已解除 "CLEARED\r\n" */
  CONTROL_ACK_LOCKED,        /* 急停锁存中，拒绝命令 "LOCKED\r\n" */
  CONTROL_ACK_CALIBRATING,   /* 正在采集标定样本 */
  CONTROL_ACK_CALHOMING,     /* Pitch正在缓慢回传感器零点 */
  CONTROL_ACK_CALIBRATED,    /* 上电自动标定已完成 */
  CONTROL_ACK_CALWAIT,       /* 至少一个未标定轴正在等待有效反馈 */
  CONTROL_ACK_CALMOVING,     /* 云台未静止，样本已重置 */
  CONTROL_ACK_CALERROR       /* 标定或回零安全检查失败 */
} ControlAck_t;

/*
 * ─── 接收缓冲区（中断上下文写入，主循环上下文读取）─────────────────
 * 双缓冲设计：中断写入 receive_line，检测到 '\n' 后整行复制到
 * pending_line 并设置 pending_line_ready 标志，主循环读取 pending_line。
 */

/* 接收行累积缓冲区（中断中逐字节追加） */
static char receive_line[CONTROL_IN_LINE_CAPACITY];  /* receive line buffer — 接收行缓冲区 */
static uint32_t receive_length;                      /* 当前已接收字节数 — received byte count */
static bool receive_overflow;                        /* 接收溢出标志 — buffer overflow flag */

/* 待处理行（由中断写入、主循环消费） */
static char pending_line[CONTROL_IN_LINE_CAPACITY];   /* pending command line — 待处理命令行 */
static volatile bool pending_line_ready;               /* 待处理行就绪标志 — line ready for processing */
static volatile bool pending_line_invalid;             /* 待处理行无效标志 — line is invalid (overflow) */
static volatile bool pending_emergency_stop;           /* 急停请求标志 — emergency stop requested */
static volatile ControlAck_t pending_ack;              /* 待发送 ACK — pending acknowledgment */

/* ACK 回复字符串模板（不含 '\0'，发送时用 sizeof()-1 去除末尾空字符） */
static uint8_t ok_reply[]      = "OK\r\n";       /* 命令成功 */
static uint8_t err_reply[]     = "ERR\r\n";      /* 格式错误 */
static uint8_t estopped_reply[] = "ESTOPPED\r\n"; /* 已急停 */
static uint8_t cleared_reply[] = "CLEARED\r\n";   /* 已清除 */
static uint8_t locked_reply[]  = "LOCKED\r\n";    /* 锁存拒绝 */
static uint8_t calibrating_reply[] = "CALIBRATING\r\n";
static uint8_t calibration_homing_reply[] = "CALHOMING\r\n";
static uint8_t calibrated_reply[] = "CALIBRATED\r\n";
static uint8_t calibration_wait_reply[] = "CALWAIT\r\n";
static uint8_t calibration_moving_reply[] = "CALMOVING\r\n";
static uint8_t calibration_error_reply[] = "CALERROR\r\n";

/* 反馈上报缓冲区与时间戳 */
static uint8_t feedback_reply[CONTROL_OUT_BUFFER_CAPACITY]; /* 反馈帧格式化缓冲区 */
static uint32_t last_feedback_tx_ms;                        /* 上次上报时间戳 — last feedback timestamp */

/*
 * 恢复中断状态。
 * 根据之前 __get_PRIMASK() 的返回值决定是否重新使能全局中断。
 * 如果进入临界区时中断本来就是关的，退出后也不应该打开。
 *
 * @param previous_primask  进入临界区前的 PRIMASK 寄存器值
 */
static void restore_interrupt_state(uint32_t previous_primask)
{
  if (previous_primask == 0U)
  {
    __enable_irq();
  }
}

/*
 * 将 receive_line 中的完整行发布到 pending_line。
 * 由 control_in_receive() 在检测到 '\n' 换行符时调用。
 *
 * 【优先级规则】
 *   "ESTOP" 具有最高优先级——使用独立的 pending_emergency_stop
 *   标志，不与普通命令共用 pending_line 队列，确保急停命令
 *   即使在有未处理普通命令的情况下也能立即响应。
 */
static void publish_received_line(void)
{
  /*
   * ESTOP 使用独立高优先级标志，即使已有普通命令排队也不会丢失。
   * 这里只发布请求，实际 CAN 操作仍在主循环 control_in() 中执行。
   */
  if (!receive_overflow
      && (receive_length == 5U)
      && ((receive_line[0] == 'E') || (receive_line[0] == 'e'))
      && ((receive_line[1] == 'S') || (receive_line[1] == 's'))
      && ((receive_line[2] == 'T') || (receive_line[2] == 't'))
      && ((receive_line[3] == 'O') || (receive_line[3] == 'o'))
      && ((receive_line[4] == 'P') || (receive_line[4] == 'p')))
  {
    pending_emergency_stop = true;
    receive_length = 0U;
    receive_overflow = false;
    return;
  }

  if (!pending_line_ready)
  {
    if (receive_overflow)
    {
      pending_line[0] = '\0';
      pending_line_invalid = true;
    }
    else
    {
      memcpy(pending_line, receive_line, receive_length);
      pending_line[receive_length] = '\0';
      pending_line_invalid = false;
    }
    pending_line_ready = true;
  }

  receive_length = 0U;
  receive_overflow = false;
}

/*
 * USB CDC 接收中断回调入口。
 *
 * 【调用方】CDC_Receive_FS() (USB CDC 接收完成中断回调)
 * 【上下文】中断上下文
 * 【功能】逐字节扫描接收数据，检测 '\n' 行尾后发布整行，其他字符
 *         追加到缓冲区，'\r' 被忽略。
 *
 * 【设计要点】
 *   - 此函数在中断中运行，除发布行 + 设置 volatile 标志外，不做任何
 *     阻塞操作或外部 API 调用，确保中断响应快速完成。
 *   - 缓冲区溢出时设置 receive_overflow 标志，该行将被丢弃。
 *
 * @param data   接收到的原始字节数组指针
 * @param length 字节数组长度
 */
void control_in_receive(const uint8_t *data, uint32_t length)
{
  uint32_t index;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    const uint8_t byte = data[index];

    if (byte == '\n')
    {
      publish_received_line();
    }
    else if (byte != '\r')
    {
      if (receive_length < (CONTROL_IN_LINE_CAPACITY - 1U))
      {
        receive_line[receive_length++] = (char)byte;
      }
      else
      {
        receive_overflow = true;
      }
    }
  }
}

/*
 * 跳过字符串中的空白字符（空格、制表符等）。
 * 用于命令解析中的容错处理，允许 "123.45, -15.30" 这样的输入。
 *
 * @param cursor  指向字符串指针的指针，解析后移动到第一个非空白字符
 */
static void skip_spaces(char **cursor)
{
  while (isspace((unsigned char)**cursor) != 0)
  {
    ++(*cursor);
  }
}

/*
 * 解析 "Yaw,Pitch" 格式的双轴角度目标字符串。
 *
 * 【格式】
 *   允许空白字符（空格/制表符）出现在逗号前后，如 "123.45 , -15.30"。
 *   格式错误包括：缺少数字、缺少逗号、逗号后有多余字符、超出范围、
 *   非有限浮点数 (NaN/Inf) 等。
 *
 * @param line             '\0' 结尾的命令行字符串
 * @param yaw_angle_deg    输出：解析后的 Yaw 目标角度（度）
 * @param pitch_angle_deg  输出：解析后的 Pitch 目标角度（度）
 * @return true  解析成功，角度已写入输出参数
 * @return false 格式错误或数值越界
 */
static bool parse_target_pair(char *line,
                              float *yaw_angle_deg,
                              float *pitch_angle_deg)
{
  char *cursor = line;
  char *end;
  float yaw;
  float pitch;

  skip_spaces(&cursor);
  yaw = strtof(cursor, &end);
  if (end == cursor)
  {
    return false;
  }

  cursor = end;
  skip_spaces(&cursor);
  if (*cursor != ',')
  {
    return false;
  }

  ++cursor;
  skip_spaces(&cursor);
  pitch = strtof(cursor, &end);
  if (end == cursor)
  {
    return false;
  }

  cursor = end;
  skip_spaces(&cursor);
  if (*cursor != '\0')
  {
    return false;
  }

  if (!isfinite(yaw) || !isfinite(pitch)
      || (yaw < CONTROL_IN_YAW_MIN_DEG)
      || (yaw > CONTROL_IN_YAW_MAX_DEG)
      || (pitch < CONTROL_IN_PITCH_MIN_DEG)
      || (pitch > CONTROL_IN_PITCH_MAX_DEG))
  {
    return false;
  }

  *yaw_angle_deg = yaw;
  *pitch_angle_deg = pitch;
  return true;
}

/*
 * 发送待处理 ACK 字符串到 USB CDC。
 * 如果 USB 发送成功，清除 pending_ack 标志；
 * 如果失败（USB 忙或未就绪），保留标志等待下一轮重试。
 */
static void transmit_pending_ack(void)
{
  uint8_t result;

  if (pending_ack == CONTROL_ACK_OK)
  {
    result = CDC_Transmit_FS(ok_reply, sizeof(ok_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_ERR)
  {
    result = CDC_Transmit_FS(err_reply, sizeof(err_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_ESTOPPED)
  {
    result = CDC_Transmit_FS(
        estopped_reply, sizeof(estopped_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_CLEARED)
  {
    result = CDC_Transmit_FS(
        cleared_reply, sizeof(cleared_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_LOCKED)
  {
    result = CDC_Transmit_FS(
        locked_reply, sizeof(locked_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_CALIBRATING)
  {
    result = CDC_Transmit_FS(
        calibrating_reply, sizeof(calibrating_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_CALHOMING)
  {
    result = CDC_Transmit_FS(
        calibration_homing_reply,
        sizeof(calibration_homing_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_CALIBRATED)
  {
    result = CDC_Transmit_FS(
        calibrated_reply, sizeof(calibrated_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_CALWAIT)
  {
    result = CDC_Transmit_FS(
        calibration_wait_reply,
        sizeof(calibration_wait_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_CALMOVING)
  {
    result = CDC_Transmit_FS(
        calibration_moving_reply,
        sizeof(calibration_moving_reply) - 1U);
  }
  else if (pending_ack == CONTROL_ACK_CALERROR)
  {
    result = CDC_Transmit_FS(
        calibration_error_reply,
        sizeof(calibration_error_reply) - 1U);
  }
  else
  {
    return;
  }

  if (result == USBD_OK)
  {
    pending_ack = CONTROL_ACK_NONE;
  }
}

/*
 * 主循环命令处理入口。
 *
 * 【执行流程】
 *   1. 重发上次未成功发送的 ACK（USB 可能当时忙）
 *   2. 在临界区中读取 pending_line 和 pending_emergency_stop 标志
 *   3. 优先级处理：
 *      a) 急停请求 → GM6020 + ChassisCAN 同时急停 → 回复 ESTOPPED
 *      b) CLEAR 命令 → 清除双模块急停 → 回复 CLEARED
 *      c) 角度目标 → 分别检查各轴在线和标定状态
 *                    → Yaw使用多圈目标, Pitch使用单圈目标
 *                    → 回复 OK/LOCKED
 *      d) 其他 → 回复 ERR
 *
 * 【调用时机】主循环 while(1) 每轮调用。
 */
void control_in(void)
{
  char line[CONTROL_IN_LINE_CAPACITY];
  bool line_invalid;
  bool line_available = false;
  bool emergency_stop_requested = false;
  bool command_applied;
  const GM6020_Feedback_t *yaw_feedback;
  const GM6020_Feedback_t *pitch_feedback;
  uint32_t previous_primask;
  float yaw_angle_deg;
  float pitch_angle_deg;

  transmit_pending_ack();

  previous_primask = __get_PRIMASK();
  __disable_irq();
  if (pending_emergency_stop)
  {
    pending_emergency_stop = false;
    pending_line_ready = false;
    emergency_stop_requested = true;
  }
  else if (pending_line_ready)
  {
    memcpy(line, pending_line, sizeof(line));
    line_invalid = pending_line_invalid;
    pending_line_ready = false;
    line_available = true;
  }
  restore_interrupt_state(previous_primask);

  if (emergency_stop_requested)
  {
    (void)GM6020_EmergencyStop();
    (void)ChassisCAN_EmergencyStop();
    pending_ack = CONTROL_ACK_ESTOPPED;
    transmit_pending_ack();
    return;
  }

  if (!line_available)
  {
    return;
  }

  if (!line_invalid && (strcmp(line, "CALSTATUS") == 0))
  {
    switch (GimbalCalibration_GetStatus())
    {
      case GIMBAL_CALIBRATION_CALIBRATED:
        pending_ack = CONTROL_ACK_CALIBRATED;
        break;

      case GIMBAL_CALIBRATION_WAITING_FEEDBACK:
        pending_ack = CONTROL_ACK_CALWAIT;
        break;

      case GIMBAL_CALIBRATION_WAITING_STILL:
        pending_ack = CONTROL_ACK_CALMOVING;
        break;

      case GIMBAL_CALIBRATION_SAMPLING:
        pending_ack = CONTROL_ACK_CALIBRATING;
        break;

      case GIMBAL_CALIBRATION_RETURNING_ZERO:
        pending_ack = CONTROL_ACK_CALHOMING;
        break;

      case GIMBAL_CALIBRATION_ERROR:
        pending_ack = CONTROL_ACK_CALERROR;
        break;

      default:
        pending_ack = CONTROL_ACK_CALERROR;
        break;
    }
  }
  else if (!line_invalid && (strcmp(line, "CLEAR") == 0))
  {
    GM6020_ClearEmergencyStop();
    ChassisCAN_ClearEmergencyStop();
    pending_ack = CONTROL_ACK_CLEARED;
  }
  else if (!line_invalid
           && parse_target_pair(
               line, &yaw_angle_deg, &pitch_angle_deg))
  {
    if (GM6020_IsEmergencyStopped())
    {
      pending_ack = CONTROL_ACK_LOCKED;
    }
    else
    {
      command_applied = false;
      yaw_feedback = GM6020_GetFeedback(GM6020_AXIS_YAW);
      pitch_feedback = GM6020_GetFeedback(GM6020_AXIS_PITCH);

      if (GimbalCalibration_IsAxisCalibrated(
              GM6020_AXIS_YAW)
          && (yaw_feedback != NULL)
          && yaw_feedback->online)
      {
        GM6020_SetMultiTurnTargetPosition(
            GM6020_AXIS_YAW, yaw_angle_deg);
        command_applied = true;
      }

      if (GimbalCalibration_IsAxisCalibrated(
              GM6020_AXIS_PITCH)
          && (pitch_feedback != NULL)
          && pitch_feedback->online)
      {
        GM6020_SetTargetPosition(
            GM6020_AXIS_PITCH, pitch_angle_deg);
        command_applied = true;
      }

      pending_ack = command_applied
          ? CONTROL_ACK_OK
          : CONTROL_ACK_LOCKED;
    }
  }
  else
  {
    pending_ack = CONTROL_ACK_ERR;
  }

  transmit_pending_ack();
}

/*
 * 将角度（度）转换为百分度 (centidegree, 0.01°) 的有符号整数。
 * 用于格式化输出时精确到小数点后两位，避免 printf 浮点格式的
 * 二进制表示误差导致的显示抖动。
 *
 * 例: 123.45° → 12345 (centidegrees)
 *     -15.30° → -1530
 *
 * @param angle_deg  角度值（度）
 * @return           百分度整数值，溢出时返回 INT32_MAX/MIN
 */
static int32_t angle_to_centidegrees(float angle_deg)
{
  const float scaled_angle = angle_deg * 100.0f;

  if (!isfinite(scaled_angle))
  {
    return 0;
  }
  if (scaled_angle >= (float)INT32_MAX)
  {
    return INT32_MAX;
  }
  if (scaled_angle <= (float)INT32_MIN)
  {
    return INT32_MIN;
  }

  return (int32_t)((scaled_angle >= 0.0f)
      ? (scaled_angle + 0.5f)
      : (scaled_angle - 0.5f));
}

/*
 * 将角度格式化为 "xxx.xx" 或 "-xxx.xx" 的定点字符串。
 * 使用百分度整数除法避免浮点 printf 的舍入误差。
 *
 * @param buffer     输出缓冲区
 * @param capacity   缓冲区容量
 * @param angle_deg  要格式化的角度（度）
 */
static void format_angle(char *buffer, size_t capacity,
                         float angle_deg)
{
  const int32_t centidegrees = angle_to_centidegrees(angle_deg);
  const uint32_t magnitude = (centidegrees < 0)
      ? (uint32_t)(-(int64_t)centidegrees)
      : (uint32_t)centidegrees;

  (void)snprintf(
      buffer,
      capacity,
      "%s%lu.%02lu",
      (centidegrees < 0) ? "-" : "",
      (unsigned long)(magnitude / 100U),
      (unsigned long)(magnitude % 100U));
}

/*
 * 主循环反馈上报入口。
 *
 * 【周期】每 CONTROL_OUT_PERIOD_MS (1000 ms) 上报一次。
 * 【格式】"FB,<yaw_deg>,<pitch_deg>,<yaw_rpm>,<pitch_rpm>,<yaw_online>,<pitch_online>,<estop>\r\n"
 *         字段说明：
 *           yaw_deg/pitch_deg: 角度 (°)，精确到 0.01，通过百分度格式化避免浮点误差
 *           yaw_rpm/pitch_rpm: 电机转速 (rpm)
 *           yaw_online/pitch_online: 电机在线状态 (1=在线, 0=离线)
 *           estop: 急停状态 (1=急停中, 0=正常)
 *
 * 【注意】Yaw/Pitch 均上报相对于标定机械零点的角度；
 *         Yaw 保留累计多圈信息。
 */
void control_out(void)
{
  const uint32_t now = HAL_GetTick();
  const GM6020_Feedback_t *yaw_feedback;
  const GM6020_Feedback_t *pitch_feedback;
  float yaw_multi_turn_deg;
  float pitch_position_deg;
  char yaw_angle[20];
  char pitch_angle[20];
  int length;

  if ((uint32_t)(now - last_feedback_tx_ms)
      < CONTROL_OUT_PERIOD_MS)
  {
    return;
  }
  last_feedback_tx_ms = now;

  yaw_feedback = GM6020_GetFeedback(GM6020_AXIS_YAW);
  pitch_feedback = GM6020_GetFeedback(GM6020_AXIS_PITCH);
  if ((yaw_feedback == NULL) || (pitch_feedback == NULL))
  {
    return;
  }

  if (!GM6020_GetMultiTurnPosition(
          GM6020_AXIS_YAW, &yaw_multi_turn_deg))
  {
    yaw_multi_turn_deg = 0.0f;
  }
  if (!GM6020_GetMultiTurnPosition(
          GM6020_AXIS_PITCH, &pitch_position_deg))
  {
    pitch_position_deg = 0.0f;
  }

  format_angle(yaw_angle, sizeof(yaw_angle),
               yaw_multi_turn_deg);
  format_angle(pitch_angle, sizeof(pitch_angle),
               pitch_position_deg);

  length = snprintf(
      (char *)feedback_reply,
      sizeof(feedback_reply),
      "FB,%s,%s,%d,%d,%u,%u,%u\r\n",
      yaw_angle,
      pitch_angle,
      (int)yaw_feedback->speed_rpm,
      (int)pitch_feedback->speed_rpm,
      yaw_feedback->online ? 1U : 0U,
      pitch_feedback->online ? 1U : 0U,
      GM6020_IsEmergencyStopped() ? 1U : 0U);

  if ((length > 0)
      && ((uint32_t)length < sizeof(feedback_reply)))
  {
    (void)CDC_Transmit_FS(feedback_reply, (uint16_t)length);
  }
}
