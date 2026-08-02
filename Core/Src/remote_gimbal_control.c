/**
 * ===========================================================================
 * @file    remote_gimbal_control.c
 * @brief   将 DBUS 摇杆量映射为云台位置目标和底盘速度目标
 * ===========================================================================
 *
 * 控制方式：
 *   CH0 → Yaw 目标角速度 → 积分得到 Yaw 多圈位置目标
 *   CH1 → Pitch 目标角速度 → 积分得到 Pitch 单圈位置目标
 *   CH2 → 底盘横移速度 vy
 *   CH3 → 底盘前进速度 vx
 *   S1上挡 → 云台与底盘急停；S1中/下挡 → 解除急停（临时映射）
 *   S2上/中/下挡 → 底盘跟随/不跟随/小陀螺模式
 *   S1下+S2上：拨轮下=连发、拨轮上边沿=单发；S1下+S2中+拨轮下=退弹
 *
 * 每个轴独立检查以下接管条件：
 *   - DBUS 最近100 ms内收到过合法帧；
 *   - 该轴已经独立完成上电机械零位标定；
 *   - 该轴GM6020在线并且全局急停未锁存。
 *
 * 条件不满足时不会继续积分。恢复控制时从电机实时位置重新建立目标，
 * 防止遥控器重连、急停解除或电机恢复后跳到旧目标。
 * ===========================================================================
 */

#include "remote_gimbal_control.h"

#include "chassis_can.h"
#include "config/control_tuning.h"
#include "config/gimbal_params.h"
#include "dbus.h"
#include "dual_m3508.h"
#include "feeder_motor.h"
#include "gimbal_calibration.h"
#include "motor_control.h"
#include "pitch_fusion.h"

#include <stdbool.h>
#include <stdint.h>

/* ─── 遥控器通道映射 (Remote Control Channel Mapping) ─── */
/* 上板视角：右摇杆横向 CH0 → Yaw，纵向 CH1 → Pitch */
#define REMOTE_GIMBAL_YAW_CHANNEL          0U    /* Yaw 通道索引 — yaw channel index (CH0) */
#define REMOTE_GIMBAL_PITCH_CHANNEL        1U    /* Pitch 通道索引 — pitch channel index (CH1) */

/* 左摇杆：横向CH2 → vy，纵向CH3 → vx。 */
#define REMOTE_CHASSIS_LATERAL_CHANNEL      2U
#define REMOTE_CHASSIS_FORWARD_CHANNEL      3U

/*
 * ─── 临时S1安全开关映射 (Temporary S1 Safety Mapping) ───
 *
 * 当前测试阶段：
 *   S1上挡     → 锁存云台和底盘急停
 *   S1中/下挡  → 解除云台和底盘急停
 *
 * 此映射是临时设置，后续功能规划改变时可整体移除或替换。
 */
#define REMOTE_GIMBAL_TEMP_S1_ESTOP_ENABLE  1U
#define REMOTE_GIMBAL_S1_INDEX              0U
#define REMOTE_CHASSIS_MODE_SWITCH_INDEX     1U
#define REMOTE_GIMBAL_SWITCH_UNKNOWN        0U

/* ─── 摇杆曲线参数 (Joystick Curve Parameters) ─── */
/*
 * 死区 (Deadzone): 去中心值 ±30 内不动作，滤除摇杆回中抖动。
 * DBUS 去中心值范围约 -660 ~ +660，30/660 ≈ 4.5% 死区。
 */
#define REMOTE_GIMBAL_DEADZONE TUNE_PITCH_REMOTE_DEADZONE

/*
 * 满量程 (Full Scale): 去中心值的最大幅值。
 * DBUS 通道值 364~1684 → 去中心 = -660 ~ +660。
 */
#define REMOTE_GIMBAL_FULL_SCALE TUNE_PITCH_REMOTE_FULL_SCALE

/* ─── 角速度映射 (Angular Rate Mapping) ─── */
/* 满杆时对应的云台逻辑目标角速度 (°/s) */
#define REMOTE_GIMBAL_YAW_MAX_RATE_DPS    180.0f /* Yaw 满杆角速度 — max yaw rate (degrees/sec) */
#define REMOTE_GIMBAL_PITCH_MAX_RATE_DPS \
    TUNE_PITCH_REMOTE_MAX_RATE_DPS

/*
 * 安装方向修正 (Mounting Direction Correction)。
 * 若实机运动方向与遥控器方向相反，只需把对应值改为 -1.0f。
 * 例: 推杆向右 → 云台向左 → 设 YAW_DIRECTION = -1.0f
 */
#define REMOTE_GIMBAL_YAW_DIRECTION        -1.0f  /* Yaw 方向修正 — +1=正向, -1=反向 */
#define REMOTE_GIMBAL_PITCH_DIRECTION      -1.0f  /* Pitch 安装方向与遥控器相反 */

/*
 * 底盘速度映射。满杆对应±5.0 m/s，发送模块再转换为Q10原始值±5120。
 * 当前坐标约定：vx向前为正，vy向左为正。
 */
#define REMOTE_CHASSIS_MAX_FORWARD_MPS       1.0f
#define REMOTE_CHASSIS_MAX_LATERAL_MPS      -1.0f
#define REMOTE_CHASSIS_FORWARD_DIRECTION     1.0f
#define REMOTE_CHASSIS_LATERAL_DIRECTION    -1.0f

/* ─── 软件限位 (Software Limits) ─── */
/* 遥控器积分位置目标的软件范围，防止目标角度无限累积溢出 */
#define REMOTE_GIMBAL_YAW_MIN_DEG      (-36000.0f) /* Yaw 最小角度 — min yaw angle (±100圈) */
#define REMOTE_GIMBAL_YAW_MAX_DEG        36000.0f  /* Yaw 最大角度 — max yaw angle */
#define REMOTE_GIMBAL_PITCH_MIN_DEG PITCH_MIN_ANGLE_DEG
#define REMOTE_GIMBAL_PITCH_MAX_DEG PITCH_MAX_ANGLE_DEG

/*
 * 单次积分时间上限 (Max Integration Delta)。
 * 限制单次积分时间 ≤ 20 ms，防止主循环长时间阻塞（如 Flash 擦除）
 * 后恢复时，摇杆目标的单次积分跃变过大导致云台突然跳动。
 * 如果真实时间差 > 20 ms，只按 20 ms 计算积分值。
 */
#define REMOTE_GIMBAL_MAX_DELTA_MS           20U   /* 单次积分上限 — max delta per integration (ms) */

/*
 * ─── 遥控器控制上下文 (Remote Control Context) ───
 * 保存 Yaw/Pitch 的积分目标角度、时间戳、激活状态。
 * 整个模块使用一个静态全局实例。
 */
typedef struct
{
  float yaw_target_deg;        /* Yaw 积分目标角度 — integrated yaw target (degrees) */
  float pitch_target_deg;      /* Pitch 积分目标角度 — integrated pitch target (degrees) */
  uint32_t last_process_ms;    /* 上次处理时间戳 — last process timestamp (ms) */
  bool axis_active[GM6020_AXIS_COUNT]; /* 各轴独立接管激活标志 */
  bool pitch_fusion_was_ready; /* Pitch反馈源变化时重新锚定遥控目标 */
  uint8_t last_s1_position;    /* 上次S1挡位，用于中/下挡解除沿检测 */
  uint8_t feeder_dial_zone;    /* 带迟滞的拨轮区域：0中、1下、2上 */
} RemoteGimbalControlContext_t;

static RemoteGimbalControlContext_t remote_control;

/*
 * ─── 内部工具函数 (Internal Utility Functions) ───
 */

/*
 * 浮点数限幅 (Float Clamp)。
 * 将 value 限制在 [minimum, maximum] 闭区间内。
 */
static float clamp_float(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static float approach_float(float current, float target,
                            float maximum_step)
{
  if (current < target - maximum_step)
  {
    return current + maximum_step;
  }
  if (current > target + maximum_step)
  {
    return current - maximum_step;
  }
  return target;
}

enum
{
  FEEDER_DIAL_CENTER = 0U,
  FEEDER_DIAL_DOWN,
  FEEDER_DIAL_UP
};

static uint8_t update_feeder_dial_zone(
    const DBUS_Data_t *dbus_data)
{
  int32_t dial;

  if ((dbus_data == NULL) || !dbus_data->dial_valid)
  {
    remote_control.feeder_dial_zone = FEEDER_DIAL_CENTER;
    return FEEDER_DIAL_CENTER;
  }

  dial = (int32_t)dbus_data->centered_dial
      * TUNE_DBUS_DIAL_DOWN_DIRECTION;
  if ((dial <= TUNE_DBUS_DIAL_RELEASE_THRESHOLD)
      && (dial >= -TUNE_DBUS_DIAL_RELEASE_THRESHOLD))
  {
    remote_control.feeder_dial_zone = FEEDER_DIAL_CENTER;
  }
  else if (dial >= TUNE_DBUS_DIAL_TRIGGER_THRESHOLD)
  {
    remote_control.feeder_dial_zone = FEEDER_DIAL_DOWN;
  }
  else if (dial <= -TUNE_DBUS_DIAL_TRIGGER_THRESHOLD)
  {
    remote_control.feeder_dial_zone = FEEDER_DIAL_UP;
  }

  return remote_control.feeder_dial_zone;
}

/*
 * 将去中心通道值归一化到 [-1.0, +1.0] (Normalize centered channel to [-1.0, +1.0])。
 *
 * 【映射曲线】
 *   Deadzone (dead)    : centered ∈ [-30, +30] → 0.0
 *   Transition (ramp)  : centered ∈ (±30, ±660] → 线性映射到 (±0, ±1]
 *
 * 【优点】
 *   死区边沿从 0 平滑起步 (smooth ramp from zero at deadzone edge)，
 *   避免刚越过死区就产生速度阶跃 (velocity step)，操作手感更线性。
 *
 * @param centered_value  DBUS 去中心值 (约 -660 ~ +660)
 * @return               归一化输出 [-1.0, +1.0]
 */
static float normalize_channel(int16_t centered_value)
{
  int32_t magnitude = centered_value;
  float normalized;

  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }

  if (magnitude <= REMOTE_GIMBAL_DEADZONE)
  {
    return 0.0f;
  }

  normalized =
      (float)(magnitude - REMOTE_GIMBAL_DEADZONE)
      / (float)(REMOTE_GIMBAL_FULL_SCALE
                - REMOTE_GIMBAL_DEADZONE);
  normalized = clamp_float(normalized, 0.0f, 1.0f);

  return (centered_value < 0) ? -normalized : normalized;
}

/*
 * 处理临时S1急停映射。
 *
 * 上挡使用电平优先策略：只要S1保持上挡，任何其他来源解除急停后都会
 * 在下一控制周期重新急停。中/下挡只在进入该挡位时解除一次，避免
 * 持续覆盖USB ESTOP等其他急停来源。
 */
static void process_temporary_s1_safety(
    const DBUS_Data_t *dbus_data)
{
#if REMOTE_GIMBAL_TEMP_S1_ESTOP_ENABLE
  uint8_t s1_position;

  if ((dbus_data == NULL)
      || !dbus_data->online
      || !dbus_data->last_frame_valid)
  {
    return;
  }

  s1_position =
      dbus_data->switch_value[REMOTE_GIMBAL_S1_INDEX];

  if (s1_position == DBUS_SWITCH_UP)
  {
    if (!GM6020_IsEmergencyStopped())
    {
      (void)GM6020_EmergencyStop();
    }
    if (!ChassisCAN_IsEmergencyStopped())
    {
      (void)ChassisCAN_EmergencyStop();
    }
    if (!FeederMotor_IsEmergencyStopped())
    {
      (void)FeederMotor_EmergencyStop();
    }
    if (!DualM3508_IsEmergencyStopped())
    {
      (void)DualM3508_EmergencyStop();
    }
  }
  else if (((s1_position == DBUS_SWITCH_MIDDLE)
            || (s1_position == DBUS_SWITCH_DOWN))
           && (s1_position != remote_control.last_s1_position))
  {
    GM6020_ClearEmergencyStop();
    ChassisCAN_ClearEmergencyStop();
    FeederMotor_ClearEmergencyStop();
    if (DualM3508_IsEmergencyStopped())
    {
      DualM3508_ClearEmergencyStop();
    }
  }

  remote_control.last_s1_position = s1_position;
#else
  (void)dbus_data;
#endif
}

/*
 * 将左摇杆映射为底盘平移速度。
 *
 * DBUS在线且最近帧合法、系统未急停时：
 *   CH3 → vx，CH2 → vy，wz保持0；
 *   S2上/中/下 → FOLLOW/NO_FOLLOW/SPIN。
 *
 * DBUS掉线、帧非法或急停时提交零速度+SOFTWARE_OFF。若底盘急停已经
 * 锁存，ChassisCAN_SetCommand()会拒绝普通命令，但急停函数中缓存的
 * 零速度命令仍会继续周期发送。
 */
static void process_chassis_control(
    const DBUS_Data_t *dbus_data)
{
  Chassis_Ctrl_Cmd_s command =
  {
    .vx = 0.0f,
    .vy = 0.0f,
    .wz = 0.0f,
    .offset_angle_rad = 0.0f,
    .chassis_mode = CHASSIS_MODE_SOFTWARE_OFF
  };

  if ((dbus_data != NULL)
      && dbus_data->online
      && dbus_data->last_frame_valid
      && !GM6020_IsEmergencyStopped())
  {
    command.vx =
        normalize_channel(
            dbus_data->centered_channel[
                REMOTE_CHASSIS_FORWARD_CHANNEL])
        * REMOTE_CHASSIS_FORWARD_DIRECTION
        * REMOTE_CHASSIS_MAX_FORWARD_MPS;
    command.vy =
        normalize_channel(
            dbus_data->centered_channel[
                REMOTE_CHASSIS_LATERAL_CHANNEL])
        * REMOTE_CHASSIS_LATERAL_DIRECTION
        * REMOTE_CHASSIS_MAX_LATERAL_MPS;

    switch (dbus_data->switch_value[
        REMOTE_CHASSIS_MODE_SWITCH_INDEX])
    {
      case DBUS_SWITCH_UP:
        command.chassis_mode = CHASSIS_MODE_FOLLOW;
        break;
      case DBUS_SWITCH_MIDDLE:
        command.chassis_mode = CHASSIS_MODE_NO_FOLLOW;
        break;
      case DBUS_SWITCH_DOWN:
        command.chassis_mode = CHASSIS_MODE_SPIN;
        break;
      default:
        command.vx = 0.0f;
        command.vy = 0.0f;
        command.chassis_mode = CHASSIS_MODE_SOFTWARE_OFF;
        break;
    }
  }

  (void)ChassisCAN_SetCommand(&command);
}

/*
 * 仅S1下+S2上（正常发射模式）请求启动双M3508摩擦轮。
 * 退弹模式和其他挡位均关闭摩擦轮。
 * 故障锁存由DualM3508模块内部处理，必须回到S1中挡后才允许复位。
 */
static void process_friction_control(
    const DBUS_Data_t *dbus_data)
{
  bool enabled;

  if ((dbus_data == NULL)
      || !dbus_data->online
      || !dbus_data->last_frame_valid)
  {
    DualM3508_DisableUntilOff();
    return;
  }

  enabled =
      !GM6020_IsEmergencyStopped()
      && !DualM3508_IsEmergencyStopped()
      && (dbus_data->switch_value[REMOTE_GIMBAL_S1_INDEX]
          == DBUS_SWITCH_DOWN)
      && (dbus_data->switch_value[
              REMOTE_CHASSIS_MODE_SWITCH_INDEX]
          == DBUS_SWITCH_UP);

  DualM3508_SetEnabled(enabled);
}

/*
 * 拨弹盘拨轮映射：
 *   S1下 + S2上 + 拨轮下 -> 连发
 *   S1下 + S2上 + 拨轮上 -> 中心到上方边沿触发单发
 *   S1下 + S2中 + 拨轮下 -> 低速反转退弹（摩擦轮关闭）
 *   其他有效组合          -> NEUTRAL
 *   DBUS/拨轮无效或S1非下 -> DISABLE
 */
static void process_feeder_control(
    const DBUS_Data_t *dbus_data)
{
  FeederRemoteCommand_t command = FEEDER_REMOTE_DISABLE;
  uint8_t dial_zone = FEEDER_DIAL_CENTER;

  if ((dbus_data != NULL)
      && dbus_data->online
      && dbus_data->last_frame_valid
      && dbus_data->dial_valid
      && !GM6020_IsEmergencyStopped()
      && (dbus_data->switch_value[REMOTE_GIMBAL_S1_INDEX]
          == DBUS_SWITCH_DOWN))
  {
    dial_zone = update_feeder_dial_zone(dbus_data);
    switch (dbus_data->switch_value[
        REMOTE_CHASSIS_MODE_SWITCH_INDEX])
    {
      case DBUS_SWITCH_UP:
        if (dial_zone == FEEDER_DIAL_DOWN)
        {
          command = FEEDER_REMOTE_CONTINUOUS;
        }
        else if (dial_zone == FEEDER_DIAL_UP)
        {
          command = FEEDER_REMOTE_SINGLE;
        }
        else
        {
          command = FEEDER_REMOTE_NEUTRAL;
        }
        break;
      case DBUS_SWITCH_MIDDLE:
        command =
            (dial_zone == FEEDER_DIAL_DOWN)
            ? FEEDER_REMOTE_REVERSE
            : FEEDER_REMOTE_NEUTRAL;
        break;
      case DBUS_SWITCH_DOWN:
      default:
        command = FEEDER_REMOTE_NEUTRAL;
        break;
    }
  }
  else
  {
    (void)update_feeder_dial_zone(NULL);
  }

  FeederMotor_SetRemoteCommand(command);
}

/*
 * 判断指定轴是否具备遥控器接管条件。
 *
 * 四个必要条件全部满足才返回true：
 *   1. DBUS 在线 (dbus_data->online) — 接收机有信号
 *   2. 指定轴零位标定完成 — 逻辑角度0°对应传感器零点
 *   3. 云台未急停 — 没有被锁存式急停阻止
 *   4. 指定轴在线 — 收到该轴有效CAN反馈
 *
 * 另一轴离线或尚未标定不会影响本轴。
 */
static bool remote_axis_is_available(
    const DBUS_Data_t *dbus_data,
    GM6020_Axis_t axis)
{
  const GM6020_Feedback_t *feedback;

  if ((dbus_data == NULL)
      || !dbus_data->online
      || !GimbalCalibration_IsAxisCalibrated(axis)
      || GM6020_IsEmergencyStopped())
  {
    return false;
  }

  feedback = GM6020_GetFeedback(axis);
  return (feedback != NULL) && feedback->online;
}

/*
 * ─── 公有 API (Public API) ───
 */

/*
 * 初始化遥控器目标积分器。
 * 清零目标角度和时间戳，等待首次接管时从电机实时位置同步。
 */
void RemoteGimbalControl_Init(void)
{
  remote_control.yaw_target_deg = 0.0f;
  remote_control.pitch_target_deg = 0.0f;
  remote_control.last_process_ms = HAL_GetTick();
  remote_control.axis_active[GM6020_AXIS_YAW] = false;
  remote_control.axis_active[GM6020_AXIS_PITCH] = false;
  remote_control.pitch_fusion_was_ready = false;
  remote_control.last_s1_position =
      REMOTE_GIMBAL_SWITCH_UNKNOWN;
  remote_control.feeder_dial_zone =
      FEEDER_DIAL_CENTER;
}

/*
 * 主循环遥控器控制处理 (Main Loop Remote Control Processing)。
 *
 * 【控制流程】
 *   1. 分别检查两轴接管条件，不满足的轴单独deactivate
 *   2. 从实时位置建立积分起点，再积分角速度下传位置目标
 *
 * 【积分公式】
 *   new_target = old_target + normalized * direction * max_rate * (delta_ms / 1000)
 *
 *   例: 摇杆半推 (normalized=0.5), Yaw max_rate=360°/s, delta=1ms
 *        → 增量 = 0.5 * 1.0 * 360 * 0.001 = 0.18° / 周期
 *
 * 【恢复时重新加锁策略】
 *   当某轴接管条件从false变为true时，只为该轴读取实时位置并重新
 *   建立积分起点；另一轴继续运行，不复用离线轴的旧目标。
 */
void RemoteGimbalControl_Process(void)
{
  const uint32_t now = HAL_GetTick();
      /* 当前系统时间，用于对摇杆角速度做时间积分。 */
  const DBUS_Data_t *dbus_data = DBUS_GetData();
      /* 最近一次已解析的遥控器快照，不直接读取DMA缓冲区。 */
  bool yaw_available;
      /* Yaw是否满足在线、标定、未急停等接管条件。 */
  bool pitch_available;
      /* Pitch是否满足自己的接管条件。 */
  bool pitch_fusion_ready;
      /* Pitch当前是否可以把融合角/速率交给电机闭环。 */
  uint32_t delta_ms;
      /* 两次处理之间的时间，限制到最大20 ms。 */
  float yaw_input;
      /* Yaw摇杆去中心并归一化后的-1~+1输入。 */
  float pitch_input;
      /* Pitch摇杆去中心并归一化后的-1~+1输入。 */
  float yaw_rate_dps;
  float pitch_rate_dps;
  float previous_target_deg;
      /* 目标更新前的角度，用来反推出本拍实际目标速度前馈。 */
  float pitch_feedback_deg;
      /* Pitch融合/编码器控制反馈角度。 */
  float pitch_feedback_rpm;
      /* Pitch融合/编码器控制反馈转速；这里只用于接口接收和状态判断。 */

  process_temporary_s1_safety(dbus_data);
  process_chassis_control(dbus_data);
  process_friction_control(dbus_data);
  process_feeder_control(dbus_data);

  yaw_available = remote_axis_is_available(
      dbus_data, GM6020_AXIS_YAW);
  pitch_available = remote_axis_is_available(
      dbus_data, GM6020_AXIS_PITCH);
  pitch_fusion_ready = PitchFusion_GetControlFeedback(
      &pitch_feedback_deg,
      &pitch_feedback_rpm);

  /*
   * 编码器与融合反馈相互切换时，旧遥控积分目标属于旧参考系。
   * 强制Pitch轴下一步从新反馈当前位置重新接管，避免恢复旧运动。
   */
  if (pitch_fusion_ready
      != remote_control.pitch_fusion_was_ready)
  {
    remote_control.axis_active[GM6020_AXIS_PITCH] = false;
    remote_control.pitch_fusion_was_ready =
        pitch_fusion_ready;
  }

  if (!yaw_available)
  {
    remote_control.axis_active[GM6020_AXIS_YAW] = false;
    GM6020_SetPositionFeedforward(
        GM6020_AXIS_YAW, 0.0f, 0.0f);
  }
  if (!pitch_available)
  {
    remote_control.axis_active[GM6020_AXIS_PITCH] = false;
    GM6020_SetPositionFeedforward(
        GM6020_AXIS_PITCH, 0.0f, 0.0f);
  }

  delta_ms = (uint32_t)(now - remote_control.last_process_ms);
  remote_control.last_process_ms = now;
  if (delta_ms > REMOTE_GIMBAL_MAX_DELTA_MS)
  {
    delta_ms = REMOTE_GIMBAL_MAX_DELTA_MS;
  }

  if (!yaw_available && !pitch_available)
  {
    return;
  }

  if (yaw_available)
  {
    if (!remote_control.axis_active[GM6020_AXIS_YAW])
    {
      if (GM6020_GetMultiTurnPosition(
              GM6020_AXIS_YAW,
              &remote_control.yaw_target_deg))
      {
        remote_control.axis_active[GM6020_AXIS_YAW] = true;
        GM6020_SetMultiTurnTargetPosition(
            GM6020_AXIS_YAW,
            remote_control.yaw_target_deg);
        GM6020_SetPositionFeedforward(
            GM6020_AXIS_YAW, 0.0f, 0.0f);
      }
    }
    else if (delta_ms > 0U)
    {
      yaw_input = normalize_channel(
          dbus_data->centered_channel[
              REMOTE_GIMBAL_YAW_CHANNEL]);
      yaw_rate_dps =
          yaw_input
          * REMOTE_GIMBAL_YAW_DIRECTION
          * REMOTE_GIMBAL_YAW_MAX_RATE_DPS;//摇杆灵敏度?
      previous_target_deg = remote_control.yaw_target_deg;
      remote_control.yaw_target_deg +=
          yaw_rate_dps
          * (float)delta_ms
          / 1000.0f;
      remote_control.yaw_target_deg = clamp_float(
          remote_control.yaw_target_deg,
          REMOTE_GIMBAL_YAW_MIN_DEG,
          REMOTE_GIMBAL_YAW_MAX_DEG);
      GM6020_SetMultiTurnTargetPosition(
          GM6020_AXIS_YAW,
          remote_control.yaw_target_deg);
      yaw_rate_dps =
          (remote_control.yaw_target_deg - previous_target_deg)
          * 1000.0f / (float)delta_ms;
      GM6020_SetPositionFeedforward(
          GM6020_AXIS_YAW,
          yaw_rate_dps / 6.0f,
          0.0f);
    }
  }

  if (pitch_available)
  {
    if (!remote_control.axis_active[GM6020_AXIS_PITCH])
    {
      bool position_available = pitch_fusion_ready;

      if (pitch_fusion_ready)
      {
        remote_control.pitch_target_deg =
            pitch_feedback_deg;
      }
      else
      {
        position_available = GM6020_GetMultiTurnPosition(
            GM6020_AXIS_PITCH,
            &remote_control.pitch_target_deg);
      }

      if (position_available)
      {
        remote_control.axis_active[GM6020_AXIS_PITCH] = true;
        GM6020_SetTargetPosition(
            GM6020_AXIS_PITCH,
            remote_control.pitch_target_deg);
        GM6020_SetPositionFeedforward(
            GM6020_AXIS_PITCH, 0.0f, 0.0f);
      }
    }
    else if (delta_ms > 0U)
    {
      pitch_input = normalize_channel(
          dbus_data->centered_channel[
              REMOTE_GIMBAL_PITCH_CHANNEL]);
      pitch_rate_dps =
          pitch_input
          * REMOTE_GIMBAL_PITCH_DIRECTION
          * REMOTE_GIMBAL_PITCH_MAX_RATE_DPS;
      previous_target_deg = remote_control.pitch_target_deg;
      if (pitch_input != 0.0f)
      {
        remote_control.pitch_target_deg +=
            pitch_rate_dps
            * (float)delta_ms
            / 1000.0f;
      }
      else
      {
        /*
         * 摇杆回中后，目标只负责平滑回到传感器水平0°。
         * 目标到达0°后保持不变；车辆颠簸引起的实际角度和角速度变化
         * 由1 kHz融合反馈立即进入两级PID，不受此回正斜坡限制。
         */
        remote_control.pitch_target_deg = approach_float(
            remote_control.pitch_target_deg,
            TUNE_PITCH_LEVEL_TARGET_DEG,
            TUNE_PITCH_LEVEL_RETURN_RATE_DPS
              * (float)delta_ms / 1000.0f);
      }
      remote_control.pitch_target_deg = clamp_float(
          remote_control.pitch_target_deg,
          REMOTE_GIMBAL_PITCH_MIN_DEG,
          REMOTE_GIMBAL_PITCH_MAX_DEG);
      GM6020_SetTargetPosition(
          GM6020_AXIS_PITCH,
          remote_control.pitch_target_deg);
      pitch_rate_dps =
          (remote_control.pitch_target_deg
           - previous_target_deg)
          * 1000.0f / (float)delta_ms;
      GM6020_SetPositionFeedforward(
          GM6020_AXIS_PITCH,
          pitch_rate_dps / 6.0f,
          0.0f);
    }
  }
}
