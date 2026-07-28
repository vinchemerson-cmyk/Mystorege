# STM32F407 双轴 GM6020 云台控制

基于 STM32F407 和 HAL 库的双轴云台控制工程。Yaw、Pitch 两台 GM6020
共用 CAN1，通过位置环与速度环串级 PID 输出合并电流命令。

## 当前功能

- Yaw、Pitch 双轴 GM6020 控制
- CAN1 反馈解析与 `0x1FE` 合并电流发送
- 单圈编码器多圈累计
- 目标角度劣弧（最短路径）处理
- 位置环、速度环串级 PID
- 等待反馈、位置控制、速度调试、故障四状态控制
- 反馈超时后零电流保护
- USB CDC 双轴位置指令输入
- CAN2底盘控制命令周期发送

## 硬件与通信

| 功能 | 配置 |
| --- | --- |
| MCU | STM32F407 |
| CAN1 | PD0 = RX，PD1 = TX，1 Mbps |
| USART6 | PG9 = RX，PG14 = TX，115200-8-N-1 |
| USB CDC | `yaw,pitch\r\n`，角度单位为度 |
| CAN2 | PB5 = RX，PB6 = TX，1 Mbps |
| Yaw反馈 | 标准帧 `0x205` |
| Pitch反馈 | 标准帧 `0x206` |
| 电流命令 | 标准帧 `0x1FE`，DLC = 8 |

`0x1FE` 数据分配：

| 字节 | 内容 |
| --- | --- |
| `DATA[0:1]` | Yaw电流，大端序 |
| `DATA[2:3]` | Pitch电流，大端序 |
| `DATA[4:7]` | 保留，填0 |

## 程序流程

```text
USB CDC接收
    ↓
control_in()
    ↓
GM6020_Process()
    ├─ 读取0x205/0x206反馈
    ├─ 更新多圈编码器
    ├─ 运行状态机
    ├─ 位置环输出目标转速
    ├─ 速度环输出目标电流
    └─ 发送0x1FE合并电流帧
```

## 双轴串口协议

开发板上电并收到两台电机的首帧 CAN 反馈后，Yaw 和 Pitch 会分别以
最短路径转到各自的编码器原始零点；单轴最多转动半圈。

发送 ASCII 文本 `yaw,pitch\r\n`。Yaw 为相对于编码器原始零点的累计
多圈角度，范围为 `-36000~36000` 度；例如 `360` 表示正向1圈，
`1080` 表示正向3圈，`-720` 表示反向2圈。Pitch 仍为单圈位置目标，
范围为 `-30~30` 度。命令有效时回复 `OK\r\n`，否则回复 `ERR\r\n`。

急停指令为 `ESTOP\r\n`，开发板回复 `ESTOPPED\r\n`。该指令锁存后会
立即将两路 GM6020 电流置零，并通过 CAN2 发送零速度和底盘软件断电
模式；后续位置命令只回复 `LOCKED\r\n`，不会驱动电机。发送
`CLEAR\r\n` 可解除急停，开发板回复 `CLEARED\r\n`。解除后云台锁定
当前位置，底盘仍保持软件断电，不会自动恢复急停前的动作。

开发板每 100 ms 主动上报一次：

```text
FB,yaw角度,pitch角度,yaw转速,pitch转速,yaw在线,pitch在线,急停\r\n
```

例如：`FB,732.34,-5.67,100,-20,1,1,0\r\n`。Yaw 上报相对于编码器零点
累计的多圈角度；Pitch 上报当前编码器角度。角度单位为度，转速单位为
rpm，在线状态 `1` 表示近期收到对应电机的 CAN 反馈，急停状态 `1`
表示急停已锁存。

## 底盘CAN2协议

CAN2每10 ms发送两帧。标准帧`0x300`的4个字段均为大端序`int16_t`
Q10定点数：

| 字节 | 内容 |
| --- | --- |
| `0:1` | `vx × 1024` |
| `2:3` | `vy × 1024` |
| `4:5` | `wz × 1024` |
| `6:7` | `offset_angle_rad × 1024` |

标准帧`0x301`的Byte 0为底盘模式：`0`软件断电、`1`跟随、
`2`不跟随、`3`小陀螺。调用`ChassisCAN_SetCommand()`更新下一周期发送值。

## 主要文件

| 路径 | 用途 |
| --- | --- |
| `Core/Src/main.c` | 初始化和裸机主循环 |
| `Core/Src/control_input.c` | USB CDC双轴串口指令解析 |
| `Core/Src/chassis_can.c` | CAN2底盘双帧编码和周期发送 |
| `Core/Src/motor_control.c` | 双轴状态机、编码器、PID和CAN控制 |
| `Core/Inc/config/gimbal_params.h` | PID、零位、限位和超时参数 |
| `CAN.ioc` | STM32CubeMX工程 |

架构图：

- [代码架构](CAN_GM6020_代码架构.drawio)
- [优化版代码架构](CAN_GM6020_代码架构_优化版.drawio)
- [指令架构](gimbal_command_architecture.drawio)

## 构建

需要 ARM GNU Toolchain、CMake 和 Ninja。

```powershell
cmake --preset Debug
cmake --build --preset Debug --target CAN
```

也可以直接使用 CLion 的 `Debug` 预设构建。

## 使用前配置

1. 在 `Core/Inc/config/gimbal_params.h` 中设置两轴PID参数。
2. 标定 `YAW_ZERO_OFFSET_DEG` 和 `PITCH_ZERO_OFFSET_DEG`。
3. 根据机械结构设置Yaw、Pitch软限位。
4. 检查 `Core/Src/main.c` 中的 `SPEED_LOOP_DEBUG_BOOT_ENABLE`：
   - `0U`：正常位置控制；
   - `1U`：上电进入固定转速调试。

当前版本该开关为 `1U`，上电会让Yaw进入100 RPM速度环调试。装机运行前请确认
云台活动范围、急停方式和电流限制。
