# STM32F407 双轴 GM6020 云台控制

基于 STM32F407 和 HAL 库的双轴云台控制工程。Yaw、Pitch 两台 GM6020
共用 CAN1，通过位置环与速度环串级 PID 输出合并电流命令；USART6 可接收
PC 端发送的鼠标目标角度。

## 当前功能

- Yaw、Pitch 双轴 GM6020 控制
- CAN1 反馈解析与 `0x1FE` 合并电流发送
- 单圈编码器多圈累计
- 目标角度劣弧（最短路径）处理
- 位置环、速度环串级 PID
- 等待反馈、位置控制、速度调试、故障四状态控制
- 反馈超时后零电流保护
- USART6 鼠标位置帧解析

## 硬件与通信

| 功能 | 配置 |
| --- | --- |
| MCU | STM32F407 |
| CAN1 | PD0 = RX，PD1 = TX，1 Mbps |
| USART6 | PG9 = RX，PG14 = TX，115200-8-N-1 |
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
USART6中断接收
    ↓
MousePositionRx_Process()
    ↓
更新Yaw目标角度
    ↓
GM6020_Process()
    ├─ 读取0x205/0x206反馈
    ├─ 更新多圈编码器
    ├─ 运行状态机
    ├─ 位置环输出目标转速
    ├─ 速度环输出目标电流
    └─ 发送0x1FE合并电流帧
```

## 鼠标串口协议

每帧8字节，小端序：

| 字节 | 内容 |
| --- | --- |
| `0` | `0xAA` |
| `1` | `0x55` |
| `2:5` | IEEE 754 `float`目标角度，单位：度 |
| `6` | 保留，必须为`0x00` |
| `7` | Byte 0至6的XOR校验 |

当前协议只更新Yaw目标；Pitch可通过
`GM6020_SetGimbalPosition()` 或扩展双角度协议控制。

## 主要文件

| 路径 | 用途 |
| --- | --- |
| `Core/Src/main.c` | 初始化和裸机主循环 |
| `Core/Src/motor_control.c` | 双轴状态机、编码器、PID和CAN控制 |
| `Core/Src/mouse_position_rx.c` | USART6鼠标指令解析 |
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
