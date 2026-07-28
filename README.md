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
- DBUS 遥控器 DMA 接收、摇杆解析与 USB CDC 监视输出
- DBUS 右摇杆控制 Yaw 云台位置目标（当前单轴测试模式）
- CAN2底盘控制命令周期发送

## 硬件与通信

| 功能 | 配置 |
| --- | --- |
| MCU | STM32F407 |
| CAN1 | PD0 = RX，PD1 = TX，1 Mbps |
| USART6 | PG9 = RX，PG14 = TX，115200-8-N-1 |
| USB CDC | `yaw,pitch\r\n`，角度单位为度 |
| DBUS | PC11 = USART3_RX，100000-8-E-1，DMA 接收 |
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

开发板上电并收到两台电机的首帧 CAN 反馈后，Yaw 和 Pitch 默认锁定
当前位置，不会在装机标定前自动转向编码器原始零点。

发送 ASCII 文本 `yaw,pitch\r\n`。Yaw 为相对于标定机械零点的累计
多圈角度，范围为 `-36000~36000` 度；例如 `360` 表示正向1圈，
`1080` 表示正向3圈，`-720` 表示反向2圈。Pitch 仍为单圈位置目标，
范围为 `-30~30` 度。命令有效时回复 `OK\r\n`，否则回复 `ERR\r\n`。

急停指令为 `ESTOP\r\n`，开发板回复 `ESTOPPED\r\n`。该指令锁存后会
立即将两路 GM6020 电流置零，并通过 CAN2 发送零速度和底盘软件断电
模式；后续位置命令只回复 `LOCKED\r\n`，不会驱动电机。发送
`CLEAR\r\n` 可解除急停，开发板回复 `CLEARED\r\n`。解除后云台锁定
当前位置，底盘仍保持软件断电，不会自动恢复急停前的动作。

## 云台机械零位标定

程序每次上电自动把当前云台姿态标定为 Yaw/Pitch 的逻辑 `0°`，不需要
串口触发，也不写入 Flash。

标定步骤：

1. 上电前把 Yaw 对正、Pitch 调至水平，并固定住云台。
2. 开发板与两台 GM6020 上电。
3. 程序在两轴 CAN 反馈有效且静止后，自动采集各 100 个新反馈帧。
   任一轴转速超过 2 rpm 或
   样本跨度超过 64 个编码器计数时，会丢弃已有样本并等待重新静止。
4. 正常约 0.1 秒可采完；建议上电后继续固定约 1 秒再放手。标定完成后
   当前位置就是 `0°`，位置环会继续保持该位置。

如需诊断，可选发送 `CALSTATUS\r\n`：

   - `CALWAIT`：等待两轴有效 CAN 反馈；
   - `CALMOVING`：云台仍在运动，样本已重置；
   - `CALIBRATING`：正在采样；
   - `CALIBRATED`：平均零位已经生效；
   - `CALERROR`：运行时应用零位失败。

采样平均使用跨 `8191→0` 展开算法，因此编码器零点附近的数据不会被
错误平均到约 `4096`。由于每次启动都会重新标定，启动时必须固定在同一
机械参考姿态；如果上电姿态不同，新的姿态就会成为新的零点。

开发板每 100 ms 主动上报一次：

```text
FB,yaw角度,pitch角度,yaw转速,pitch转速,yaw在线,pitch在线,急停\r\n
```

例如：`FB,732.34,-5.67,100,-20,1,1,0\r\n`。Yaw 上报相对于标定零点
累计的多圈角度；Pitch 上报当前编码器角度。角度单位为度，转速单位为
rpm，在线状态 `1` 表示近期收到对应电机的 CAN 反馈，急停状态 `1`
表示急停已锁存。

## DBUS 遥控器调试输出

开发板通过 USART3 + DMA 接收固定 18 字节 DBUS 帧，主循环解析四个
摇杆通道和两个三挡开关，并以 20 Hz 通过 USB CDC 虚拟串口输出：

```text
RC,frame,online,valid,ch0,ch1,ch2,ch3,s1,s2,c0,c1,c2,c3,invalid\r\n
```

- `frame`：已经处理的完整 18 字节帧数；
- `online`：最近 100 ms 内收到过合法帧时为 `1`；
- `valid`：最近处理的一帧通过范围检查时为 `1`；
- `ch0~ch3`：摇杆原始值，正常约 `364~1684`，回中约 `1024`；
- `s1/s2`：三挡开关编码，上=`1`、下=`2`、中=`3`；
- `c0~c3`：去中心后的摇杆值，正常约 `-660~+660`；
- `invalid`：累计非法帧数。

例如摇杆全部回中时可能看到：

```text
RC,123,1,1,1024,1024,1024,1024,1,3,0,0,0,0,0
```

电脑连接开发板 USB 虚拟串口后，用串口助手以 ASCII 文本显示即可。
建议界面选择 `115200、8-N-1`（USB CDC 实际不依赖该波特率），行尾为
CRLF。未连接接收机或连续 100 ms 没有合法帧时，`online` 为 `0`。

## DBUS 云台控制

当前 `GIMBAL_YAW_ONLY_TEST_MODE=1U`，只要求 Yaw 电机和遥控器在线。
上电采集 Yaw 的 100 帧零点后，右摇杆横向控制 Yaw：

| 通道 | 动作 | 满杆目标速度 |
| --- | --- | --- |
| `CH0` | Yaw 左右运动 | `360°/s` |

摇杆去中心值在 `±30` 内视为回中，回中后保持当前目标角度。Yaw 使用
多圈位置目标。遥控器掉线、急停、电机离线或标定未完成时停止更新；
恢复后从编码器当前位置重新接管，避免跳回旧目标。单轴测试期间
`0x1FE` 的 Pitch 电流槽始终强制为零。如果 Yaw 实机方向相反，修改
`remote_gimbal_control.c` 中的 `REMOTE_GIMBAL_YAW_DIRECTION` 为
`-1.0f`。完成单轴测试后，把 `gimbal_params.h` 中的
`GIMBAL_YAW_ONLY_TEST_MODE` 改为 `0U` 即可恢复双轴逻辑。

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
| `Core/Src/dbus.c` | DBUS DMA接收、摇杆解包、校验与在线判断 |
| `Core/Src/dbus_monitor.c` | 20 Hz USB CDC遥控器调试输出 |
| `Core/Src/remote_gimbal_control.c` | DBUS摇杆死区、速度映射与目标位置积分 |
| `Core/Src/gimbal_calibration.c` | 上电自动采集100帧并设置机械零位 |
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
