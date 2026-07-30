# STM32F407 双轴 GM6020 云台控制

基于 STM32F407 和 HAL 库的双轴云台控制工程。以上板开发板为视角，
Yaw GM6020 使用 CAN1，Pitch GM6020 使用 CAN2；两台电机都设为 ID 2。

## 当前功能

- Yaw、Pitch 双轴 GM6020 控制
- CAN1/Yaw、CAN2/Pitch 独立反馈解析与 `0x1FE` 电流发送
- 单圈编码器多圈累计
- 目标角度劣弧（最短路径）处理
- 位置环、速度环串级 PID
- 等待反馈、位置控制、速度调试、故障四状态控制
- 反馈超时后零电流保护
- USB CDC 双轴位置指令输入
- DBUS 遥控器 DMA 接收、摇杆解析与 USB CDC 监视输出
- DBUS右摇杆分别控制Yaw、Pitch云台位置目标
- DBUS左摇杆控制底盘前进/横移速度，CAN1周期发送控制命令
- BMI088初始化、1 kHz六轴采集，以及Pitch编码器/IMU融合诊断

## 硬件与通信

| 功能 | 配置 |
| --- | --- |
| MCU | STM32F407 |
| CAN1 | PD0 = RX，PD1 = TX，1 Mbps；连接Yaw ID 2和底盘通信线路 |
| USART6 | PG9 = RX，PG14 = TX，115200-8-N-1 |
| USB CDC | `yaw,pitch\r\n`，角度单位为度 |
| DBUS | PC11 = USART3_RX，100000-8-E-1，DMA 接收 |
| CAN2 | PB5 = RX，PB6 = TX，1 Mbps；连接Pitch GM6020 ID 2 |
| BMI088 SPI1 | PB3 = SCK，PB4 = MISO，PA7 = MOSI，5.25 Mbps，Mode 0 |
| BMI088片选 | PA4 = Accel CS，PB0 = Gyro CS |
| Yaw反馈 | 上板CAN1标准帧 `0x206` |
| Pitch反馈 | 上板CAN2标准帧 `0x206` |
| 电流命令 | CAN1、CAN2 各自发送标准帧 `0x1FE`，DLC = 8 |

两条总线上的 `0x1FE` 数据分配相同：

| 字节 | 内容 |
| --- | --- |
| `DATA[0:1]` | 填0 |
| `DATA[2:3]` | 本总线 ID 2 电机电流，大端序 |
| `DATA[4:7]` | 填0 |

## 程序流程

```text
USB CDC接收
    ↓
control_in()
    ↓
GM6020_Process()
    ├─ CAN1读取Yaw 0x206，CAN2读取Pitch 0x206
    ├─ 更新多圈编码器
    ├─ 运行状态机
    ├─ 位置环输出目标转速
    ├─ 速度环输出目标电流
    └─ 在对应轴的CAN总线上发送0x1FE电流帧
```

## 双轴串口协议

开发板上电后，Yaw或Pitch收到自己的首帧CAN反馈便独立锁定当前位置，
不会在装机标定前自动转向编码器原始零点。

发送 ASCII 文本 `yaw,pitch\r\n`。Yaw 为相对于标定机械零点的累计
多圈角度，范围为 `-36000~36000` 度；例如 `360` 表示正向1圈，
`1080` 表示正向3圈，`-720` 表示反向2圈。Pitch 仍为单圈位置目标，
范围为 `-31.0~18.5` 度。命令会分别应用到已经在线并完成标定的轴；至少
一轴成功接收时回复`OK\r\n`，两轴都不可用时回复`LOCKED\r\n`，格式
错误时回复`ERR\r\n`。

急停指令为 `ESTOP\r\n`，开发板回复 `ESTOPPED\r\n`。该指令锁存后会
立即将两路 GM6020 电流置零，并通过 CAN1 发送零速度和底盘软件断电
模式；后续位置命令只回复 `LOCKED\r\n`，不会驱动电机。发送
`CLEAR\r\n` 可解除急停，开发板回复 `CLEARED\r\n`。解除后云台锁定
当前位置，底盘仍保持软件断电，不会自动恢复急停前的动作。

## 云台机械零位标定

程序每次上电自动标定，不需要串口触发，也不写入Flash。Yaw仍以开机
姿态为逻辑`0°`；Pitch改为由BMI088重力角决定，传感器水平为逻辑`0°`。

标定步骤：

1. 上电前把Yaw对正并固定云台；Pitch可以不在水平位置，但必须静止且
   位于安全机械范围内。
2. 开发板与两台 GM6020 上电。
3. 程序分别等待每个轴数据：Yaw采集100个新编码器反馈；Pitch同步采集
   100个新编码器反馈和100个有效BMI088重力角样本。任一轴不会等待
   另一轴。电机转速超过2 rpm、Pitch陀螺角速度超过静止门限、加速度
   模长不可信或编码器样本跨度超过64时，只重置对应轴样本。
4. 正常约0.1秒可采完；程序从零点生效时继续等待1秒供操作者放手。
   Pitch随后从当前位置开始，以`10°/s`的斜坡目标缓慢回到传感器`0°`。
   目标最多领先实际位置`2°`，到达`±0.5°`且低速保持100 ms后才允许
   遥控器或串口接管Pitch。

如需诊断，可选发送 `CALSTATUS\r\n`：

   - `CALWAIT`：至少一个未标定轴仍在等待有效CAN反馈；
   - `CALMOVING`：云台仍在运动，样本已重置；
   - `CALIBRATING`：正在采样；
   - `CALHOMING`：Pitch正在缓慢回到传感器零点；
   - `CALIBRATED`：零位已经生效且Pitch回零完成；
   - `CALERROR`：零位应用失败、启动姿态越界或回零超时。

编码器采样平均使用跨`8191→0`展开算法，因此编码器零点附近的数据不会
被错误平均到约`4096`。Yaw每次启动仍需固定在同一机械参考姿态；Pitch
零位由重力方向决定，不再随开机俯仰姿态变化。如果BMI088安装面与真实
机械水平存在固定偏差，可修改
`Core/Inc/config/pitch_fusion_config.h`中的
`PITCH_SENSOR_LEVEL_OFFSET_DEG`。

回零期间若触发急停或Pitch反馈掉线，斜坡暂停；恢复后先以当前位置重新
建立目标，再继续回零，不会追赶暂停前的旧目标。Pitch实测运行软限位为
`-31.0°~+18.5°`，上电回零允许从`-35.0°~+23.0°`包络内向安全区恢复，
但禁止继续向机械端点外侧运动。超过启动包络时进入`CALERROR`且不回零；
8秒内未完成回零时进入`CALERROR`并锁存急停。相关参数集中在
`pitch_fusion_config.h`中。

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
摇杆通道和两个三挡开关。当前RC数据已经验证，
`freertos.c`中的`DBUS_Monitor_Process()`调用暂时注释，不再通过USB
CDC周期输出；需要再次调试时取消注释即可。启用后的20 Hz输出格式为：

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

## BMI088原始数据与Pitch融合诊断

`imuTask` 每1 ms通过SPI读取一次加速度计和陀螺仪，并发布一致的数据
快照。Pitch完成机械零位和500个静止IMU样本标定后，融合器持续计算
角度、角速度和诊断量。当前安全验证阶段
`PITCH_FUSION_CONTROL_ENABLE=0U`，电机闭环仍使用编码器角度和电机RPM，
融合结果不会接管Pitch控制。Yaw控制不受影响。当前BMI088配置为：

- 加速度计：`±6 g`、800 Hz、Normal带宽；
- 陀螺仪：`±1000 °/s`、1000 Hz ODR、116 Hz带宽；
- 温度约每秒读取一次，单位为毫摄氏度。

USB CDC每20 ms发送一行，原始数据和Pitch融合数据交替输出，因此两种
格式各约25 Hz：

```text
IMU_RAW,count,ax,ay,az,gx,gy,gz,temp_mC,error_count\r\n
PITCH_FUSION,S=状态,CAL=标定样本,AR=原始重力角,AA=对齐重力角,G=陀螺角速度,E=编码器角度,ER=编码器角速度,F=融合角度,FR=融合角速度,B=安装架扰动角速度,I=加速度创新量,R=恢复样本数,H=健康位\r\n
```

其中六轴字段均为有符号16位原始值。换算关系约为：

```text
accel_g = accel_raw / 5460
gyro_dps = gyro_raw / 32.768
temperature_C = temp_mC / 1000
```

静止平放时，三个加速度轴中应有一个轴接近 `±5460`，其余接近0；
三个陀螺仪轴应接近0。`count` 应持续递增，正常时
`error_count` 应保持0。`PITCH_FUSION`中的角度和角速度均放大100倍后
以整数发送，例如`F=1234`表示融合Pitch为`12.34°`。

融合状态`S`：

| 数值 | 状态 |
| --- | --- |
| `0` | 等待BMI088或Pitch电机数据 |
| `1` | 数据有效，但云台还未静止 |
| `2` | 正在静止标定，`CAL`应增长到500 |
| `3` | 融合正常运行，掉线恢复也已重新通过健康样本检查 |
| `4` | 已标定但数据失效，或正在等待掉线后的安全重捕获 |

`H`的三位依次表示IMU有效、Pitch电机有效、加速度可信；其中加速度必须
同时满足模长门限和创新量门限，正常运行应为`111`。`I`放大100倍，表示
加速度角与陀螺预测角之差；绝对值超过`1200`时，本拍不使用加速度修正。
数据掉线后`R`需连续增长到20，状态才从`4`恢复为`3`。默认假设BMI088
的Y轴为Pitch陀螺轴、X/Z用于重力角计算。
如果实机方向不一致，只修改
`Core/Inc/config/pitch_fusion_config.h`中的轴索引和方向。

Pitch融合使用编码器建立机械零位与IMU安装角偏差，运行中用陀螺
积分预测、用可信的重力角低频修正。`B = FR - ER`用于观察底盘或安装架
对Pitch轴的扰动。以下闭环保护参数已经保留，但只有把
`PITCH_FUSION_CONTROL_ENABLE`改为`1U`后才生效：

- 控制用融合角相对编码器角最多偏离`±5°`；
- 融合角速度相对电机反馈最多修正`±180°/s`；
- 融合工作时Pitch目标速度最多`30 rpm`；
- 融合工作时Pitch电流命令最多`±3000`。

首次上电保持云台静止至少1秒，先观察`AR/AA/G/E/ER/F/I/R/H`。缓慢抬头
时`AA、G、E、F`应同号；静止或匀速缓慢运动时`I`不应长期超过门限。
确认轴向、符号、动态数据和掉线恢复均正确后，再单独启用融合闭环。
本阶段暂不启用数据就绪中断、SPI DMA和IMU恒温加热。

如果初始化配置寄存器失败，会输出首个失败位置：

```text
IMU,CONFIG_ERROR,ACC=0x1E,GYRO=0x0F,CFG=GYRO:0x10,W=0x02,R=0x82,ERR=0
```

`CFG`表示器件和寄存器地址，`W/R`分别表示写入值与回读值。配置未成功
时采集任务不会把每次跳过采样误计入`error_count`。

## DBUS 云台控制

当前`GIMBAL_YAW_ONLY_TEST_MODE=0U`，启用Yaw和Pitch双轴控制。以上板
开发板为视角，Yaw通过CAN1、Pitch通过CAN2通信，两台GM6020均为ID 2。
上电分别采集100帧零点后，CH0控制Yaw、CH1控制Pitch：

| 通道 | 动作 | 满杆目标速度 |
| --- | --- | --- |
| `CH0` | Yaw 左右运动 | `360°/s` |
| `CH1` | Pitch 上下运动 | `30°/s` |

两个通道都使用`±30`死区，回中后各自保持目标角度。Yaw使用多圈位置
目标，范围为`±36000°`；Pitch使用单圈位置目标并限制在
`-31.0°~+18.5°`。
遥控器掉线或急停会停止两轴更新。除此之外，每轴只检查自己的反馈和
标定状态：Yaw离线不影响Pitch，Pitch离线不影响Yaw；恢复的轴从自己的
编码器当前位置重新接管，避免跳回旧目标。

当前实机方向修正为Yaw `+1.0f`、Pitch `-1.0f`。如果后续更改电机安装
方向，可修改`remote_gimbal_control.c`中的
`REMOTE_GIMBAL_YAW_DIRECTION`或`REMOTE_GIMBAL_PITCH_DIRECTION`。
需要重新进行Yaw单轴测试时，可暂时把
`GIMBAL_YAW_ONLY_TEST_MODE`改回`1U`。

当前还启用了临时S1安全映射：

| S1挡位 | 动作 |
| --- | --- |
| 上挡 | 锁存云台GM6020急停，并令CAN1底盘零速、软件断电 |
| 中挡 | 解除云台与底盘急停 |
| 下挡 | 解除云台与底盘急停 |

S1上挡具有持续优先权；上挡期间即使USB收到`CLEAR`，也会在下一控制
周期重新急停。中/下挡只在切换进入该挡位时解除一次，不会持续覆盖
USB的`ESTOP`。该映射标记为临时设置，后续可以重新分配S1功能。

S2用于选择底盘模式：

| S2挡位 | `0x301`模式值 | 底盘模式 |
| --- | --- | --- |
| 上挡 | `1` | 跟随模式 |
| 中挡 | `2` | 不跟随模式 |
| 下挡 | `3` | 小陀螺模式 |

## 底盘CAN1协议

CAN1与Yaw电机共享物理总线；底盘模块每10 ms发送两帧，不接收Yaw反馈。
标准帧`0x300`的4个字段均为大端序`int16_t` Q10定点数：

| 字节 | 内容 |
| --- | --- |
| `0:1` | `vx × 1024` |
| `2:3` | `vy × 1024` |
| `4:5` | `wz × 1024` |
| `6:7` | `offset_angle_rad × 1024` |

标准帧`0x301`的Byte 0为底盘模式：`0`软件断电、`1`跟随、
`2`不跟随、`3`小陀螺。调用`ChassisCAN_SetCommand()`更新下一周期发送值。

当前遥控映射使用左摇杆：

| 遥控通道 | 动作 | 满杆命令 |
| --- | --- | --- |
| `CH3`纵向 | 底盘前进速度`vx` | `±5.0 m/s`，Q10为`±5120` |
| `CH2`横向 | 底盘横移速度`vy` | `±5.0 m/s`，Q10为`±5120` |

两个通道共用去中心`±30`死区，`wz`和`offset_angle_rad`暂时保持0，底盘
模式由S2选择。遥控掉线、最近帧非法或急停时发送零速度和软件断电模式。
若实机前后或左右方向相反，只修改
`Core/Src/remote_gimbal_control.c`中的对应`DIRECTION`符号。

## 主要文件

| 路径 | 用途 |
| --- | --- |
| `Core/Src/main.c` | 外设、业务模块及 FreeRTOS 内核初始化 |
| `Core/Src/freertos.c` | 1 ms云台控制任务、1 ms IMU采集任务及RTOS故障钩子 |
| `Core/Src/bmi088.c` | BMI088初始化、寄存器配置和六轴原始数据采集 |
| `Core/Src/bmi088_monitor.c` | USB CDC BMI088原始数据、Pitch融合与故障输出 |
| `Core/Src/pitch_fusion.c` | Pitch静止标定、互补滤波和受限闭环反馈 |
| `Core/Src/control_input.c` | USB CDC双轴串口指令解析 |
| `Core/Src/dbus.c` | DBUS DMA接收、摇杆解包、校验与在线判断 |
| `Core/Src/dbus_monitor.c` | 20 Hz USB CDC遥控器调试输出 |
| `Core/Src/remote_gimbal_control.c` | DBUS摇杆死区、速度映射与目标位置积分 |
| `Core/Src/gimbal_calibration.c` | 上电采样、传感器零位设置及Pitch慢速回零 |
| `Core/Src/chassis_can.c` | CAN1底盘双帧编码和周期发送 |
| `Core/Src/motor_control.c` | 双轴状态机、Pitch融合反馈、PID和CAN控制 |
| `Core/Inc/config/gimbal_params.h` | PID、零位、限位和超时参数 |
| `Core/Inc/config/pitch_fusion_config.h` | Pitch融合、传感器零位及慢速回零参数 |
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
4. 检查 `Core/Inc/config/pitch_fusion_config.h`中的BMI088轴向、Pitch
   编码器方向和融合闭环限制。
5. 检查 `Core/Src/main.c` 中的 `SPEED_LOOP_DEBUG_BOOT_ENABLE`：
   - `0U`：正常位置控制；
   - `1U`：上电进入固定转速调试。

当前版本该开关为`0U`，不会上电进入固定转速调试。装机运行前仍需确认
云台活动范围、急停方式、BMI088方向和Pitch融合限制。
