# STM32H7 Robot Control Framework

基于 STM32H723、STM32 HAL 和 C++20 的机器人控制框架，使用 xmake 与 Arm GNU Toolchain 交叉编译。

当前 `Robot` 已接入：

- 3 路独立 FDCAN
- 4 个 DJI M3508 底盘电机
- 1 个 RobStride RS01
- 1 个 DM4310
- 麦克纳姆轮运动学与四轮速度 PID
- Topic 消息通道、CAN 路由和固定容量电机发送调度器

当前电机均接在 CAN2；驱动和发送节点通过总线引用绑定，后续可调整到其他总线。

## 目录结构

```text
app/          应用入口、中断桥接和主循环调度
bsp/HAL/      CubeMX 生成的启动代码、HAL 和链接脚本
components/   RingBuffer、Topic、日志等通用组件
control/      PID 等通用控制算法
devices/      电机、IMU、LED 等具体设备驱动
platform/     CAN、SPI、I2C、UART、时间基准和 RTT 封装
robot/        设备组合、Topic、底盘控制和电机发送调度
third_party/  SEGGER RTT 等第三方代码
```

## 核心设计

- 中断只接收 CAN 帧并写入对应总线的 SPSC RingBuffer。
- CAN 帧在主循环中按总线独立路由，设备驱动负责协议解析。
- `StateTopic<T>` 保存设备最新反馈，`CommandTopic<T>` 保存最新控制命令。
- Topic 名称仅用于调试和标识，实际连接由 `Robot` 构造时的引用绑定完成。
- 不使用动态内存、异常和 RTTI；队列、路由及发送任务均为固定容量。

## 反馈数据流

```text
FDCAN FIFO0 中断
  -> CanBus::onRxFifo0Interrupt()
  -> 对应 CAN 总线的 SPSC RingBuffer
  -> Robot::processCanRx()
  -> 对应总线的 CanRouter
  -> 电机 handleCanFrame()
  -> 解析为 MotorState
  -> StateTopic<MotorState>
  -> 控制器或上层逻辑读取
```

每条 CAN 总线拥有独立的 `CanBus`、接收队列和 `CanRouter`。Router 根据帧格式、帧类型以及精确 ID 或掩码进行分发。

## 底盘命令数据流

```text
ChassisVelCmd Topic
  -> ChassisController
  -> 麦克纳姆轮运动学解算
  -> 4 个 WheelVelTarget Topic
  -> 4 个 WheelVelController
  -> 速度反馈 + PID
  -> 4 个 MotorCommand::Torque Topic
  -> DjiGroupTx（四个槽位合成一帧）
  -> MotorTxScheduler
  -> CanBus::send()
  -> CAN 0x200 控制帧
```

底盘控制只发布轮端目标速度，不直接依赖 M3508。电机类型、总线和 Topic 的对应关系集中在 `Robot` 中完成。

## RS01 / DM4310 命令数据流

RS01 和 DM4310 使用 MIT 命令，不经过底盘运动学和轮速 PID：

```text
MotorCommand::Mit Topic
  -> MitMotorTx<Motor>
  -> 电机协议编码
  -> MotorTxScheduler
  -> 对应 CanBus
```

`MotorTxScheduler` 只调度统一的发送任务，不了解具体电机协议。DJI 使用 `DjiGroupTx`，RS/DM 使用 `MitMotorTx`。

命令过期、反馈过期或设备报告故障时，发送节点使用对应模式的零命令。RS01 和 DM4310 初始化采用“零命令 -> 使能 -> 零命令”的顺序。

## 主循环顺序

```text
Robot::processCanRx()       更新 StateTopic
Robot::processControllers() 执行底盘解算和轮速 PID
Robot::processMotorTx()     调度并发送电机命令
```

控制和 CAN 发送均在主循环执行，不在中断中运行。

## 环境要求

- STM32CubeMX
- xmake 2.8.5 或更高版本
- Arm GNU Toolchain，提供 `arm-none-eabi-gcc`
- C11 / C++20

目标芯片为 STM32H723，使用 Cortex-M7、硬件双精度 FPU 和 hard-float ABI；链接脚本为 `bsp/HAL/rccs_slave/STM32H723XG_FLASH.ld`。

## 配置与构建

工具链路径不包含 `bin` 目录：

```bash
xmake f --sdk="<arm-gnu-toolchain-path>" -m debug
xmake -r
```

Debug 固件输出为 `build/cross/arm/debug/application.elf`。

Release 构建：

```bash
xmake f -m release && xmake
```

Release 固件输出为 `build/cross/arm/release/application.elf`。

CubeMX 代码生成到 `bsp/HAL`。工程会读取其中的 Makefile，并自动加入 HAL 源文件、宏和头文件路径。

## 下载与调试

可使用 STM32CubeProgrammer、OpenOCD、J-Link 或其他支持 STM32H723 的工具下载和调试；运行日志通过 SEGGER RTT 输出。

## 扩展状态

仓库还包含 VESC、GO1、BMI088 和 WS2812 等驱动，但尚未全部接入当前 `Robot` 的 Topic、反馈路由和发送调度数据流。

## License

GPL-3.0 License
