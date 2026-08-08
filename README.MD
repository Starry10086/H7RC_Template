# STM32H7 Robot Control Framework

基于 STM32H723、STM32 HAL 和 C++20 的机器人控制框架，使用 xmake 与 Arm GNU Toolchain 交叉编译。

当前 `Robot` 已接入：

- 3 路独立 FDCAN
- 4 个 DJI M3508 底盘电机
- 1 个 RobStride RS01
- 1 个 DM4310
- 麦克纳姆轮运动学与四轮速度 PID
- Topic 消息通道、CAN 路由和固定容量电机发送调度器
- SPI2 DMA 与 BMI088 六轴 IMU，200 Hz 发布 `ImuState`

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

## CAN 反馈数据流

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

## SPI 总线框架

SPI 总线采用“设备提交事务、总线统一调度、DMA 异步完成”的结构。设备驱动不直接调用 HAL，也不等待 DMA 完成。

```text
设备准备长期存在的 SpiTransfer
  -> SpiDevice::submit()
  -> SpiBus::submit()
  -> 参数和 DMA 地址检查
  -> 固定容量指针队列（默认 8 项）
  -> SpiBus::startNext()
  -> 当前事务变为 Active，CS 拉低
  -> HAL_SPI_TransmitReceive_DMA()
  -> CPU 继续执行主循环
  -> HAL SPI DMA 完成中断
  -> interrupt_bridge 转发事件，ISR 释放 CS
  -> SpiBus::process()
  -> 事务变为 Completed/Failed/TimedOut
  -> BMI088::process() 解析 RX 缓冲区
```

### SPI 框架的职责

`SpiBus` 是一条物理 SPI 总线的唯一调度器：

- 同一时间只启动一个 DMA 事务。
- 事务队列只保存 `SpiTransfer*`，不复制数据。
- 启动事务前拉低该事务的 CS。
- DMA 完成、HAL 错误和超时后拉高 CS。
- 主循环中的 `process(now_ms)` 负责状态收尾和启动下一笔事务。
- 中断回调不解析设备协议，只记录事件。

事务状态遵循：

```text
Idle -> Queued -> Active -> Completed/Failed/TimedOut/Aborted
```

`SpiDevice` 表示挂在同一条总线上的一个从设备。多个设备共享 `SpiBus`，但每个设备拥有独立 CS：

```cpp
platform::SpiDevice bmi_accel{spi2, accel_cs};
platform::SpiDevice bmi_gyro{spi2, gyro_cs};
```

设备提交的是长期存在的缓冲区和事务描述符。禁止使用局部 DMA 数组：

```cpp
struct Bmi088DmaStorage {
    alignas(32) std::array<uint8_t, 16> tx{};
    alignas(32) std::array<uint8_t, 16> rx{};
};
```

该存储由 `app/dma_storage.cpp` 使用 `DMA_BUFFER` 放入链接脚本的 `.dma_buffer`，位于 RAM_D2。事务处于 `Queued` 或 `Active` 时，不得修改其缓冲区，也不能重复提交同一个 `SpiTransfer`。

### 如何使用 SPI 框架

一个设备驱动通常持有以下对象：

```text
SpiBus 引用
SpiDevice（总线 + CS）
DMA Storage 引用
SpiTransfer（长期存在）
```

提交一次读取的过程是：

```text
填充 tx[0] 为寄存器读地址
  -> 清空 rx
  -> device.submit(transfer, tx, rx, size, timeout)
  -> 进入 XxxDataWait 状态
  -> 等待 transfer.state 成为 Completed
  -> 读取 rx
  -> 解析并发布 StateTopic
```

纯发送也使用 `HAL_SPI_TransmitReceive_DMA()`，因此必须提供同长度的 RX 丢弃缓冲区。设备协议解析只能发生在主循环，不能发生在 SPI ISR 中。

### BMI088 的 SPI 数据流

`Robot` 在构造时把 `spi2_`、两个 CS、DMA 存储和 `imu.bmi088.state` 引用注入 `Bmi088`。BMI088 的两个内部器件共用 SPI2，但使用不同 CS。

```text
Robot::init()
  -> Bmi088::init() 启动非阻塞状态机
  -> 等待上电
  -> 加速度计第一次 SPI 访问，切换 I2C 到 SPI
  -> 读取 Accel ID = 0x1E
  -> 读取 Gyro ID = 0x0F
  -> 写入加速度计和陀螺仪配置
  -> Ready
```

运行阶段每 5 ms 执行一次：

```text
Ready
  -> 加速度计 Burst Read
  -> AccelDataWait
  -> 解析 rx[2..7]
  -> 陀螺仪 Burst Read
  -> GyroDataWait
  -> 解析 rx[1..6]
  -> 单位换算
  -> 发布 imu.bmi088.state
  -> Ready
```

加速度计 SPI 读取需要一个额外 dummy byte，所以六轴读取的帧结构不同：

```text
Accel Read: 地址 + dummy + 6 字节数据，数据从 rx[2] 开始
Gyro Read:  地址 + 6 字节数据，数据从 rx[1] 开始
```

BMI088 驱动发布的单位为：

```text
accel_m_s2：m/s²
gyro_rad_s：rad/s
```

当前 BMI088 配置为：

```text
加速度计 ODR：200 Hz，量程：±6 g
陀螺仪带宽：200 Hz / 23 Hz，量程：±2000 deg/s
采样周期：5 ms
```

状态数据通过 `messaging::StateTopic<device::ImuState>` 保存最新值。日志、姿态估计器和其他控制逻辑都从 Topic 读取，不直接访问 BMI088 的 DMA 缓冲区。

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
SpiBus::process(now_ms)     收尾 SPI DMA 并启动下一笔事务
Robot::processDevices()     推进 BMI088 状态机并发布 ImuState
Robot::processControllers() 执行底盘解算和轮速 PID
Robot::processMotorTx()     调度并发送电机命令
```

`app/interrupt_bridge.cpp` 中的 HAL 回调只把 SPI 事件转发给 `SpiBus`。控制、SPI 事务收尾、BMI088 数据解析和 CAN 发送均在主循环执行，不在中断中运行。

## DMA 与 SPI2 配置

SPI2 使用 8 位、Mode 0、MSB first 和 TX/RX DMA。DMA 缓冲区必须满足：

- 全局或静态长期存在。
- 放在 `.dma_buffer`，不能使用普通 DTCM 成员或局部数组。
- 起始地址按 32 字节对齐。
- 位于 STM32H723 DMA 可访问的 RAM_D2。

链接脚本中的 `.dma_buffer` 位于 `0x30000000` 开始的 RAM_D2；MPU 将该区域配置为可访问、不可缓存区域。`SpiBus::submit()` 会用 `isDmaBuffer()` 拒绝空指针、零长度、越界或不在 DMA 区域内的缓冲区。

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

仓库还包含 VESC、GO1 和 WS2812 等驱动，但尚未全部接入当前 `Robot` 的 Topic、反馈路由和发送调度数据流。BMI088 已接入 SPI2 DMA、`Robot::processDevices()` 和 IMU StateTopic；四元数姿态估计器仍属于后续控制层工作。

## License

GPL-3.0 License
