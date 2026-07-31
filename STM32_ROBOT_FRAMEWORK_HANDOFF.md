# STM32H7 机器人控制框架交接文档

> 最后核对日期：2026-07-28
>
> 工程路径：`/Users/starry/Documents/STM32_Project/stm32_H7_xmake`
>
> 本文档是新对话的技术上下文。它记录已经实机验证的内容、只完成代码但尚未实机验证的内容、尚未完成的内容，以及已经讨论并确定的架构取舍。

## 0. 新对话请先发送这段话

```text
请完整读取仓库根目录的 STM32_ROBOT_FRAMEWORK_HANDOFF.md，并检查文档提到的当前代码与 git status。当前工作树包含我还没有提交的修改，禁止 reset、checkout、清理或覆盖这些修改。

请严格区分“已实机验证”“仅完成代码”“尚未实现”。不要重新引入 MotorEndpoint、std::span、复杂模板、工厂或过度抽象。保持当前简单分层：驱动生成 can::Frame，Robot 持有具体设备并选择 CAN 总线，CanBus::send() 唯一负责交给 HAL，App 负责测试或主循环调度。

先向我总结你理解的当前状态和下一步，再从交接文档的 P0 待办开始。修改前读取当前文件，修改后运行 xmake -r，并详细解释代码逻辑和安全影响。
```

说明：新建对话后，平台不会自动把旧对话的每一句消息注入新对话。本文档不能保存界面层面的完整聊天记录，但已经把会影响后续设计和实现的技术结论、试验结果、用户偏好和未完成项集中保留下来。

## 1. 用户目标和协作偏好

用户正在学习并搭建一个 STM32H7 裸机 C++ 机器人框架，希望最终支持多种电机、传感器、控制器和多路 CAN，同时希望每一步都能理解“为什么这样写”。

后续协作需要遵循：

1. 用户已经允许助手在明确提出“直接帮我修改”时直接编辑代码。
2. 修改前必须读取当前代码，因为用户会在两次对话之间手动改文件。
3. 修改后必须编译，并解释数据流、协议位布局、对象关系和安全影响。
4. 不允许回退、清理或覆盖工作树中用户已有的修改。
5. 用户倾向容易阅读、容易测试的显式代码，不接受为了“通用”而引入大量间接层。
6. 当前明确不要引入 `MotorEndpoint`、`std::span`、复杂模板接口、运行时工厂或一套重复的通用 `sendFrame()` 包装。
7. 测试阶段允许 `App` 直接调用 `Robot` 的电机发送函数，暂时不要求建立完整控制命令总线。

## 2. 工程环境

- MCU：STM32H723
- 底层：STM32 HAL / CMSIS
- CubeMX 工程：`bsp/HAL/rccs_slave`
- 构建系统：xmake
- 工具链：`arm-none-eabi-gcc/g++`
- C：C11
- C++：C++20
- 运行方式：裸机 `while (1)`，没有 RTOS
- C++ 配置：禁用异常、RTTI、线程安全局部静态初始化锁
- 调试：SEGGER J-Link + RTT，实机日志已经正常工作
- 三路 FDCAN：Classic CAN、Normal mode、1 Mbps
- 每路 FDCAN：RX FIFO0 32 个元素、TX FIFO 32 个元素
- `CanBus` 的每路软件 RX SPSC 队列逻辑容量：64 帧

`main.c` 已经接入应用层：

```c
MX_GPIO_Init();
MX_SPI2_Init();
MX_FDCAN1_Init();
MX_FDCAN2_Init();
MX_FDCAN3_Init();

if (!App_Init()) {
    Error_Handler();
}

while (1) {
    App_Process();
}
```

HAL 的 FIFO0 回调通过 `app/interrupt_bridge.cpp` 转到对应 `CanBus`：

```text
HAL_FDCAN_RxFifo0Callback
    -> App::onFdcanRxFifo0Interrupt
    -> CanBus::onRxFifo0Interrupt
```

## 3. 已确认的框架边界

当前接受的结构不是“每个动作都再包一层”，而是下面这四层：

```text
devices 电机驱动
    解析反馈帧
    把协议参数编码成 can::Frame
              |
              v
Robot
    持有本机器人实际使用的电机对象
    绑定反馈路由
    决定每台电机属于哪条 CAN 总线
    提供少量有明确含义的发送方法
              |
              v
CanBus::send(const can::Frame&)
    校验通用 CAN 属性
    转换为 FDCAN_TxHeaderTypeDef
    唯一调用 HAL_FDCAN_AddMessageToTxFifoQ 的地方
              |
              v
App
    初始化
    调度 1 kHz 测试命令
    读取状态、记录日志、以后运行控制器
```

这样划分的理由：

- 电机驱动只理解协议，不依赖 `FDCAN_HandleTypeDef`，可以独立做字节级测试。
- `CanBus` 只理解通用 CAN，不知道 RS、DM 或 DJI。
- `Robot` 知道实际接线关系，因此这里选择 `can1_`、`can2_` 或 `can3_` 是合理的。
- `App` 不拼协议字节，只调用“使能、停止、控制”等动作。
- 当前规模下允许 `Robot::enableRs01()` 这类少量重复函数；可读性比过早抽象更重要。

当前明确不采用：

- 不新增 `MotorEndpoint` 来绑定设备和总线。
- 不用 `std::span<const double>` 作为 1 到 4 台 DJI 电机的测试接口。
- 不让每个驱动直接持有 `CanBus`，否则协议驱动与 STM32/HAL 绑定过紧。
- 不再包装一个和 `CanBus::send()` 功能重复的通用 `Robot::sendFrame()`。

## 4. 当前目录职责

```text
bsp/HAL/rccs_slave/              CubeMX、HAL、启动代码、中断入口

platform/can/
    can_types.hpp                can::Frame、标准/扩展 ID、数据/远程帧
    can_router.hpp               固定容量软件路由

platform/stm32/
    can_bus.hpp/.cpp             HAL FDCAN 收发、ISR 队列、统计
    timebase.hpp/.cpp            毫秒时间基准

components/
    containers/                  SPSC 队列
    messaging/                   StateTopic
    logging/                     RTT 日志接口和节流日志

devices/motors/
    motor_state.hpp              统一电机状态
    robostride/rs_motor.hpp      RS01/RS05 通用驱动
    dm/dm_motor.hpp              DM4310/DM4340 驱动
    dji/dji_motor.hpp            DJI 单电机反馈解析和力矩编码
    dji/dji_command_group.hpp    DJI 4 槽位组帧
    vesc/vesc_motor.hpp          旧接口，尚未迁移

robot/
    robot_topics.hpp             本机器人实际状态话题
    robot.hpp/.cpp               设备对象、路由、总线归属、发送函数

app/
    app.hpp/.cpp                 当前台架测试和主循环调度
    app_entry.h                  C/C++ ABI 入口
    interrupt_bridge.cpp         HAL 回调桥接
```

## 5. CAN 基础设施：已经完成

### 5.1 `can::Frame`

`platform/can/can_types.hpp` 中统一使用完整帧：

```cpp
struct Frame {
    uint32_t id{0};
    IdFormat id_format{IdFormat::Standard};
    FrameKind kind{FrameKind::Data};
    uint8_t length{0};
    std::array<uint8_t, 8> data{};
};
```

这是之前“电机驱动为什么不应该只返回 `uint64_t`”讨论后的结论。完整帧同时携带 ID、标准/扩展格式、帧类型、长度和字节数据，不会在调用者处丢失协议语义，也避免主机字节序造成的混乱。

### 5.2 接收路径

三路 CAN 独立接收：

```text
CAN 硬件
  -> FDCAN FIFO0 中断
  -> 对应 CanBus 的 SPSC ring
  -> App 主循环
  -> Robot::processCanRx()
  -> 对应 Router::dispatch(frame)
  -> 电机 handleCanFrame(frame)
  -> StateTopic<MotorState>
```

约束：

- ISR 只取 HAL 帧、转换为 `can::Frame`、压入队列。
- ISR 不解析电机协议、不写日志、不运行 PID。
- `Robot::processCanRx()` 每次主循环每条总线最多处理 16 帧，避免某一总线永久占住主循环。
- `Router` 只负责找到接收对象，协议合法性仍由驱动再次检查。

### 5.3 发送路径

`platform::CanBus::send(const can::Frame&)` 已经实现并通过实机发送验证：

- 只允许已经 `start()` 的总线发送。
- 支持 Classic CAN 的标准 ID 和扩展 ID。
- 验证长度不超过 8 字节。
- 验证标准 ID `<= 0x7FF`、扩展 ID `<= 0x1FFFFFFF`。
- 转换 DLC、ID 类型和帧类型。
- 统一调用 `HAL_FDCAN_AddMessageToTxFifoQ()`。
- 成功入硬件 FIFO 后增加 `queued_transmit_frames`。
- 软件校验失败增加 `rejected_transmit_frames`。
- HAL 失败增加 `hal_error`。

注意：当前 `send()` 返回 `true` 只表示 HAL 已接受该帧进入发送 FIFO，不保证对端电机已经收到、应答或执行。最终成功需要结合反馈的新鲜度、错误计数和电机状态判断。

### 5.4 软件 Router

`can::Router<MaxRoutes>` 已实现：

- `bindExact()`：完整 ID 精确匹配。
- `bindMask()`：只比较掩码选中的位。
- 拒绝非法 ID、非法 mask、零 mask、容量溢出和重叠路由。
- `dispatch()` 找到第一条匹配路由后调用设备的 `handleCanFrame()`。
- 有已绑定、已路由、未处理帧统计。

当前每条总线 `router_capacity_ = 8`，FDCAN2 已使用 6 条路由：4 个 DJI、1 个 RS01、1 个 DM4310。

## 6. 消息和状态模型：已经完成基础部分

统一状态：

```cpp
struct MotorState {
    float position_rad{0.0F};
    float velocity_rad_s{0.0F};
    float torque_nm{0.0F};
    float temperature_c{0.0F};
    uint32_t fault_code{0U};
};
```

`StateTopic<T>` 保存最新值、毫秒时间戳和递增 sequence：

```cpp
StateSample<device::MotorState> sample{};

if (topic.read(sample)) {
    const bool online = messaging::isFresh(
        now_ms,
        sample.timestamp_ms,
        timeout_ms);
}
```

状态话题不自动删除旧数据。`read()` 为真只表示曾经收到过数据，是否在线必须再看 `timestamp_ms`。

未来控制命令应该采用“最新命令 + 超时”的模型，而不是把历史控制命令排队执行。`CommandTopic` 尚未实现。

## 7. 当前 Robot 的实际设备和接线

当前所有测试电机都放在 FDCAN2：

| 对象 | 类型 | 发送 ID | 反馈路由 | 当前状态 |
|---|---|---:|---:|---|
| `chassis_left_front_` | DJI M3508 | 组 ID `0x200` 槽 1 | 标准 `0x201` | 反馈代码完成 |
| `chassis_right_front_` | DJI M3508 | 组 ID `0x200` 槽 2 | 标准 `0x202` | 反馈已实机验证 |
| `chassis_right_back_` | DJI M3508 | 组 ID `0x200` 槽 3 | 标准 `0x203` | 反馈代码完成 |
| `chassis_left_back_` | DJI M3508 | 组 ID `0x200` 槽 4 | 标准 `0x204` | 反馈代码完成 |
| `rs01_` | RobStride RS01 | 扩展 ID，motor `0x03` | motor `0x03` + host `0xFD` | 收发实机成功 |
| `dm4310_` | DM-J4310-2EC MIT | 标准 `0x01` | 标准 `0x11` | 收发实机成功 |

构造配置的核心内容：

```cpp
rs01_{
    device::RsMotor::Config{
        device::RsMotor::Type::RS01,
        0x03U}.set_host_id(0xFDU),
    topics_.rs01_state}

dm4310_{
    device::DmMotor::Config{
        device::DmMotor::Type::DM_J4310_2EC,
        0x01U}.set_control_mode(
            device::DmMotor::ControlMode::MIT),
    topics_.dm4310_state}
```

当前 `Robot` 发送 API 保持直接、显式：

```cpp
bool enableRs01() noexcept;
bool stopRs01() noexcept;
bool sendRs01MitControl(float p, float v, float kp,
                        float kd, float torque) noexcept;

bool enableDm4310() noexcept;
bool disableDm4310() noexcept;
bool sendDm4310MitControl(float p, float v, float kp,
                          float kd, float torque) noexcept;

bool sendDjiTorqueCommands(
    double motor_1_torque_nm,
    double motor_2_torque_nm = 0.0,
    double motor_3_torque_nm = 0.0,
    double motor_4_torque_nm = 0.0) noexcept;
```

`Robot` 的这些方法不是协议实现。它们只是把对应驱动生成的 `can::Frame` 交给正确的 `CanBus`。

## 8. RS01 / RS05：已完成并实机验证

### 8.1 驱动现状

原来的 `devices/motors/robostride/rs01.hpp` 已替换为通用的 `rs_motor.hpp`，同一驱动通过 `Type::RS01` / `Type::RS05` 使用不同限幅。

已实现：

- 使能帧 `makeEnableFrame()`
- 停止帧 `makeStopFrame(clear_fault)`
- 设置机械零点帧
- 主动上报开关帧
- MIT 控制帧
- 反馈类型 2 和主动上报类型 24 的解析
- 位置、速度、力矩、温度、故障位、运行状态解析
- 反向配置和可选多圈角度
- 所有发送接口返回完整 `can::Frame`

RS01 限制：位置约 `+-12.57 rad`、速度 `+-44 rad/s`、力矩 `+-17 Nm`、`Kp 0..500`、`Kd 0..5`。

RS05 限制：位置约 `+-12.57 rad`、速度 `+-50 rad/s`、力矩 `+-5.5 Nm`、`Kp 0..500`、`Kd 0..5`。

### 8.2 实机验证记录

- RS05：motor ID `0x01`，host ID `0xFD`，FDCAN2，收发成功。
- RS01：motor ID `0x03`，host ID `0xFD`，FDCAN2，收发成功。
- 当前代码保留的是 RS01 ID `0x03`，不是 RS05。
- 当前 RS01 和 DM4310 同时连接 FDCAN2，各自 1 kHz 零力矩发送，用户已确认成功。

已核对过的典型扩展 ID：

```text
RS05 ID=1, host=FD
enable      0x0300FD01
MIT zero    0x01800001
stop        0x0400FD01

RS01 ID=3, host=FD
enable      0x0300FD03
MIT zero    0x01800003
stop        0x0400FD03
```

### 8.3 `receiveRouteId()` 和 `receiveRouteMask()` 的含义

这是上一段讨论的重点。

RS 反馈扩展 ID 的低 16 位固定表示：

```text
bit 15..8 = motor ID
bit  7..0 = host ID
```

对于当前 RS01：

```cpp
receiveRouteId() = (0x03U << 8U) | 0xFDU;
                 = 0x03FDU;

receiveRouteMask() = 0x0000FFFFU;
```

绑定代码：

```cpp
can2_router_.bindMask(
    can::IdFormat::Extended,
    rs01_.receiveRouteId(),
    rs01_.receiveRouteMask(),
    rs01_);
```

Router 实际比较：

```cpp
((frame.id ^ route.id) & route.mask) == 0U
```

也就是只要求收到帧的低 16 位等于 `0x03FD`。高位中的通信类型、故障位和运行状态会变化，所以不能做完整 29 位精确匹配。路由找到 RS01 后，`RsMotor::handleCanFrame()` 还会再次验证扩展数据帧、8 字节、通信类型 2/24、motor ID 和 host ID。

## 9. DM4310 / DM4340：DM4310 已实机验证

### 9.1 已完成的驱动改造

`DmMotor` 的发送接口已经从零散整数/`uint64_t` 改成完整 `can::Frame`：

- `makeControlFrame()`
- `makeMitControlFrame()`
- `makePositionVelocityFrame()`
- `makeVelocityFrame()`
- `makeEnableFrame()`
- `makeDisableFrame()`
- `makeSaveZeroPositionFrame()`
- `makeClearErrorFrame()`

配置中已有命令 ID：

```cpp
device::DmMotor::Config{
    device::DmMotor::Type::DM_J4310_2EC,
    0x01U}
```

当前 DM4310：

- 主机发送：标准 ID `0x01`
- 电机反馈：标准 ID `0x11`
- 控制模式：MIT
- FDCAN2

MIT 零力矩/零增益帧已经做过字节检查：

```text
ID: 0x01 standard
DATA: 7F FF 7F F0 00 00 07 FF
```

特殊命令：

```text
enable:  FF FF FF FF FF FF FF FC
disable: FF FF FF FF FF FF FF FD
zero:    FF FF FF FF FF FF FF FE
clear:   FF FF FF FF FF FF FF FB
```

位置速度模式和速度模式的 `float` 明确按小端字节写入，避免依赖 `uint64_t` 的主机内存布局。

### 9.2 实机验证记录

- DM4310 在 FDCAN2 上使用 command ID `0x01`、feedback ID `0x11`，发送、使能、状态接收成功。
- 已成功以 1 kHz 连续发送 MIT 零力矩控制。
- 用户曾得到稳定反馈，例如：

```text
DM4310 ONLINE pos=2.114 vel=0.011 torque=-0.003 temp=32.0 fault=0x01
```

这里的 `0x01` 当前来自反馈 `data[0]` 的高 4 位。它可能包含运行状态语义，不能在没有重新核对协议手册前直接解释为“发生故障”。这个字段的准确命名和语义仍是待办。

### 9.3 DM4340 当前状态

代码的 `Type::DM_J4340_2EC` 和相应力矩/减速比配置已经存在，但没有创建 Robot 对象，也没有做本轮实机验证。不能把 DM4310 的验证结果自动等同于 DM4340。

## 10. DJI：反馈已验证，发送代码已完成但尚未完成实机命令验证

### 10.1 单电机驱动

`DjiMotor::handleCanFrame()` 解析 8 字节反馈并发布 `MotorState`。

`DjiMotor::encodeTorqueCommand(double)` 负责：

- 检查 NaN，NaN 返回 0。
- 按当前电机配置限制最大力矩。
- 把 Nm 转成 DJI 原始电流命令。
- 返回主机序的 `int16_t`，不在这里预先交换字节。

字节序由组帧函数统一处理。这样单电机只关心“这个力矩对应什么原始命令”，组帧函数只关心“4 个 `int16_t` 怎样放入 8 字节”。

### 10.2 组帧

`devices/motors/dji/dji_command_group.hpp` 已支持：

```cpp
enum class DjiCommandGroup : uint16_t {
    m3508_m2006_201_to_204 = 0x200U,
    m3508_m2006_205_to_208 = 0x1FFU,
    gm6020_current_205_to_208 = 0x1FEU,
    gm6020_current_209_to_20b = 0x2FEU,
    gm6020_voltage_205_to_208 = 0x1FFU,
    gm6020_voltage_209_to_20b = 0x2FFU
};
```

统一函数：

```cpp
constexpr can::Frame makeDjiCommandFrame(
    DjiCommandGroup group,
    const std::array<int16_t, 4>& commands) noexcept;
```

一个 DJI 组命令永远是 4 个槽位、8 字节，每个 `int16_t` 按高字节在前写入。只发送 1 到 3 台电机时，其余槽位必须填 0，不能缩短 DLC，也不能改变槽位顺序。

`Robot::sendDjiTorqueCommands()` 当前固定对应 M3508/M2006 `0x201..0x204` 和发送 ID `0x200`：

```cpp
robot.sendDjiTorqueCommands(torque1);             // 后三槽为 0
robot.sendDjiTorqueCommands(torque1, torque2);    // 后两槽为 0
robot.sendDjiTorqueCommands(t1, t2, t3, t4);
```

已经完成的验证：

- DJI `0x202` M3508/C620 反馈在实机上接收成功。
- `0x200`、`0x1FF` 组 ID 的正数、负数和边界值做过主机字节级断言。
- ARM 工程能够编译这套接口。

尚未完成：

- 当前 `App` 没有周期调用 `sendDjiTorqueCommands()`。
- 没有在本轮实机上验证 DJI 力矩发送和电机实际响应。
- 当前 Robot 只暴露了 `0x200` 的 4 台 M3508/M2006 发送封装；`0x1FF` 等组帧能力存在，但尚未接入 Robot 的实际设备配置。

## 11. 当前 App 的实际运行行为

当前 `App` 是台架测试程序，不是最终控制系统。

`App::init()` 的顺序：

```text
初始化 RTT
-> Robot::init() 绑定路由并启动三路 CAN
-> 给 RS01 发送一次 MIT 零位置/零速度/零增益/零力矩帧
-> 给 DM4310 发送一次 MIT 零位置/零速度/零增益/零力矩帧
-> 使能 RS01
-> 使能 DM4310
-> 打开两个 1 kHz 零力矩流标志
```

`App::process()`：

```text
处理三路 CAN 接收
-> 每经过 1 ms 给 RS01 发送一次零力矩 MIT 帧
-> 每经过 1 ms 给 DM4310 发送一次零力矩 MIT 帧
-> 如果 RS 发送失败，关闭其发送流并请求 stop
-> 如果 DM 发送失败，关闭其发送流并请求 disable
-> 可选地读取 StateTopic 和打印节流日志
```

调度使用：

```cpp
if (static_cast<uint32_t>(now_ms - last_command_ms_) >= 1U) {
    last_command_ms_ = now_ms;
    // 只发送一次，不补发错过的多个周期
}
```

这避免主循环阻塞恢复后把旧周期突发补发到 CAN 总线。

非常重要：

- 当前固件上电后会自动使能 RS01 和 DM4310。
- `p=0, v=0, kp=0, kd=0, torque=0` 表示 MIT 零输出，不是位置保持。
- “零力矩”不等于急停，也不能代替硬件断电、急停和机械安全措施。
- 当前 RS01/DM4310 状态日志和 CAN2 统计大多被注释了，当前唯一启用的状态日志是 DJI `0x202`。日志被注释不影响两个 1 kHz 电机命令继续发送。

## 12. 已经确认成功的测试汇总

### 已实机验证

1. SEGGER RTT 初始化、浮点格式化和节流日志正常。
2. FDCAN2 Classic CAN 1 Mbps 接收链路正常。
3. `CanBus::send()` 能发送标准帧和扩展帧。
4. DJI M3508/C620 的 `0x202` 反馈可以接收、路由、解析并发布状态。
5. DM4310，command ID `0x01`、feedback ID `0x11`，可使能、发送 MIT 控制并接收状态。
6. DM4310 1 kHz 零力矩 MIT 控制成功。
7. RS05，motor ID `0x01`、host `0xFD`，可使能、控制、停止并接收状态。
8. RS01，motor ID `0x03`、host `0xFD`，可使能、控制、停止并接收状态。
9. 当前 RS01 与 DM4310 共用 FDCAN2、各自 1 kHz 零力矩控制，用户确认成功。

### 只完成代码或离线测试

1. DJI `0x200` / `0x1FF` 等组命令的完整 `can::Frame` 生成。
2. DJI 1 到 4 个扭矩参数的简单 Robot API。
3. DM4340 类型配置。
4. RS/DM/DJI 的若干边界值和字节序主机断言。

### 尚未验证

1. DJI 力矩命令的本轮实机发送与电机响应。
2. DM4340 实机。
3. 多台 RS、多台 DM 混合挂在同一总线时的完整带宽和长期稳定性。
4. 总线拥塞、掉线和 bus-off 的系统级恢复策略。

## 13. 一条总线挂多个 RS05、RS01、DM4310、DM4340 时的方向

这个问题已经讨论过，但尚未实现。

不要为每台电机无限增加：

```text
enableRs01A()
enableRs01B()
enableRs05A()
...
```

也暂时不引入 `MotorEndpoint`。当实际数量确定后，可在 `Robot` 中按协议放置明确的对象数组或引用数组，并用下标选择：

```cpp
std::array<device::RsMotor*, rs_count> rs_motors_;
std::array<device::DmMotor*, dm_count> dm_motors_;

bool enableRs(std::size_t index) noexcept;
bool stopRs(std::size_t index) noexcept;
bool sendRsMitControl(std::size_t index, ... ) noexcept;

bool enableDm(std::size_t index) noexcept;
bool disableDm(std::size_t index) noexcept;
bool sendDmMitControl(std::size_t index, ... ) noexcept;
```

这里数组只是减少同协议设备的重复 API，不改变现有分层。每个对象仍保存自己的型号、ID、限幅、状态话题；`Robot` 仍决定它属于哪条总线。

实现前必须满足：

- 每个 RS 对 `(motor_id, host_id)` 必须能被路由唯一识别。
- 每个 DM 的 command ID 和 feedback ID 必须按协议设置且不与同总线其他标准帧冲突。
- 每增加一台需要反馈的电机，就增加一条软件路由；当前容量 8 很快会不够。
- 如果不同对象分布在不同 CAN，总线归属不能只靠一个全局数组下标隐式猜测，应在 Robot 的明确配置中表达。

带宽必须先预算。Classic CAN 1 Mbps 下，一帧 8 字节数据帧考虑仲裁、CRC、填充和帧间隔后远大于 64 bit。当前两台电机各 1 kHz 命令已经产生约 2000 帧/秒，再叠加两台反馈。继续加入多台 1 kHz 命令和主动反馈可能迅速接近或超过总线能力。

可选策略：

- 普通测试先降到 500 Hz 或更低。
- 不同电机错开发送相位，避免同一毫秒集中入 FIFO。
- 将电机分散到 FDCAN1/2/3。
- 持续观察 `queued_transmit_frames`、`rejected_transmit_frames`、`hal_error`、RX drop、TEC/REC 和 bus-off。

## 14. 当前未完成项和优先级

### P0：下一步先处理的正确性和安全问题

1. 修复 `Robot::init()` 的返回值检查。

当前代码调用 6 次路由绑定和 3 次 `CanBus::start()`，但忽略所有返回值，最后无条件 `return true`。如果路由重叠、容量不足或某路 CAN 启动失败，`App` 仍会继续使能电机。应该逐项检查并在失败时返回 `false`。

2. 给控制命令增加真正的超时/失联保护。

当前只有“本地 `send()` 返回 false”时 stop/disable。如果主循环还在成功入 FIFO，但电机反馈已经丢失，系统不会停止。需要规定每台电机反馈超时阈值，在超时、fault 或 bus-off 时停止继续控制并请求 stop/disable。

3. 把自动上电使能改成明确的测试开关或测试状态机。

当前一上电就使能两台电机。继续做非零力矩、位置或速度测试前，至少需要 `Idle -> NeutralSent -> Enabled -> Running -> Fault/Stopped` 这样的简单状态，避免初始化失败一半后行为不清晰。

4. 恢复必要的 CAN2 统计和 RS/DM 新鲜度日志。

目前这些日志被注释，排查 1 kHz 双电机长期运行时看不到发送拒绝、接收掉帧和状态老化。日志必须节流，不能每毫秒打印。

### P1：完成电机层

1. 在低风险条件下实机验证 DJI `sendDjiTorqueCommands()`，先发全 0，再逐步给单个槽位极小命令。
2. 若实际要挂多台 RS/DM，再按已经确定的“协议内数组 + 下标接口”扩展，不提前设计未知数量。
3. 增加 Router 容量或根据实际总线分别设置容量，并检查绑定结果。
4. 核对 DM 反馈高 4 位字段的准确协议语义，重命名 `ERR`，明确正常运行状态与故障位。
5. 根据实际需要验证 DM4340，不要只依赖 DM4310 的测试结果。
6. 根据总线实际设备配置 HAL 硬件过滤器；当前 `CanBus::start()` 使用全局过滤接收所有未匹配标准/扩展数据帧，适合调试但不是最终配置。

### P2：迁移剩余驱动和进入控制框架

1. 迁移 VESC：它仍使用旧的 `CANCommand { can_id, uint64_t, length }`、`std::chrono` 和原子 `uint64_t` 状态缓存；应改成 `can::Frame`、`handleCanFrame()` 和 `StateTopic<MotorState>`。
2. 决定 Unitree、BMI088 等设备怎样进入统一状态模型。
3. 设计 `CommandTopic`：最新目标、时间戳、超时和来源优先级。
4. 建立固定周期控制循环，在控制周期中读取最新状态、判断新鲜度、运行 PID、发布最新命令。
5. 再实现 chassis 运动学和多电机协调，而不是直接在驱动里做底盘逻辑。
6. 整理 PID 文件状态。当前 `control/pid/pid.cpp` 和 `fuzzy_pid.cpp` 在工作树中被删除，而头文件仍在；`xmake` 可能提示 `control/**.cpp` 无匹配文件。不要擅自恢复删除文件，应先确认用户是否准备改为头文件实现。

## 15. 推荐的下一次实际推进顺序

建议新对话按下面顺序推进，每一步都单独编译和台架验证：

1. 先检查 `git status` 和当前 `Robot::init()`，只修复路由/CAN 启动返回值，不顺手重构其他文件。
2. 恢复节流后的 RS01、DM4310 和 CAN2 统计日志，运行现有双电机零力矩测试，确认 `reject=0`、`drop=0`、`hal=0` 且反馈持续新鲜。
3. 加一个简单测试状态机和反馈超时保护，验证拔掉某台电机或断开 CAN 时能进入停止路径。
4. 关闭或隔离 RS/DM 自动测试，再单独验证 DJI 全零组命令；确认正确后只对一个槽位施加很小的命令。
5. 得到最终实际电机数量、ID 和总线分配后，再实现多 RS/DM 数组接口并做总线负载预算。
6. 电机层稳定后再迁移 VESC、实现 CommandTopic 和 PID 控制循环。

不建议下一步立刻做大型“通用电机管理器”。当前最有价值的是先补齐初始化失败处理、反馈超时和实机诊断。

## 16. 当前工作树注意事项

交接时工作树不是干净状态，包含已暂存和未暂存修改。典型状态包括：

```text
M  STM32_ROBOT_FRAMEWORK_HANDOFF.md
MM app/app.cpp
M  app/app.hpp
M  components/containers/spsc_ring_buffer.hpp
M  components/messaging/state_topic.hpp
D  control/pid/fuzzy_pid.cpp
M  control/pid/fuzzy_pid.hpp
D  control/pid/pid.cpp
M  control/pid/pid.hpp
A  devices/motors/dji/dji_command_group.hpp
M  devices/motors/dji/dji_motor.hpp
M  devices/motors/dm/dm_motor.hpp
D  devices/motors/robostride/rs01.hpp
A  devices/motors/robostride/rs_motor.hpp
M  platform/can/can_router.hpp
M  platform/stm32/can_bus.cpp
M  platform/stm32/can_bus.hpp
M  platform/stm32/timebase.hpp
MM robot/robot.cpp
MM robot/robot.hpp
M  robot/robot_topics.hpp
```

规则：

- 这些修改都应视为用户当前成果，不得 `git reset --hard`、`git checkout --`、批量清理或覆盖。
- `MM` 表示同一文件同时有已暂存和未暂存修改；读取普通文件得到的是当前工作树最终内容。
- 新对话不能只看 `git diff`，还要同时看 `git diff --cached` 和文件本身。
- 当前 `HEAD` 仍是较早的接收链路提交，许多已验证成果还没有提交。

交接当日已经执行完整的 `xmake -r`，`application.elf` 构建成功。唯一构建告警是：

```text
warning: ./xmake.lua:33: cannot match add_files("control/**.cpp") in target(application)
```

这与当前两个 PID `.cpp` 被删除的工作树状态一致，不是 CAN 或电机代码编译失败。

## 17. 安全约束

所有后续实机动作都必须遵守：

1. 上电前确认 CANH/CANL、共地、波特率、终端电阻和电源电压。
2. 第一次非零输出时电机卸载或悬空，并准备硬件急停/断电。
3. 先发送中性帧，再使能；先验证反馈在线，再逐步增加极小目标。
4. `send() == true` 不代表电机已经执行。
5. 反馈存在不代表电机已经使能；使能与状态反馈是两个独立事实。
6. 控制退出、反馈超时、故障、总线错误和主循环异常都应进入 stop/disable 路径。
7. 不要同时开启多套 1 kHz 测试而不计算总线负载。
8. 修改电机 ID、机械零点、参数保存等持久化命令前必须单独确认，不能混入普通启动流程。

## 18. 一句话状态

框架已经完成了 `FDCAN ISR -> SPSC -> Router -> 电机解析 -> StateTopic` 的接收链路，以及 `电机生成 can::Frame -> Robot 选择 FDCAN2 -> CanBus::send -> HAL` 的发送链路；DM4310、RS05、RS01 已完成实机收发，RS01 与 DM4310 双 1 kHz 零力矩测试成功，DJI 反馈成功且组发送代码已完成但尚未实机命令验证。下一步不是继续堆抽象，而是补上 `Robot::init()` 失败检查、反馈超时/故障停止、可控的测试状态机和诊断，然后再根据真实电机数量扩展多电机接口。
