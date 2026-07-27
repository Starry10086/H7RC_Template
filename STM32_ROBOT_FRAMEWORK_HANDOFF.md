# STM32H7 机器人控制框架对话交接文档

> 更新时间：2026-07-26
>
> 用途：在新对话中恢复目前已经讨论并实现的机器人控制框架上下文。
>
> 新对话建议直接发送：
>
> `请先完整读取仓库根目录的 STM32_ROBOT_FRAMEWORK_HANDOFF.md，再检查文档中提到的当前代码。请延续原来的教学方式，从“统一 MotorState 和 DJI 电机反馈接入”继续，不要重新推翻架构，也不要直接替我修改代码，先把需要手写的代码和原理详细讲给我。`

## 1. 用户目标和协作方式

用户正在学习如何为 STM32 机器人项目搭建一个可维护、可扩展的 C++ 框架。用户希望理解每一层为什么这样设计，并亲手抄写代码，而不是一次性让助手生成完整工程。

后续对话应遵循以下方式：

1. 每次只推进一个能够独立理解、独立编译的小步骤。
2. 先给出完整代码，再用通俗语言解释数据流、对象关系和每个成员的作用。
3. 用户写完后，先读取用户当前代码并做审查，不要直接覆盖或重构。
4. 审查后运行 `xmake -r`，但要注意：未被业务代码使用的模板可能没有被实例化，需要额外做最小实例化检查。
5. 不要推翻已经确认的分层和数据流，除非发现明确的技术问题，并说明原因。
6. 当前工作树包含大量用户自己的新增和修改，不能回退或清理这些改动。

## 2. 工程环境

工程路径：

```text
/Users/starry/Documents/STM32_Project/stm32_H7_xmake
```

技术环境：

- MCU：STM32H723
- 底层：STM32 HAL / CMSIS
- 构建系统：xmake
- 工具链：`arm-none-eabi-gcc/g++`
- 语言：C11、C++20
- 当前运行模式：裸机 `while (1)`，没有 RTOS
- CubeMX 工程：`bsp/HAL/rccs_slave`
- C++ 异常已关闭：`-fno-exceptions`
- RTTI 已关闭：`-fno-rtti`
- 局部静态初始化锁已关闭：`-fno-threadsafe-statics`
- 启用了 `-Wall -Wextra -Werror -pedantic-errors`
- `xmake.lua` 已包含 `app`、`components`、`control`、`devices`、`platform`、`robot` 下的源文件

当前 CubeMX 已初始化：

- GPIO
- SPI2
- FDCAN1
- FDCAN2
- FDCAN3

三路 FDCAN 当前均使用经典 CAN，RX FIFO0 为 32 个元素，TX FIFO 为 32 个元素。`main.c` 已调用 `MX_FDCAN1_Init()`、`MX_FDCAN2_Init()`、`MX_FDCAN3_Init()`，但还没有调用 `App_Init()` 和 `App_Process()`。

## 3. 项目涉及的设备和功能

CAN 设备：

- DJI M3508
- DJI GM6020
- DJI M2006
- 达妙 DM4310
- 达妙 DM4340
- VESC
- 工程中还已有 Robostride RS01 驱动

其他外设：

- RS485：宇树 GO1 电机
- SPI：BMI088 IMU
- WS2812
- UART
- I2C

控制和通用组件：

- 普通 PID
- 模糊 PID
- 底盘运动学解算
- 环形缓冲区
- 后续还会有机器人模式、控制器和安全管理逻辑

## 4. 已确定的总体分层

目录按“职责”划分，不按“设计模式名字”划分。不要建立 `factory/`、`observer/`、`singleton/` 这种目录。

```text
bsp/
    HAL/rccs_slave/            CubeMX、HAL、启动文件、中断入口

platform/
    can/                       与芯片无关的 CAN 帧和路由定义
    stm32/                     STM32 HAL 外设封装

components/
    containers/                SPSC 队列、字节 ringbuffer 等通用容器
    messaging/                 StateTopic、后续 CommandTopic/EventTopic

devices/
    motors/dji/                DJI 协议解析和命令编码
    motors/dm/                 达妙协议解析和命令编码
    motors/vesc/               VESC 协议解析和命令编码
    motors/robostride/         RS01 协议
    motors/unitree/            GO1 RS485 协议
    imu/                       BMI088 驱动
    led/                       WS2812 驱动

control/
    pid/                       可复用 PID 算法
    chassis/                   后续放底盘运动学解算

robot/
    robot_topics.hpp           机器人实际使用的话题对象
    robot.hpp/.cpp             设备对象、路由绑定和机器人级组合
    controllers/ 或 subsystems/ 后续放底盘、云台等固定周期控制逻辑

app/
    app.hpp/.cpp               最外层生命周期和主循环调度
    app_entry.h                C 到 C++ 的 ABI 桥
    interrupt_bridge.cpp       HAL 中断回调到 C++ 对象的桥接
```

各层边界：

- `bsp` 只负责芯片生成代码和 HAL 中断入口。
- `platform` 负责“怎么收发一帧”，不理解电机协议。
- `devices` 负责“这一帧对于某个设备是什么意思”，不运行底盘控制策略。
- `components` 提供与具体设备无关的固定容量容器和消息机制。
- `control` 放 PID、滤波、运动学等可复用算法。
- `robot` 描述这台机器人实际有哪些设备、ID、话题和控制子系统。
- `app` 决定初始化顺序和每次主循环调用顺序，是最外层组合根。

### 关于 `config` 文件夹

`config` 不是必须的，也不是没有用。当前阶段可以把少量电机 ID、反向和减速比配置直接放在 `Robot` 的初始化代码中。只有出现以下情况时再建立 `config/`：

- 有多种机器人或板卡型号
- 同一套代码需要多套电机布局
- 参数数量太多，已经影响 `Robot` 的可读性
- 参数需要集中生成或统一审查

`config` 只保存配置，不保存控制逻辑、驱动代码或运行状态。

## 5. 已确定的消息模型

话题必须是强类型 C++ 对象，不能在运行时通过字符串查找对象。

例如：

```cpp
messaging::StateTopic<MotorState> left_front_state{"motor.left_front.state"};
```

字符串名字只用于日志、调试和可读性；真正的连接关系通过对象引用、模板类型和显式绑定建立。

三类数据采用不同模型：

| 数据类型 | 推荐模型 | 原因 |
|---|---|---|
| 传感器/电机状态 | `StateTopic<T>`，只保留最新值 | 控制器需要最新状态，不需要把旧状态逐条补完 |
| 电机控制目标 | 后续 `CommandTopic<T>`，最新命令加时间戳/超时 | 高频控制命令不应排队，否则会执行过时命令 |
| 故障、按键、一次性动作 | 后续 `EventTopic<T>` 或固定队列 | 每个事件通常都需要处理一次 |
| ISR 到主循环的原始字节/帧 | SPSC ring buffer | 用于跨中断边界安全搬运数据 |

### StateTopic 的超时结论

`StateTopic` 自己不负责自动超时，也不应该在超时后偷偷清空数据。它保存：

- 最新值
- 发布时间戳
- sequence 版本号

消费者使用 `isFresh(now, timestamp, timeout)` 判断数据是否足够新。不同消费者可以使用不同阈值：

- 控制器可能要求 10 ms 或 20 ms 内的新反馈
- 设备在线检测可能允许 100 ms 或 200 ms

因此时间戳是必要的，但“超时策略”属于控制器或设备监督器，不属于通用 `StateTopic`。

## 6. CAN 反馈方向的最终数据流

用户已经选择“三条 CAN 总线各自拥有独立 Router”。总线信息由 Router 对象本身隐含，不需要再放进路由键中。

```text
FDCAN1 中断 -> CanBus1 的 SPSC ring -> 主循环 -> Router1 -> CAN1 上对应电机
FDCAN2 中断 -> CanBus2 的 SPSC ring -> 主循环 -> Router2 -> CAN2 上对应电机
FDCAN3 中断 -> CanBus3 的 SPSC ring -> 主循环 -> Router3 -> CAN3 上对应电机
```

完整反馈链路：

```text
CAN 硬件收到帧
    -> HAL_FDCAN_RxFifo0Callback
    -> 对应 CanBus::onRxFifo0Interrupt()
    -> ISR 把 can::Frame 放入该总线自己的 SPSC ring
    -> App/Robot 主循环调用 CanBus::popReceived()
    -> 该总线自己的 Router::dispatch(frame)
    -> Router 按 IdFormat + FrameKind + ID/mask 找到接收者
    -> 对应电机的 handleCanFrame(frame)
    -> 电机校验长度并解析协议
    -> 电机发布 MotorState 到自己的 StateTopic
    -> 固定周期控制器读取 StateTopic
    -> 检查时间戳是否新鲜
    -> PID/运动学计算
```

关键约束：

- 中断中不解析电机协议、不发布话题、不运行 PID。
- 不给每个电机建立一个原始 CAN ringbuffer；每条 CAN 总线一个接收 ringbuffer 即可。
- Router 只负责找到接收对象，不负责理解 DJI、DM 或 VESC 数据格式。
- 电机驱动必须再次检查帧长度和必要的协议字段。
- 收到状态后不要立刻在回调里运行控制器；控制器应按固定周期读取最新状态。
- 主循环每次处理 CAN 帧时后续应设置预算，避免总线繁忙时饿死 PID、IMU 和其他任务。

## 7. CAN 命令方向的架构决定

反馈方向和命令方向不能简单反向复用同一条调用链。

```text
遥控/规划目标
    -> Robot 控制器
    -> PID/运动学
    -> 发布 MotorCommand
    -> 安全检查/限幅/离线处理
    -> 电机协议编码
    -> 对应总线发送调度器
    -> FDCAN 或 RS485
```

原因：

- DJI 多个电机电流命令需要合并到 `0x200`、`0x1FF` 等组帧中，不能让每个电机随时独立发送。
- DM、RS01、VESC 的命令 ID 和数据格式不同。
- GO1 使用 RS485，需要轮询、半双工方向控制和超时调度。
- 控制命令需要统一做超时归零、急停、使能状态和限幅。

命令发送框架还没有实现，当前优先完成反馈方向。

## 8. 已经完成并验证的代码

### 8.1 `components/containers/spsc_ring_buffer.hpp`

已实现固定容量单生产者、单消费者队列：

- ISR 调用 `pushFromIsr()`
- 主循环调用 `pop()`
- 使用 `std::atomic<uint32_t>` 和 acquire/release 内存序
- 内部使用 `Capacity + 1` 个槽位区分空和满
- 对外有效容量是 `Capacity`
- 没有动态内存

它只适用于一个生产者和一个消费者。当前 CAN 用法是同一条总线的 ISR 生产、主循环消费，符合要求。

### 8.2 `components/messaging/state_topic.hpp`

已实现：

- `StateSample<T>`：值、`timestamp_ms`、`sequence`
- `publish()`
- `read()`
- `readIfNew()`
- `name()`
- `isFresh()`

当前设计默认所有 publish/read 都发生在主循环上下文，所以没有使用原子变量。不能直接从 ISR 发布。

### 8.3 `platform/can/can_types.hpp`

已定义：

- `IdFormat::Standard/Extended`
- `FrameKind::Data/Remote`
- `Frame`：ID、ID 类型、帧类型、长度和 8 字节数据

当前只支持经典 CAN 的最多 8 字节数据。

### 8.4 `platform/stm32/can_bus.hpp/.cpp`

已实现：

- 每个 `CanBus` 对象绑定一个 `FDCAN_HandleTypeDef`
- 每条总线拥有自己的 `SpscRingBuffer<can::Frame, 64>`
- `start()` 启动 FDCAN 并打开 FIFO0 新消息通知
- `onRxFifo0Interrupt()` 从硬件 FIFO 读取经典 CAN 帧并压入队列
- `popReceived()` 供主循环读取
- 统计收到帧、软件队列丢帧、无效帧和 HAL 错误

注意当前真实状态：

- `CanBus::start()` 通过 `HAL_FDCAN_ConfigGlobalFilter()` 接收所有未匹配的标准帧和扩展帧到 FIFO0，同时拒绝远程帧。
- 最近取消的是 Router 的全匹配软件路由，不等于已经收紧 FDCAN 硬件过滤器。
- 如果后续总线负载较大，可以再根据已绑定 ID 配置硬件 filter；初期调通时全接收更容易排查。

### 8.5 `platform/can/can_router.hpp`

已实现固定容量 `Router<MaxRoutes>`：

- `bindExact()`：精确 ID 绑定
- `bindMask()`：掩码范围绑定
- `dispatch()`：按格式、帧类型和 ID 进行首个匹配
- 统计绑定数、成功路由帧数和无人处理帧数
- 不使用虚函数、动态内存、`std::function` 或字符串查找
- 回调采用函数指针加 `void* context`

已经确认正确的规则：

- 标准帧 ID 只能使用低 11 位
- 扩展帧 ID 只能使用低 29 位
- mask 也不能超出对应位宽
- 普通路由拒绝 `mask == 0`
- `id &= mask` 会清除不参与匹配的位
- 路由重叠公式正确：`((route.id ^ id) & (route.mask & mask)) == 0`
- 相同 ID 的标准帧、扩展帧、数据帧和远程帧属于不同匹配空间
- 因为绑定阶段拒绝重叠，所以 `dispatch()` 首次匹配后返回是确定的

用户决定不额外编写 `maxId()` 函数，这是可以的；当前局部 `valid_bits` 已完成相同的范围检查。

Router 当前没有 wildcard、fallback 或 tap/sniffer 功能。用户已经决定现阶段不全盘领取所有 CAN 消息，因此不需要加入这些功能。

最近一次代码审查的唯一建议是给以下两个公开函数添加 `[[nodiscard]]`，避免调用者静默忽略绑定失败：

```cpp
template<typename Receiver>
[[nodiscard]] bool bindExact(...);

template<typename Receiver>
[[nodiscard]] bool bindMask(...);
```

所有绑定结果都必须在 `Robot::init()` 中检查。绑定失败的可能原因包括容量不足、ID/mask 非法和路由重叠。

Router 保存接收对象的裸指针，因此接收对象必须比 Router 活得更久。路由只应在初始化阶段绑定，不能在主循环分发期间修改。

## 9. 最近一次验证结果

2026-07-26 已完成：

- `xmake -r` 整个工程交叉编译和链接成功
- 使用项目的 C++20、`-Werror`、禁异常配置专门实例化 Router，编译成功
- 额外行为测试通过：精确匹配、掩码匹配、零 mask 拒绝、非法 ID 拒绝、非法 mask 拒绝、重叠拒绝、标准/扩展帧隔离、数据/远程帧隔离、容量上限和统计值

尚未完成真实硬件收帧测试。

## 10. 当前已有电机驱动的状态

已有驱动文件包含较多协议换算和命令生成代码，应尽量复用已验证的公式，不要一次性全部重写：

```text
devices/motors/dji/dji_motor.hpp
devices/motors/dm/dm_motor.hpp
devices/motors/vesc/vesc_motor.hpp
devices/motors/robostride/rs01.hpp
devices/motors/unitree/go1_motor.hpp/.cpp
```

CAN 电机驱动目前仍采用旧方式：

```text
store_status(uint64_t)
    -> std::atomic<uint64_t> 保存原始帧
    -> update_status() 延后解析
    -> std::chrono 判断在线
```

这个方式还没有接入新 Router，需要逐步迁移为：

```text
Router::dispatch(frame)
    -> motor.handleCanFrame(frame)
    -> 主循环中直接解析 frame.data
    -> 发布 StateTopic<MotorState>
```

迁移原因：

- ISR 和电机之间已经由 SPSC ringbuffer 隔离，电机解析全在主循环中，不再需要额外的 `atomic<uint64_t>`。
- Cortex-M7 上不能理所当然地认为 64 位原子操作无锁。
- 裸机工程不适合依赖桌面语义的 `std::chrono::steady_clock`；应使用项目 timebase 或 `HAL_GetTick()`。
- 原始 CAN 数据不需要在 CanBus 队列、电机 atomic 和 StateTopic 中重复缓存三次。

### 关于每种电机的文件和对象数量

每种电机协议保留一组 `.hpp/.cpp` 即可，不需要每个实际电机创建一份源文件。实际电机数量由 `Robot` 中的对象或 `std::array` 表达。

```cpp
std::array<DjiMotor, 4> chassis_motors;
std::array<DmMotor, 2> joint_motors;
```

同一种类型适合放在同一个 `std::array`。不同 C++ 类型通常使用不同数组；没有明确的运行时多态需求时，不必为了放进同一个数组引入基类和工厂。

工厂模式也不是 SPI、CAN 或电机框架的必需条件。当前设备类型和数量在编译期已知，显式构造、模板、`std::array` 和依赖注入更简单。只有运行时需要根据板卡、EEPROM 配置或探测结果选择不同实现时，才值得引入工厂。

## 11. 还没有实现的部分

以下文件目前存在但内容为空：

```text
app/app.cpp
app/app.hpp
app/app_entry.h
app/interrupt_bridge.cpp
robot/robot.cpp
robot/robot.hpp
robot/robot_topics.hpp
```

因此当前还没有真正连通：

- `main.c -> App_Init/App_Process`
- `HAL_FDCAN_RxFifo0Callback -> CanBus`
- `CanBus -> 每条总线的 Router`
- `Router -> motor.handleCanFrame()`
- `motor -> StateTopic<MotorState>`
- `StateTopic -> 固定周期控制器`

另外还未实现：

- 统一的 `MotorState`
- `MotorCommand` 和 `CommandTopic`
- 电机发送调度器
- `CanBus::send()` 或独立 CAN TX 封装
- DJI 组帧发送
- 离线监督、安全停机和命令超时
- 底盘控制器和底盘运动学接线
- 三路 CAN 的实际 Robot 对象和 ID 配置
- 主循环任务周期和每轮 CAN 帧处理预算
- FDCAN 硬件 ID 过滤策略

## 12. 下一步应从这里继续

当前优先完成反馈方向，不要先写工厂、命令总线或完整底盘控制。

推荐顺序：

1. 给 Router 的 `bindExact()` 和 `bindMask()` 添加 `[[nodiscard]]`，重新编译。
2. 新建统一状态类型 `devices/motors/motor_state.hpp`。
3. 只选择 DJI 驱动作为第一种迁移对象，实现 `handleCanFrame(const can::Frame&)`。
4. DJI 驱动在主循环中直接解析 8 字节反馈并发布自己的 `StateTopic<MotorState>`。
5. 在 `RobotTopics` 中创建具体电机状态话题。
6. 在 `Robot` 中创建 DJI 电机对象，并将对应状态话题引用注入电机。
7. 在每条 CAN 总线自己的 Router 中绑定对应反馈 ID。
8. 实现主循环从每条 `CanBus` 取帧并交给相应 Router，加入每轮帧数预算。
9. 实现 HAL 回调桥和 `App_Init/App_Process`，打通真实硬件反馈。
10. 第一种 DJI 电机完整跑通后，再按同一接口迁移 DM、VESC 和 RS01。

建议的初版统一状态：

```cpp
namespace librmcs::device {

struct MotorState {
    float position_rad{0.0F};
    float velocity_rad_s{0.0F};
    float torque_nm{0.0F};
    float temperature_c{0.0F};
    uint32_t fault_code{0};
};

}
```

时间戳和 sequence 已经由 `StateSample<MotorState>` 保存，因此不要在 `MotorState` 里重复保存。在线状态也优先根据时间戳新鲜度推导，不要保存一个可能过期的 `online = true`。

DJI 迁移时应保留现有角度、速度、电流/力矩、减速比、反向和多圈角度计算公式，只替换原始数据进入方式和状态输出方式。

初版接口方向：

```cpp
class DjiMotor {
public:
    DjiMotor(const Config& config,
             messaging::StateTopic<MotorState>& state_topic);

    void handleCanFrame(const can::Frame& frame) noexcept;
};
```

Router 已经匹配了 ID，但 `handleCanFrame()` 仍应检查：

- `frame.id_format == IdFormat::Standard`
- `frame.kind == FrameKind::Data`
- `frame.length == 8`

发布时间可以先使用项目 timebase 或 `HAL_GetTick()`。如果未来非常在意软件队列等待造成的时间偏差，可以再给 `can::Frame` 增加接收时间戳；这不是当前第一步的阻塞项。

## 13. 推荐的最终对象关系

下面是方向，不要求下一步一次完成：

```text
App
├── CanBus1
├── CanBus2
├── CanBus3
└── Robot（持有或引用三条总线）
    ├── RobotTopics
    ├── Router1
    ├── Router2
    ├── Router3
    ├── DJI/DM/VESC/RS01 电机对象或数组
    ├── BMI088/GO1/WS2812 等设备
    └── ChassisController 等固定周期控制子系统
```

成员声明顺序必须保证被引用对象先构造：

- Topic 先于持有 Topic 引用的电机对象
- 电机对象先于保存电机指针的 Router 生命周期结束
- HAL 外设完成 CubeMX 初始化之后，才能调用 `CanBus::start()`
- 应先创建对象和绑定路由，再启动 CAN 接收通知

## 14. 当前阶段不要做的事情

- 不要因为设备种类多就立刻引入复杂工厂和继承层次。
- 不要用字符串作为实时话题查找键。
- 不要在中断中解析电机、发布状态、跑 PID 或发送成组命令。
- 不要给每个电机分配一个原始帧 ringbuffer。
- 不要把高频状态和高频控制命令都做成排队消息。
- 不要使用 `new/delete`、`std::function`、`std::shared_ptr` 或异常。
- 不要一次迁移全部电机驱动；先用 DJI 跑通端到端反馈链路。
- 不要直接修改 CubeMX 生成代码的非 `USER CODE` 区域，C/C++ 连接使用明确的 C ABI 桥。

## 15. 补充文档

仓库根目录的 `STM32_DESIGN_PATTERNS_CONTEXT.md` 保存了更早期关于模块化、回调、观察者、发布订阅、工厂、单例和命令模式的基础解释，但其中的“工程尚未建立应用层”等状态已经过时。

新对话应以本交接文档记录的当前工程状态为准，需要回顾设计模式概念时再参考旧文档。
