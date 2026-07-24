# STM32 设计模式与嵌入式 C++ 对话上下文

> 用途：在新对话中提供本次讨论的完整技术上下文。
>
> 建议新对话开场：`请先阅读仓库根目录的 STM32_DESIGN_PATTERNS_CONTEXT.md，并基于其中的工程现状和约束继续实现。`

## 1. 用户目标

用户正在学习如何在 STM32 工程中实际使用以下设计模式：

- 发布订阅模式（Publish/Subscribe）
- 观察者模式（Observer）
- 工厂模式（Factory）
- 单例模式（Singleton）
- 回调模式（Callback）
- 模块化模式（Modular Design）
- 命令模式（Command）

后续讨论进一步确认：用户考虑使用 C++ 实现这些模式，希望方案适合真实 STM32 工程，而不是照搬桌面软件写法。

## 2. 当前工程现状

工程路径：

```text
D:\STM32-Study\stm32_H7_xmake
```

已确认的技术栈：

- MCU：STM32H723
- 底层库：STM32 HAL / CMSIS
- 构建系统：xmake
- 工具链：`arm-none-eabi-gcc/g++`
- 语言配置：C11、C++20
- 运行模型：当前是裸机 `while (1)` 主循环，尚未引入 RTOS
- CubeMX 工程位于 `bsp/HAL/rccs_slave`
- `main.c` 当前只完成基础 HAL、系统时钟和 GPIO 初始化
- 应用层、服务层和可复用组件目录尚未建立

当前 `xmake.lua` 已包含以下关键配置：

- Cortex-M7、硬件浮点
- C++20
- 关闭 C++ 异常：`-fno-exceptions`
- 关闭 RTTI：`-fno-rtti`
- 关闭局部静态变量的线程安全初始化锁：`-fno-threadsafe-statics`
- 启用函数和数据段垃圾回收
- 自动读取 CubeMX Makefile 中的 HAL 源文件和包含目录

需要注意：`xmake.lua` 中应用层 C++ 文件的 `add_files` 目前仍是注释或尚未正式配置。

## 3. 总体结论

C++能够让设计模式表达得更清晰、更类型安全，尤其适合模块化、工厂、观察者、命令和依赖注入。但 C++ 不会自动让程序更好，也不意味着应该使用动态内存、复杂继承或桌面标准库风格。

推荐采用混合架构：

```text
CubeMX / HAL / CMSIS       使用 C，尽量不手工修改生成代码
驱动封装 / 服务 / 应用层   使用受约束的嵌入式 C++
main.c                     通过 C ABI 桥接调用 C++ 应用入口
```

第一阶段最值得落地的组合是：

```text
模块化 + HAL 回调桥接 + 静态事件队列 + App::init()/App::process()
```

不要为了“使用设计模式”而同时引入全部模式。模式应该解决已经存在的依赖、扩展或调度问题。

## 4. 各模式的含义和适用场景

| 模式 | 解决的问题 | STM32 常见用途 | 使用建议 |
|---|---|---|---|
| 模块化 | 隔离职责、状态和硬件细节 | LED、按键、电机、通信、传感器 | 所有工程都应该先做好 |
| 回调 | 工作完成后反向通知调用方 | UART、DMA、定时器、ADC完成 | 一对一通知优先使用 |
| 观察者 | 一个明确对象通知多个观察者 | 按键状态、连接状态、配置变化 | 适合模块内部或局部关系 |
| 发布订阅 | 互不认识的模块通过事件中心通信 | 故障、温度更新、通信状态 | 跨业务模块解耦时使用 |
| 工厂 | 根据配置选择不同实现 | BMP280/SHT30、不同板卡 | 只有存在多种实现时使用 |
| 单例 | 确保系统只有一个实例 | 配置、日志、事件总线 | 严格限制，优先显式传依赖 |
| 命令 | 把“动作”封装成可排队请求 | UART/CAN命令、按键动作 | 需要排队、延迟或统一入口时使用 |

### 4.1 回调、观察者和发布订阅的区别

这三者容易混淆：

- 回调是一种实现机制，通常是函数指针加 `context`，一般用于一对一通知。
- 观察者是一种一对多关系，被观察对象知道并保存观察者列表。
- 发布订阅通过事件总线或消息代理转发，发布者和订阅者互相不知道。
- 观察者和发布订阅底层都可以使用回调实现，但它们描述的是不同的依赖关系。

选择原则：

```text
驱动完成通知一个使用者       -> 回调
一个模块通知多个明确依赖方   -> 观察者
多个业务模块需要完全解耦     -> 发布订阅
```

## 5. 推荐工程目录

设计模式是实现方法，不建议建立 `observer/`、`singleton/`、`factory/` 之类的目录。应按职责组织：

```text
app/
    app.cpp
    app.hpp
    app_entry.h
    app_events.hpp
drivers/
    gpio/
        led.cpp
        led.hpp
        button.cpp
        button.hpp
    sensor/
        sensor.hpp
        bmp280.cpp
        sht30.cpp
services/
    sensor_service.cpp
    alarm_service.cpp
    communication_service.cpp
components/
    event_bus.hpp
    command_queue.hpp
bsp/HAL/rccs_slave/
    Core/
    Drivers/
```

各层职责：

- `bsp`：CubeMX 和芯片相关代码。
- `drivers`：对 HAL 和具体器件进行对象化封装，不包含业务规则。
- `components`：事件总线、固定队列等通用基础设施。
- `services`：传感器采集、故障处理、通信等业务服务。
- `app`：创建对象、连接依赖、决定初始化和调度顺序。

## 6. 推荐的 C/C++ 边界

不建议直接把 CubeMX 的 `main.c` 改成 `main.cpp`，因为重新生成代码时容易产生维护问题。推荐增加一个 C ABI 桥接头文件：

```c
// app/app_entry.h
#ifndef APP_ENTRY_H
#define APP_ENTRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void App_Init(void);
void App_Process(void);
void App_OnButtonInterrupt(uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif
```

`main.c` 只在 CubeMX USER CODE 区调用：

```c
/* USER CODE BEGIN Includes */
#include "app_entry.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
App_Init();
/* USER CODE END 2 */

while (1)
{
    /* USER CODE BEGIN 3 */
    App_Process();
    /* USER CODE END 3 */
}
```

C++ 文件提供实现：

```cpp
namespace {
App app;
}

extern "C" void App_Init(void)
{
    app.init();
}

extern "C" void App_Process(void)
{
    app.process();
}
```

## 7. App 作为组合根

推荐让 `App` 统一持有对象并显式注入依赖：

```cpp
class App final {
public:
    void init();
    void process();

private:
    EventBus event_bus_;
    Led led_;
    Button button_;
    AlarmService alarm_{led_, event_bus_};
};
```

优点：

- 对象生命周期清楚。
- 初始化顺序集中管理。
- 依赖关系在构造函数中可见。
- 比各模块到处调用单例更容易测试。

全局 `App` 对象的构造函数只能初始化普通数据和保存引用。硬件访问必须放到 `App::init()`，并确保它在 `HAL_Init()` 和 CubeMX 外设初始化之后调用。

## 8. C++ 实现建议

### 8.1 模块化

使用类封装内部状态和 HAL 细节：

```cpp
class Led final {
public:
    void init();
    void set(bool enabled);
    void toggle();

private:
    bool enabled_{false};
};
```

业务模块不得直接操作 LED 对应 GPIO，应只依赖 `Led` 的公开接口。

### 8.2 回调

驱动层优先使用没有堆分配的函数指针加上下文：

```cpp
using RxCallback = void (*)(const uint8_t *data,
                            uint16_t length,
                            void *context);
```

一般不建议在底层驱动中使用 `std::function`，因为它可能带来更大代码体积，并且某些捕获对象可能触发动态分配。

### 8.3 观察者

可以使用接口类和固定容量观察者数组：

```cpp
class IButtonObserver {
public:
    virtual ~IButtonObserver() = default;
    virtual void onButtonChanged(bool pressed) = 0;
};

std::array<IButtonObserver *, 4> observers_{};
```

通知通常是同步的，因此观察者处理函数必须很快。需要耗时操作时，应改为投递事件或命令。

### 8.4 发布订阅

建议定义明确的事件类型，并使用固定容量订阅表和静态环形队列：

```cpp
struct TemperatureUpdated {
    float celsius;
};

struct CommunicationLost {
    uint8_t channel;
};
```

事件总线至少需要以下能力：

```cpp
subscribe<Event>(handler);
post(event);
postFromISR(event);
dispatch();
```

`postFromISR()` 只能复制小型事件到队列；`dispatch()` 在主循环中调用订阅者。

### 8.5 工厂

只有一个功能存在多个实现时再引入接口和工厂：

```cpp
class ISensor {
public:
    virtual ~ISensor() = default;
    virtual bool init() = 0;
    virtual bool read(float &value) = 0;
};
```

嵌入式工厂应返回静态对象引用、指针或值对象，不应通过 `new` 创建长期对象。

### 8.6 单例

单例仅适用于确实全系统唯一的资源。即便是唯一资源，也优先由 `App` 持有并传给使用者。

局部静态单例在本工程中没有编译器线程安全保护，因为已经使用 `-fno-threadsafe-statics`。进入 RTOS 后，不能假设多任务或中断同时首次访问是安全的。

### 8.7 命令

命令需要频繁入队时，推荐轻量值类型，而不是动态创建多态对象：

```cpp
struct Command {
    void (*execute)(void *context, int32_t argument);
    void *context;
    int32_t argument;
};
```

串口、CAN 和按键可以生成同一种命令，再由主循环中的固定容量命令队列统一执行。

## 9. 中断和并发约束

所有 HAL 中断回调应遵循：

- 读取必要状态。
- 复制少量数据或写入静态队列。
- 设置标志后尽快退出。
- 不调用耗时业务逻辑。
- 不同步遍历事件订阅者。
- 不执行命令。
- 不使用 `printf`、`HAL_Delay`、`new/delete` 或可能阻塞的锁。

队列需要明确：

- 固定容量。
- 满队列处理策略，例如丢弃最新、覆盖最旧或记录故障。
- 主循环与 ISR 之间的临界区策略。
- 是否允许多个生产者。

STM32H7 使用 DMA 时还要单独考虑 D-Cache 一致性、缓冲区对齐和内存区域，这不是设计模式本身能够解决的问题。

## 10. 嵌入式 C++ 使用规则

推荐：

- `std::array`
- `std::span`
- `std::optional`
- 小型值类型
- `enum class`
- `constexpr`
- 引用或非拥有指针表达依赖
- 固定容量容器和队列

谨慎或避免：

- `new/delete`
- `std::vector`
- `std::string`
- `std::function`
- `std::shared_ptr`
- 异常
- RTTI 和 `dynamic_cast`
- 复杂的全局静态初始化

类、命名空间、引用、普通模板通常可以做到零额外运行时开销。虚函数会增加一次间接调用，并让每个对象保存虚表指针；低频业务路径通常可以接受，但高频采样和中断路径应谨慎。模板也可能因为多次实例化增加 Flash 占用，需要查看最终 ELF/map 文件。

## 11. xmake 后续配置

应用层建立后，需要在 `target("application")` 中加入类似配置：

```lua
add_files(
    "app/**.cpp",
    "drivers/**.cpp",
    "services/**.cpp"
)

add_includedirs(
    "app",
    "drivers",
    "services",
    "components"
)
```

如果 `components` 中存在 `.cpp` 文件，也应加入 `add_files("components/**.cpp")`。只有头文件的模板组件不需要加入源文件列表。

## 12. 推荐实施顺序

1. 建立 `app`、`drivers`、`services`、`components` 目录。
2. 建立 `app_entry.h`，让 `main.c` 调用 C++ 的 `App_Init/App_Process`。
3. 先实现一个简单 `Led` 和 `Button` C++ 模块。
4. HAL 中断回调只向应用层投递按键或通信事件。
5. 实现固定容量事件队列，在 `App::process()` 中分发。
6. 有多个明确观察者时再加入观察者接口。
7. 出现多个传感器或板卡实现时再加入工厂。
8. 出现 UART/CAN/按键统一动作入口时加入命令队列。
9. 构建并检查 Flash、RAM、警告和链接 map。
10. 后续若引入 FreeRTOS，再重新定义队列、临界区和模块线程归属。

## 13. 尚未完成的工作

本次聊天只完成了工程分析和架构建议，尚未实际执行以下修改：

- 尚未创建 `app/drivers/services/components` 目录。
- 尚未创建 C ABI 桥接文件。
- 尚未实现事件总线或命令队列。
- 尚未封装具体硬件驱动。
- 尚未修改 `xmake.lua` 纳入应用层文件。
- 尚未运行新增 C++ 架构的交叉编译验证。

新对话可以从“实现一个最小可编译的 C++ 应用骨架”开始，建议第一批文件仅包括：

```text
app/app.hpp
app/app.cpp
app/app_entry.h
drivers/led.hpp
drivers/led.cpp
```

先打通 `main.c -> C ABI -> C++ App -> Led`，编译成功后再逐步加入事件总线、观察者、工厂和命令队列。
