#pragma once

#include "devices/motors/dji/dji_motor.hpp"
#include "platform/can/can_router.hpp"
#include "platform/stm32/can_bus.hpp"
#include "robot/robot_topics.hpp"

#include <cstddef>

namespace librmcs::robot{

class Robot final{
public:
    Robot(platform::CanBus& can1,
          platform::CanBus& can2,
          platform::CanBus& can3) noexcept;

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    [[nodiscard]] bool init() noexcept;
    void processCanRx() noexcept;

    [[nodiscard]] RobotTopics& topics() noexcept { return topics_; }

private:
    static constexpr std::size_t router_capacity_ = 8U;     // 每个CAN总线最多绑定8条路由,可以调整大小
    static constexpr std::size_t rx_budget_per_bus_ = 16U;  // 每次主循环中，每条 CAN 总线最多处理多少个已经接收到的软件队列帧，可以调整大小

    using CanRouter = can::Router<router_capacity_>;

    static void processCanBus(platform::CanBus& bus, CanRouter& router) noexcept;

    platform::CanBus& can1_;
    platform::CanBus& can2_;
    platform::CanBus& can3_;

    RobotTopics topics_{};

    device::DjiMotor chassis_left_front_;
    device::DjiMotor chassis_right_front_;
    device::DjiMotor chassis_right_back_;
    device::DjiMotor chassis_left_back_;

    CanRouter can1_router_{};
    CanRouter can2_router_{};
    CanRouter can3_router_{};
};

}