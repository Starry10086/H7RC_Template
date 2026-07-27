#pragma once

#include "platform/stm32/can_bus.hpp"
#include "robot/robot.hpp"

namespace librmcs::app {

class App final {
public:
    App(FDCAN_HandleTypeDef& can1_handle,
        FDCAN_HandleTypeDef& can2_handle,
        FDCAN_HandleTypeDef& can3_handle) noexcept;

    App(const App&) = delete;
    App& operator=(const App&) = delete;

    [[nodiscard]] bool init() noexcept;
    void process() noexcept;

    void onFdcanRxFifo0Interrupt(
        FDCAN_HandleTypeDef& handle) noexcept;

private:
    platform::CanBus can1_;
    platform::CanBus can2_;
    platform::CanBus can3_;

    robot::Robot robot_;

    bool initialized_{false};
};

[[nodiscard]] App& instance() noexcept;

} // namespace librmcs::app