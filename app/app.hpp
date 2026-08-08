#pragma once

#include "platform/stm32/can_bus.hpp"
#include "platform/stm32/spi_bus.hpp"
#include "robot/robot.hpp"
#include "spi.h"
#include "stm32h7xx_hal_spi.h"

#include <cstdint>

namespace app {

class App final {
public:
    App() noexcept;

    App(const App&) = delete;
    App& operator=(const App&) = delete;

     bool init() noexcept;
    void process() noexcept;

    void onFdcanRxFifo0Interrupt(FDCAN_HandleTypeDef& handle) noexcept;
    void onSpiTxRxCompleteInterrupt(SPI_HandleTypeDef& handle) noexcept;
    void onSpiErrorInterrupt(SPI_HandleTypeDef& handle) noexcept;
    void onSpiAbortCompleteInterrupt(SPI_HandleTypeDef& handle) noexcept;

private:
    platform::CanBus can1_{hfdcan1};
    platform::CanBus can2_{hfdcan2};
    platform::CanBus can3_{hfdcan3};

    platform::SpiBus spi2_{hspi2};

    robot::Robot robot_;

    bool initialized_{false};
};

 App& instance() noexcept;

} // namespace app
