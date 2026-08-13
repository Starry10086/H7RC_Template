#pragma once

#include "platform/stm32/can_bus.hpp"
#include "platform/stm32/spi_bus.hpp"
#include "platform/stm32/i2c_bus.hpp"
#include "platform/stm32/uart_port.hpp"
#include "robot/robot.hpp"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_i2c.h"
#include "dma_storage.hpp"

#include <cstdint>

namespace app {

namespace{
    inline constexpr platform::UartPortConfig rx_only_uart{
        .enable_rx = true,
        .enable_tx = false
    };
}

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
    void onI2cTransferCompleteInterrupt(I2C_HandleTypeDef& handle) noexcept;
    void onI2cErrorInterrupt(I2C_HandleTypeDef& handle) noexcept;
    void onI2cAbortCompleteInterrupt(I2C_HandleTypeDef& handle) noexcept;
    void onUartRxEventInterrupt(UART_HandleTypeDef& handle, uint16_t dma_position) noexcept;
    void onUartTxCompleteInterrupt(UART_HandleTypeDef& handle) noexcept;
    void onUartErrorInterrupt(UART_HandleTypeDef& handle, uint32_t hal_error) noexcept;
    void onUartTxAbortCompleteInterrupt(UART_HandleTypeDef& handle) noexcept;

private:
    platform::CanBus can1_{hfdcan1};
    platform::CanBus can2_{hfdcan2};
    platform::CanBus can3_{hfdcan3};

    platform::SpiBus spi2_{hspi2};
    platform::I2cBus i2c2_{hi2c2};
    platform::UartPort uart10_{huart10, app::dma_storage::uart10, rx_only_uart};

    robot::Robot robot_;

    bool initialized_{false};
};

 App& instance() noexcept;

} // namespace app
