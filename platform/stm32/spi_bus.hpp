#pragma once

#include "components/containers/fixed_pointer_queue.hpp"
#include "platform/stm32/async_transfer.hpp"
#include "platform/stm32/dma_buffer.hpp"

#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace platform{

struct SpiChipSelect{
    GPIO_TypeDef* port{nullptr};
    uint16_t pin{0U};
    bool active_low{true};

    bool valid() const noexcept {
        return port != nullptr && pin != 0U;
    }

    void setActive(bool active) const noexcept {
        if(!valid()){
            return;
        }

        GPIO_PinState state{GPIO_PIN_RESET};
        if(active_low){
            state = active ? GPIO_PIN_RESET : GPIO_PIN_SET;
        }else{
            state = active ? GPIO_PIN_SET : GPIO_PIN_RESET;
        }

        HAL_GPIO_WritePin(port, pin, state);
    }
};

struct SpiTransfer{
    const uint8_t* tx_buffer{nullptr};
    uint8_t* rx_buffer{nullptr};
    uint16_t size{0U};
    uint32_t timeout_ms{0U};
    uint32_t start_time_ms{0U};

    SpiChipSelect chip_select{};

    TransferState state{TransferState::Idle};
    TransferError error{TransferError::None};

    void configure(const uint8_t* tx, uint8_t* rx, uint16_t length, uint32_t timeout, SpiChipSelect cs) noexcept{
        tx_buffer = tx;
        rx_buffer = rx;
        size = length;
        timeout_ms = timeout;
        chip_select = cs;
        error = TransferError::None;
    }

    void reset() noexcept{
        tx_buffer = nullptr;
        rx_buffer = nullptr;
        size = 0U;
        timeout_ms = 0U;
        start_time_ms = 0U;
        chip_select = {};
        state = TransferState::Idle;
        error = TransferError::None;
    }
};

enum class SpiEvent : uint8_t {
    None,
    Complete,
    Error,
    AbortComplete
};

class SpiBus final{
public:
    static constexpr std::size_t queue_capacity = 8U;
    explicit SpiBus(SPI_HandleTypeDef& handle) noexcept;

    SpiBus(const SpiBus&) = delete;
    SpiBus& operator=(const SpiBus&) = delete;

    bool submit(SpiTransfer& transfer) noexcept;
    void process(uint32_t now_ms) noexcept;

    void onTxRxCompleteInterrupt() noexcept;
    void onErrorInterrupt() noexcept;
    void onAbortCompleteInterrupt() noexcept;

    SPI_HandleTypeDef& handle() noexcept { return handle_; }
    TransferStats stats() const noexcept { return stats_; }
private:
    void startNext(uint32_t now_ms) noexcept;
    void processActive(uint32_t now_ms) noexcept;
    void finishActive(SpiEvent event) noexcept;
    void timeoutActive() noexcept;

    SPI_HandleTypeDef& handle_;
    container::FixedPointerQueue<SpiTransfer, queue_capacity> transfer_queue_;  // 最多存queue_capacity条事务
    SpiTransfer* active_transfer_{nullptr}; // 当前正在执行的事务
    std::atomic<uint8_t> event_{static_cast<uint8_t>(SpiEvent::None)};
    bool abort_requested_{false};
    TransferStats stats_{};
};

}
