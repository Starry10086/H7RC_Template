#pragma once

#include "components/containers/fixed_pointer_queue.hpp"
#include "platform/stm32/async_transfer.hpp"
#include "platform/stm32/dma_buffer.hpp"

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_i2c.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace platform{
    enum class I2cOperation : uint8_t{
        MasterTransmit,
        MasterReceive,
        MemoryWrite,
        MemoryRead
    };

    enum class I2cMemoryAddressSize : uint8_t{
        Bits8,
        Bits16
    };

    struct I2cTransfer{
        I2cOperation operation{I2cOperation::MasterTransmit};
        uint8_t device_address_7bit{0U};

        uint16_t memory_address{0U};
        I2cMemoryAddressSize memory_address_size{I2cMemoryAddressSize::Bits8};

        uint8_t* tx_buffer{nullptr};
        uint8_t* rx_buffer{nullptr};

        uint16_t size{0U};
        uint32_t timeout_ms{0U};
        uint32_t start_time_ms{0U};

        TransferState state{TransferState::Idle};
        TransferError error{TransferError::None};

        void configure(I2cOperation op, uint8_t device_addr, uint16_t mem_addr, I2cMemoryAddressSize mem_addr_size,
                       uint8_t* tx, uint8_t* rx, uint16_t length, uint32_t timeout) noexcept{
            operation = op;
            device_address_7bit = device_addr;
            memory_address = mem_addr;
            memory_address_size = mem_addr_size;
            tx_buffer = tx;
            rx_buffer = rx;
            size = length;
            timeout_ms = timeout;
            error = TransferError::None;
        }

        void reset() noexcept{
            operation = I2cOperation::MasterTransmit;
            device_address_7bit = 0U;
            memory_address = 0U;
            memory_address_size = I2cMemoryAddressSize::Bits8;
            tx_buffer = nullptr;
            rx_buffer = nullptr;
            size = 0U;
            timeout_ms = 0U;
            start_time_ms = 0U;
            state = TransferState::Idle;
            error = TransferError::None;
        }
    };

class I2cBus final{
public:
    static constexpr std::size_t queue_capacity = 8U;

    explicit I2cBus(I2C_HandleTypeDef& handle) noexcept;

    I2cBus(const I2cBus&) = delete;
    I2cBus& operator=(const I2cBus&) = delete;

    bool submit(I2cTransfer& transfer) noexcept;
    void process(uint32_t now_ms) noexcept;

    void onTransferCompleteInterrupt() noexcept;
    void onErrorInterrupt() noexcept;
    void onAbortCompleteInterrupt() noexcept;

    I2C_HandleTypeDef& handle() noexcept {
        return handle_;
    }

    const TransferStats& stats() const noexcept {
        return stats_;
    }

private:
    enum class AbortReason : uint8_t{
        None,
        Timeout
    };

    static constexpr uint32_t event_complete = 1UL << 0U;
    static constexpr uint32_t event_error = 1UL << 1U;
    static constexpr uint32_t event_abort_complete = 1UL << 2U;

    void startNext(uint32_t now_ms) noexcept;
    void processActive(uint32_t now_ms) noexcept;
    void handleEvents(uint32_t events) noexcept;
    void endActive(TransferState state, TransferError error) noexcept;
    void timeoutActive() noexcept;

    static bool validMemoryAddressSize(I2cMemoryAddressSize size) noexcept;
    static uint16_t toHalMemoryAddressSize(I2cMemoryAddressSize size) noexcept;

    I2C_HandleTypeDef& handle_;

    container::FixedPointerQueue<I2cTransfer, queue_capacity> transfer_queue_;  // 最多存queue_capacity条事务
    I2cTransfer* active_transfer_{nullptr};
    std::atomic<uint32_t> pending_events_{0U};
    AbortReason abort_reason_{AbortReason::None};
    TransferStats stats_{};
};
}
