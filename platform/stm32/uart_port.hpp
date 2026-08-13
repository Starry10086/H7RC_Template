#pragma once

#include "components/containers/fixed_pointer_queue.hpp"
#include "components/containers/spsc_ring_buffer.hpp"
#include "platform/stm32/async_transfer.hpp"
#include "platform/stm32/dma_buffer.hpp"

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>


namespace platform{

struct UartPortConfig{
    bool enable_rx{true};
    bool enable_tx{true};
};

struct UartDmaStorage{
    static constexpr std::size_t rx_capacity = 512U;
    alignas(32) std::array<uint8_t, rx_capacity> rx{};
};

struct UartTxTransfer{
    const uint8_t* buffer{nullptr};
    uint16_t size{0U};
    uint32_t timeout_ms{0U};
    uint32_t start_time_ms{0U};

    TransferState state{TransferState::Idle};
    TransferError error{TransferError::None};

    void config(const uint8_t* data, uint16_t length, uint32_t timeout) noexcept{
        buffer = data;
        size = length;
        timeout_ms = timeout;
        error = TransferError::None;
    }

    void reset() noexcept{
        buffer = nullptr;
        size = 0U;
        timeout_ms = 0U;
        start_time_ms = 0U;
        state = TransferState::Idle;
        error = TransferError::None;
    }
};

struct UartPortStats{
    uint32_t rx_bytes{0U};          // 成功放入软件 SPSC 的字节数。
    uint32_t rx_dropped_bytes{0U};  // 软件 SPSC 满时，未能进入软件环的新字节数
    uint32_t tx_bytes{0U};          // 成功发送的字节数
    uint32_t framing_errors{0U};    // 帧结构错误数
    uint32_t noise_errors{0U};      // 噪声错误数
    uint32_t overrun_errors{0U};    // 溢出错误数
    uint32_t dma_errors{0U};        // DMA传输错误数

    TransferStats tx{};
};

class UartPort final{
public:
    static constexpr std::size_t rx_dma_capacity = UartDmaStorage::rx_capacity;
    static constexpr std::size_t rx_ring_capacity = 1024U;
    static constexpr std::size_t tx_queue_capacity = 8U;

    UartPort(UART_HandleTypeDef& handle, UartDmaStorage& dma_storage, UartPortConfig config) noexcept;

    UartPort(const UartPort&) = delete;
    UartPort& operator=(const UartPort&) = delete;

    bool init() noexcept;
    bool submit(UartTxTransfer& transfer) noexcept;
    void process(uint32_t now_ms) noexcept;
    std::size_t available() const noexcept;
    std::size_t read(uint8_t* destination, std::size_t capacity) noexcept;

    void onRxEventInterrupt(uint16_t dma_position) noexcept;
    void onTxCompleteInterrupt() noexcept;
    void onErrorInterrupt(uint32_t hal_error) noexcept;
    void onTxAbortCompleteInterrupt() noexcept;

    UART_HandleTypeDef& handle() noexcept { return handle_; }
    UartPortStats stats() const noexcept;
private:
    enum class TxAbortReason : uint8_t{
        None,
        Timeout
    };

    static constexpr uint32_t tx_event_complete = 1UL << 0U;
    static constexpr uint32_t tx_event_error = 1UL << 1U;
    static constexpr uint32_t tx_event_abort_complete = 1UL << 2U;
    
    bool startReception() noexcept;
    void pushRxRangeFromIsr(uint16_t begin, uint16_t end) noexcept;
    void startNextTx(uint32_t now_ms) noexcept;
    void processActiveTx(uint32_t now_ms) noexcept;
    void timeoutActiveTx() noexcept;
    void handleTxEvents(uint32_t event) noexcept;
    void endActiveTx(TransferState state, TransferError error) noexcept;

    UART_HandleTypeDef& handle_;
    UartDmaStorage& dma_storage_;
    UartPortConfig config_;
    container::SpscRingBuffer<uint8_t, rx_ring_capacity> rx_ring_;
    container::FixedPointerQueue<UartTxTransfer, tx_queue_capacity> tx_queue_;
    // USART 和 RX DMA IRQ 必须具有相同抢占优先级，禁止两个 RX 回调嵌套
    uint16_t last_dma_position_{0U};// HAL RxEvent 的 Size 是 DMA 当前写入位置，不是新增长度。
    UartTxTransfer* active_tx_{nullptr};
    TxAbortReason tx_abort_reason_{TxAbortReason::None};
    std::atomic<uint32_t> pending_tx_events_{0U};
    std::atomic<uint32_t> rx_restart_requested_{0U};
    bool started_{false};

    std::atomic<uint32_t> rx_bytes_{0U};
    std::atomic<uint32_t> rx_dropped_bytes_{0U};
    std::atomic<uint32_t> tx_bytes_{0U};
    std::atomic<uint32_t> framing_errors_{0U};
    std::atomic<uint32_t> noise_errors_{0U};
    std::atomic<uint32_t> overrun_errors_{0U};
    std::atomic<uint32_t> dma_errors_{0U};

    // 只在主循环更新。
    TransferStats tx_stats_{};
};
}
