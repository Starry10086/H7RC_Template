#include "platform/stm32/uart_port.hpp"

#include "platform/stm32/async_transfer.hpp"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_uart_ex.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace platform{

    UartPort::UartPort(UART_HandleTypeDef& handle, UartDmaStorage& dma_storage, UartPortConfig config) noexcept
        : handle_(handle)
        , dma_storage_(dma_storage) 
        , config_(config){}

    bool UartPort::init() noexcept{
        if(started_){
            return true;
        }

        if(config_.enable_rx){
            if(handle_.hdmarx == nullptr || 
               handle_.hdmarx->Init.Mode != DMA_CIRCULAR ||
               !isDmaBuffer(dma_storage_.rx.data(), dma_storage_.rx.size())){
                return false;
            }
        }

        if(config_.enable_tx){
            if(handle_.hdmatx == nullptr || handle_.hdmatx->Init.Mode != DMA_NORMAL){
                return false;
            }
        }

        rx_ring_.clear();

        last_dma_position_ = 0U;
        pending_tx_events_.store(0U, std::memory_order_relaxed);
        rx_restart_requested_.store(0U, std::memory_order_relaxed);
        if(config_.enable_rx && !startReception()){
            return false;
        }

        started_ = true;
        return true;
    }

    bool UartPort::startReception() noexcept{
        if(handle_.RxState != HAL_UART_STATE_READY){
            return false;
        }

        last_dma_position_ = 0U;
        __HAL_UART_CLEAR_FLAG(&handle_, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF | UART_CLEAR_IDLEF);
        __HAL_UART_SEND_REQ(&handle_, UART_RXDATA_FLUSH_REQUEST);
        const HAL_StatusTypeDef resule = HAL_UARTEx_ReceiveToIdle_DMA(&handle_, dma_storage_.rx.data(), static_cast<uint16_t>(dma_storage_.rx.size()));
        if(resule != HAL_OK){
            return false;
        }
        return true;
    }

    bool UartPort::submit(UartTxTransfer& transfer) noexcept {
        if (!started_ || !config_.enable_tx) {
            return false;
        }

        if (transfer.state != TransferState::Idle) {
            return false;
        }

        if (transfer.buffer == nullptr ||
            transfer.size == 0U ||
            transfer.timeout_ms == 0U) {
            transfer.error = TransferError::InvalidArgument;
            transfer.state = TransferState::Failed;
            return false;
        }

        if (!isDmaBuffer(transfer.buffer, transfer.size)) {
            transfer.error = TransferError::InvalidDmaBuffer;
            transfer.state = TransferState::Failed;
            return false;
        }

        if (!tx_queue_.push(transfer)) {
            ++tx_stats_.queue_rejected;
            // 队列满时保持 Idle，调用者之后可以重试。
            return false;
        }

        transfer.error = TransferError::None;
        transfer.state = TransferState::Queued;

        return true;
    }

    void UartPort::process(uint32_t now_ms) noexcept{
        if(!started_){
            return;
        }

        // HAL 错误回调只提出恢复请求，真正启动 DMA 仍然放在主循环中执行
        if(config_.enable_rx &&
           rx_restart_requested_.exchange(0U, std::memory_order_acq_rel) != 0U){
            if(!startReception()){
                // 如果重新启动接收失败，则将 rx_restart_requested_ 设置为 1，以便在下一次调用 process() 时再次尝试。
                rx_restart_requested_.store(1U, std::memory_order_relaxed);
            }
        }

        const uint32_t events = pending_tx_events_.exchange(0U, std::memory_order_acq_rel);
        if(active_tx_ != nullptr && events != 0U){
            handleTxEvents(events);
        }
        if(active_tx_ != nullptr){
            processActiveTx(now_ms);
        }
        if(active_tx_ == nullptr){
            startNextTx(now_ms);
        }
    }

    std::size_t UartPort::available() const noexcept{
        return rx_ring_.size();
    }

    std::size_t UartPort::read(uint8_t* destination, std::size_t capacity) noexcept{
        if(destination == nullptr || capacity == 0U){
            return 0U;
        }
        const std::size_t requested = std::min(capacity, rx_ring_capacity);
        return rx_ring_.pop(destination, requested);
    }

    // HAL_UARTEx_RxEventCallback() 的 Size 表示 DMA 当前写入位置，有效范围是1 ... rx_dma_capacity
    void UartPort::onRxEventInterrupt(uint16_t dma_position) noexcept {
        if (dma_position > static_cast<uint16_t>(rx_dma_capacity)) {
            dma_errors_.fetch_add(1U,std::memory_order_relaxed);
            return;
        }

        // UART错误已经发生，DMA即将被重新启动。
        // 不再接收当前DMA周期产生的迟到事件。
        if (rx_restart_requested_.load(std::memory_order_acquire) != 0U) {
            return;
        }

        const HAL_UART_RxEventTypeTypeDef event_type = HAL_UARTEx_GetRxEventType(&handle_);
        const uint16_t previous = last_dma_position_;

        /*
        * Circular DMA刚启动时，如果RX脚悬空产生空IDLE，
        * HAL可能回调Size=rx_dma_capacity。
        *
        * 这不代表真的收到了512字节。
        * 真正收满512字节时会先产生TC事件。
        */
        if (event_type == HAL_UART_RXEVENT_IDLE &&
            dma_position == static_cast<uint16_t>(rx_dma_capacity) &&
            previous == 0U) {
            return;
        }

        if (dma_position == previous) {
            return;
        }

        if (dma_position > previous) {
            pushRxRangeFromIsr(previous, dma_position);
        } 
        else{
            pushRxRangeFromIsr(previous,static_cast<uint16_t>(rx_dma_capacity));
            if (dma_position != 0U) {
                pushRxRangeFromIsr(0U,dma_position);
            }
        }

        last_dma_position_ = dma_position;
    }

    void UartPort::pushRxRangeFromIsr(uint16_t begin, uint16_t end) noexcept{
        if(begin == end){
            return;
        }

        const std::size_t count = static_cast<std::size_t>(end - begin);
        const std::size_t accepted = rx_ring_.pushFromIsr(dma_storage_.rx.data() + begin, count);
        rx_bytes_.fetch_add(static_cast<uint32_t>(accepted), std::memory_order_relaxed);
        rx_dropped_bytes_.fetch_add(static_cast<uint32_t>(count - accepted), std::memory_order_relaxed);
    }

    void UartPort::startNextTx(uint32_t now_ms) noexcept{
        UartTxTransfer* next = tx_queue_.pop();
        if(next == nullptr){
            return;
        }

        active_tx_ = next;
        active_tx_->state = TransferState::Active;
        active_tx_->error = TransferError::None;
        active_tx_->start_time_ms = now_ms;
        tx_abort_reason_ = TxAbortReason::None;

        const HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(&handle_, active_tx_->buffer, static_cast<uint16_t>(active_tx_->size));
        if(result == HAL_OK){
            return;
        }
        endActiveTx(TransferState::Failed, result == HAL_BUSY ? TransferError::HalBusy : TransferError::HalError);
    }

    void UartPort::processActiveTx(uint32_t now_ms) noexcept{
        if(active_tx_ == nullptr){
            return;
        }

        // 已经提出 Abort 请求，等待 AbortComplete，不能重复调用 HAL_UART_AbortTransmit_IT。
        if(tx_abort_reason_ != TxAbortReason::None){
            return;
        }

        const uint32_t elapsed = now_ms - active_tx_->start_time_ms;
        if(elapsed < active_tx_->timeout_ms){
            return;
        }
        timeoutActiveTx();
    }
    
    void UartPort::timeoutActiveTx() noexcept{
        if(active_tx_ == nullptr || tx_abort_reason_ != TxAbortReason::None){
            return;
        }

        tx_abort_reason_ = TxAbortReason::Timeout;
        const HAL_StatusTypeDef result = HAL_UART_AbortTransmit_IT(&handle_);
        if(result == HAL_OK){
            return;
        }
        endActiveTx(TransferState::TimedOut, TransferError::HalError);
    }

    void UartPort::handleTxEvents(uint32_t event) noexcept{
        if(active_tx_ == nullptr){
            return;
        }

        const bool complete = (event & tx_event_complete) != 0U;
        const bool error = (event & tx_event_error) != 0U;
        const bool abort_complete = (event & tx_event_abort_complete) != 0U;

        // 正在等待超时中止完成。
        if (tx_abort_reason_ == TxAbortReason::Timeout) {
            if (abort_complete) {
                endActiveTx(TransferState::TimedOut,TransferError::Timeout);
            }

            // 超时一旦确定，由超时语义接管。
            // 中途出现的 Error 或迟到的 Complete
            // 不允许释放事务，必须等待 AbortComplete。
            return;
        }
        if(complete){
            endActiveTx(TransferState::Completed, TransferError::None);
        }
        if(error){
            endActiveTx(TransferState::Failed, TransferError::HalError);
            return;
        }
        if(abort_complete){
            endActiveTx(TransferState::Failed, TransferError::HalError);
            return;
        }
    }

    void UartPort::endActiveTx(TransferState state, TransferError error) noexcept{
        if(active_tx_ == nullptr){
            return;
        }

        const uint16_t transfer_size = active_tx_->size;
        active_tx_->state = state;
        active_tx_->error = error;

        switch(state){
            case TransferState::Completed:
                ++tx_stats_.completed;
                tx_bytes_.fetch_add(static_cast<uint32_t>(transfer_size), std::memory_order_relaxed);
            break;
            case TransferState::Failed:
                ++tx_stats_.failed;
            break;
            case TransferState::TimedOut:
                ++tx_stats_.timed_out;
            break;
            default:
                // 其他状态不应该出现在这里
            break;
        }

        active_tx_ = nullptr;
        tx_abort_reason_ = TxAbortReason::None;
    }

    void UartPort::onTxCompleteInterrupt() noexcept{
        // HAL_UART_txCpltCallback 发生在 USART 最终 TC，不是 DMA 刚刚搬完最后一个字节时
        pending_tx_events_.fetch_or(tx_event_complete, std::memory_order_release);
    }

    void UartPort::onErrorInterrupt(uint32_t hal_error) noexcept {
        if ((hal_error & HAL_UART_ERROR_FE) != 0U) {
            framing_errors_.fetch_add(
                1U,
                std::memory_order_relaxed);
        }
        if ((hal_error & HAL_UART_ERROR_NE) != 0U) {
            noise_errors_.fetch_add(
                1U,
                std::memory_order_relaxed);
        }
        if ((hal_error & HAL_UART_ERROR_ORE) != 0U) {
            overrun_errors_.fetch_add(
                1U,
                std::memory_order_relaxed);
        }
        if ((hal_error & HAL_UART_ERROR_DMA) != 0U) {
            dma_errors_.fetch_add(
                1U,
                std::memory_order_relaxed);
        }
        constexpr uint32_t rx_error_mask =
            HAL_UART_ERROR_PE |
            HAL_UART_ERROR_FE |
            HAL_UART_ERROR_NE |
            HAL_UART_ERROR_ORE |
            HAL_UART_ERROR_DMA |
            HAL_UART_ERROR_RTO;

        if (config_.enable_rx && (hal_error & rx_error_mask) != 0U) {
            // 错误发生后，不复制 DMA 中尚未发布的尾部。
            // 这部分数据的完整性已经不再可靠。
            rx_restart_requested_.store(
                1U,
                std::memory_order_release);
        }
    }

    void UartPort::onTxAbortCompleteInterrupt() noexcept {
        pending_tx_events_.fetch_or(tx_event_abort_complete,std::memory_order_release);
    }

    UartPortStats UartPort::stats() const noexcept {
        UartPortStats result{};

        result.rx_bytes = rx_bytes_.load(std::memory_order_relaxed);
        result.rx_dropped_bytes = rx_dropped_bytes_.load(std::memory_order_relaxed);
        result.tx_bytes = tx_bytes_.load( std::memory_order_relaxed);
        result.framing_errors = framing_errors_.load(std::memory_order_relaxed);
        result.noise_errors = noise_errors_.load(std::memory_order_relaxed);
        result.overrun_errors = overrun_errors_.load(std::memory_order_relaxed);
        result.dma_errors = dma_errors_.load(std::memory_order_relaxed);
        result.tx = tx_stats_;

        return result;
    }
}
