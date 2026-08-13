#include "platform/stm32/spi_bus.hpp"
#include "platform/stm32/async_transfer.hpp"
#include "stm32h7xx_hal_def.h"
#include <cstdint>

namespace platform{

    SpiBus::SpiBus(SPI_HandleTypeDef& handle) noexcept
    : handle_(handle)
    {}

    bool SpiBus::submit(SpiTransfer& transfer) noexcept{
        if(transfer.state != TransferState::Idle){
            return false;
        }

        if(transfer.tx_buffer == nullptr ||
           transfer.rx_buffer == nullptr ||
           transfer.size == 0U ||
           transfer.timeout_ms == 0U){
           transfer.error = TransferError::InvalidArgument;
           transfer.state = TransferState::Failed;
           return false;
        }

        if(!transfer.chip_select.valid()){
            transfer.error = TransferError::InvalidArgument;
            transfer.state = TransferState::Failed;
            return false;
        }

        if(!isDmaBuffer(transfer.tx_buffer, transfer.size) ||
           !isDmaBuffer(transfer.rx_buffer, transfer.size)){
            transfer.error = TransferError::InvalidDmaBuffer;
            transfer.state = TransferState::Failed;
            return false;
        }

        if(!transfer_queue_.push(transfer)){
            ++stats_.queue_rejected;
            return false;
        }

        transfer.state = TransferState::Queued;
        return true;
    }

    void SpiBus::process(uint32_t now_ms) noexcept{
        const uint32_t events = pending_events_.exchange(0U, std::memory_order_acq_rel);

        if(active_transfer_ != nullptr && events != 0U){
            handleEvents(events);
        }
        if(active_transfer_ != nullptr){
            processActive(now_ms);
        }
        if(active_transfer_ == nullptr){
            startNext(now_ms);
        }
    }

    void SpiBus::startNext(uint32_t now_ms) noexcept{
        SpiTransfer* next = transfer_queue_.pop();

        if(next == nullptr){
            return;
        }

        active_transfer_ = next;
        active_transfer_->state = TransferState::Active;
        active_transfer_->start_time_ms = now_ms;
        active_transfer_->error = TransferError::None;
        abort_reason_ = AbortReason::None;
        active_transfer_->chip_select.setActive(true);

        const HAL_StatusTypeDef result = HAL_SPI_TransmitReceive_DMA(
            &handle_,
            active_transfer_->tx_buffer,
            active_transfer_->rx_buffer,
            active_transfer_->size);
        if(result == HAL_OK){
            return;
        }

        endActive(TransferState::Failed, result == HAL_BUSY ? TransferError::HalBusy : TransferError::HalError);
    }

    void SpiBus::processActive(uint32_t now_ms) noexcept{
        if(active_transfer_ == nullptr){
            return;
        }
        if(abort_reason_ != AbortReason::None){
            return;
        }

        const uint32_t elapsed_ms = static_cast<uint32_t>(now_ms - active_transfer_->start_time_ms);
        if(elapsed_ms < active_transfer_->timeout_ms){
            return;
        }

        timeoutActive();
    }

    void SpiBus::timeoutActive() noexcept{
        if (active_transfer_ == nullptr || abort_reason_ != AbortReason::None) {
            return;
        }

        active_transfer_->chip_select.setActive(false);
        abort_reason_ = AbortReason::Timeout;

        const HAL_StatusTypeDef result = HAL_SPI_Abort_IT(&handle_);
        if (result == HAL_OK) {
            return;
        }

        endActive(TransferState::TimedOut, TransferError::Timeout);
    }

    void SpiBus::handleEvents(uint32_t event) noexcept{
        if(active_transfer_ == nullptr){
            return;
        }

        const bool complete = (event & event_complete) != 0U;
        const bool error = (event & event_error) != 0U;
        const bool abort_complete = (event & event_abort_complete) != 0U;

        if(abort_reason_ == AbortReason::Timeout){
            if(abort_complete){
                endActive(TransferState::TimedOut, TransferError::Timeout);
            }
            // 超时中止过程中必须等待 AbortComplete，不能因为中间 Error 提前启动下一笔事务。
            return;
        }

        if(error){
            endActive(TransferState::Failed, TransferError::HalError);
            return;
        }
        if(abort_complete){
            endActive(TransferState::Failed, TransferError::HalError);
            return;
        }
        if(complete){
            endActive(TransferState::Completed, TransferError::None);
        }
    }

    void SpiBus::endActive(TransferState state, TransferError error) noexcept{
        if(active_transfer_ == nullptr){
            return;
        }

        active_transfer_->chip_select.setActive(false);
        active_transfer_->state = state;
        active_transfer_->error = error;

        switch(state){
            case TransferState::Completed:
                ++stats_.completed;
            break;
            case TransferState::Failed:
                ++stats_.failed;
            break;
            case TransferState::TimedOut:
                ++stats_.timed_out;
            break;
            default:
                // 其他状态不应该在 endActive() 中出现
            break;
        }

        active_transfer_ = nullptr;
        abort_reason_ = AbortReason::None;
    }

    void SpiBus::onTxRxCompleteInterrupt() noexcept{
        if(active_transfer_ != nullptr){
            active_transfer_->chip_select.setActive(false);
        }
        pending_events_.fetch_or(event_complete,std::memory_order_release);
    }
    void SpiBus::onErrorInterrupt() noexcept{
        if(active_transfer_ != nullptr){
            active_transfer_->chip_select.setActive(false);
        }
        pending_events_.fetch_or(event_error,std::memory_order_release);
    }
    void SpiBus::onAbortCompleteInterrupt() noexcept{
        if(active_transfer_ != nullptr){
            active_transfer_->chip_select.setActive(false);
        }
        pending_events_.fetch_or(event_abort_complete,std::memory_order_release);
    }
}