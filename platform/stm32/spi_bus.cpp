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
        const auto event = static_cast<SpiEvent>(event_.exchange(static_cast<uint8_t>(SpiEvent::None),std::memory_order_acq_rel));

        if(active_transfer_ != nullptr && event != SpiEvent::None){
            finishActive(event);
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
        abort_requested_ = false;
        active_transfer_->chip_select.setActive(true);

        const HAL_StatusTypeDef result = HAL_SPI_TransmitReceive_DMA(
            &handle_,
            active_transfer_->tx_buffer,
            active_transfer_->rx_buffer,
            active_transfer_->size);
        if(result == HAL_OK){
            return;
        }

        active_transfer_->chip_select.setActive(false);
        active_transfer_->error = result == HAL_BUSY ? TransferError::HalBusy : TransferError::HalError;

        active_transfer_->state = TransferState::Failed;
        active_transfer_ = nullptr;
        ++stats_.failed;
    }

    void SpiBus::processActive(uint32_t now_ms) noexcept{
        if(active_transfer_ == nullptr ||
           abort_requested_){
            return;
        }

        const uint32_t elapsed_ms = static_cast<uint32_t>(now_ms - active_transfer_->start_time_ms);
        if(elapsed_ms <= active_transfer_->timeout_ms){
            return;
        }

        timeoutActive();
    }

    void SpiBus::timeoutActive() noexcept{
        if (active_transfer_ == nullptr) {
            return;
        }

        active_transfer_->chip_select.setActive(false);
        abort_requested_ = true;

        const HAL_StatusTypeDef result = HAL_SPI_Abort_IT(&handle_);
        if (result == HAL_OK) {
            return;
        }

        active_transfer_->error = TransferError::Timeout;
        active_transfer_->state = TransferState::TimedOut;
        active_transfer_ = nullptr;
        abort_requested_ = false;
        ++stats_.timed_out;
    }

    void SpiBus::finishActive(SpiEvent event) noexcept{
        if(active_transfer_ == nullptr){
            return;
        }

        active_transfer_->chip_select.setActive(false);

        switch(event){
            case SpiEvent::Complete:
                active_transfer_->state = TransferState::Completed;
                active_transfer_->error = TransferError::None;
                ++stats_.completed;
            break;
            case SpiEvent::Error:
                active_transfer_->state = abort_requested_ ? TransferState::TimedOut : TransferState::Failed;
                active_transfer_->error = abort_requested_ ? TransferError::Timeout : TransferError::HalError;
                abort_requested_ ? ++stats_.timed_out : ++stats_.failed;
            break;
            case SpiEvent::AbortComplete:
                active_transfer_->state = abort_requested_ ? TransferState::TimedOut : TransferState::Aborted;
                active_transfer_->error = abort_requested_ ? TransferError::Timeout : TransferError::HalError;
                abort_requested_ ? ++stats_.timed_out : ++stats_.failed;
            break;
            case SpiEvent::None:
            return;
        }

        active_transfer_ = nullptr;
        abort_requested_ = false;
    }

    void SpiBus::onTxRxCompleteInterrupt() noexcept{
        if(active_transfer_ != nullptr){
            active_transfer_->chip_select.setActive(false);
        }
        event_.store(static_cast<uint8_t>(SpiEvent::Complete),std::memory_order_release);
    }
    void SpiBus::onErrorInterrupt() noexcept{
        if(active_transfer_ != nullptr){
            active_transfer_->chip_select.setActive(false);
        }
        event_.store(static_cast<uint8_t>(SpiEvent::Error),std::memory_order_release);
    }
    void SpiBus::onAbortCompleteInterrupt() noexcept{
        if(active_transfer_ != nullptr){
            active_transfer_->chip_select.setActive(false);
        }
        event_.store(static_cast<uint8_t>(SpiEvent::AbortComplete),std::memory_order_release);
    }
}