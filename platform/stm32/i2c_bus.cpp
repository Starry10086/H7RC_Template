#include "platform/stm32/i2c_bus.hpp"

#include "platform/stm32/async_transfer.hpp"
#include "stm32h7xx_hal_def.h"
#include <cstdint>

namespace platform{

namespace{
    constexpr uint16_t toHalDeviceAddress(uint8_t device_address_7bit) noexcept{
        return static_cast<uint16_t>(device_address_7bit << 1U);
    }
}

    I2cBus::I2cBus(I2C_HandleTypeDef& handle) noexcept
    : handle_(handle){}

    bool I2cBus::validMemoryAddressSize(I2cMemoryAddressSize size) noexcept{
        return size == I2cMemoryAddressSize::Bits8 || size == I2cMemoryAddressSize::Bits16;
    }

    uint16_t I2cBus::toHalMemoryAddressSize(I2cMemoryAddressSize size) noexcept{
        if(size == I2cMemoryAddressSize::Bits8){
            return I2C_MEMADD_SIZE_8BIT;
        }

        return I2C_MEMADD_SIZE_16BIT;
    }

    bool I2cBus::submit(I2cTransfer& transfer) noexcept{
        if(transfer.state != TransferState::Idle){
            return false;
        }

        if (transfer.device_address_7bit > 0x7FU ||
            transfer.size == 0U ||
            transfer.timeout_ms == 0U ||
            !validMemoryAddressSize(transfer.memory_address_size)) {
            transfer.error = TransferError::InvalidArgument;
            transfer.state = TransferState::Failed;
            return false;
        }

        const bool needs_tx = transfer.operation == I2cOperation::MasterTransmit || transfer.operation == I2cOperation::MemoryWrite;
        const bool needs_rx = transfer.operation == I2cOperation::MasterReceive || transfer.operation == I2cOperation::MemoryRead;

        if(needs_tx && (transfer.tx_buffer == nullptr || !isDmaBuffer(transfer.tx_buffer, transfer.size))){
            transfer.error = TransferError::InvalidDmaBuffer;
            transfer.state = TransferState::Failed;
            return false;
        }
        if(needs_rx && (transfer.rx_buffer == nullptr || !isDmaBuffer(transfer.rx_buffer, transfer.size))){
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

    void I2cBus::process(uint32_t now_ms) noexcept{
        const I2cEvent event = static_cast<I2cEvent>(event_.exchange(static_cast<uint8_t>(I2cEvent::None),std::memory_order_acq_rel));

        if(active_transfer_ != nullptr && event != I2cEvent::None){
            finishActive(event);
        }
        if(active_transfer_ != nullptr){
            processActive(now_ms);
        }
        if(active_transfer_ == nullptr){
            startNext(now_ms);
        }
    }

    void I2cBus::startNext(uint32_t now_ms) noexcept{
        I2cTransfer* next = transfer_queue_.pop();

        if(next == nullptr){
            return;
        }

        active_transfer_ = next;
        active_transfer_->state = TransferState::Active;
        active_transfer_->start_time_ms = now_ms;
        active_transfer_->error = TransferError::None;
        abort_requested_ = false;

        const uint16_t hal_address = toHalDeviceAddress(active_transfer_->device_address_7bit);

        HAL_StatusTypeDef result = HAL_ERROR;

        switch(active_transfer_->operation){
            case I2cOperation::MasterTransmit:
                result = HAL_I2C_Master_Transmit_DMA(
                    &handle_,
                    hal_address,
                    active_transfer_->tx_buffer,
                    active_transfer_->size);
            break;
            case I2cOperation::MasterReceive:
                result = HAL_I2C_Master_Receive_DMA(
                    &handle_,
                    hal_address,
                    active_transfer_->rx_buffer,
                    active_transfer_->size);
            break;
            case I2cOperation::MemoryWrite:
                result = HAL_I2C_Mem_Write_DMA(
                    &handle_,
                    hal_address,
                    active_transfer_->memory_address,
                    toHalMemoryAddressSize(active_transfer_->memory_address_size),
                    active_transfer_->tx_buffer,
                    active_transfer_->size);
            break;
            case I2cOperation::MemoryRead:
                result = HAL_I2C_Mem_Read_DMA(
                    &handle_,
                    hal_address,
                    active_transfer_->memory_address,
                    toHalMemoryAddressSize(active_transfer_->memory_address_size),
                    active_transfer_->rx_buffer,
                    active_transfer_->size);
            break;
        }

        if(result == HAL_OK){
            return;
        }

        active_transfer_->error = result == HAL_BUSY ? TransferError::HalBusy : TransferError::HalError;
        active_transfer_->state = TransferState::Failed;
        active_transfer_ = nullptr;
        ++stats_.failed;
    }

    void I2cBus::processActive(uint32_t now_ms) noexcept{
        if(active_transfer_ == nullptr || abort_requested_){
            return;
        }

        const uint32_t elapsed_ms = static_cast<uint32_t>(now_ms - active_transfer_->start_time_ms);
        if(elapsed_ms <= active_transfer_->timeout_ms){
            return;
        }

        timeoutActive();
    }

    void I2cBus::timeoutActive() noexcept{
        if(active_transfer_ == nullptr){
            return;
        }

        abort_requested_ = true;

        const HAL_StatusTypeDef result = HAL_I2C_Master_Abort_IT(&handle_, toHalDeviceAddress(active_transfer_->device_address_7bit));
        if(result == HAL_OK){
            return;
        }

        active_transfer_->error = TransferError::Timeout;
        active_transfer_->state = TransferState::TimedOut;
        active_transfer_ = nullptr;
        abort_requested_ = false;
        ++stats_.timed_out;
    }

    void I2cBus::finishActive(I2cEvent event) noexcept{
        if(active_transfer_ == nullptr){
            return;
        }

        switch(event){
            case I2cEvent::Complete:
                active_transfer_->state = TransferState::Completed;
                active_transfer_->error = TransferError::None;
                ++stats_.completed;
            break;
            case I2cEvent::Error:
                active_transfer_->state = abort_requested_ ? TransferState::TimedOut : TransferState::Failed;
                active_transfer_->error = abort_requested_ ? TransferError::Timeout : TransferError::HalError;
                if (abort_requested_) {
                    ++stats_.timed_out;
                } else {
                    ++stats_.failed;
                }
            break;
            case I2cEvent::AbortComplete:
                active_transfer_->state = TransferState::TimedOut;
                active_transfer_->error = TransferError::Timeout;
                ++stats_.timed_out;
            break;
            case I2cEvent::None:
                return;
        }
        active_transfer_ = nullptr;
        abort_requested_ = false;
    }

    void I2cBus::onTransferCompleteInterrupt() noexcept {
        event_.store(static_cast<uint8_t>(I2cEvent::Complete),std::memory_order_release);
    }

    void I2cBus::onErrorInterrupt() noexcept {
        event_.store(static_cast<uint8_t>(I2cEvent::Error),std::memory_order_release);
    }

    void I2cBus::onAbortCompleteInterrupt() noexcept {
        event_.store(static_cast<uint8_t>(I2cEvent::AbortComplete),std::memory_order_release);
    }


}