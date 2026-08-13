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

    void I2cBus::startNext(uint32_t now_ms) noexcept{
        I2cTransfer* next = transfer_queue_.pop();

        if(next == nullptr){
            return;
        }

        active_transfer_ = next;
        active_transfer_->state = TransferState::Active;
        active_transfer_->start_time_ms = now_ms;
        active_transfer_->error = TransferError::None;
        abort_reason_ = AbortReason::None;

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

        endActive(TransferState::Failed,result == HAL_BUSY ? TransferError::HalBusy : TransferError::HalError);
    }

    void I2cBus::processActive(uint32_t now_ms) noexcept{
        if(active_transfer_ == nullptr || abort_reason_ != AbortReason::None){
            return;
        }

        const uint32_t elapsed_ms = static_cast<uint32_t>(now_ms - active_transfer_->start_time_ms);
        if(elapsed_ms < active_transfer_->timeout_ms){
            return;
        }

        timeoutActive();
    }

    void I2cBus::timeoutActive() noexcept{
        if(active_transfer_ == nullptr || abort_reason_ != AbortReason::None){
            return;
        }

        abort_reason_ = AbortReason::Timeout;

        const HAL_StatusTypeDef result = HAL_I2C_Master_Abort_IT(&handle_, toHalDeviceAddress(active_transfer_->device_address_7bit));
        if(result == HAL_OK){
            return;
        }

        endActive(TransferState::TimedOut, TransferError::Timeout);
    }

    void I2cBus::handleEvents(uint32_t events) noexcept{
        if(active_transfer_ == nullptr){
            return;
        }

        const bool complete = (events & event_complete) != 0U;
        const bool error = (events & event_error) != 0U;
        const bool abort_complete = (events & event_abort_complete) != 0U;

        if(abort_reason_ == AbortReason::Timeout){
            if(abort_complete){
                endActive(TransferState::TimedOut, TransferError::Timeout);
            }
            // 超时中止期间必须等待 AbortComplete，不能因中间 Error 提前启动下一笔事务。
            return;
        }

        if(error){
            endActive(TransferState::Failed, TransferError::HalError);
            return;
        }

        if(abort_complete){
            // 没有发起中止却收到 AbortComplete，说明内部状态异常。
            endActive(TransferState::Failed, TransferError::HalError);
            return;
        }

        if(complete){
            endActive(TransferState::Completed, TransferError::None);
        }
    }

    void I2cBus::endActive(TransferState state, TransferError error) noexcept{
        if(active_transfer_ == nullptr){
            return;
        }

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
                // 其他状态不应该在 endActive() 中出现。
            break;
        }

        active_transfer_ = nullptr;
        abort_reason_ = AbortReason::None;
    }

    void I2cBus::onTransferCompleteInterrupt() noexcept {
        pending_events_.fetch_or(event_complete, std::memory_order_release);
    }

    void I2cBus::onErrorInterrupt() noexcept {
        pending_events_.fetch_or(event_error, std::memory_order_release);
    }

    void I2cBus::onAbortCompleteInterrupt() noexcept {
        pending_events_.fetch_or(event_abort_complete, std::memory_order_release);
    }


}
