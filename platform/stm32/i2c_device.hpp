#pragma once

#include "platform/stm32/i2c_bus.hpp"

#include <cstdint>

namespace platform{

class I2cDevice final{
public:
    I2cDevice(I2cBus& bus, uint8_t device_address) noexcept
    : bus_(bus)
    , device_address(device_address) {}

    uint8_t deviceAddress() const noexcept { return device_address; }

    bool masterTransmit(I2cTransfer& transfer, uint8_t* data, uint16_t size, uint32_t timeout_ms) noexcept{
        return submit(transfer, I2cOperation::MasterTransmit, 0U, I2cMemoryAddressSize::Bits8, data, nullptr, size, timeout_ms);
    }

    bool masterReceive(I2cTransfer& transfer, uint8_t* data, uint16_t size, uint32_t timeout_ms) noexcept{
        return submit(transfer, I2cOperation::MasterReceive, 0U, I2cMemoryAddressSize::Bits8, nullptr, data, size, timeout_ms);
    }

    bool memoryWrite(I2cTransfer& transfer, uint16_t memory_address, I2cMemoryAddressSize memory_address_size,
                     uint8_t* data, uint16_t size, uint32_t timeout_ms) noexcept{
        return submit(transfer, I2cOperation::MemoryWrite, memory_address, memory_address_size, data, nullptr, size, timeout_ms);
    }

    bool memoryRead(I2cTransfer& transfer, uint16_t memory_address, I2cMemoryAddressSize memory_address_size,
                    uint8_t* data, uint16_t size, uint32_t timeout_ms) noexcept{
        return submit(transfer, I2cOperation::MemoryRead, memory_address, memory_address_size, nullptr, data, size, timeout_ms);
    }

private:
    bool submit(I2cTransfer& transfer, I2cOperation operation, uint16_t memory_address, I2cMemoryAddressSize memory_address_size,
                uint8_t* tx, uint8_t* rx, uint16_t size, uint32_t timeout_ms) noexcept{
        if(transfer.state != TransferState::Idle){
            return false;
        }

        transfer.configure(operation, device_address, memory_address, memory_address_size,
                           tx, rx, size, timeout_ms);
        return bus_.submit(transfer);
    }

    I2cBus& bus_;
    uint8_t device_address{0U};
};

}