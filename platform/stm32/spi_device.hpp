#pragma once

#include "platform/stm32/spi_bus.hpp"

#include <cstdint>

namespace platform{

class SpiDevice final{
public:
    SpiDevice(SpiBus& bus, SpiChipSelect chip_select) noexcept
    : bus_(bus)
    , chip_select_(chip_select) {}

    bool submit(SpiTransfer& transfer, const uint8_t* tx, uint8_t* rx, uint16_t size, uint32_t timeout_ms) noexcept{
        if(transfer.state != TransferState::Idle){
            return false;
        }

        transfer.configure(tx, rx, size, timeout_ms, chip_select_);
        return bus_.submit(transfer);
    }

    SpiChipSelect chipSelect() const noexcept { return chip_select_; }
private:
    SpiBus& bus_;
    SpiChipSelect chip_select_;
};

}