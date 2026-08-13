#pragma once

#include "devices/imu/bmi088.hpp"
#include "platform/stm32/uart_port.hpp"

namespace app::dma_storage{
    extern device::Bmi088DmaStorage bmi088;
    extern platform::UartDmaStorage uart10;
}