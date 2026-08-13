#include "app/dma_storage.hpp"
#include "platform/stm32/dma_buffer.hpp"

namespace app::dma_storage{
    DMA_BUFFER device::Bmi088DmaStorage bmi088{};
    DMA_BUFFER platform::UartDmaStorage uart10{};
}