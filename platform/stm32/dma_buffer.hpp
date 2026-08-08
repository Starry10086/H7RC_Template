#pragma once

#include <cstddef>

#if defined(__GNUC__)
#define DMA_BUFFER \
    __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define DMA_BUFFER
#endif

namespace platform{
    bool isDmaBuffer(const void* data, std::size_t size) noexcept;
}