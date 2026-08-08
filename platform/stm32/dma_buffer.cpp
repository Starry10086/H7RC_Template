#include "platform/stm32/dma_buffer.hpp"
#include <cstdint>

extern "C"{
extern uint8_t __dma_buffer_start__;
extern uint8_t __dma_buffer_end__;
}

namespace platform{
    bool isDmaBuffer(const void* data, std::size_t size) noexcept{
        if(data == nullptr || size == 0U){
            return false;
        }

        const auto begin = reinterpret_cast<uintptr_t>(data);
        const auto section_begin = reinterpret_cast<uintptr_t>(&__dma_buffer_start__);
        const auto section_end = reinterpret_cast<uintptr_t>(&__dma_buffer_end__);

        if(begin < section_begin || begin >= section_end){
            return false;
        }
        return size <= section_end - begin;
    }

}