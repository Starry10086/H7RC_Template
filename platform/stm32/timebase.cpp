#include "platform/stm32/timebase.hpp"

#include "stm32h7xx_hal.h"

namespace platform {

uint32_t nowMs() noexcept {
    return HAL_GetTick();
}

} // namespace platform