#include "platform/segger/rtt.hpp"

#include "SEGGER_RTT.h"

namespace librmcs::platform::rtt {

namespace {

constexpr unsigned kTerminalUpBuffer{0U};

} // namespace

void init() noexcept {
    SEGGER_RTT_Init();

    (void)SEGGER_RTT_SetFlagsUpBuffer(
        kTerminalUpBuffer,
        SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

std::size_t write(std::string_view text) noexcept {
    if (text.empty()) {
        return 0U;
    }

    const auto bytes_written = SEGGER_RTT_Write(
        kTerminalUpBuffer,
        text.data(),
        static_cast<unsigned>(text.size()));

    return static_cast<std::size_t>(bytes_written);
}

} // namespace librmcs::platform::rtt