#include "components/logging/log.hpp"

#include "platform/segger/rtt.hpp"
#include "platform/stm32/timebase.hpp"

#include <cstdarg>
#include <cstdio>
#include <string_view>

namespace librmcs::logging {

namespace {

constexpr std::size_t kLineCapacity{256U};

const char* levelName(Level level) noexcept {
    switch (level) {
    case Level::Info:
        return "INFO";
    case Level::Warn:
        return "WARN";
    case Level::Error:
        return "ERROR";
    }

    return "UNKNOWN";
}

} // namespace

void log(Level level, const char* format, ...) noexcept {
    if (format == nullptr) {
        return;
    }

    char line[kLineCapacity]{};

    const int prefix_length = std::snprintf(
        line,
        kLineCapacity,
        "[%10lu] [%s] ",
        static_cast<unsigned long>(platform::nowMs()),
        levelName(level));

    if (prefix_length < 0) {
        return;
    }

    std::size_t used = static_cast<std::size_t>(prefix_length);
    if (used >= kLineCapacity) {
        used = kLineCapacity - 1U;
    }

    std::va_list arguments;
    va_start(arguments, format);

    const std::size_t remaining = kLineCapacity - used;
    const int message_length = std::vsnprintf(
        line + used,
        remaining,
        format,
        arguments);

    va_end(arguments);

    if (message_length < 0) {
        return;
    }

    const auto message_size =
        static_cast<std::size_t>(message_length);

    if (message_size >= remaining) {
        used = kLineCapacity - 1U;
    } else {
        used += message_size;
    }

    if (used == 0U || line[used - 1U] != '\n') {
        if (used < kLineCapacity - 1U) {
            line[used] = '\n';
            ++used;
        } else {
            line[kLineCapacity - 2U] = '\n';
        }
    }

    (void)platform::rtt::write(
        std::string_view{line, used});
}

bool shouldLog(
    ThrottleState& state,
    uint32_t period_ms) noexcept {

    const uint32_t now_ms = platform::nowMs();

    if (!state.initialized ||
        static_cast<uint32_t>(
            now_ms - state.last_log_ms) >= period_ms) {

        state.last_log_ms = now_ms;
        state.initialized = true;
        return true;
    }

    return false;
}

} // namespace librmcs::logging