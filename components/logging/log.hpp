#pragma once

#include <cstdint>

namespace librmcs::logging {

enum class Level : uint8_t {
    Info,
    Warn,
    Error,
};

struct ThrottleState {
    uint32_t last_log_ms{0U};
    bool initialized{false};
};

bool shouldLog(
    ThrottleState& state,
    uint32_t period_ms) noexcept;

#if defined(__GNUC__)
__attribute__((format(printf, 2, 3)))
#endif
void log(Level level, const char* format, ...) noexcept;

} // namespace librmcs::logging

#define LOG_INFO(...)                                                     \
    ::librmcs::logging::log(::librmcs::logging::Level::Info, __VA_ARGS__)

#define LOG_WARN(...)                                                     \
    ::librmcs::logging::log(::librmcs::logging::Level::Warn, __VA_ARGS__)

#define LOG_ERROR(...)                                                    \
    ::librmcs::logging::log(::librmcs::logging::Level::Error, __VA_ARGS__)

#define LOG_INFO_THROTTLE(PERIOD_MS, ...)                              \
    do {                                                               \
        static ::librmcs::logging::ThrottleState throttle_state{};     \
        if (::librmcs::logging::shouldLog(                             \
                throttle_state,                                       \
                static_cast<uint32_t>(PERIOD_MS))) {                   \
            LOG_INFO(__VA_ARGS__);                                     \
        }                                                              \
    } while (false)

#define LOG_WARN_THROTTLE(PERIOD_MS, ...)                              \
    do {                                                               \
        static ::librmcs::logging::ThrottleState throttle_state{};     \
        if (::librmcs::logging::shouldLog(                             \
                throttle_state,                                       \
                static_cast<uint32_t>(PERIOD_MS))) {                   \
            LOG_WARN(__VA_ARGS__);                                     \
        }                                                              \
    } while (false)
