#pragma once

#include <cstddef>
#include <string_view>

namespace platform::rtt {

void init() noexcept;

std::size_t write(std::string_view text) noexcept;

} // namespace platform::rtt