#pragma once

#include <cstddef>
#include <string_view>

namespace librmcs::platform::rtt {

void init() noexcept;

std::size_t write(std::string_view text) noexcept;

} // namespace librmcs::platform::rtt