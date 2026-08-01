#pragma once

#include "platform/can/can_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace device {

enum class DjiCommandGroup : uint16_t {
    m3508_m2006_201_to_204 = 0x200U,
    m3508_m2006_205_to_208 = 0x1FFU,

    gm6020_current_205_to_208 = 0x1FEU,
    gm6020_current_209_to_20b = 0x2FEU,
    gm6020_voltage_205_to_208 = 0x1FFU,
    gm6020_voltage_209_to_20b = 0x2FFU
};

constexpr can::Frame makeDjiCommandFrame(DjiCommandGroup group, const std::array<int16_t, 4>& commands) noexcept {
    can::Frame frame{
        .id = static_cast<uint16_t>(group),
        .id_format = can::IdFormat::Standard,
        .kind = can::FrameKind::Data,
        .length = 8U,
        .data = {}
    };

    for (std::size_t index = 0U; index < commands.size(); ++index) {
        const uint16_t raw =
            static_cast<uint16_t>(commands[index]);
        const std::size_t offset = index * 2U;

        frame.data[offset] =
            static_cast<uint8_t>((raw >> 8U) & 0xFFU);
        frame.data[offset + 1U] =
            static_cast<uint8_t>(raw & 0xFFU);
    }

    return frame;
}

} // namespace device
