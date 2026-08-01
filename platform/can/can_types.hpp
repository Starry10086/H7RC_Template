#pragma once 

#include <array>
#include <cstdint>

namespace can{

enum class IdFormat : uint8_t {
    Standard,
    Extended
};

enum class FrameKind : uint8_t{
    Data,
    Remote
};

struct Frame{
    uint32_t id{0};
    IdFormat id_format{IdFormat::Standard};
    FrameKind kind{FrameKind::Data};
    uint8_t length{0};
    std::array<uint8_t, 8> data{};
};

}