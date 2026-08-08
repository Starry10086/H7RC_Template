#pragma once

#include <array>
#include <cstdint>

namespace device{
    struct ImuState{
        std::array<float, 3> accel_m_s2{};  // 单位 m/s^2
        std::array<float, 3> gyro_rad_s{};  // 单位 rad/s
        uint32_t fault_code{0U};
    };
}