#pragma once

#include <cstdint>

namespace robot{

enum class ChassisType : uint8_t {
    Mecanum,
    Omni4
};

struct ChassisConfig{
    ChassisType type{ChassisType::Mecanum};

    float wheel_radius_m{0.0f}; // 轮子半径，单位：米
    float half_wheel_base_m{0.0f};   // 轴距，前轮中心线到后轮中心线的距离的一半，单位：米
    float half_wheel_track_m{0.0f};  // 轮距，左轮中心线到右轮中心线的距离的一半，单位：米

    float max_wheel_vel_rad_s{0.0f}; // 最大轮子角速度，单位：弧度/秒
};

struct ChassisVelCmd{
    float vx_m_s{0.0f}; // 前进为正，单位：米/秒
    float vy_m_s{0.0f}; // 向左为正，单位：米/秒
    float omega_rad_s{0.0f}; // 逆时针为正，单位：弧度/秒
};

struct WheelVelTarget{
    float vel_rad_s{0.0f}; // 轮子角速度，单位：弧度/秒
};
}