#pragma once

#include <cstdint>

namespace librmcs::device{

struct MotorState{
    float pos_rad{0.0f};        // 电机位置，单位：弧度
    float vel_rad_s{0.0f};      // 电机速度，单位：弧度/秒
    float torque_nm{0.0f};           // 电机扭矩，单位：牛·米
    float temperature_c{0.0f};       // 电机温度，单位：摄氏度
    uint32_t fault_code{0};          // 电机故障码，0表示无故障
};

}