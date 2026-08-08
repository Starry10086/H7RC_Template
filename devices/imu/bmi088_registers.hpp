#pragma once

#include <cstdint>

namespace device::bmi088_register{

    inline constexpr uint8_t read_bit = 0x80U;
    inline constexpr uint8_t write_mask = 0x7FU;
    inline constexpr uint8_t dummy_byte = 0x00U;

    // Chip ID
    inline constexpr uint8_t accel_chip_id_reg = 0x00U;
    inline constexpr uint8_t gyro_chip_id_reg = 0x00U;

    inline constexpr uint8_t expected_accel_chip_id = 0x1EU;
    inline constexpr uint8_t expected_gyro_chip_id = 0x0FU;

    // 数据寄存器起始地址
    inline constexpr uint8_t accel_xyz_reg = 0x12U;
    inline constexpr uint8_t gyro_xyz_reg = 0x02U;

    // 加速度计配置
    inline constexpr uint8_t accel_conf_reg = 0x40U;
    inline constexpr uint8_t accel_range_reg = 0x41U;
    inline constexpr uint8_t accel_power_conf_reg = 0x7CU;
    inline constexpr uint8_t accel_power_ctrl_reg = 0x7DU;

    // 陀螺仪配置
    inline constexpr uint8_t gyro_range_reg = 0x0FU;
    inline constexpr uint8_t gyro_bandwidth_reg = 0x10U;

    // 200HZ
    inline constexpr uint8_t accel_normal_200_hz = 0xA9U;
    inline constexpr uint8_t accel_range_6g = 0x01U;
    inline constexpr uint8_t accel_power_active = 0x00U;
    inline constexpr uint8_t accel_measurement_enable = 0x04U;

    inline constexpr uint8_t gyro_range_2000_dps = 0x00U;
    inline constexpr uint8_t gyro_200_hz_23_hz = 0x04U;

    constexpr uint8_t makeReadAddress(uint8_t address) noexcept{
        return static_cast<uint8_t>(address | read_bit);
    }

    constexpr uint8_t makeWriteAddress(uint8_t address) noexcept{
        return static_cast<uint8_t>(address & write_mask);
    }
}