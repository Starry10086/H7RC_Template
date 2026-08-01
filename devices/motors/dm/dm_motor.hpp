#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>

#include "components/messaging/state_topic.hpp"
#include "devices/motors/motor_state.hpp"
#include "platform/can/can_types.hpp"
#include "platform/stm32/timebase.hpp"

/*
@breif DM电机返回的和设置的参数均以经过减速比转换后的输出端参数
*/

namespace device {
class DmMotor {
public:
    enum class Type : uint8_t {DM_J4310_2EC,DM_J4340_2EC};
    enum class ControlMode : uint8_t {MIT,Pos_Vel,Vel};

    struct Config{
        explicit Config(Type motor_type,
                        uint16_t command_id_value = 0x01U)
        : control_mode(ControlMode::MIT)
        , command_id(command_id_value)
        , reduction_ratio(reduction_ratio_init(motor_type))
        , KP_Max(500.0)
        , KD_Max(5.0)
        , Pos_Max(12.5)
        , Vel_Max(45.0)
        , Tor_Max(max_torque_init(motor_type))
        ,reversed(false)
        ,multi_turn_angle_enabled(false) {}

        Config& set_reversed() { return reversed = true, *this; }
        Config& set_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
        Config& set_reduction_ratio(float value) { return reduction_ratio = value, *this; }
        Config& set_control_mode(ControlMode value) { return control_mode = value, *this; }
        Config& set_command_id(uint16_t value) { return command_id = value, *this; }
    
        ControlMode control_mode;
        uint16_t command_id;
        float reduction_ratio;
        float KP_Max;
        float KD_Max;
        float Pos_Max;
        float Vel_Max;
        float Tor_Max;
        bool reversed;
        bool multi_turn_angle_enabled;
    };

    DmMotor() = delete;
    DmMotor(const Config& config, messaging::StateTopic<MotorState>& state_topic) noexcept
    : state_topic_(state_topic){
        configure(config);
    }

    //禁用拷贝
    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;

    void configure(const Config& config) noexcept{
        control_mode_ = config.control_mode;
        command_id_ = config.command_id;
        reduction_ratio = config.reduction_ratio;
        KP_Max = config.KP_Max;
        KD_Max = config.KD_Max;
        Pos_Max = config.Pos_Max;
        Vel_Max = config.Vel_Max;
        Tor_Max = config.Tor_Max;
        reversed = config.reversed;
        multi_turn_angle_enabled = config.multi_turn_angle_enabled;
        reversed_sign = config.reversed ? -1.0 : 1.0;
    }

    void handleCanFrame(const can::Frame& frame) noexcept {
        if (frame.id_format != can::IdFormat::Standard ||
            frame.kind != can::FrameKind::Data ||
            frame.length != 8U) {
            return;
        }

        const auto& data = frame.data;

        /*
        * data[0]:
        * 高4位：电机状态/故障码
        * 低4位：反馈中的电机ID
        */
        ERR = static_cast<uint8_t>((data[0] >> 4U) & 0x0FU);

        const uint16_t raw_position =
            static_cast<uint16_t>(
                (static_cast<uint16_t>(data[1]) << 8U) |
                static_cast<uint16_t>(data[2]));

        const uint16_t raw_velocity =
            static_cast<uint16_t>(
                (static_cast<uint16_t>(data[3]) << 4U) |
                (static_cast<uint16_t>(data[4]) >> 4U));

        const uint16_t raw_torque =
            static_cast<uint16_t>(
                (static_cast<uint16_t>(data[4] & 0x0FU) << 8U) |
                static_cast<uint16_t>(data[5]));

        Angle = uint_to_float(
                    raw_position,
                    -Pos_Max,
                    Pos_Max,
                    16) *
                reversed_sign;

        VEL = uint_to_float(
                raw_velocity,
                -Vel_Max,
                Vel_Max,
                12) *
            reversed_sign;

        TOR = uint_to_float(
                raw_torque,
                -Tor_Max,
                Tor_Max,
                12) *
            reversed_sign;

        T_MOS = static_cast<float>(data[6]);
        T_Rotor = static_cast<float>(data[7]);

        state_topic_.publish(
            MotorState{
                .pos_rad = Angle,
                .vel_rad_s = VEL,
                .torque_nm = TOR,
                .temperature_c = T_Rotor,
                .fault_code = static_cast<uint32_t>(ERR),
            },
            platform::nowMs());
    }

    can::Frame makeControlFrame(float target_position,
                                float target_velocity,
                                float target_kp,
                                float target_kd,
                                float feedforward_torque) const noexcept {
        switch (control_mode_) {
        case ControlMode::MIT:
            return makeMitControlFrame(
                target_position,
                target_velocity,
                target_kp,
                target_kd,
                feedforward_torque);
        case ControlMode::Pos_Vel:
            return makePositionVelocityFrame(
                target_position,
                target_velocity);
        case ControlMode::Vel:
            return makeVelocityFrame(target_velocity);
        }

        return makeDataFrame();
    }

    can::Frame makeMitControlFrame(float target_position,
                                   float target_velocity,
                                   float target_kp,
                                   float target_kd,
                                   float feedforward_torque) const noexcept {
        target_position = clampFinite(
            target_position * reversed_sign,
            -Pos_Max,
            Pos_Max);
        target_velocity = clampFinite(
            target_velocity * reversed_sign,
            -Vel_Max,
            Vel_Max);
        target_kp = clampFinite(target_kp, 0.0F, KP_Max);
        target_kd = clampFinite(target_kd, 0.0F, KD_Max);
        feedforward_torque = clampFinite(
            feedforward_torque * reversed_sign,
            -Tor_Max,
            Tor_Max);

        const uint16_t raw_position = float_to_uint(
            target_position, -Pos_Max, Pos_Max, 16U);
        const uint16_t raw_velocity = float_to_uint(
            target_velocity, -Vel_Max, Vel_Max, 12U);
        const uint16_t raw_kp = float_to_uint(
            target_kp, 0.0F, KP_Max, 12U);
        const uint16_t raw_kd = float_to_uint(
            target_kd, 0.0F, KD_Max, 12U);
        const uint16_t raw_torque = float_to_uint(
            feedforward_torque, -Tor_Max, Tor_Max, 12U);

        can::Frame frame = makeDataFrame();
        frame.data[0] = static_cast<uint8_t>(raw_position >> 8U);
        frame.data[1] = static_cast<uint8_t>(raw_position & 0xFFU);
        frame.data[2] = static_cast<uint8_t>(raw_velocity >> 4U);
        frame.data[3] = static_cast<uint8_t>(
            ((raw_velocity & 0x0FU) << 4U) |
            ((raw_kp >> 8U) & 0x0FU));
        frame.data[4] = static_cast<uint8_t>(raw_kp & 0xFFU);
        frame.data[5] = static_cast<uint8_t>(raw_kd >> 4U);
        frame.data[6] = static_cast<uint8_t>(
            ((raw_kd & 0x0FU) << 4U) |
            ((raw_torque >> 8U) & 0x0FU));
        frame.data[7] = static_cast<uint8_t>(raw_torque & 0xFFU);
        return frame;
    }

    can::Frame makePositionVelocityFrame(float target_position, float target_velocity) const noexcept {
        target_position = clampFinite(
            target_position * reversed_sign,
            -Pos_Max,
            Pos_Max);
        target_velocity = clampFinite(
            target_velocity * reversed_sign,
            -Vel_Max,
            Vel_Max);

        can::Frame frame = makeDataFrame();
        writeLittleEndianFloat(frame.data, 0U, target_position);
        writeLittleEndianFloat(frame.data, 4U, target_velocity);
        return frame;
    }

    can::Frame makeVelocityFrame(float target_velocity) const noexcept {
        target_velocity = clampFinite(
            target_velocity * reversed_sign,
            -Vel_Max,
            Vel_Max);

        can::Frame frame = makeDataFrame();
        writeLittleEndianFloat(frame.data, 0U, target_velocity);
        return frame;
    }

    can::Frame makeEnableFrame() const noexcept {
        return makeSpecialCommandFrame(0xFCU);
    }

    can::Frame makeDisableFrame() const noexcept {
        return makeSpecialCommandFrame(0xFDU);
    }

    can::Frame makeSaveZeroPositionFrame() const noexcept {
        return makeSpecialCommandFrame(0xFEU);
    }

    can::Frame makeClearErrorFrame() const noexcept {
        return makeSpecialCommandFrame(0xFBU);
    }
    float angle() const { return Angle; }
    float velocity() const { return VEL; }
    float torque() const { return TOR; }
    float temperature_mos() const { return T_MOS; }
    float temperature_rotor() const { return T_Rotor; }
    float max_torque() const { return Tor_Max; }
    uint8_t get_err() const { return ERR; }


private:
    can::Frame makeDataFrame() const noexcept {
        can::Frame frame{};
        frame.id = command_id_;
        frame.id_format = can::IdFormat::Standard;
        frame.kind = can::FrameKind::Data;
        frame.length = 8U;
        return frame;
    }

    can::Frame makeSpecialCommandFrame(uint8_t command) const noexcept {
        can::Frame frame = makeDataFrame();
        frame.data.fill(uint8_t{0xFFU});
        frame.data[7] = command;
        return frame;
    }

    static void writeLittleEndianFloat(std::array<uint8_t, 8>& data, uint8_t offset, float value) noexcept {
        static_assert(sizeof(float) == sizeof(uint32_t));

        const uint32_t raw = std::bit_cast<uint32_t>(value);
        data[offset] = static_cast<uint8_t>(raw & 0xFFU);
        data[offset + 1U] =
            static_cast<uint8_t>((raw >> 8U) & 0xFFU);
        data[offset + 2U] =
            static_cast<uint8_t>((raw >> 16U) & 0xFFU);
        data[offset + 3U] =
            static_cast<uint8_t>((raw >> 24U) & 0xFFU);
    }

    static float clampFinite(float value, float minimum, float maximum) noexcept {
        if (!std::isfinite(value)) {
            return 0.0F;
        }

        return std::clamp(value, minimum, maximum);
    }

    static float reduction_ratio_init(Type motor_type) {
        switch (motor_type) {
            case Type::DM_J4310_2EC: return 10.0;
            case Type::DM_J4340_2EC: return 40.0;
            default: return 0.0;
        }
    }
    static float max_torque_init(Type motor_type) {
        switch (motor_type) {
            case Type::DM_J4310_2EC: return 12.5;
            case Type::DM_J4340_2EC: return 27.0;
            default: return 0.0;
        }
    }
    static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
        const float span = x_max - x_min;
        const float offset = x_min;
        return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + offset;
    }

    static uint16_t float_to_uint(float value, float minimum, float maximum, uint8_t bits) noexcept {
        value = clampFinite(value, minimum, maximum);

        const float span = maximum - minimum;
        const uint32_t raw_max =
            (uint32_t{1U} << bits) - uint32_t{1U};

        return static_cast<uint16_t>(
            (value - minimum) *
            static_cast<float>(raw_max) / span);
    }

    messaging::StateTopic<MotorState>& state_topic_;

    uint8_t ERR{0U};
    float Angle{0.0f};
    float VEL{0.0f};
    float TOR{0.0f};
    float T_MOS{0.0f};
    float T_Rotor{0.0f};
    float reversed_sign{1.0f};
    
    ControlMode control_mode_{ControlMode::MIT};
    uint16_t command_id_{0x01U};
    float reduction_ratio{1.0f};
    float KP_Max{500.0f};
    float KD_Max{5.0f};
    float Pos_Max{12.5f};
    float Vel_Max{45.0f};
    float Tor_Max{0.0f};
    bool reversed{false};
    bool multi_turn_angle_enabled{false};
};
}
