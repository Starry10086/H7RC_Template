#pragma once

#include "components/messaging/state_topic.hpp"
#include "devices/motors/motor_state.hpp"
#include "platform/can/can_types.hpp"
#include "platform/stm32/timebase.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace librmcs::device {

class RsMotor final {
public:
    enum class Type : uint8_t {
        RS01,
        RS05
    };

    enum class CommunicationType : uint8_t {
        get_device_id = 0,
        mit_control = 1,
        feedback = 2,
        motor_enable = 3,
        motor_stop = 4,
        set_mechanical_zero = 6,
        active_report = 24
    };

    enum class OperatingState : uint8_t {
        reset = 0,
        calibration = 1,
        running = 2,
        unknown = 3
    };

    enum class FaultFlag : uint8_t {
        under_voltage = 1U << 0U,
        phase_current = 1U << 1U,
        over_temperature = 1U << 2U,
        magnetic_encoder = 1U << 3U,
        overload = 1U << 4U,
        uncalibrated = 1U << 5U
    };

    struct Limits {
        float position_min;
        float position_max;
        float velocity_min;
        float velocity_max;
        float torque_min;
        float torque_max;
        float kp_min;
        float kp_max;
        float kd_min;
        float kd_max;
    };

    struct Config {
        explicit constexpr Config(Type type_value,
                                  uint8_t motor_id_value) noexcept
            : type(type_value)
            , motor_id(motor_id_value) {
        }

        constexpr Config& set_host_id(uint8_t value) noexcept {
            host_id = value;
            return *this;
        }

        constexpr Config& set_reversed(bool value = true) noexcept {
            reversed = value;
            return *this;
        }

        constexpr Config& enable_multi_turn_angle(
            bool value = true) noexcept {
            multi_turn_angle_enabled = value;
            return *this;
        }

        Type type;
        uint8_t motor_id;
        uint8_t host_id{0xFD};
        bool reversed{false};
        bool multi_turn_angle_enabled{false};
    };

    RsMotor() = delete;

    RsMotor(const Config& config,
            messaging::StateTopic<MotorState>& state_topic) noexcept
        : state_topic_(state_topic) {
        configure(config);
    }

    RsMotor(const RsMotor&) = delete;
    RsMotor& operator=(const RsMotor&) = delete;

    void configure(const Config& config) noexcept {
        type_ = config.type;
        limits_ = limitsFor(config.type);

        motor_id_ = config.motor_id;
        host_id_ = config.host_id;

        reversed_sign_ = config.reversed ? -1.0F : 1.0F;
        multi_turn_angle_enabled_ =
            config.multi_turn_angle_enabled;

        position_rad_ = 0.0F;
        velocity_rad_s_ = 0.0F;
        torque_nm_ = 0.0F;
        temperature_c_ = 0.0F;

        last_raw_position_rad_ = 0.0F;
        multi_turn_position_rad_ = 0.0F;

        feedback_fault_flags_ = 0U;
        operating_state_ = OperatingState::unknown;
        first_feedback_ = true;
    }

    /*
     * 接收反馈帧的低 16 位：
     *
     * bit15~8 = 电机 ID
     * bit7~0  = 主机 ID
     *
     * 通信类型、故障位和运行状态不参与 Router 匹配，
     * 由 handleCanFrame() 再次检查。
     */
    uint32_t receiveRouteId() const noexcept {
        return
            (static_cast<uint32_t>(motor_id_) << 8U) |
            static_cast<uint32_t>(host_id_);
    }

    static constexpr uint32_t receiveRouteMask() noexcept {
        return 0x0000FFFFU;
    }

    void handleCanFrame(const can::Frame& frame) noexcept {
        if (frame.id_format != can::IdFormat::Extended ||
            frame.kind != can::FrameKind::Data ||
            frame.length != 8U) {
            return;
        }

        const uint8_t communication_type =
            static_cast<uint8_t>((frame.id >> 24U) & 0x1FU);

        /*
         * 新固件主动上报通常使用通信类型 2。
         * 部分旧固件的主动上报应答可能使用通信类型 24，
         * 所以这里兼容两种情况。
         */
        if (communication_type !=
                static_cast<uint8_t>(
                    CommunicationType::feedback) &&
            communication_type !=
                static_cast<uint8_t>(
                    CommunicationType::active_report)) {
            return;
        }

        const uint8_t received_motor_id =
            static_cast<uint8_t>((frame.id >> 8U) & 0xFFU);

        const uint8_t received_host_id =
            static_cast<uint8_t>(frame.id & 0xFFU);

        if (received_motor_id != motor_id_ ||
            received_host_id != host_id_) {
            return;
        }

        feedback_fault_flags_ =
            static_cast<uint8_t>((frame.id >> 16U) & 0x3FU);

        operating_state_ = static_cast<OperatingState>(
            (frame.id >> 22U) & 0x03U);

        const uint16_t raw_position =
            readBigEndianU16(frame.data[0], frame.data[1]);

        const uint16_t raw_velocity =
            readBigEndianU16(frame.data[2], frame.data[3]);

        const uint16_t raw_torque =
            readBigEndianU16(frame.data[4], frame.data[5]);

        const uint16_t raw_temperature =
            readBigEndianU16(frame.data[6], frame.data[7]);

        const float raw_position_rad =
            uint16ToFloat(
                raw_position,
                limits_.position_min,
                limits_.position_max);

        if (first_feedback_) {
            last_raw_position_rad_ = raw_position_rad;
            multi_turn_position_rad_ = raw_position_rad;
            first_feedback_ = false;
        } else if (multi_turn_angle_enabled_) {
            const float delta =
                wrapPositionDelta(
                    raw_position_rad -
                    last_raw_position_rad_);

            multi_turn_position_rad_ += delta;
        }

        if (multi_turn_angle_enabled_) {
            position_rad_ =
                multi_turn_position_rad_ * reversed_sign_;
        } else {
            position_rad_ =
                raw_position_rad * reversed_sign_;
        }

        last_raw_position_rad_ = raw_position_rad;

        velocity_rad_s_ =
            uint16ToFloat(
                raw_velocity,
                limits_.velocity_min,
                limits_.velocity_max) *
            reversed_sign_;

        torque_nm_ =
            uint16ToFloat(
                raw_torque,
                limits_.torque_min,
                limits_.torque_max) *
            reversed_sign_;

        /*
         * 协议中温度原始值 = 摄氏度 * 10。
         * 例如 325 表示 32.5 摄氏度。
         */
        temperature_c_ =
            static_cast<float>(raw_temperature) * 0.1F;

        state_topic_.publish(
            MotorState{
                .pos_rad = position_rad_,
                .vel_rad_s = velocity_rad_s_,
                .torque_nm = torque_nm_,
                .temperature_c = temperature_c_,
                .fault_code =
                    static_cast<uint32_t>(
                        feedback_fault_flags_)
            },
            platform::nowMs());
    }

    can::Frame makeEnableFrame() const noexcept {
        return makeCommandFrame(
            CommunicationType::motor_enable,
            static_cast<uint16_t>(host_id_));
    }

    can::Frame makeStopFrame(
        bool clear_fault = false) const noexcept {
        can::Frame frame = makeCommandFrame(
            CommunicationType::motor_stop,
            static_cast<uint16_t>(host_id_));

        frame.data[0] = clear_fault ? 1U : 0U;
        return frame;
    }

    can::Frame makeSetMechanicalZeroFrame() const noexcept {
        can::Frame frame = makeCommandFrame(
            CommunicationType::set_mechanical_zero,
            static_cast<uint16_t>(host_id_));

        frame.data[0] = 1U;
        return frame;
    }

    can::Frame makeActiveReportFrame(bool enabled) const noexcept {
        can::Frame frame = makeCommandFrame(
            CommunicationType::active_report,
            static_cast<uint16_t>(host_id_));

        /*
         * 这是官方协议要求的固定数据，不是测试占位值。
         */
        frame.data = {
            1U, 2U, 3U, 4U,
            5U, 6U,
            enabled ? uint8_t{1U} : uint8_t{0U},
            0U
        };

        return frame;
    }

    can::Frame makeMitControlFrame(float target_position_rad,
                                   float target_velocity_rad_s,
                                   float target_kp,
                                   float target_kd,
                                   float feedforward_torque_nm) const noexcept {
        target_position_rad = clampFinite(
            target_position_rad * reversed_sign_,
            limits_.position_min,
            limits_.position_max);

        target_velocity_rad_s = clampFinite(
            target_velocity_rad_s * reversed_sign_,
            limits_.velocity_min,
            limits_.velocity_max);

        feedforward_torque_nm = clampFinite(
            feedforward_torque_nm * reversed_sign_,
            limits_.torque_min,
            limits_.torque_max);

        target_kp = clampFinite(
            target_kp,
            limits_.kp_min,
            limits_.kp_max);

        target_kd = clampFinite(
            target_kd,
            limits_.kd_min,
            limits_.kd_max);

        const uint16_t raw_torque =
            floatToUint16(
                feedforward_torque_nm,
                limits_.torque_min,
                limits_.torque_max);

        can::Frame frame = makeCommandFrame(
            CommunicationType::mit_control,
            raw_torque);

        writeBigEndianU16(
            frame.data,
            0U,
            floatToUint16(
                target_position_rad,
                limits_.position_min,
                limits_.position_max));

        writeBigEndianU16(
            frame.data,
            2U,
            floatToUint16(
                target_velocity_rad_s,
                limits_.velocity_min,
                limits_.velocity_max));

        writeBigEndianU16(
            frame.data,
            4U,
            floatToUint16(
                target_kp,
                limits_.kp_min,
                limits_.kp_max));

        writeBigEndianU16(
            frame.data,
            6U,
            floatToUint16(
                target_kd,
                limits_.kd_min,
                limits_.kd_max));

        return frame;
    }

    float position() const noexcept {
        return position_rad_;
    }

    float velocity() const noexcept {
        return velocity_rad_s_;
    }

    float torque() const noexcept {
        return torque_nm_;
    }

    float temperature() const noexcept {
        return temperature_c_;
    }

    float maxTorque() const noexcept {
        return limits_.torque_max;
    }

    uint8_t faultFlags() const noexcept {
        return feedback_fault_flags_;
    }

    bool hasFault(FaultFlag fault) const noexcept {
        const uint8_t mask = static_cast<uint8_t>(fault);
        return (feedback_fault_flags_ & mask) != 0U;
    }

    bool hasAnyFault() const noexcept {
        return feedback_fault_flags_ != 0U;
    }

    OperatingState operatingState() const noexcept {
        return operating_state_;
    }

    uint8_t motorId() const noexcept {
        return motor_id_;
    }

    Type type() const noexcept {
        return type_;
    }

private:
    static constexpr Limits limitsFor(Type type) noexcept {
        switch (type) {
        case Type::RS01:
            return Limits{
                .position_min = -12.57F,
                .position_max = 12.57F,
                .velocity_min = -44.0F,
                .velocity_max = 44.0F,
                .torque_min = -17.0F,
                .torque_max = 17.0F,
                .kp_min = 0.0F,
                .kp_max = 500.0F,
                .kd_min = 0.0F,
                .kd_max = 5.0F
            };

        case Type::RS05:
            return Limits{
                .position_min = -12.57F,
                .position_max = 12.57F,
                .velocity_min = -50.0F,
                .velocity_max = 50.0F,
                .torque_min = -5.5F,
                .torque_max = 5.5F,
                .kp_min = 0.0F,
                .kp_max = 500.0F,
                .kd_min = 0.0F,
                .kd_max = 5.0F
            };
        }

        return limitsFor(Type::RS01);
    }

    static constexpr uint16_t readBigEndianU16(uint8_t high,uint8_t low) noexcept {
        return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
    }

    static void writeBigEndianU16(std::array<uint8_t, 8>& data, uint8_t offset, uint16_t value) noexcept {
        data[offset] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
        data[offset + 1U] = static_cast<uint8_t>(value & 0xFFU);
    }

    static float uint16ToFloat(uint16_t value, float minimum, float maximum) noexcept {
        constexpr float raw_max = 65535.0F;

        return minimum + static_cast<float>(value) * (maximum - minimum) / raw_max;
    }

    static float clampFinite(float value, float minimum, float maximum) noexcept {
        if (!std::isfinite(value)) {
            return 0.0F;
        }

        return std::clamp(value, minimum, maximum);
    }

    static uint16_t floatToUint16(float value, float minimum, float maximum) noexcept {
        constexpr float raw_max = 65535.0F;
        value = clampFinite(value, minimum, maximum);
        const float scaled = (value - minimum) * raw_max / (maximum - minimum);

        return static_cast<uint16_t>(scaled + 0.5F);
    }

    float wrapPositionDelta(float delta) const noexcept {
        const float position_range = limits_.position_max - limits_.position_min;

        if (delta > position_range * 0.5F) {
            delta -= position_range;
        } else if (delta <= -position_range * 0.5F) {
            delta += position_range;
        }

        return delta;
    }

    static constexpr uint32_t makeExtendedId(CommunicationType type, uint16_t data, uint8_t target_id) noexcept {
        return
            (static_cast<uint32_t>(type) << 24U) |
            (static_cast<uint32_t>(data) << 8U) |
            static_cast<uint32_t>(target_id);
    }

    can::Frame makeCommandFrame(CommunicationType type, uint16_t data) const noexcept {
        can::Frame frame{};

        frame.id = makeExtendedId(
            type,
            data,
            motor_id_);

        frame.id_format = can::IdFormat::Extended;
        frame.kind = can::FrameKind::Data;
        frame.length = 8U;

        return frame;
    }

    messaging::StateTopic<MotorState>& state_topic_;

    Type type_{Type::RS01};
    Limits limits_{limitsFor(Type::RS01)};

    uint8_t motor_id_{0U};
    uint8_t host_id_{0xFD};

    float reversed_sign_{1.0F};
    bool multi_turn_angle_enabled_{false};
    bool first_feedback_{true};

    float position_rad_{0.0F};
    float velocity_rad_s_{0.0F};
    float torque_nm_{0.0F};
    float temperature_c_{0.0F};

    float last_raw_position_rad_{0.0F};
    float multi_turn_position_rad_{0.0F};

    uint8_t feedback_fault_flags_{0U};
    OperatingState operating_state_{
        OperatingState::unknown};
};

} // namespace librmcs::device
