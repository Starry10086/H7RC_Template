#pragma once

#include "components/messaging/state_topic.hpp"
#include "devices/motors/motor_state.hpp"
#include "platform/can/can_types.hpp"
#include "platform/stm32/timebase.hpp"

#include <cmath>
#include <algorithm>
#include <bit>
#include <cstdint>
#include <numbers>

namespace device {

class DjiMotor {
public:
    enum class Type : uint8_t { GM6020, GM6020_VOLTAGE, M3508, M2006 };

    struct Config {
        explicit Config(Type motor_type) {
            this->encoder_zero_point = 0;
            this->motor_type = motor_type;
            switch (motor_type) {
            case Type::GM6020:
            case Type::GM6020_VOLTAGE: reduction_ratio = 1.0; break;
            case Type::M3508: reduction_ratio = 3591.0 / 187.0; break;
            case Type::M2006: reduction_ratio = 36.0; break;
            }
            this->reversed = false;
            this->multi_turn_angle_enabled = false;
        }

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point;
        double reduction_ratio;
        bool reversed;
        bool multi_turn_angle_enabled;
    };

    DjiMotor(const Config& config,
             messaging::StateTopic<MotorState>& state_topic) noexcept
        : state_topic_(state_topic)
        , angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0)
        , temperature_(0.0){
            configure(config);
        }

    DjiMotor(const DjiMotor&) = delete;
    DjiMotor& operator=(const DjiMotor&) = delete;

    void configure(const Config& config) noexcept{
        encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += raw_angle_max_;

        double sign = config.reversed ? -1 : 1;

        raw_angle_to_angle_coefficient_ =
            sign / config.reduction_ratio / raw_angle_max_ * 2 * std::numbers::pi;
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            sign / config.reduction_ratio / 60 * 2 * std::numbers::pi;
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;

        double torque_constant, raw_current_max, current_max;
        switch (config.motor_type) {
        case Type::GM6020:
            torque_constant = 0.741;
            raw_current_max = 16384.0;
            current_max = 3.0;
            break;
        case Type::GM6020_VOLTAGE:
            torque_constant = 0.741;
            raw_current_max = 25000.0;
            current_max = 3.0;
            break;
        case Type::M3508:
            torque_constant = 0.3 * 187.0 / 3591.0;
            raw_current_max = 16384.0;
            current_max = 20.0;
            break;
        case Type::M2006:
            torque_constant = 0.18 * 1.0 / 36.0;
            raw_current_max = 16384.0;
            current_max = 10.0;
            break;
        }

        raw_current_to_torque_coefficient_ =
            sign * config.reduction_ratio * torque_constant / raw_current_max * current_max;
        torque_to_raw_current_coefficient_ = 1 / raw_current_to_torque_coefficient_;

        max_torque_ = 1 * config.reduction_ratio * torque_constant * current_max;

        last_raw_angle_ = 0;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_ = 0;
    }

    void handleCanFrame(const can::Frame& frame) noexcept{
        if (frame.id_format != can::IdFormat::Standard ||
            frame.kind != can::FrameKind::Data ||
            frame.length != 8U) {
            return;
        }

        const uint16_t raw_angle_value = readBigEndianU16(frame.data[0], frame.data[1]);
        if(raw_angle_value >= static_cast<uint16_t>(raw_angle_max_)){
            return;
        }

        const int raw_angle = static_cast<int>(raw_angle_value);
        const int16_t raw_velocity = readBigEndianI16(frame.data[2], frame.data[3]);
        const int16_t raw_current = readBigEndianI16(frame.data[4], frame.data[5]);
        temperature_ = frame.data[6];

        int calibrated_raw_angle = raw_angle - encoder_zero_point_;
        if (calibrated_raw_angle < 0) {
            calibrated_raw_angle += raw_angle_max_;
        }

        if(!multi_turn_angle_enabled_){
            angle_ = raw_angle_to_angle_coefficient_ *
                 static_cast<double>(calibrated_raw_angle);

            if (angle_ < 0.0) {
                angle_ += 2.0 * std::numbers::pi;
            }
        }
        else{
            auto diff =
            (static_cast<int64_t>(calibrated_raw_angle) - angle_multi_turn_) % raw_angle_max_;

            if (diff <= -raw_angle_max_ / 2) {
                diff += raw_angle_max_;
            } else if (diff > raw_angle_max_ / 2) {
                diff -= raw_angle_max_;
            }

            angle_multi_turn_ += diff;
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }

        last_raw_angle_ = raw_angle;

        velocity_ = raw_velocity_to_velocity_coefficient_ *
                    static_cast<double>(raw_velocity);

        torque_ = raw_current_to_torque_coefficient_ *
                static_cast<double>(raw_current);

        state_topic_.publish(
            MotorState{
                .pos_rad = static_cast<float>(angle_),
                .vel_rad_s = static_cast<float>(velocity_),
                .torque_nm = static_cast<float>(torque_),
                .temperature_c = static_cast<float>(temperature_),
                .fault_code = 0U,
            },
        platform::nowMs());
    }

    int16_t encodeTorqueCommand(double control_torque) const noexcept {
        if (std::isnan(control_torque)) {
            return 0;
        }

        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        const double raw_current = std::round(
            torque_to_raw_current_coefficient_ * control_torque);
        return static_cast<int16_t>(raw_current);
    }

    int calibrate_zero_point() {
        angle_multi_turn_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    double temperature() const { return temperature_; }

private:
    static constexpr uint16_t readBigEndianU16(uint8_t high, uint8_t low) noexcept{
        return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
    }

    static constexpr int16_t readBigEndianI16(uint8_t high, uint8_t low) noexcept{
        return std::bit_cast<int16_t>(readBigEndianU16(high, low));
    }

    messaging::StateTopic<MotorState>& state_topic_;
    static constexpr int raw_angle_max_ = 8192;
    int encoder_zero_point_, last_raw_angle_;

    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_current_to_torque_coefficient_, torque_to_raw_current_coefficient_;

    double angle_;
    double velocity_;
    double torque_;
    double max_torque_;
    double temperature_;
};

} // namespace device
