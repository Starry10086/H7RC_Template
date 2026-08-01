#pragma once

#include "components/messaging/command_topic.hpp"
#include "components/messaging/state_topic.hpp"
#include "devices/motors/motor_command.hpp"
#include "devices/motors/motor_state.hpp"
#include "platform/stm32/can_bus.hpp"
#include "robot/motor_tx/motor_command_guard.hpp"

#include <cstdint>

namespace robot::motor_tx{

template<typename Motor>
class MitMotorTx final{
public:
    MitMotorTx(platform::CanBus& bus,
               Motor& motor,
               messaging::CommandTopic<device::MotorCommand>& command_topic,
               messaging::StateTopic<device::MotorState>& state_topic,
               uint32_t feedback_timeout_ms) noexcept
    : bus_(bus)
    , motor_(motor)
    , command_topic_(command_topic)
    , state_topic_(state_topic)
    , feedback_timeout_ms_(feedback_timeout_ms) {}

    bool process(uint32_t now_ms) noexcept{
        const auto command = readMotorCmd(command_topic_, state_topic_, device::MotorCommandMode::Mit, feedback_timeout_ms_, now_ms);
        return send(command);
    }

    bool sendZero()noexcept{
        return send(device::MotorCommand{
            .mode = device::MotorCommandMode::Mit
        });
    }
    
    bool enable()noexcept{
        return bus_.send(motor_.makeEnableFrame());
    }
private:
    bool send(const device::MotorCommand& cmd) noexcept{
        return bus_.send(motor_.makeMitControlFrame(
            cmd.pos_rad,
            cmd.vel_rad_s,
            cmd.kp,
            cmd.kd,
            cmd.torque_nm
        ));
    }

    platform::CanBus& bus_;
    Motor& motor_;

    messaging::CommandTopic<device::MotorCommand>& command_topic_;
    messaging::StateTopic<device::MotorState>& state_topic_;

    uint32_t feedback_timeout_ms_ = 100U;
};

}