#pragma once

#include "components/messaging/command_topic.hpp"
#include "components/messaging/state_topic.hpp"
#include "devices/motors/motor_command.hpp"
#include "devices/motors/motor_state.hpp"

#include <cstdint>

namespace librmcs::robot::motor_tx{

inline device::MotorCommand readMotorCmd(messaging::CommandTopic<device::MotorCommand>& command_topic,
                                      messaging::StateTopic<device::MotorState>& state_topic,
                                      device::MotorCommandMode expected_mode,
                                      uint32_t feedback_timeout_ms,
                                      uint32_t now_ms) noexcept {
    const device::MotorCommand zero_command{
        .mode = expected_mode
    };

    messaging::CommandSample<device::MotorCommand> cmd_sample;
    messaging::StateSample<device::MotorState> state_sample;

    if(!command_topic.readFresh(now_ms, cmd_sample)){
        return zero_command;
    }
    if(cmd_sample.value.mode != expected_mode){
        return zero_command;
    }
    if(!state_topic.read(state_sample)){
        return zero_command;
    }
    if(!messaging::isFresh(now_ms, state_sample.timestamp_ms, feedback_timeout_ms)){
        return zero_command;
    }
    if(state_sample.value.fault_code != 0U){
        return zero_command;
    }
    
    return cmd_sample.value;
}

}