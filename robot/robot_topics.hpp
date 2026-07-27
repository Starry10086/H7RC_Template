#pragma once

#include "components/messaging/state_topic.hpp"
#include "devices/motors/motor_state.hpp"

namespace librmcs::robot{

struct RobotTopics{
    messaging::StateTopic<device::MotorState> chassis_left_front_state{"motor.chassis.left_front.state"};
    messaging::StateTopic<device::MotorState> chassis_right_front_state{"motor.chassis.right_front.state"};
    messaging::StateTopic<device::MotorState> chassis_right_back_state{"motor.chassis.right_back.state"};
    messaging::StateTopic<device::MotorState> chassis_left_back_state{"motor.chassis.left_back.state"};
};
}