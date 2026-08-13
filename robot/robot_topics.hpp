#pragma once

#include "components/messaging/state_topic.hpp"
#include "components/messaging/command_topic.hpp"
#include "devices/motors/motor_state.hpp"
#include "devices/motors/motor_command.hpp"
#include "robot/chassis/chassis_types.hpp"
#include "devices/imu/imu_state.hpp"
#include "devices/laser/laser_distance.hpp"
#include <stdint.h>

namespace robot{

inline constexpr uint32_t motor_command_timeout_ms = 100U;
inline constexpr uint32_t chassis_command_timeout_ms = 100U;
inline constexpr uint32_t wheel_vel_command_timeout_ms = 100U;

struct WheelTopics{
    messaging::StateTopic<device::MotorState> state;
    messaging::CommandTopic<WheelVelTarget> vel_target;
    messaging::CommandTopic<device::MotorCommand> command;
};

struct ChassisTopics{
    messaging::CommandTopic<ChassisVelCmd> vel_cmd{"chassis.vel_cmd", chassis_command_timeout_ms};
    WheelTopics left_front{
        .state{"motor.chassis.left_front.state"},
        .vel_target{"chassis.left_front.vel_target", wheel_vel_command_timeout_ms},
        .command{"motor.chassis.left_front.command", motor_command_timeout_ms}
    };
    WheelTopics right_front{
        .state{"motor.chassis.right_front.state"},
        .vel_target{"chassis.right_front.vel_target", wheel_vel_command_timeout_ms},
        .command{"motor.chassis.right_front.command", motor_command_timeout_ms}
    };
    WheelTopics right_back{
        .state{"motor.chassis.right_back.state"},
        .vel_target{"chassis.right_back.vel_target", wheel_vel_command_timeout_ms},
        .command{"motor.chassis.right_back.command", motor_command_timeout_ms}
    };
    WheelTopics left_back{
        .state{"motor.chassis.left_back.state"},
        .vel_target{"chassis.left_back.vel_target", wheel_vel_command_timeout_ms},
        .command{"motor.chassis.left_back.command", motor_command_timeout_ms}
    };
};

struct MotorTopics{
    messaging::StateTopic<device::MotorState> state;
    messaging::CommandTopic<device::MotorCommand> command;
};

struct RobotTopics{
    ChassisTopics chassis{};

    MotorTopics rs01{
        .state{"motor.rs01.state"},
        .command{"motor.rs01.command", motor_command_timeout_ms}
    };

    MotorTopics dm4310{
        .state{"motor.dm4310.state"},
        .command{"motor.dm4310.command", motor_command_timeout_ms}
    };

    messaging::StateTopic<device::ImuState> imu_state{"imu.bmi088.state"};
    messaging::StateTopic<device::LaserDistance> mtl1_distance{"mtl1.distance.state"};
};
}
