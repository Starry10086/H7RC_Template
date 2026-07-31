#include "robot/controller/wheel_velocity_controller.hpp"
#include "robot/chassis/chassis_types.hpp"

namespace librmcs::robot{

WheelVelController::WheelVelController(const WheelVelControllerConfig& config,
                                   messaging::CommandTopic<WheelVelTarget>& target_topic,
                                   messaging::StateTopic<device::MotorState>& state_topic,
                                   messaging::CommandTopic<device::MotorCommand>& command_topic) noexcept
    : config_(config)
    , target_topic_(target_topic)
    , state_topic_(state_topic)
    , command_topic_(command_topic) 
    , pid_(config.kp, config.ki, config.kd){
    
    pid_.output_max = config.max_torque_nm;
    pid_.output_min = -config.max_torque_nm;
    pid_.integral_max = config.integral_limit;
    pid_.integral_min = -config.integral_limit;
}

void WheelVelController::process(uint32_t now_ms) noexcept{
    if(now_ms - last_update_ms_ < config_.update_period_ms){
        return;
    }
    last_update_ms_ = now_ms;

    messaging::CommandSample<WheelVelTarget> target_sample{};
    messaging::StateSample<device::MotorState> state_sample{};

    if (!target_topic_.readFresh(now_ms, target_sample) ||
        !state_topic_.read(state_sample) ||
        !messaging::isFresh(now_ms,state_sample.timestamp_ms,config_.feedback_timeout_ms) ||
        state_sample.value.fault_code != 0U) {
        reset(now_ms);
        return;
    }

    const float target_vel = target_sample.value.vel_rad_s;
    const float feedback_vel = state_sample.value.vel_rad_s;

    const double vel_err = static_cast<double>(target_vel) - static_cast<double>(feedback_vel);
    const double output_torque = pid_.update(vel_err);
    publishTorque(static_cast<float>(output_torque), now_ms);
}

void WheelVelController::reset(uint32_t now_ms) noexcept{
    pid_.reset();
    publishTorque(0.0f, now_ms);
}

void WheelVelController::publishTorque(float torque, uint32_t now_ms) noexcept{
    command_topic_.publish(
        device::MotorCommand{
            .mode = device::MotorCommandMode::Torque,
            .torque_nm = torque
        },
        now_ms
    );
}

}