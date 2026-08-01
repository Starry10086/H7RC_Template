#include "robot/chassis/chassis_controller.hpp"
#include "robot/chassis/chassis_types.hpp"

#include <algorithm>

namespace robot{

ChassisController::ChassisController(const ChassisConfig& config,
                                   messaging::CommandTopic<ChassisVelCmd>& chassis_cmd,
                                   messaging::CommandTopic<WheelVelTarget>& lf_target,
                                   messaging::CommandTopic<WheelVelTarget>& rf_target,
                                   messaging::CommandTopic<WheelVelTarget>& rb_target,
                                   messaging::CommandTopic<WheelVelTarget>& lb_target) noexcept
    : config_(config)
    , chassis_cmd_(chassis_cmd)
    , lf_target_(lf_target)
    , rf_target_(rf_target)
    , rb_target_(rb_target)
    , lb_target_(lb_target) {}

void ChassisController::setTargetVel(const ChassisVelCmd& cmd, uint32_t now_ms) noexcept{
    chassis_cmd_.publish(cmd, now_ms);
}

void ChassisController::stop(uint32_t now_ms) noexcept{
    ChassisVelCmd cmd{};
    chassis_cmd_.publish(cmd, now_ms);
}

std::array<float, 4> ChassisController::calculateWheelVel(const ChassisVelCmd& cmd) const noexcept{
    std::array<float, 4> wheel_vel{};

    if(config_.wheel_radius_m <= 0.0f){
        return wheel_vel;
    }

    switch(config_.type){
        case ChassisType::Mecanum:{
            const float rotation_radius = config_.half_wheel_base_m + config_.half_wheel_track_m;
            const float vx = cmd.vx_m_s;
            const float vy = cmd.vy_m_s;
            const float omega = cmd.omega_rad_s;

            wheel_vel[0] = (vx - vy - rotation_radius * omega) / config_.wheel_radius_m; // left front
            wheel_vel[1] = (vx + vy + rotation_radius * omega) / config_.wheel_radius_m; // right front
            wheel_vel[2] = (vx - vy + rotation_radius * omega) / config_.wheel_radius_m; // right back
            wheel_vel[3] = (vx + vy - rotation_radius * omega) / config_.wheel_radius_m; // left back
        }
        break;
        case ChassisType::Omni4:
            
        break;
    }
    return wheel_vel;
}

void ChassisController::limitWheelVel(std::array<float, 4>& wheel_vel) const noexcept{
    for(auto& vel : wheel_vel){
        vel = std::clamp(vel, -config_.max_wheel_vel_rad_s, config_.max_wheel_vel_rad_s);
    }
}

void ChassisController::publishWheelVel(const std::array<float, 4>& wheel_vel, uint32_t now_ms) noexcept{
    lf_target_.publish(WheelVelTarget{.vel_rad_s = wheel_vel[0]}, now_ms);
    rf_target_.publish(WheelVelTarget{.vel_rad_s = wheel_vel[1]}, now_ms);
    rb_target_.publish(WheelVelTarget{.vel_rad_s = wheel_vel[2]}, now_ms);
    lb_target_.publish(WheelVelTarget{.vel_rad_s = wheel_vel[3]}, now_ms);
}

void ChassisController::process(uint32_t now_ms) noexcept{
    if(now_ms - last_update_ms_ < update_period_ms_){
        return;
    }
    last_update_ms_ = now_ms;

    messaging::CommandSample<ChassisVelCmd> cmd_sample{};
    ChassisVelCmd cmd{};

    if(chassis_cmd_.readFresh(now_ms, cmd_sample)){
        cmd = cmd_sample.value;
    }

    auto wheel_vel = calculateWheelVel(cmd);
    limitWheelVel(wheel_vel);
    publishWheelVel(wheel_vel, now_ms);
}

}