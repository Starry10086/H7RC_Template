#pragma once

#include "components/messaging/command_topic.hpp"
#include "robot/chassis/chassis_types.hpp"

#include <array>
#include <cstdint>

namespace robot {

class ChassisController final{
public:
    ChassisController(const ChassisConfig& config,
                      messaging::CommandTopic<ChassisVelCmd>& chassis_cmd,
                      messaging::CommandTopic<WheelVelTarget>& lf_target,
                      messaging::CommandTopic<WheelVelTarget>& rf_target,
                      messaging::CommandTopic<WheelVelTarget>& rb_target,
                      messaging::CommandTopic<WheelVelTarget>& lb_target) noexcept;

    void setTargetVel(const ChassisVelCmd& cmd, uint32_t now_ms) noexcept;
    void stop(uint32_t now_ms) noexcept;
    void process(uint32_t now_ms) noexcept;
private:
   std::array<float, 4> calculateWheelVel(const ChassisVelCmd& cmd) const noexcept;
   void limitWheelVel(std::array<float, 4>& wheel_vel) const noexcept;
   void publishWheelVel(const std::array<float, 4>& wheel_vel, uint32_t now_ms) noexcept;

    ChassisConfig config_;
    messaging::CommandTopic<ChassisVelCmd>& chassis_cmd_;
    messaging::CommandTopic<WheelVelTarget>& lf_target_;
    messaging::CommandTopic<WheelVelTarget>& rf_target_;
    messaging::CommandTopic<WheelVelTarget>& rb_target_;
    messaging::CommandTopic<WheelVelTarget>& lb_target_;

    uint32_t last_update_ms_{0U};
    static constexpr uint32_t update_period_ms_ = 1U;
};
}
