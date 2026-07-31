#pragma once

#include "components/messaging/command_topic.hpp"
#include "components/messaging/state_topic.hpp"
#include "control/pid/pid.hpp"
#include "devices/motors/motor_command.hpp"
#include "devices/motors/motor_state.hpp"
#include "robot/chassis/chassis_types.hpp"

#include <cstdint>

namespace librmcs::robot {

struct WheelVelControllerConfig{
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};

    // PID 最大允许输出的轮端力矩。
    double max_torque_nm{0.0};

    // PID 内部累计误差的最大绝对值。
    // 设置为 0 表示不允许积分累计。
    double integral_limit{0.0};

    // 电机反馈超过这个时间没有更新，就认为反馈失效。
    uint32_t feedback_timeout_ms{100U};

    // 轮速控制器执行周期。
    uint32_t update_period_ms{1U};
};

class WheelVelController final{
public:
    WheelVelController(const WheelVelControllerConfig& config,
                       messaging::CommandTopic<WheelVelTarget>& target_topic,
                       messaging::StateTopic<device::MotorState>& state_topic,
                       messaging::CommandTopic<device::MotorCommand>& command_topic) noexcept;

    void process(uint32_t now_ms) noexcept;
    void reset(uint32_t now_ms) noexcept;
private:
    void publishTorque(float torque, uint32_t now_ms) noexcept;

    WheelVelControllerConfig config_;
    // ChassisController 发布的轮速目标。
    messaging::CommandTopic<WheelVelTarget>& target_topic_;
    // 电机驱动发布的反馈状态。
    messaging::StateTopic<device::MotorState>& state_topic_;
    // 发布给 Robot::processMotorTx() 的最终电机命令。
    messaging::CommandTopic<device::MotorCommand>& command_topic_;
    rmcs_core::controller::pid::PidCalculator pid_;

    uint32_t last_update_ms_{0U};
};

}