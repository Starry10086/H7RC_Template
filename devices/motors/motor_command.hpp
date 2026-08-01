#pragma once

#include <cstdint>

namespace device {

enum class MotorCommandMode : uint8_t {
    Torque,
    Velocity,
    Position,
    PositionVelocity,
    Mit
};

struct MotorCommand {
    MotorCommandMode mode{MotorCommandMode::Torque};

    float pos_rad{0.0F};
    float vel_rad_s{0.0F};
    float torque_nm{0.0F};       // MIT 模式下表示前馈力矩
    float kp{0.0F};
    float kd{0.0F};
};

} // namespace device

// | 电机  | 接受的命令        | 下发方式            | 命令过期         
// | DJI  | Torque           | 4 个 Topic 合成一帧 | 对应槽位写 0 
// | DM   | MIT/速度/位置速度  | 每个电机一帧.       | 发零力矩安全帧 
// | RS01 | MIT.             | 每个电机一帧        | 发零力矩、零增益帧   
// | VESC | 力矩/速度/位置     | 每个电机一帧.       | 发零电流或零力矩 
// | GO1  | MIT 类命令        | RS485 轮询调度     | 停止发送并进入安全状态 