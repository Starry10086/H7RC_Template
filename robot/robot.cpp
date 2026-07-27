#include "robot/robot.hpp"

#include "platform/can/can_types.hpp"

namespace librmcs::robot {

Robot::Robot(platform::CanBus& can1,
             platform::CanBus& can2,
             platform::CanBus& can3) noexcept
    : can1_(can1)
    , can2_(can2)
    , can3_(can3)
    , topics_{}
    , chassis_left_front_{
          device::DjiMotor::Config{device::DjiMotor::Type::M3508},
          topics_.chassis_left_front_state}
    , chassis_right_front_{
          device::DjiMotor::Config{device::DjiMotor::Type::M3508}.enable_multi_turn_angle(),
          topics_.chassis_right_front_state}
    , chassis_right_back_{
          device::DjiMotor::Config{device::DjiMotor::Type::M3508},
          topics_.chassis_right_back_state}
    , chassis_left_back_{
          device::DjiMotor::Config{device::DjiMotor::Type::M3508},
          topics_.chassis_left_back_state} {
}

bool Robot::init() noexcept {
    if (!can2_router_.bindExact(
            can::IdFormat::Standard,
            0x201U,
            chassis_left_front_)) {
        return false;
    }

    if (!can2_router_.bindExact(
            can::IdFormat::Standard,
            0x202U,
            chassis_right_front_)) {
        return false;
    }

    if (!can2_router_.bindExact(
            can::IdFormat::Standard,
            0x203U,
            chassis_right_back_)) {
        return false;
    }

    if (!can2_router_.bindExact(
            can::IdFormat::Standard,
            0x204U,
            chassis_left_back_)) {
        return false;
    }

    // 后续在这里绑定 CAN2 设备：
    // can2_router_.bindExact(...);

    // 后续在这里绑定 CAN3 设备：
    // can3_router_.bindExact(...);

    if (!can1_.start()) {
        return false;
    }

    if (!can2_.start()) {
        return false;
    }

    if (!can3_.start()) {
        return false;
    }

    return true;
}

void Robot::processCanBus(
    platform::CanBus& bus,
    CanRouter& router) noexcept {
    can::Frame frame{};
    std::size_t processed_frames = 0U;

    while (processed_frames < rx_budget_per_bus_ &&
           bus.popReceived(frame)) {
        (void)router.dispatch(frame);
        ++processed_frames;
    }
}

void Robot::processCanRx() noexcept {
    processCanBus(can1_, can1_router_);
    processCanBus(can2_, can2_router_);
    processCanBus(can3_, can3_router_);
}

} // namespace librmcs::robot