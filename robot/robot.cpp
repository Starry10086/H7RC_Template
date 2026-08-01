#include "robot/robot.hpp"
#include "devices/motors/dji/dji_command_group.hpp"
#include "devices/motors/dji/dji_motor.hpp"
#include "devices/motors/dm/dm_motor.hpp"
#include "devices/motors/robostride/rs_motor.hpp"
#include "platform/can/can_types.hpp"
#include "platform/stm32/can_bus.hpp"
#include "robot/chassis/chassis_types.hpp"
#include "robot/controller/wheel_velocity_controller.hpp"

#include <stdint.h>

namespace robot {

constexpr ChassisConfig chassis_config{
    .type = ChassisType::Mecanum,
    .wheel_radius_m = 0.076f,
    .half_wheel_base_m = 0.2f,
    .half_wheel_track_m = 0.2f,
    .max_wheel_vel_rad_s = 40.0f
};

constexpr WheelVelControllerConfig wheel_vel_config{
    .kp = 0.5,
    .ki = 0.0,
    .kd = 0.0,
    .max_torque_nm = 3.0,
    .integral_limit = 0.0,
    .feedback_timeout_ms = 100U,
    .update_period_ms = 1U
};

Robot::Robot(platform::CanBus& can1,
             platform::CanBus& can2,
             platform::CanBus& can3) noexcept
    : can1_(can1)
    , can2_(can2)
    , can3_(can3)
    , topics_{}
    , chassis_motors_{
        device::DjiMotor{
            device::DjiMotor::Config{device::DjiMotor::Type::M3508},
            topics_.chassis.left_front.state},
        device::DjiMotor{
            device::DjiMotor::Config{device::DjiMotor::Type::M3508},
            topics_.chassis.right_front.state},
        device::DjiMotor{
            device::DjiMotor::Config{device::DjiMotor::Type::M3508},
            topics_.chassis.right_back.state},
        device::DjiMotor{
            device::DjiMotor::Config{device::DjiMotor::Type::M3508},
            topics_.chassis.left_back.state}
        }
    , rs01_{
        device::RsMotor::Config{
        device::RsMotor::Type::RS01,
        0x03U}.set_host_id(0xFDU),
        topics_.rs01.state}
    , dm4310_{
        device::DmMotor::Config{
        device::DmMotor::Type::DM_J4310_2EC,
        0x01U}.set_control_mode(
        device::DmMotor::ControlMode::MIT),
        topics_.dm4310.state}
    , chassis_controller_{
        chassis_config,
        topics_.chassis.vel_cmd,
        topics_.chassis.left_front.vel_target,
        topics_.chassis.right_front.vel_target,
        topics_.chassis.right_back.vel_target,
        topics_.chassis.left_back.vel_target}
    , wheel_vel_controllers_{
        WheelVelController{
            wheel_vel_config,
            topics_.chassis.left_front.vel_target,
            topics_.chassis.left_front.state,
            topics_.chassis.left_front.command},
        WheelVelController{
            wheel_vel_config,
            topics_.chassis.right_front.vel_target,
            topics_.chassis.right_front.state,
            topics_.chassis.right_front.command},
        WheelVelController{
            wheel_vel_config,
            topics_.chassis.right_back.vel_target,
            topics_.chassis.right_back.state,
            topics_.chassis.right_back.command},
        WheelVelController{
            wheel_vel_config,
            topics_.chassis.left_back.vel_target,
            topics_.chassis.left_back.state,
            topics_.chassis.left_back.command}
        }
    , chassis_dji_tx_{
        can2_,
        device::DjiCommandGroup::m3508_m2006_201_to_204,
        std::array<motor_tx::DjiTxSlot, 4>{
            motor_tx::DjiTxSlot{
                .motor = &chassis_motors_[0],
                .command_topic = &topics_.chassis.left_front.command,
                .state_topic = &topics_.chassis.left_front.state},
            motor_tx::DjiTxSlot{
                .motor = &chassis_motors_[1],
                .command_topic = &topics_.chassis.right_front.command,
                .state_topic = &topics_.chassis.right_front.state},
            motor_tx::DjiTxSlot{
                .motor = &chassis_motors_[2],
                .command_topic = &topics_.chassis.right_back.command,
                .state_topic = &topics_.chassis.right_back.state},
            motor_tx::DjiTxSlot{
                .motor = &chassis_motors_[3],
                .command_topic = &topics_.chassis.left_back.command,
                .state_topic = &topics_.chassis.left_back.state}
        },
        feedback_timeout_ms_}
    , rs01_tx{
        can2_,
        rs01_,
        topics_.rs01.command,
        topics_.rs01.state,
        feedback_timeout_ms_}
    , dm4310_tx_{
        can2_,
        dm4310_,
        topics_.dm4310.command,
        topics_.dm4310.state,
        feedback_timeout_ms_}
    {}

bool Robot::init() noexcept {
    can2_router_.bindExact(can::IdFormat::Standard,0x201U,chassis_motors_[0]);
    can2_router_.bindExact(can::IdFormat::Standard,0x202U,chassis_motors_[1]);
    can2_router_.bindExact(can::IdFormat::Standard,0x203U,chassis_motors_[2]);
    can2_router_.bindExact(can::IdFormat::Standard,0x204U,chassis_motors_[3]);
    can2_router_.bindMask(can::IdFormat::Extended,rs01_.receiveRouteId(),rs01_.receiveRouteMask(),rs01_);
    can2_router_.bindExact(can::IdFormat::Standard,0x11U,dm4310_);

    motor_tx_scheduler_.clear();
    motor_tx_scheduler_.add(chassis_dji_tx_);
    motor_tx_scheduler_.add(rs01_tx);
    motor_tx_scheduler_.add(dm4310_tx_);

    can1_.start();
    can2_.start();
    can3_.start();

    return true;
}

bool Robot::processMotorTx(uint32_t now_ms) noexcept {
    if (static_cast<uint32_t>(now_ms - last_motor_tx_ms_) < motor_tx_period_ms_) {
        return true;
    }

    last_motor_tx_ms_ = now_ms;

    return motor_tx_scheduler_.process(now_ms);
}

void Robot::processCanBus(platform::CanBus& bus,
                          CanRouter& router) noexcept {
                          can::Frame frame{};
                          std::size_t processed_frames = 0U;

    while (processed_frames < rx_budget_per_bus_ && bus.popReceived(frame)) {
        (void)router.dispatch(frame);
        ++processed_frames;
    }
}

void Robot::sendAllZeroCommands() noexcept{
    chassis_dji_tx_.sendZero();
    rs01_tx.sendZero();
    dm4310_tx_.sendZero();
}

void Robot::processCanRx() noexcept {
    processCanBus(can1_, can1_router_);
    processCanBus(can2_, can2_router_);
    processCanBus(can3_, can3_router_);
}

void Robot::armMotorOutputs() noexcept {
    sendAllZeroCommands();
    rs01_tx.enable();
    dm4310_tx_.enable();
    sendAllZeroCommands();
}

void Robot::processControllers(uint32_t now_ms) noexcept {
    chassis_controller_.process(now_ms);

    for (auto& controller : wheel_vel_controllers_) {
        controller.process(now_ms);
    }
}

} // namespace robot
