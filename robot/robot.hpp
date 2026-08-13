#pragma once

#include "devices/laser/MTL1.hpp"
#include "devices/motors/dji/dji_motor.hpp"
#include "devices/motors/dm/dm_motor.hpp"
#include "devices/motors/robostride/rs_motor.hpp"
#include "platform/can/can_router.hpp"
#include "platform/stm32/can_bus.hpp"
#include "platform/stm32/uart_port.hpp"
#include "robot/robot_topics.hpp"
#include "robot/chassis/chassis_controller.hpp"
#include "robot/controller/wheel_velocity_controller.hpp"
#include "robot/motor_tx/dji_group_tx.hpp"
#include "robot/motor_tx/mit_motor_tx.hpp"
#include "robot/motor_tx/motor_tx_scheduler.hpp"
#include "devices/imu/bmi088.hpp"

#include <cstddef>

namespace robot{

class Robot final{
public:
    Robot(platform::CanBus& can1,
          platform::CanBus& can2,
          platform::CanBus& can3,
          platform::SpiBus& spi2,
          platform::UartPort& uart10,
          device::Bmi088DmaStorage& bmi088_dma,
          const device::Bmi088Config& bmi088_config,
          const device::MTL1Config& mtl1_config) noexcept;

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    bool init() noexcept;
    void processCanRx() noexcept;
    bool processMotorTx(uint32_t now_ms) noexcept;
    void processControllers(uint32_t now_ms) noexcept;
    void processDevices(uint32_t now_ms) noexcept;

    void armMotorOutputs() noexcept;

    RobotTopics& topics() noexcept { return topics_; }

private:
    static constexpr std::size_t router_capacity_ = 16U;     // 每个CAN总线最多绑定16条路由,可以调整大小
    static constexpr std::size_t rx_budget_per_bus_ = 16U;  // 每次主循环中，每条 CAN 总线最多处理多少个已经接收到的软件队列帧，可以调整大小

    using CanRouter = can::Router<router_capacity_>;

    static void processCanBus(platform::CanBus& bus, CanRouter& router) noexcept;
    void sendAllZeroCommands() noexcept;

    platform::CanBus& can1_;
    platform::CanBus& can2_;
    platform::CanBus& can3_;

    RobotTopics topics_{};

    std::array<device::DjiMotor, 4> chassis_motors_;         // lf，rf，rb，lb
    device::RsMotor rs01_;
    device::DmMotor dm4310_;

    device::Bmi088 bmi088_;
    device::MTL1 mtl1_;
    
    ChassisController chassis_controller_;
    std::array<WheelVelController, 4> wheel_vel_controllers_;// lf，rf，rb，lb

    motor_tx::DjiGroupTx chassis_dji_tx_;
    motor_tx::MitMotorTx<device::RsMotor> rs01_tx;
    motor_tx::MitMotorTx<device::DmMotor> dm4310_tx_;

    motor_tx::MotorTxScheduler<16> motor_tx_scheduler_;

    CanRouter can1_router_{};
    CanRouter can2_router_{};
    CanRouter can3_router_{};

    uint32_t last_motor_tx_ms_{0U};

    static constexpr uint32_t motor_tx_period_ms_ = 1U;
    static constexpr uint32_t feedback_timeout_ms_ = 100U;
};

}
