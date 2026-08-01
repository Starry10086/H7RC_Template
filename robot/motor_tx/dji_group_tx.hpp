#pragma once

#include "components/messaging/command_topic.hpp"
#include "components/messaging/state_topic.hpp"
#include "devices/motors/motor_command.hpp"
#include "devices/motors/motor_state.hpp"
#include "devices/motors/dji/dji_command_group.hpp"
#include "devices/motors/dji/dji_motor.hpp"
#include "platform/stm32/can_bus.hpp"
#include "robot/motor_tx/motor_command_guard.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace robot::motor_tx{

struct DjiTxSlot{
    device::DjiMotor* motor{nullptr};
    messaging::CommandTopic<device::MotorCommand>* command_topic{nullptr};
    messaging::StateTopic<device::MotorState>* state_topic{nullptr};
};

class DjiGroupTx final{
public:
    DjiGroupTx(platform::CanBus& bus,
               device::DjiCommandGroup group,
               const std::array<DjiTxSlot, 4>& slots,
               uint32_t feedback_timeout_ms) noexcept
    : bus_(bus)
    , group_(group)
    , slots_(slots)
    , feedback_timeout_ms_(feedback_timeout_ms) {}

    bool process(uint32_t now_ms) noexcept{
        std::array<int16_t, 4> raw_cmd{};

        for(std::size_t index = 0U; index < slots_.size(); ++index){
            const DjiTxSlot& slot = slots_[index];
            if(slot.motor == nullptr || slot.command_topic == nullptr || slot.state_topic == nullptr){
                continue;
            }

            const auto cmd = readMotorCmd(*slot.command_topic, *slot.state_topic, device::MotorCommandMode::Torque, feedback_timeout_ms_, now_ms);
            raw_cmd[index] = slot.motor->encodeTorqueCommand(cmd.torque_nm);
        }
        return bus_.send(device::makeDjiCommandFrame(group_, raw_cmd));
    }

    bool sendZero() noexcept{
        return bus_.send(device::makeDjiCommandFrame(group_, std::array<int16_t, 4>{}));
    }
private:
    platform::CanBus& bus_;
    device::DjiCommandGroup group_;
    std::array<DjiTxSlot, 4> slots_{};
    uint32_t feedback_timeout_ms_{100U};
};

}