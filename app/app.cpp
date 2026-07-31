#include "app/app.hpp"
#include "app/app_entry.h"
#include "platform/segger/rtt.hpp"
#include "components/logging/log.hpp"
#include "components/messaging/state_topic.hpp"
#include "devices/motors/motor_state.hpp"
#include "platform/stm32/timebase.hpp"
#include "stm32h7xx_hal_fdcan.h"

namespace librmcs::app {

namespace {

App application{hfdcan1, hfdcan2, hfdcan3};

} // namespace

App::App(FDCAN_HandleTypeDef& can1_handle,
         FDCAN_HandleTypeDef& can2_handle,
         FDCAN_HandleTypeDef& can3_handle) noexcept
    : can1_(can1_handle)
    , can2_(can2_handle)
    , can3_(can3_handle)
    , robot_(can1_, can2_, can3_) {
}

bool App::init() noexcept {
    if (initialized_) {
        return true;
    }

    platform::rtt::init();

    if (!robot_.init()) {
        LOG_ERROR("Robot initialization failed");
        return false;
    }

    robot_.armMotorOutputs();

    initialized_ = true;
    LOG_INFO(
        "Robot initialized; RS01 and DM4310 "
        "zero-torque streams started");

    return true;
}

void App::process() noexcept {
    if (!initialized_) {
        return;
    }

    // 先处理 CAN，设备收到反馈后会更新对应的 StateTopic。
    robot_.processCanRx();

    const uint32_t now_ms = platform::nowMs();

    static uint32_t last_test_cmd_ms = 0U;
    if(static_cast<uint32_t>(now_ms - last_test_cmd_ms) >= 20U){
        last_test_cmd_ms = now_ms;

        robot_.topics().chassis.vel_cmd.publish(
            robot::ChassisVelCmd{
                .vx_m_s = 1.0f,
                .vy_m_s = 0.0f,
                .omega_rad_s = 0.0f},
            now_ms);
    }

    robot_.processControllers(now_ms);

    if (!robot_.processMotorTx(now_ms)) {
        LOG_WARN_THROTTLE(
            1000U,
            "One or more motor frames failed to queue");
    }

    auto& topics = robot_.topics();

    messaging::StateSample<device::MotorState> sample{};

    // if (topics.rs01_state.read(sample)) {
    //     const uint32_t age_ms =
    //         static_cast<uint32_t>(
    //             now_ms - sample.timestamp_ms);
    //     const bool online = messaging::isFresh(
    //         now_ms, sample.timestamp_ms, 10U);

    //     LOG_INFO_THROTTLE(
    //         200U,
    //         "RS01 %s pos=%.3f vel=%.3f "
    //         "torque=%.3f temp=%.1f faults=0x%02lX age=%lu ms",
    //         online ? "ONLINE" : "STALE",
    //         static_cast<double>(sample.value.position_rad),
    //         static_cast<double>(sample.value.velocity_rad_s),
    //         static_cast<double>(sample.value.torque_nm),
    //         static_cast<double>(sample.value.temperature_c),
    //         static_cast<unsigned long>(sample.value.fault_code),
    //         static_cast<unsigned long>(age_ms));
    // } else {
    //     LOG_WARN_THROTTLE(
    //         1000U,
    //         "RS01(motor=0x03 host=0xFD) NO_DATA");
    // }

    // if (topics.dm4310_state.read(sample)) {
    //     const uint32_t age_ms =
    //         static_cast<uint32_t>(
    //             now_ms - sample.timestamp_ms);
    //     const bool online = messaging::isFresh(
    //         now_ms, sample.timestamp_ms, 10U);

    //     LOG_INFO_THROTTLE(
    //         200U,
    //         "DM4310 %s pos=%.3f vel=%.3f "
    //         "torque=%.3f temp=%.1f fault=0x%02lX age=%lu ms",
    //         online ? "ONLINE" : "STALE",
    //         static_cast<double>(sample.value.position_rad),
    //         static_cast<double>(sample.value.velocity_rad_s),
    //         static_cast<double>(sample.value.torque_nm),
    //         static_cast<double>(sample.value.temperature_c),
    //         static_cast<unsigned long>(sample.value.fault_code),
    //         static_cast<unsigned long>(age_ms));
    // } else {
    //     LOG_WARN_THROTTLE(
    //         1000U,
    //         "DM4310(tx=0x01 feedback=0x11) NO_DATA");
    // }

    if (topics.chassis.right_front.state.read(sample)) {
        const uint32_t age_ms =
            static_cast<uint32_t>(now_ms - sample.timestamp_ms);
        const bool online = messaging::isFresh(
            now_ms, sample.timestamp_ms, 100U);

        LOG_INFO_THROTTLE(
            200U,
            "RF(0x202) %s pos=%.3f rad vel=%.3f rad/s "
            "torque=%.3f Nm temp=%.1f C age=%lu ms",
            online ? "ONLINE" : "STALE",
            static_cast<double>(sample.value.pos_rad),
            static_cast<double>(sample.value.vel_rad_s),
            static_cast<double>(sample.value.torque_nm),
            static_cast<double>(sample.value.temperature_c),
            static_cast<unsigned long>(age_ms));
    } else {
        LOG_WARN_THROTTLE(
            1000U,
            "RF(0x202) NO_DATA");
    }
}

void App::onFdcanRxFifo0Interrupt(
    FDCAN_HandleTypeDef& handle) noexcept {
    if (&handle == &can1_.handle()) {
        can1_.onRxFifo0Interrupt();
    } else if (&handle == &can2_.handle()) {
        can2_.onRxFifo0Interrupt();
    } else if (&handle == &can3_.handle()) {
        can3_.onRxFifo0Interrupt();
    }
}

App& instance() noexcept {
    return application;
}

} // namespace librmcs::app

extern "C" bool App_Init(void) {
    return librmcs::app::instance().init();
}

extern "C" void App_Process(void) {
    librmcs::app::instance().process();
}
