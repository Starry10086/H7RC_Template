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
    LOG_INFO("RTT backend ready, float test = %.3f", 1.234);

    if (!robot_.init()) {
        LOG_ERROR("Robot initialization failed");
        return false;
    }

    initialized_ = true;
    LOG_INFO("Robot initialized");

    return true;
}

void App::process() noexcept {
    if (!initialized_) {
        return;
    }

    // 先处理 CAN，DjiMotor 收到反馈后会更新对应的 StateTopic。
    robot_.processCanRx();

    const uint32_t now_ms = platform::nowMs();
    auto& topics = robot_.topics();

    messaging::StateSample<device::MotorState> sample{};

    if (topics.chassis_left_front_state.read(sample)) {
        const uint32_t age_ms =
            static_cast<uint32_t>(now_ms - sample.timestamp_ms);
        const bool online = messaging::isFresh(
            now_ms, sample.timestamp_ms, 100U);

        LOG_INFO_THROTTLE(
            200U,
            "LF(0x201) %s pos=%.3f rad vel=%.3f rad/s "
            "torque=%.3f Nm temp=%.1f C age=%lu ms",
            online ? "ONLINE" : "STALE",
            static_cast<double>(sample.value.position_rad),
            static_cast<double>(sample.value.velocity_rad_s),
            static_cast<double>(sample.value.torque_nm),
            static_cast<double>(sample.value.temperature_c),
            static_cast<unsigned long>(age_ms));
    } else {
        LOG_WARN_THROTTLE(
            1000U,
            "LF(0x201) NO_DATA");
    }

    if (topics.chassis_right_front_state.read(sample)) {
        const uint32_t age_ms =
            static_cast<uint32_t>(now_ms - sample.timestamp_ms);
        const bool online = messaging::isFresh(
            now_ms, sample.timestamp_ms, 100U);

        LOG_INFO_THROTTLE(
            200U,
            "RF(0x202) %s pos=%.3f rad vel=%.3f rad/s "
            "torque=%.3f Nm temp=%.1f C age=%lu ms",
            online ? "ONLINE" : "STALE",
            static_cast<double>(sample.value.position_rad),
            static_cast<double>(sample.value.velocity_rad_s),
            static_cast<double>(sample.value.torque_nm),
            static_cast<double>(sample.value.temperature_c),
            static_cast<unsigned long>(age_ms));
    } else {
        LOG_WARN_THROTTLE(
            1000U,
            "RF(0x202) NO_DATA");
    }

    if (topics.chassis_right_back_state.read(sample)) {
        const uint32_t age_ms =
            static_cast<uint32_t>(now_ms - sample.timestamp_ms);
        const bool online = messaging::isFresh(
            now_ms, sample.timestamp_ms, 100U);

        LOG_INFO_THROTTLE(
            200U,
            "RB(0x203) %s pos=%.3f rad vel=%.3f rad/s "
            "torque=%.3f Nm temp=%.1f C age=%lu ms",
            online ? "ONLINE" : "STALE",
            static_cast<double>(sample.value.position_rad),
            static_cast<double>(sample.value.velocity_rad_s),
            static_cast<double>(sample.value.torque_nm),
            static_cast<double>(sample.value.temperature_c),
            static_cast<unsigned long>(age_ms));
    } else {
        LOG_WARN_THROTTLE(
            1000U,
            "RB(0x203) NO_DATA");
    }

    if (topics.chassis_left_back_state.read(sample)) {
        const uint32_t age_ms =
            static_cast<uint32_t>(now_ms - sample.timestamp_ms);
        const bool online = messaging::isFresh(
            now_ms, sample.timestamp_ms, 100U);

        LOG_INFO_THROTTLE(
            200U,
            "LB(0x204) %s pos=%.3f rad vel=%.3f rad/s "
            "torque=%.3f Nm temp=%.1f C age=%lu ms",
            online ? "ONLINE" : "STALE",
            static_cast<double>(sample.value.position_rad),
            static_cast<double>(sample.value.velocity_rad_s),
            static_cast<double>(sample.value.torque_nm),
            static_cast<double>(sample.value.temperature_c),
            static_cast<unsigned long>(age_ms));
    } else {
        LOG_WARN_THROTTLE(
            1000U,
            "LB(0x204) NO_DATA");
    }
    // const auto can2_stats = can2_.stats();

    // FDCAN_ProtocolStatusTypeDef protocol{};
    // FDCAN_ErrorCountersTypeDef counters{};

    // (void)HAL_FDCAN_GetProtocolStatus(
    //     &can2_.handle(), &protocol);

    // (void)HAL_FDCAN_GetErrorCounters(
    //     &can2_.handle(), &counters);

    // LOG_INFO_THROTTLE(
    //     1000U,
    //     "CAN2 rx=%lu invalid=%lu drop=%lu sw_hal=%lu "
    //     "hal=0x%08lX act=0x%02lX lec=%lu "
    //     "rec=%lu tec=%lu passive=%lu warn=%lu busoff=%lu",
    //     static_cast<unsigned long>(can2_stats.received_frames),
    //     static_cast<unsigned long>(can2_stats.invalid_frames),
    //     static_cast<unsigned long>(can2_stats.dropped_frames),
    //     static_cast<unsigned long>(can2_stats.hal_error),
    //     static_cast<unsigned long>(
    //         HAL_FDCAN_GetError(&can2_.handle())),
    //     static_cast<unsigned long>(protocol.Activity),
    //     static_cast<unsigned long>(protocol.LastErrorCode),
    //     static_cast<unsigned long>(counters.RxErrorCnt),
    //     static_cast<unsigned long>(counters.TxErrorCnt),
    //     static_cast<unsigned long>(protocol.ErrorPassive),
    //     static_cast<unsigned long>(protocol.Warning),
    //     static_cast<unsigned long>(protocol.BusOff));
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