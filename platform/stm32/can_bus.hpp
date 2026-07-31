#pragma once

#include "components/containers/spsc_ring_buffer.hpp"
#include "platform/can/can_types.hpp"

#include "fdcan.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace librmcs::platform{

struct CanBusState{
    uint32_t received_frames{0};    // 记录接收到的帧数
    uint32_t dropped_frames{0};     // 记录丢弃的帧数
    uint32_t invalid_frames{0};     // 记录无效帧数
    uint32_t hal_error{0};          // 记录HAL错误数
    uint32_t queued_transmit_frames{0}; // 记录HAL 已经接受并放入硬件FIFO的帧数
    uint32_t rejected_transmit_frames{0}; // 记录因未启动，长度，ID或枚举值非法而被软件拒绝的帧数
};

class CanBus final{
public:
    static constexpr std::size_t rx_queue_capacity = 64;

    explicit CanBus(FDCAN_HandleTypeDef& handle)
    : handle_(handle) {}

    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    // 必须在MX_FDCANx_Init()之后调用。
     bool start() noexcept;
    // 只能从主循环调用，不要在中断中发送。
    bool send(const can::Frame& frame) noexcept;
    // 只能从对应FDCAN的FIFO0中断回调调用
    void onRxFifo0Interrupt() noexcept;
    // 只能从主循环调用。
    bool popReceived(can::Frame& frame) noexcept;
    CanBusState stats() const noexcept;
    FDCAN_HandleTypeDef& handle() noexcept { return handle_; }

private:
    FDCAN_HandleTypeDef& handle_;
    container::SpscRingBuffer<can::Frame, rx_queue_capacity> rx_queue_;
    std::atomic<uint32_t> received_frames_{0};
    std::atomic<uint32_t> dropped_frames_{0};
    std::atomic<uint32_t> invalid_frames_{0};
    std::atomic<uint32_t> hal_errors_{0};
    std::atomic<uint32_t> queued_transmit_frames_{0U};
    std::atomic<uint32_t> rejected_transmit_frames_{0U};
    bool started_{false};
};
}