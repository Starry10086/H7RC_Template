#pragma once

#include <cstdint>

namespace platform{

enum class TransferState : uint8_t {
    Idle,       // 传输空闲
    Queued,     // 传输已排队，等待发送
    Active,     // 传输正在进行中
    Completed,  // 传输已完成
    Failed,     // 传输失败
    TimedOut,   // 传输超时
};

enum class TransferError : uint8_t {
    None,               // 无错误
    InvalidArgument,    // 无效参数
    InvalidDmaBuffer,   // DMA缓冲区无效
    HalBusy,            // HAL忙
    HalError,           // HAL错误
    Timeout,            // 超时
    RxOverrun           // 接收缓冲区溢出
};

constexpr bool isTransferPending(TransferState state) noexcept{
    return state == TransferState::Queued || state == TransferState::Active;
}

constexpr bool isTransferTerminal(TransferState state) noexcept{
    return state == TransferState::Completed || state == TransferState::Failed ||
           state == TransferState::TimedOut;
}

constexpr bool isTransferSuccessful(TransferState state) noexcept {
    return state == TransferState::Completed;
}

constexpr bool isTransferUnsuccessful(TransferState state) noexcept {
    return state == TransferState::Failed ||
           state == TransferState::TimedOut;
}

struct TransferStats{
    uint32_t completed{0U};
    uint32_t failed{0U};
    uint32_t timed_out{0U};
    uint32_t queue_rejected{0U};
};

/*
* 事务状态变化如下：
Idle
  -> Queued
  -> Active
  -> Completed / Failed / TimedOut
  -> reset()
  -> Idle

禁止在 Queued 或 Active 时修改 DMA 缓冲区
*/

}