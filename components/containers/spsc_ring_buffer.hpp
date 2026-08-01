#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace container{

template<typename T, std::size_t Capacity>
class SpscRingBuffer final {
    static_assert(Capacity > 0);
    static_assert(std::atomic<uint32_t>::is_always_lock_free);

public:
    bool pushFromIsr(const T& value) noexcept {
        const uint32_t write = write_index_.load(std::memory_order_relaxed);
        const uint32_t next = increment(write);

        if(next == read_index_.load(std::memory_order_acquire)){
            return false;
        }

        storage_[write] = value;
        write_index_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& value) noexcept{
        const uint32_t read = read_index_.load(std::memory_order_relaxed);

        if(read == write_index_.load(std::memory_order_acquire)){
            return false;
        }

        value = storage_[read];
        read_index_.store(increment(read), std::memory_order_release);
        return true;
    }

     bool empty() const noexcept {
        return read_index_.load(std::memory_order_acquire) == write_index_.load(std::memory_order_acquire);
    }

    static constexpr std::size_t capacity() noexcept {
        return Capacity;
    }

private:
    static constexpr uint32_t increment(uint32_t index) noexcept {
        return index == Capacity ? 0U : index + 1U;
    }

    // 多一个槽位用于区分满和空的状态
    std::array<T, Capacity + 1U> storage_{};

    alignas(4) std::atomic<uint32_t> write_index_{0};
    alignas(4) std::atomic<uint32_t> read_index_{0};
};

}