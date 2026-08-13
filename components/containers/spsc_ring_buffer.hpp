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
    // ISR 单元素写入。环满时返回 false，不覆盖旧数据。
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

    // ISR 批量写入。
    // 返回实际写入数量。空间不足时只写入前面的字节，后面的新字节由调用者视为丢弃。
    std::size_t pushFromIsr(const T* values, std::size_t count) noexcept{
        if(values == nullptr || count == 0U){
            return 0U;
        }

        const uint32_t write = write_index_.load(std::memory_order_relaxed);
        const uint32_t read = read_index_.load(std::memory_order_acquire);

        const std::size_t used = distance(read, write);
        const std::size_t free = Capacity - used;
        const std::size_t accepted = std::min(count, free);
        
        uint32_t position = write;
        for(std::size_t i = 0U; i < accepted; ++i){
            storage_[position] = values[i];
            position = increment(position);
        }

        write_index_.store(position, std::memory_order_release);
        return accepted;
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

    std::size_t pop(T* destination, std::size_t capacity) noexcept{
        if(destination == nullptr || capacity == 0U){
            return 0U;
        }

        const uint32_t read = read_index_.load(std::memory_order_relaxed);
        const uint32_t write = write_index_.load(std::memory_order_acquire);

        const std::size_t used = distance(read, write);
        const std::size_t count = std::min(capacity, used);

        uint32_t position = read;
        for(std::size_t i = 0U; i < count; ++i){
            destination[i] = storage_[position];
            position = increment(position);
        }

        read_index_.store(position, std::memory_order_release);
        return count;
    }

    std::size_t size() const noexcept {
        const uint32_t read = read_index_.load(std::memory_order_acquire);
        const uint32_t write = write_index_.load(std::memory_order_acquire);
        return distance(read, write);
    }

    void clear() noexcept{
        const uint32_t write = write_index_.load(std::memory_order_acquire);
        read_index_.store(write, std::memory_order_release);
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

    static constexpr std::size_t distance(uint32_t read, uint32_t write) noexcept {
        if(write >= read){
            return static_cast<std::size_t>(write - read);
        }
        return static_cast<std::size_t>(Capacity + 1U - (read - write));
    }

    // 多一个槽位用于区分满和空的状态
    std::array<T, Capacity + 1U> storage_{};

    alignas(4) std::atomic<uint32_t> write_index_{0};
    alignas(4) std::atomic<uint32_t> read_index_{0};
};

}