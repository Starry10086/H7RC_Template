#pragma once

#include <array>
#include <cstddef>

namespace container{

template<typename T, std::size_t Capacity>
class FixedPointerQueue final{
    static_assert(Capacity > 0U);
public:
    bool push(T& value) noexcept{
        if(size_ == Capacity){
            return false;
        }

        storage_[write_index_] = &value;
        write_index_ = increment(write_index_);
        ++size_;
        return true;
    }

    T* pop() noexcept{
        if(size_ == 0U){
            return nullptr;
        }

        T* value = storage_[read_index_];
        read_index_ = increment(read_index_);
        --size_;

        return value;
    }

    void clear() noexcept{
        storage_.fill(nullptr);
        read_index_ = 0U;
        write_index_ = 0U;
        size_ = 0U;
    }

    bool isEmpty() const noexcept{
        return size_ == 0U;
    }

    bool isFull() const noexcept{
        return size_ == Capacity;
    }

    std::size_t size() const noexcept{
        return size_;
    }

    static constexpr std::size_t capacity() noexcept{
        return Capacity;
    }

private:
    static constexpr std::size_t increment(std::size_t index) noexcept{
        return index + 1U == Capacity ? 0U : index + 1U;
    }

    std::array<T*, Capacity> storage_{};
    std::size_t read_index_{0U};
    std::size_t write_index_{0U};
    std::size_t size_{0U};
};
}