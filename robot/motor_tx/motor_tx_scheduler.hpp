#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace librmcs::robot::motor_tx{

struct MotorTxJob{
    using ProcessFunction = bool (*)(void*, uint32_t) noexcept;
    void* object{nullptr};
    ProcessFunction process_function{nullptr};

    template<typename Transmitter>
    static MotorTxJob make(Transmitter& transmitter) noexcept{
        return MotorTxJob{
            .object = &transmitter,
            .process_function = [](void* object, uint32_t now_ms) noexcept -> bool {
                return static_cast<Transmitter*>(object)->process(now_ms);
            }
        };
    }

    bool process(uint32_t now_ms) const noexcept{
        if(object == nullptr || process_function == nullptr){
            return false;
        }
        return process_function(object, now_ms);
    }
};

template<std::size_t Capacity>
class MotorTxScheduler{
public:
    template<typename Transmitter>
    bool add(Transmitter& transmitter) noexcept{
        if(size_ >= Capacity){
            return false;
        }
        jobs_[size_] = MotorTxJob::make(transmitter);
        ++size_;
        return true;
    }

    bool process(uint32_t now_ms) noexcept{
        bool all_success = true;
        for(std::size_t index = 0U; index < size_; ++index){
            const bool success = jobs_[index].process(now_ms);
            if(!success){
                all_success = false;
            }
        }

        return all_success;
    }

    void clear() noexcept{
        size_ = 0U;
    }

    std::size_t size() const noexcept{
        return size_;
    }

private:
    std::array<MotorTxJob, Capacity> jobs_{};
    std::size_t size_{0};
};

}