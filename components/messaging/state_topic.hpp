#pragma once

#include <cstdint>
#include <string_view>

namespace librmcs::messaging {

template<typename T>
struct StateSample{
    T value{};
    uint32_t timestamp_ms{0};   // 记录采样时间戳，单位毫秒
    uint32_t sequence{0};       // 每发一次新数据，版本号就 +1。用来区分数据有没有更新过
};

template<typename T>
class StateTopic{
public:
    explicit constexpr StateTopic(std::string_view name) noexcept
    : name_(name) {}

    void publish(const T& value, uint32_t now_ms) noexcept {
        sample_.value = value;
        sample_.timestamp_ms = now_ms;
        ++sample_.sequence;
        valid_ = true;
    }

    [[nodiscard]] bool read(StateSample<T>& output) const noexcept{
        if(!valid_){
            return false;
        }

        output = sample_;
        return true;
    }

    [[nodiscard]] bool readIfNew(uint32_t& last_sequence, StateSample<T>& output) const noexcept{
        if(!valid_ || last_sequence == sample_.sequence){
            return false;
        }

        output = sample_;
        last_sequence = sample_.sequence;
        return true;
    }

    [[nodiscard]] std::string_view name() const noexcept {
        return name_;
    }
private:
    std::string_view name_;
    StateSample<T> sample_{};
    bool valid_{false};
};

[[nodiscard]] constexpr bool isFresh(uint32_t now_ms,
                                     uint32_t sample_time_ms,
                                     uint32_t timeout_ms) noexcept {
    return static_cast<uint32_t>(now_ms - sample_time_ms) <= timeout_ms;
}

}