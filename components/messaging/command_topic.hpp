#pragma once

#include <cstdint>
#include <string_view>

namespace librmcs::messaging{

template<typename T>
struct CommandSample{
    T value{};
    uint32_t timestamp_ms{0U};
    uint32_t sequence{0U};
};

template<typename T>
class CommandTopic{
public:
    constexpr explicit CommandTopic(std::string_view name, uint32_t timeout_ms) noexcept
        : name_(name)
        , timeout_ms_(timeout_ms) {}

    void publish(const T& value, uint32_t now_ms) noexcept{
        sample_.value = value;
        sample_.timestamp_ms = now_ms;
        sample_.sequence++;
        valid_ = true;
    }

    bool readFresh(uint32_t now_ms, CommandSample<T>& output) const noexcept{
        if(!valid_)
            return false;

        const uint32_t age_ms = static_cast<uint32_t>(now_ms - sample_.timestamp_ms);
        if(age_ms > timeout_ms_){
            return false;
        }

        output = sample_;
        return true;
    }

    void invalidate() noexcept{
        valid_ = false;
    }

    std::string_view name() const noexcept{
        return name_;
    }

    uint32_t timeoutMs() const noexcept{
        return timeout_ms_;
    }

private:
    std::string_view name_;
    uint32_t timeout_ms_{0U};
    CommandSample<T> sample_{};
    bool valid_{false};
};

}