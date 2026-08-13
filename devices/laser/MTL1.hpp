#pragma once

#include "components/messaging/state_topic.hpp"
#include "devices/laser/laser_distance.hpp"
#include "platform/stm32/uart_port.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace device {

struct MTL1Config {
    // 每轮 process() 最多消费的 UART 字节数，避免设备数据淹没主循环。
    std::size_t rx_budget_bytes{256U};
    uint32_t offline_timeout_ms{100U};
};

struct MTL1Stats {
    uint32_t rx_bytes{0U};
    uint32_t distance_frames{0U};
    uint32_t fault_frames{0U};
    uint32_t checksum_errors{0U};
    uint32_t unsupported_frames{0U};
    uint32_t parser_resets{0U};
    uint32_t uart_discontinuities{0U};
    uint32_t uart_dropped_bytes{0U};
};

class MTL1 final {
public:
    MTL1(
        platform::UartPort& uart,
        messaging::StateTopic<LaserDistance>& distance_topic,
        const MTL1Config& config = {}) noexcept
        : uart_(uart)
        , distance_topic_(distance_topic)
        , config_(config) {}

    MTL1(const MTL1&) = delete;
    MTL1& operator=(const MTL1&) = delete;

    bool init() noexcept {
        if (initialized_) {
            return true;
        }

        if (config_.rx_budget_bytes == 0U ||
            config_.offline_timeout_ms == 0U) {
            return false;
        }

        resetParser(false);
        stats_ = {};
        last_uart_stats_ = uart_.stats();
        last_frame_ms_ = 0U;
        has_received_frame_ = false;
        initialized_ = true;
        return true;
    }

    // 由主循环调用。UartPort::process(now_ms) 应当先于本函数执行。
    void process(uint32_t now_ms) noexcept {
        if (!initialized_) {
            return;
        }

        checkUartContinuity();

        std::array<uint8_t, read_chunk_size_> bytes{};
        std::size_t remaining = config_.rx_budget_bytes;

        while (remaining != 0U) {
            const std::size_t request =
                remaining < bytes.size() ? remaining : bytes.size();
            const std::size_t received = uart_.read(bytes.data(), request);

            if (received == 0U) {
                break;
            }

            stats_.rx_bytes += static_cast<uint32_t>(received);
            remaining -= received;

            for (std::size_t i = 0U; i < received; ++i) {
                consumeByte(bytes[i], now_ms);
            }
        }
    }

    bool initialized() const noexcept {
        return initialized_;
    }

    bool online(uint32_t now_ms) const noexcept {
        return initialized_ &&
               has_received_frame_ &&
               static_cast<uint32_t>(now_ms - last_frame_ms_) <=
                   config_.offline_timeout_ms;
    }

    uint32_t lastFrameTimeMs() const noexcept {
        return last_frame_ms_;
    }

    MTL1Stats stats() const noexcept {
        return stats_;
    }

private:
    static constexpr uint8_t header_0_ = 0xB4U;
    static constexpr uint8_t header_1_ = 0x69U;
    static constexpr std::size_t frame_size_ = 8U;
    static constexpr std::size_t read_chunk_size_ = 64U;

    static constexpr bool isDistanceFunction(uint8_t function) noexcept {
        return function == 0x02U ||
               function == 0x03U ||
               function == 0x04U;
    }

    static constexpr bool isFaultFunction(uint8_t function) noexcept {
        return function == 0x82U ||
               function == 0x83U ||
               function == 0x84U;
    }

    static uint8_t calculateBcc(
        const uint8_t* data,
        std::size_t size) noexcept {
        uint8_t result = 0U;

        for (std::size_t i = 0U; i < size; ++i) {
            result ^= data[i];
        }

        return result;
    }

    static uint32_t readUint32Be(const uint8_t* data) noexcept {
        return (static_cast<uint32_t>(data[0]) << 24U) |
               (static_cast<uint32_t>(data[1]) << 16U) |
               (static_cast<uint32_t>(data[2]) << 8U) |
               static_cast<uint32_t>(data[3]);
    }

    void consumeByte(uint8_t byte, uint32_t now_ms) noexcept {
        if (frame_length_ == 0U) {
            if (byte == header_0_) {
                frame_[0] = byte;
                frame_length_ = 1U;
            }
            return;
        }

        if (frame_length_ == 1U) {
            if (byte == header_1_) {
                frame_[1] = byte;
                frame_length_ = 2U;
            } else if (byte != header_0_) {
                frame_length_ = 0U;
            }
            return;
        }

        frame_[frame_length_] = byte;
        ++frame_length_;

        if (frame_length_ != frame_size_) {
            return;
        }

        if (calculateBcc(frame_.data(), frame_size_ - 1U) !=
            frame_[frame_size_ - 1U]) {
            ++stats_.checksum_errors;
            recoverAfterInvalidFrame();
            return;
        }

        handleFrame(now_ms);
        frame_length_ = 0U;
    }

    void handleFrame(uint32_t now_ms) noexcept {
        const uint8_t function = frame_[2];
        const uint32_t value = readUint32Be(&frame_[3]);

        if (isDistanceFunction(function)) {
            distance_topic_.publish(LaserDistance{.distance_mm = value}, now_ms);
            ++stats_.distance_frames;
        } else if (isFaultFunction(function)) {
            ++stats_.fault_frames;
        } else {
            ++stats_.unsupported_frames;
            return;
        }

        last_frame_ms_ = now_ms;
        has_received_frame_ = true;
    }

    // BCC 失败时保留当前窗口中可能出现的新帧头，避免再丢一整帧。
    void recoverAfterInvalidFrame() noexcept {
        for (std::size_t begin = 1U;
             begin + 1U < frame_size_;
             ++begin) {
            if (frame_[begin] != header_0_ ||
                frame_[begin + 1U] != header_1_) {
                continue;
            }

            const std::size_t remaining = frame_size_ - begin;
            for (std::size_t i = 0U; i < remaining; ++i) {
                frame_[i] = frame_[begin + i];
            }
            frame_length_ = remaining;
            return;
        }

        if (frame_[frame_size_ - 1U] == header_0_) {
            frame_[0] = header_0_;
            frame_length_ = 1U;
        } else {
            frame_length_ = 0U;
        }
    }

    void checkUartContinuity() noexcept {
        const platform::UartPortStats current = uart_.stats();

        const uint32_t dropped_delta =
            current.rx_dropped_bytes - last_uart_stats_.rx_dropped_bytes;

        const bool receive_error =
            current.framing_errors != last_uart_stats_.framing_errors ||
            current.noise_errors != last_uart_stats_.noise_errors ||
            current.overrun_errors != last_uart_stats_.overrun_errors ||
            current.dma_errors != last_uart_stats_.dma_errors;

        if (dropped_delta != 0U || receive_error) {
            stats_.uart_dropped_bytes += dropped_delta;
            ++stats_.uart_discontinuities;
            resetParser(true);
        }

        last_uart_stats_ = current;
    }

    void resetParser(bool count_reset) noexcept {
        frame_length_ = 0U;
        if (count_reset) {
            ++stats_.parser_resets;
        }
    }

    platform::UartPort& uart_;
    messaging::StateTopic<LaserDistance>& distance_topic_;
    MTL1Config config_{};

    std::array<uint8_t, frame_size_> frame_{};
    std::size_t frame_length_{0U};

    platform::UartPortStats last_uart_stats_{};
    MTL1Stats stats_{};

    uint32_t last_frame_ms_{0U};
    bool has_received_frame_{false};
    bool initialized_{false};
};

} // namespace device
