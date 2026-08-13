#pragma once

#include "components/messaging/state_topic.hpp"
#include "devices/laser/laser_distance.hpp"
#include "platform/stm32/uart_port.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace device {

struct Stp23Config {
    std::size_t rx_budget_bytes{512U};
    uint32_t offline_timeout_ms{100U};

    // 0 表示只过滤 distance == 0 的无效点。
    uint8_t minimum_intensity{0U};
};

struct Stp23Stats {
    uint32_t rx_bytes{0U};
    uint32_t valid_frames{0U};
    uint32_t frames_without_measurement{0U};
    uint32_t crc_errors{0U};
    uint32_t parser_resets{0U};
    uint32_t uart_discontinuities{0U};
    uint32_t uart_dropped_bytes{0U};
};

class Stp23 final {
public:
    Stp23(
        platform::UartPort& uart,
        messaging::StateTopic<LaserDistance>& distance_topic,
        const Stp23Config& config = {}) noexcept
        : uart_(uart)
        , distance_topic_(distance_topic)
        , config_(config) {}

    Stp23(const Stp23&) = delete;
    Stp23& operator=(const Stp23&) = delete;

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

    Stp23Stats stats() const noexcept {
        return stats_;
    }

private:
    static constexpr uint8_t header_ = 0x54U;
    static constexpr uint8_t version_length_ = 0x2CU;
    static constexpr std::size_t point_count_ = 12U;
    static constexpr std::size_t frame_size_ = 47U;
    static constexpr std::size_t points_offset_ = 6U;
    static constexpr std::size_t point_size_ = 3U;
    static constexpr std::size_t end_angle_offset_ = 42U;
    static constexpr std::size_t timestamp_offset_ = 44U;
    static constexpr std::size_t read_chunk_size_ = 128U;

    static_assert(
        points_offset_ + point_count_ * point_size_ == end_angle_offset_);
    static_assert(timestamp_offset_ + 2U + 1U == frame_size_);

    inline static constexpr std::array<uint8_t, 256U> crc_table_{
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
        0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
        0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
        0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
        0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
        0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
        0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
        0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
        0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
        0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
        0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
        0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
        0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
        0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
        0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
        0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
        0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
        0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
        0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
        0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
        0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
        0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
        0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
        0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
        0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
    };

    static uint16_t readUint16Le(const uint8_t* data) noexcept {
        return static_cast<uint16_t>(data[0]) |
               static_cast<uint16_t>(
                   static_cast<uint16_t>(data[1]) << 8U);
    }

    static uint8_t calculateCrc8(
        const uint8_t* data,
        std::size_t size) noexcept {
        uint8_t crc = 0U;

        for (std::size_t i = 0U; i < size; ++i) {
            crc = crc_table_[crc ^ data[i]];
        }

        return crc;
    }

    void consumeByte(uint8_t byte, uint32_t now_ms) noexcept {
        if (frame_length_ == 0U) {
            if (byte == header_) {
                frame_[0] = byte;
                frame_length_ = 1U;
            }
            return;
        }

        if (frame_length_ == 1U) {
            if (byte == version_length_) {
                frame_[1] = byte;
                frame_length_ = 2U;
            } else if (byte != header_) {
                frame_length_ = 0U;
            }
            return;
        }

        frame_[frame_length_] = byte;
        ++frame_length_;

        if (frame_length_ != frame_size_) {
            return;
        }

        if (calculateCrc8(frame_.data(), frame_size_ - 1U) !=
            frame_[frame_size_ - 1U]) {
            ++stats_.crc_errors;
            recoverAfterInvalidFrame();
            return;
        }

        publishFrame(now_ms);
        frame_length_ = 0U;
    }

    void publishFrame(uint32_t now_ms) noexcept {
        uint32_t distance_sum_mm = 0U;
        uint32_t valid_point_count = 0U;

        for(std::size_t i = 0U; i < point_count_; ++i){
            const std::size_t offset = points_offset_ + i * point_size_;
            const uint16_t distance_mm = readUint16Le(&frame_[offset]);
            const uint8_t intensity = frame_[offset + 2U];

            if(distance_mm == 0U || intensity < config_.minimum_intensity){
                continue;
            }

            distance_sum_mm += distance_mm;
            ++valid_point_count;
        }

        if(valid_point_count != 0U){
            const uint32_t average_mm = (distance_sum_mm + valid_point_count / 2U) / valid_point_count;
            distance_topic_.publish(LaserDistance{.distance_mm = average_mm}, now_ms);
            ++stats_.valid_frames;
        }
        else{
            ++stats_.frames_without_measurement;
        }

        last_frame_ms_ = now_ms;
        has_received_frame_ = true;
    }

    // CRC 失败后在旧窗口里寻找下一个 0x54 0x2C，快速恢复帧同步。
    void recoverAfterInvalidFrame() noexcept {
        for (std::size_t begin = 1U;
             begin + 1U < frame_size_;
             ++begin) {
            if (frame_[begin] != header_ ||
                frame_[begin + 1U] != version_length_) {
                continue;
            }

            const std::size_t remaining = frame_size_ - begin;
            for (std::size_t i = 0U; i < remaining; ++i) {
                frame_[i] = frame_[begin + i];
            }
            frame_length_ = remaining;
            return;
        }

        if (frame_[frame_size_ - 1U] == header_) {
            frame_[0] = header_;
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
    Stp23Config config_{};

    std::array<uint8_t, frame_size_> frame_{};
    std::size_t frame_length_{0U};

    platform::UartPortStats last_uart_stats_{};
    Stp23Stats stats_{};

    uint32_t last_frame_ms_{0U};
    bool has_received_frame_{false};
    bool initialized_{false};
};

// 兼容旧代码；新代码建议使用符合工程命名风格的 Stp23。
using STP_23 = Stp23;

} // namespace device
