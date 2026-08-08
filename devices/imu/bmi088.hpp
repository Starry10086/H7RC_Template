#pragma once

#include "platform/stm32/async_transfer.hpp"
#include "platform/stm32/spi_bus.hpp"
#include "platform/stm32/spi_device.hpp"
#include "components/messaging/state_topic.hpp"
#include "devices/imu/imu_state.hpp"

#include <array>
#include <cstdint>

namespace device{
    struct Bmi088DmaStorage{
        alignas(32) std::array<uint8_t, 16> tx{};
        alignas(32) std::array<uint8_t, 16> rx{};
    };

    struct Bmi088Config{
        platform::SpiChipSelect accel_chip_select{};
        platform::SpiChipSelect gyro_chip_select{};
        uint32_t boot_wait_ms{30U};
        uint32_t power_change_wait_ms{5U};
        uint32_t transfer_timeout_ms{5U};
        uint32_t sample_period_ms{5U};  // 5m 对应200HZ
    };

    enum class Bmi088State: uint8_t{
        Uninit,         // 未初始化
        BootStart,      // 开始启动等待
        BootWait,       // 等待启动完成

        AccelSwitchReq, // 提交加速度计接口切换请求，BMI088 的加速度计上电默认处于 I2C 模式
        AccelSwitchWait,// 等待加速度计接口切换完成
        AccelIdReq,     // 提交加速度计 ID 读取请求
        AccelIdWait,    // 等待加速度计 ID 读取完成
        GyroIdReq,      // 提交陀螺仪 ID 读取请求
        GyroIdWait,     // 等待陀螺仪 ID 读取完成

        ConfigReq,      // 提交配置请求
        ConfigWait,     // 等待配置完成
        ConfigDelay,    // 配置完成后等待一段时间，确保 BMI088 内部稳定

        Ready,          // 就绪
        AccelDataWait,  // 等待加速度计数据读取完成
        GyroDataWait,   // 等待陀螺仪数据读取完成

        Fault           // 故障
    };

    enum class Bmi088Fault : uint8_t{
        None,
        InvalidConfig,  // 配置参数无效
        TransferFailed, // 传输失败
        BadAccelId,     // 加速度计 ID 不正确
        BadGyroId       // 陀螺仪 ID 不正确
    };

class Bmi088 final{
public:
    Bmi088(platform::SpiBus& spi_bus,
           const Bmi088Config& config,
           Bmi088DmaStorage& dma_storage,
           messaging::StateTopic<ImuState>& state_topic) noexcept;

    bool init()noexcept;
    void process(uint32_t now_ms) noexcept;

    bool ready() const noexcept { return inited_ && state_ != Bmi088State::Fault; }
    bool hasFault() const noexcept { return state_ == Bmi088State::Fault; }

    Bmi088State state() const noexcept { return state_; }
    Bmi088Fault fault() const noexcept { return fault_; }

    uint8_t accelChipId() const noexcept { return accel_chip_id_; }
    uint8_t gyroChipId() const noexcept { return gyro_chip_id_; }

    platform::TransferError lastTransferError() const noexcept { return last_transfer_error_; }
private:
    bool submitAccelRead(uint8_t address, uint8_t data_size) noexcept;
    bool submitGyroRead(uint8_t address, uint8_t data_size) noexcept;

    bool submitAccelWrite(uint8_t address, uint8_t value) noexcept;
    bool submitGyroWrite(uint8_t address, uint8_t value) noexcept;

    bool consumeCompletedTransfer() noexcept;
    void parseAccel() noexcept;
    void parseGyro() noexcept;
    void publishState(uint32_t now_ms) noexcept;

    static int16_t readInt16Le(const uint8_t* data) noexcept;

    void fail(Bmi088Fault fault) noexcept;
    static bool deadlineReached(uint32_t now_ms, uint32_t deadline_ms)noexcept;

    Bmi088Config config_{};
    platform::SpiDevice accel_device_;
    platform::SpiDevice gyro_device_;
    Bmi088DmaStorage& dma_storage_;

    messaging::StateTopic<ImuState>& state_topic_;
    ImuState imu_state_{};

    platform::SpiTransfer transfer_{};

    Bmi088State state_{Bmi088State::Uninit};
    Bmi088Fault fault_{Bmi088Fault::None};
    platform::TransferError last_transfer_error_{platform::TransferError::None};

    uint32_t next_sample_ms_{0U};
    uint32_t sample_count_{0U};
    uint8_t config_step_{0u};
    bool inited_{false};

    uint32_t deadline_ms_{0U};
    uint8_t accel_chip_id_{0U};
    uint8_t gyro_chip_id_{0U};
};
}