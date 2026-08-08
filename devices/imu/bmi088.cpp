#include "devices/imu/bmi088.hpp"
#include "devices/imu/bmi088_registers.hpp"
#include "platform/stm32/spi_bus.hpp"
#include <cstdint>
#include <array>

namespace device{

namespace{
    enum class ConfigTarget : uint8_t{
        Accel,
        Gyro
    };

    struct ConfigItem{
        ConfigTarget target;
        uint8_t address;
        uint8_t value;
        uint32_t wait_ms;
    };

    constexpr std::array<ConfigItem, 6U> config_items{
        ConfigItem{ConfigTarget::Accel, 
                       bmi088_register::accel_power_conf_reg, 
                       bmi088_register::accel_power_active, 
                       5U},
        ConfigItem{ConfigTarget::Accel,
                        bmi088_register::accel_power_ctrl_reg,
                        bmi088_register::accel_measurement_enable,
                        5U},
        ConfigItem{ConfigTarget::Accel,
                        bmi088_register::accel_conf_reg,
                        bmi088_register::accel_normal_200_hz,
                        0U}, 
        ConfigItem{ConfigTarget::Accel,
                        bmi088_register::accel_range_reg,
                        bmi088_register::accel_range_6g,
                        0U},
        ConfigItem{ConfigTarget::Gyro,
                        bmi088_register::gyro_range_reg,
                        bmi088_register::gyro_range_2000_dps,
                        0U},
        ConfigItem{ConfigTarget::Gyro,
                        bmi088_register::gyro_bandwidth_reg,
                        bmi088_register::gyro_200_hz_23_hz,
                        0U}
    };

    constexpr float standard_gravity_m_s2 = 9.80665F;

    // BMI088 加速度计 ±6g：5460 LSB/g
    constexpr float accel_m_s2_per_lsb = standard_gravity_m_s2 / 5460.0F;

    // BMI088 陀螺仪 ±2000 deg/s：16.384 LSB/(deg/s)
    constexpr float gyro_rad_s_per_lsb = 0.017453292519943F / 16.384F;
}
Bmi088::Bmi088(platform::SpiBus& spi_bus,
               const Bmi088Config& config,
               Bmi088DmaStorage& dma_storage,
               messaging::StateTopic<ImuState>& state_topic) noexcept
    : config_(config)
    , accel_device_(spi_bus, config.accel_chip_select)
    , gyro_device_(spi_bus, config.gyro_chip_select)
    , dma_storage_(dma_storage)
    , state_topic_(state_topic){}

    bool Bmi088::init() noexcept{
        if(inited_)
            return true;

        if (!config_.accel_chip_select.valid() ||
            !config_.gyro_chip_select.valid() ||
            config_.transfer_timeout_ms == 0U ||
            config_.sample_period_ms == 0U) {
            fail(Bmi088Fault::InvalidConfig);
            return false;
        }

        accel_device_.chipSelect().setActive(false);
        gyro_device_.chipSelect().setActive(false);

        transfer_.reset();

        fault_ = Bmi088Fault::None;
        last_transfer_error_ =
            platform::TransferError::None;

        accel_chip_id_ = 0U;
        gyro_chip_id_ = 0U;
        config_step_ = 0U;
        sample_count_ = 0U;
        next_sample_ms_ = 0U;
        deadline_ms_ = 0U;
        inited_ = false;
        state_ = Bmi088State::BootStart;

        return true;
    }


    void Bmi088::process(uint32_t now_ms) noexcept {
        switch (state_) {
            case Bmi088State::Uninit:
            case Bmi088State::Fault:
                return;

            case Bmi088State::BootStart:
                accel_device_.chipSelect().setActive(false);
                gyro_device_.chipSelect().setActive(false);

                deadline_ms_ =
                    now_ms + config_.boot_wait_ms;

                state_ = Bmi088State::BootWait;
                return;

            case Bmi088State::BootWait:
                if (!deadlineReached(now_ms, deadline_ms_)) {
                    return;
                }

                state_ = Bmi088State::AccelSwitchReq;
                return;

            case Bmi088State::AccelSwitchReq:
                /*
                * 加速度计上电默认是 I2C 模式。
                * 第一次 SPI 访问用于切换到 SPI 模式。
                */
                if (submitAccelRead(
                        bmi088_register::accel_chip_id_reg,
                        1U)) {
                    state_ = Bmi088State::AccelSwitchWait;
                    return;
                }

                if (transfer_.state == platform::TransferState::Failed) {
                    (void)consumeCompletedTransfer();
                    fail(Bmi088Fault::TransferFailed);
                }
                return;

            case Bmi088State::AccelSwitchWait:
                if (!platform::isTransferTerminal(
                        transfer_.state)) {
                    return;
                }

                if (!consumeCompletedTransfer()) {
                    fail(Bmi088Fault::TransferFailed);
                    return;
                }

                state_ = Bmi088State::AccelIdReq;
                return;

            case Bmi088State::AccelIdReq:
                if (submitAccelRead(
                        bmi088_register::accel_chip_id_reg,
                        1U)) {
                    state_ = Bmi088State::AccelIdWait;
                    return;
                }

                if (transfer_.state == platform::TransferState::Failed) {
                    (void)consumeCompletedTransfer();
                    fail(Bmi088Fault::TransferFailed);
                }
                return;

            case Bmi088State::AccelIdWait:
                if (!platform::isTransferTerminal(
                        transfer_.state)) {
                    return;
                }

                if (!consumeCompletedTransfer()) {
                    fail(Bmi088Fault::TransferFailed);
                    return;
                }

                accel_chip_id_ = dma_storage_.rx[2];

                if (accel_chip_id_ !=
                    bmi088_register::expected_accel_chip_id) {
                    fail(Bmi088Fault::BadAccelId);
                    return;
                }

                state_ = Bmi088State::GyroIdReq;
                return;

            case Bmi088State::GyroIdReq:
                if (submitGyroRead(
                        bmi088_register::gyro_chip_id_reg,
                        1U)) {
                    state_ = Bmi088State::GyroIdWait;
                    return;
                }

                if (transfer_.state == platform::TransferState::Failed) {
                    (void)consumeCompletedTransfer();
                    fail(Bmi088Fault::TransferFailed);
                }
                return;

            case Bmi088State::GyroIdWait:
                if (!platform::isTransferTerminal(
                        transfer_.state)) {
                    return;
                }

                if (!consumeCompletedTransfer()) {
                    fail(Bmi088Fault::TransferFailed);
                    return;
                }

                gyro_chip_id_ = dma_storage_.rx[1];

                if (gyro_chip_id_ !=
                    bmi088_register::expected_gyro_chip_id) {
                    fail(Bmi088Fault::BadGyroId);
                    return;
                }

                config_step_ = 0U;
                state_ = Bmi088State::ConfigReq;
                return;

            case Bmi088State::ConfigReq: {
                if (config_step_ >= config_items.size()) {
                    inited_ = true;
                    next_sample_ms_ = now_ms;
                    state_ = Bmi088State::Ready;
                    return;
                }

                const ConfigItem& item =
                    config_items[config_step_];

                bool submitted = false;

                if (item.target == ConfigTarget::Accel) {
                    submitted = submitAccelWrite(
                        item.address,
                        item.value);
                } else {
                    submitted = submitGyroWrite(
                        item.address,
                        item.value);
                }

                if (submitted) {
                    state_ = Bmi088State::ConfigWait;
                    return;
                }

                if (transfer_.state == platform::TransferState::Failed) {
                    (void)consumeCompletedTransfer();
                    fail(Bmi088Fault::TransferFailed);
                }

                return;
            }

            case Bmi088State::ConfigWait:
                if (!platform::isTransferTerminal(
                        transfer_.state)) {
                    return;
                }

                if (!consumeCompletedTransfer()) {
                    fail(Bmi088Fault::TransferFailed);
                    return;
                }

                deadline_ms_ =
                    now_ms +
                    config_items[config_step_].wait_ms;

                ++config_step_;

                if (config_items[config_step_ - 1U].wait_ms != 0U) {
                    state_ = Bmi088State::ConfigDelay;
                } else {
                    state_ = Bmi088State::ConfigReq;
                }

                return;

            case Bmi088State::ConfigDelay:
                if (deadlineReached(now_ms, deadline_ms_)) {
                    state_ = Bmi088State::ConfigReq;
                }
                return;

            case Bmi088State::Ready:
                if (!deadlineReached(now_ms, next_sample_ms_)) {
                    return;
                }

                if (submitAccelRead(
                        bmi088_register::accel_xyz_reg,
                        6U)) {
                    state_ = Bmi088State::AccelDataWait;
                    return;
                }

                if (transfer_.state == platform::TransferState::Failed) {
                    (void)consumeCompletedTransfer();
                    fail(Bmi088Fault::TransferFailed);
                }
                return;

            case Bmi088State::AccelDataWait:
                if (!platform::isTransferTerminal(
                        transfer_.state)) {
                    return;
                }

                if (!consumeCompletedTransfer()) {
                    fail(Bmi088Fault::TransferFailed);
                    return;
                }

                parseAccel();

                /*
                * 加速度计完成后，立即读取陀螺仪。
                * 如果队列暂时满，则放弃本次采样，
                * 下一周期重新开始。
                */
                if (submitGyroRead(
                        bmi088_register::gyro_xyz_reg,
                        6U)) {
                    state_ = Bmi088State::GyroDataWait;
                } else if (transfer_.state ==
                        platform::TransferState::Failed) {
                    (void)consumeCompletedTransfer();
                    fail(Bmi088Fault::TransferFailed);
                } else {
                    next_sample_ms_ = now_ms;
                    state_ = Bmi088State::Ready;
                }

                return;

            case Bmi088State::GyroDataWait:
                if (!platform::isTransferTerminal(
                        transfer_.state)) {
                    return;
                }

                if (!consumeCompletedTransfer()) {
                    fail(Bmi088Fault::TransferFailed);
                    return;
                }

                parseGyro();
                publishState(now_ms);

                ++sample_count_;
                next_sample_ms_ =
                    now_ms + config_.sample_period_ms;

                state_ = Bmi088State::Ready;
                return;
        }
    }

    bool Bmi088::consumeCompletedTransfer() noexcept{
        last_transfer_error_ = transfer_.error;

        const bool success = transfer_.state == platform::TransferState::Completed;

        transfer_.reset();

        return success;
    }

    int16_t Bmi088::readInt16Le(const uint8_t* data) noexcept{
        const uint16_t raw = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U);

        if(raw >= 0x8000U){
            return static_cast<int16_t>(static_cast<int32_t>(raw) - 0x10000);
        }

        return static_cast<int16_t>(raw);
    }

    void Bmi088::parseAccel() noexcept {
        const int16_t raw_x = readInt16Le(&dma_storage_.rx[2]);
        const int16_t raw_y = readInt16Le(&dma_storage_.rx[4]);
        const int16_t raw_z = readInt16Le(&dma_storage_.rx[6]);

        imu_state_.accel_m_s2[0] = static_cast<float>(raw_x) * (9.80665F / 5460.0F);
        imu_state_.accel_m_s2[1] = static_cast<float>(raw_y) * (9.80665F / 5460.0F);
        imu_state_.accel_m_s2[2] = static_cast<float>(raw_z) * (9.80665F / 5460.0F);
    }

    void Bmi088::parseGyro() noexcept {
        const int16_t raw_x = readInt16Le(&dma_storage_.rx[1]);
        const int16_t raw_y = readInt16Le(&dma_storage_.rx[3]);
        const int16_t raw_z = readInt16Le(&dma_storage_.rx[5]);

        constexpr float deg_to_rad = 0.017453292519943F;

        const float scale = deg_to_rad / 16.384F;

        imu_state_.gyro_rad_s[0] = static_cast<float>(raw_x) * scale;
        imu_state_.gyro_rad_s[1] = static_cast<float>(raw_y) * scale;
        imu_state_.gyro_rad_s[2] = static_cast<float>(raw_z) * scale;
    }

    void Bmi088::publishState(uint32_t now_ms) noexcept{
        imu_state_.fault_code = 0U;
        state_topic_.publish(imu_state_, now_ms);
    }

    void Bmi088::fail(Bmi088Fault fault) noexcept{
        fault_ = fault;
        inited_ = false;
        state_ = Bmi088State::Fault;

        accel_device_.chipSelect().setActive(false);
        gyro_device_.chipSelect().setActive(false);
    }

    bool Bmi088::deadlineReached(uint32_t now_ms, uint32_t deadline_ms) noexcept{
        return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
    }

    bool Bmi088::submitAccelRead(uint8_t address, uint8_t data_size) noexcept{
        const uint16_t transfer_size = static_cast<uint16_t>(data_size + 2U);

        if(data_size == 0U || transfer_size > dma_storage_.tx.size()){
            return false;
        }

        dma_storage_.tx.fill(bmi088_register::dummy_byte);
        dma_storage_.rx.fill(0U);

        dma_storage_.tx[0] = bmi088_register::makeReadAddress(address);
        return accel_device_.submit(transfer_, dma_storage_.tx.data(), dma_storage_.rx.data(), transfer_size, config_.transfer_timeout_ms);
    }

    bool Bmi088::submitGyroRead(uint8_t address, uint8_t data_size) noexcept{
        const uint16_t transfer_size = static_cast<uint16_t>(data_size + 1U);

        if(data_size == 0U || transfer_size > dma_storage_.tx.size()){
            return false;
        }

        dma_storage_.tx.fill(bmi088_register::dummy_byte);
        dma_storage_.rx.fill(0U);

        dma_storage_.tx[0] = bmi088_register::makeReadAddress(address);

        return gyro_device_.submit(transfer_, dma_storage_.tx.data(), dma_storage_.rx.data(), transfer_size, config_.transfer_timeout_ms);
    }

    bool Bmi088::submitAccelWrite(uint8_t address, uint8_t value) noexcept{
        dma_storage_.tx.fill(0U);
        dma_storage_.rx.fill(0U);

        dma_storage_.tx[0] = bmi088_register::makeWriteAddress(address);
        dma_storage_.tx[1] = value;

        return accel_device_.submit(transfer_, dma_storage_.tx.data(), dma_storage_.rx.data(), 2U, config_.transfer_timeout_ms);
    }

    bool Bmi088::submitGyroWrite(uint8_t address, uint8_t value) noexcept{
        dma_storage_.tx.fill(0U);
        dma_storage_.rx.fill(0U);

        dma_storage_.tx[0] = bmi088_register::makeWriteAddress(address);
        dma_storage_.tx[1] = value;

        return gyro_device_.submit(transfer_, dma_storage_.tx.data(), dma_storage_.rx.data(), 2U, config_.transfer_timeout_ms);
    }
    
}
