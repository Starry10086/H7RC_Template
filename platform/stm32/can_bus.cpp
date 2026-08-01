#include "platform/stm32/can_bus.hpp"
#include "platform/can/can_types.hpp"
#include "stm32h7xx_hal_fdcan.h"

#include <array>
#include <cstdint>

namespace {
    bool decodeClassicCanLength(uint32_t dlc,uint8_t& length) noexcept {
        switch (dlc) {
        case FDCAN_DLC_BYTES_0: length = 0; return true;
        case FDCAN_DLC_BYTES_1: length = 1; return true;
        case FDCAN_DLC_BYTES_2: length = 2; return true;
        case FDCAN_DLC_BYTES_3: length = 3; return true;
        case FDCAN_DLC_BYTES_4: length = 4; return true;
        case FDCAN_DLC_BYTES_5: length = 5; return true;
        case FDCAN_DLC_BYTES_6: length = 6; return true;
        case FDCAN_DLC_BYTES_7: length = 7; return true;
        case FDCAN_DLC_BYTES_8: length = 8; return true;
        default:
            length = 0;
            return false;
        }
    }

    bool decodeIdFormat(uint32_t hal_id_type,can::IdFormat& format) noexcept {
        if (hal_id_type == FDCAN_STANDARD_ID) {
            format = can::IdFormat::Standard;
            return true;
        }

        if (hal_id_type == FDCAN_EXTENDED_ID) {
            format = can::IdFormat::Extended;
            return true;
        }

        return false;
    }

    bool decodeFrameKind(uint32_t hal_frame_type,can::FrameKind& kind) noexcept {
        if (hal_frame_type == FDCAN_DATA_FRAME) {
            kind = can::FrameKind::Data;
            return true;
        }

        if (hal_frame_type == FDCAN_REMOTE_FRAME) {
            kind = can::FrameKind::Remote;
            return true;
        }

        return false;
    }

    bool encodeClassicCanLength(uint8_t length,uint32_t& dlc) noexcept {
        switch (length) {
            case 0U: dlc = FDCAN_DLC_BYTES_0; return true;
            case 1U: dlc = FDCAN_DLC_BYTES_1; return true;
            case 2U: dlc = FDCAN_DLC_BYTES_2; return true;
            case 3U: dlc = FDCAN_DLC_BYTES_3; return true;
            case 4U: dlc = FDCAN_DLC_BYTES_4; return true;
            case 5U: dlc = FDCAN_DLC_BYTES_5; return true;
            case 6U: dlc = FDCAN_DLC_BYTES_6; return true;
            case 7U: dlc = FDCAN_DLC_BYTES_7; return true;
            case 8U: dlc = FDCAN_DLC_BYTES_8; return true;
            default:
                dlc = FDCAN_DLC_BYTES_0;
                return false;
        }
    }

    bool encodeIdFormat(can::IdFormat format,uint32_t& hal_id_format) noexcept {
        switch (format) {
            case can::IdFormat::Standard:
                hal_id_format = FDCAN_STANDARD_ID;
                return true;
            case can::IdFormat::Extended:
                hal_id_format = FDCAN_EXTENDED_ID;
                return true;
        }
        return false;
    }

    bool encodeFrameKind(can::FrameKind kind,uint32_t& hal_frame_kind) noexcept {
        switch (kind) {
        case can::FrameKind::Data:
            hal_frame_kind = FDCAN_DATA_FRAME;
            return true;
        case can::FrameKind::Remote:
            hal_frame_kind = FDCAN_REMOTE_FRAME;
            return true;
        }
        return false;
    }

    bool isValidCanId(uint32_t id,can::IdFormat format) noexcept {
        switch (format) {
            case can::IdFormat::Standard:
                return id <= 0x7FFU;
            case can::IdFormat::Extended:
                return id <= 0x1FFFFFFFU;
        }
        return false;
    }
}

namespace platform{
    bool CanBus::start() noexcept{
        if(started_){
            return true;
        }

        if(HAL_FDCAN_ConfigGlobalFilter(
            &handle_,
            FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK){
        hal_errors_.fetch_add(1,std::memory_order_relaxed);
            return false;
        }

        if(HAL_FDCAN_Start(&handle_) != HAL_OK){
            hal_errors_.fetch_add(1,std::memory_order_relaxed);
            return false;
        }

        if(HAL_FDCAN_ActivateNotification(
            &handle_,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0) != HAL_OK){
            hal_errors_.fetch_add(1,std::memory_order_relaxed);
            HAL_FDCAN_Stop(&handle_);
            return false;
        }

        started_ = true;
        return true;
    }

    bool CanBus::send(const can::Frame& frame) noexcept{
        if(!started_ || frame.length > 8U){
            rejected_transmit_frames_.fetch_add(1,std::memory_order_relaxed);
            return false;
        }

        uint32_t data_length = FDCAN_DLC_BYTES_0;
        uint32_t id_format = FDCAN_STANDARD_ID;
        uint32_t frame_kind = FDCAN_DATA_FRAME;

        const bool valid = encodeClassicCanLength(frame.length,data_length) &&
                           encodeIdFormat(frame.id_format,id_format) &&
                           encodeFrameKind(frame.kind,frame_kind) &&
                           isValidCanId(frame.id,frame.id_format);
        if(!valid){
            rejected_transmit_frames_.fetch_add(1,std::memory_order_relaxed);
            return false;
        }

        FDCAN_TxHeaderTypeDef header{};
        header.Identifier = frame.id;
        header.IdType = id_format;
        header.TxFrameType = frame_kind;
        header.DataLength = data_length;
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        header.MessageMarker = 0U;

        if(HAL_FDCAN_AddMessageToTxFifoQ(&handle_,&header,frame.data.data()) != HAL_OK){
            hal_errors_.fetch_add(1,std::memory_order_relaxed);
            return false;
        }

        queued_transmit_frames_.fetch_add(1,std::memory_order_relaxed);
        return true;
    }

    void CanBus::onRxFifo0Interrupt() noexcept{
        while(HAL_FDCAN_GetRxFifoFillLevel(
            &handle_,
            FDCAN_RX_FIFO0) > 0){
            FDCAN_RxHeaderTypeDef header{};
            std::array<uint8_t, 8> payload{};

            if(HAL_FDCAN_GetRxMessage(
                &handle_,
                FDCAN_RX_FIFO0,
                &header,
                payload.data()) != HAL_OK){
                    hal_errors_.fetch_add(1,std::memory_order_relaxed);
                    break;
            }

            can::Frame frame{};
            frame.id = header.Identifier;
            frame.data = payload;
            
            const bool valid = header.FDFormat == FDCAN_CLASSIC_CAN &&
                                 decodeClassicCanLength(header.DataLength,frame.length) &&
                                 decodeIdFormat(header.IdType,frame.id_format) &&
                                 decodeFrameKind(header.RxFrameType,frame.kind);
            if(!valid){
                invalid_frames_.fetch_add(1,std::memory_order_relaxed);
                continue;
            }
            if(!rx_queue_.pushFromIsr(frame)){
                dropped_frames_.fetch_add(1,std::memory_order_relaxed);
                continue;
            }
            received_frames_.fetch_add(1,std::memory_order_relaxed);
        }
    }

    bool CanBus::popReceived(can::Frame& frame) noexcept{
        return rx_queue_.pop(frame);
    }

    CanBusState CanBus::stats() const noexcept{
        return CanBusState{
            .received_frames = received_frames_.load(std::memory_order_relaxed),
            .dropped_frames = dropped_frames_.load(std::memory_order_relaxed),
            .invalid_frames = invalid_frames_.load(std::memory_order_relaxed),
            .hal_error = hal_errors_.load(std::memory_order_relaxed),
            .queued_transmit_frames = queued_transmit_frames_.load(std::memory_order_relaxed),
            .rejected_transmit_frames = rejected_transmit_frames_.load(std::memory_order_relaxed)
        };
    }
}