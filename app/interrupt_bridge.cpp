#include "app/app.hpp"

#include "stm32h7xx_hal_fdcan.h"

#include <cstdint>

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs){
    if(hfdcan == nullptr){
        return;
    }

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0){
        return;
    }

    app::instance().onFdcanRxFifo0Interrupt(*hfdcan);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){
    if(hspi == nullptr){
        return;
    }

    app::instance().onSpiTxRxCompleteInterrupt(*hspi);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi){
    if(hspi == nullptr){
        return;
    }

    app::instance().onSpiErrorInterrupt(*hspi);
}

extern "C" void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef* hspi){
    if(hspi == nullptr){
        return;
    }

    app::instance().onSpiAbortCompleteInterrupt(*hspi);
}