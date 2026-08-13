#include "app/app.hpp"
#include "stm32h7xx_hal_uart.h"

#include <cstdint>

// FDCAN
extern "C"{
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs){
        if(hfdcan == nullptr) return;

        if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0){
            return;
        }
        app::instance().onFdcanRxFifo0Interrupt(*hfdcan);
    }
} 

// SPI
extern "C" {
    void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){
        if(hspi == nullptr) return;
        app::instance().onSpiTxRxCompleteInterrupt(*hspi);
    }

    void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi){
        if(hspi == nullptr) return;
        app::instance().onSpiErrorInterrupt(*hspi);
    }

    void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef* hspi){
        if(hspi == nullptr) return;
        app::instance().onSpiAbortCompleteInterrupt(*hspi);
    }
} 

// IIC
extern "C" {
    void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c){
        if(hi2c == nullptr) return;
        app::instance().onI2cTransferCompleteInterrupt(*hi2c);
    }

    void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c){
        if(hi2c == nullptr) return;
        app::instance().onI2cTransferCompleteInterrupt(*hi2c);
    }

    void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
        if(hi2c == nullptr) return;
        app::instance().onI2cTransferCompleteInterrupt(*hi2c);
    }

    void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
        if(hi2c == nullptr) return;
        app::instance().onI2cTransferCompleteInterrupt(*hi2c);
    }

    void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
        if(hi2c == nullptr) return;
        app::instance().onI2cErrorInterrupt(*hi2c);
    }

    void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c){
        if(hi2c == nullptr) return;
        app::instance().onI2cAbortCompleteInterrupt(*hi2c);
    }
}

// UART
extern "C"{
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size){
        if(huart == nullptr) return;
        app::instance().onUartRxEventInterrupt(*huart, Size);
    }

    void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart){
        if(huart == nullptr) return;
        app::instance().onUartTxCompleteInterrupt(*huart);
    }

    void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart){
        if(huart == nullptr) return;

        const uint32_t hal_error = HAL_UART_GetError(huart);
        app::instance().onUartErrorInterrupt(*huart, hal_error);
    }

    void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart){
        if(huart == nullptr) return;
        app::instance().onUartTxAbortCompleteInterrupt(*huart);
    }
}

