/*
 * icm_42688_p_spi.h
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */

#ifndef INC_ICM_42688_P_SENSOR_H_
#define INC_ICM_42688_P_SENSOR_H_
#include "stm32l0xx_hal.h"

/************************* Defines etc. ************************/
#define FIFO_PACKET_SIZE 			16
#define FIFO_MAX_SIZE				2048
#define FIFO_PACKET_SIZE_MODIFIED	15
#define DIRECT_DATA_SIZE			12

/************************* Variables etc. ************************/

/************************* Functions ************************/
HAL_StatusTypeDef ICM42688ReadSingle(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t *pRxData);
HAL_StatusTypeDef ICM42688ReadIMU(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t *pRxData);
HAL_StatusTypeDef ICM42688ReadFIFO(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t *dataBuffer);
HAL_StatusTypeDef ICM42688Write(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t data);
HAL_StatusTypeDef ICM42688Setup(uint8_t sensorNumber, SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ICM42688FlushFIFO(uint8_t sensorNumber, SPI_HandleTypeDef *hspi);
#endif /* INC_ICM_42688_P_SENSOR_H_ */
