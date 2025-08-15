/*
 * icm_42688_p_sensor.c
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */
#include "icm_42688_p_spi.h"
#include "stm32h5xx_hal.h"
#include "sensor_array.h"

//Variables

//Functions
HAL_StatusTypeDef icm_42688_read_single(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t *pRxData){
	HAL_StatusTypeDef status;
	const uint8_t TxData[2] = {sensorRegister | 0x80 , 0x00};
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)TxData, (uint8_t*)pRxData, 2, 1000);
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET);
	return status;
}
HAL_StatusTypeDef icm_42688_write(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t data){
	HAL_StatusTypeDef status;
	const uint8_t TxData[2] = {sensorRegister, data};
	uint8_t RxData[2] = {0x00, 0x00};
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)TxData, (uint8_t*)RxData, 2, 1000);
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET);
	return status;
}
HAL_StatusTypeDef icm_42688_setup(uint8_t sensorNumber){
	return HAL_OK;
}
