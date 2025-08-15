/*
 * icm_42688_p_sensor.h
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */

#ifndef INC_ICM_42688_P_SENSOR_H_
#define INC_ICM_42688_P_SENSOR_H_
#include "stm32h5xx_hal.h"

//Defines


//Variables


//Functions
HAL_StatusTypeDef icm_42688_read_single(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t *pRxData);
HAL_StatusTypeDef icm_42688_write(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t data);
HAL_StatusTypeDef icm_42688_setup(uint8_t sensorNumber);
#endif /* INC_ICM_42688_P_SENSOR_H_ */
