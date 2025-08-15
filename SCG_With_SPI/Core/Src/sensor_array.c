/*
 * sensor_array.c
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */

#include "icm_42688_p_spi.h"
#include "sensor_array.h"
#include "stm32h5xx_hal.h"

//Defines etc.
const SensorCS sensorCSPin[NUM_SENSORS] = {
    {GPIOC, GPIO_PIN_1}, // SENSOR_0
    /*{GPIOA, GPIO_PIN_1}, // SENSOR_1
    {GPIOA, GPIO_PIN_2}, // SENSOR_2
    {GPIOA, GPIO_PIN_3}, // SENSOR_3
    {GPIOB, GPIO_PIN_0}, // SENSOR_4
    {GPIOB, GPIO_PIN_1}, // SENSOR_5
    {GPIOB, GPIO_PIN_2}, // SENSOR_6
    {GPIOB, GPIO_PIN_3}, // SENSOR_7
    {GPIOC, GPIO_PIN_0}, // SENSOR_8
    {GPIOC, GPIO_PIN_1}, // SENSOR_9
    {GPIOC, GPIO_PIN_2}, // SENSOR_10
    {GPIOC, GPIO_PIN_3}, // SENSOR_11
    {GPIOD, GPIO_PIN_0}, // SENSOR_12
    {GPIOD, GPIO_PIN_1}, // SENSOR_13
    {GPIOD, GPIO_PIN_2}, // SENSOR_14
    {GPIOD, GPIO_PIN_3}  // SENSOR_15*/
};

//Variables
uint8_t spi_rx[2] = {0x00, 0x00};

//Functions
HAL_StatusTypeDef SetupSensors(SPI_HandleTypeDef *hspi){
	HAL_StatusTypeDef status = HAL_OK;
	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		HAL_StatusTypeDef status_inside = ICM42688ReadSingle(i, hspi, 0x75, (uint8_t*)spi_rx);
		if (status_inside != HAL_OK) {
			status = HAL_ERROR;
		}
	}
	return status;
}

void SetCSStartup(){
	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		HAL_GPIO_WritePin(sensorCSPin[i].Port, sensorCSPin[i].Pin, GPIO_PIN_SET);
	}
}


