/*
 * sensor_array.c
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */

#include "icm_42688_p_spi.h"
#include "sensor_array.h"
#include "stm32h5xx_hal.h"

/************************* Defines etc. ************************/
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
/************************* Variables etc. ************************/
uint8_t spi_rx[2] = {0x00, 0x00};

/************************* Functions ************************/
/**
  * @brief  sets up all sensor for SPI communication with CLKIN for a specific ODR & full scale range. sensorCSPin has to be setup first
  * @param  hspi   : pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef SetupSensors(SPI_HandleTypeDef *hspi){
	HAL_StatusTypeDef status = HAL_OK;
	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		HAL_StatusTypeDef status_inside = ICM42688Setup(i, hspi);
		if (status_inside != HAL_OK) {
			status = HAL_ERROR;
		}
	}
	return status;
}

HAL_StatusTypeDef ReadIMUs(SPI_HandleTypeDef *hspi, uint8_t *pRxData){
	for (uint8_t currentSensor = 0; currentSensor < NUM_SENSORS; currentSensor++) {
		HAL_StatusTypeDef status = ICM42688ReadIMUs(currentSensor, hspi, (uint8_t*)pRxData);
		if (status == HAL_ERROR) {
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

/**
  * @brief  reads WhoAmI register of all sensors. TODO: implement save location for read data
  * @param  hspi   : pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef CheckWhoAmI(SPI_HandleTypeDef *hspi){
	HAL_StatusTypeDef status = HAL_OK;
	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		HAL_StatusTypeDef status_inside = ICM42688CheckWhoAmI(i, hspi);
		if (status_inside != HAL_OK) {
			status = HAL_ERROR;
		}
	}
	return status;
}

/**
  * @brief  set all CS pins to high. They have to be initialized as low, so the sensor doesn't start in I2C/I3C mode
  * @param  none
  * @retval none
  */
void SetCSStartup(){
	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		HAL_GPIO_WritePin(sensorCSPin[i].Port, sensorCSPin[i].Pin, GPIO_PIN_SET);
	}
}


