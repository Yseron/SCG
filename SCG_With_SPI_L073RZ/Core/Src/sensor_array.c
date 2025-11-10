/*
 * sensor_array.c
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */

#include "icm_42688_p_spi.h"
#include "sensor_array.h"
#include "stm32l0xx_hal.h"
#include "uart.h"

/************************* Defines etc. ************************/
const SensorCS sensorCSPin[NUM_SENSORS] = {
	    {GPIOB, GPIO_PIN_4}, // SENSOR_1
	    {GPIOB, GPIO_PIN_4}, // SENSOR_1
	    {GPIOB, GPIO_PIN_4}, // SENSOR_1
	    {GPIOB, GPIO_PIN_4}, // SENSOR_1
	    {GPIOB, GPIO_PIN_4}, // SENSOR_1
	    {GPIOB, GPIO_PIN_4}, // SENSOR_1
	    {GPIOB, GPIO_PIN_4}, // SENSOR_1
	    {GPIOB, GPIO_PIN_4}, // SENSOR_1
    /*{GPIOB, GPIO_PIN_5}, // SENSOR_2
    {GPIOB, GPIO_PIN_4}, // SENSOR_0
    {GPIOB, GPIO_PIN_5}, // SENSOR_1
    {GPIOB, GPIO_PIN_4}, // SENSOR_0
    {GPIOB, GPIO_PIN_5}, // SENSOR_1
    {GPIOB, GPIO_PIN_4}, // SENSOR_0
    {GPIOB, GPIO_PIN_5}, // SENSOR_1
    */
};
/************************* Variables etc. ************************/
uint16_t dataBufferPosition = 0;

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

/**
  * @brief  reads all sensors directly and saves them in dataBuffer, the 12 data bytes are read and a sensor number byte is prepended
  * @param  hspi   : pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @param dataBuffer : large buffer to read all sensors a single time, size should be according to the modified FIFO packet
  * @retval HAL status
  */
HAL_StatusTypeDef ReadSensors(SPI_HandleTypeDef *hspi, uint8_t *dataBuffer){
	for (uint8_t currentSensor = 0; currentSensor < NUM_SENSORS; currentSensor++) {
		HAL_StatusTypeDef status = ICM42688ReadFIFO(currentSensor, hspi, dataBuffer + (currentSensor * FIFO_PACKET_SIZE_MODIFIED)); //Inputs pointer to start of the space for the data of the current sensor
		if (status == HAL_ERROR) {
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

/**
  * @brief  reads all FIFOs and saves them in dataBuffer, FIFO packet of 16 bytes is set and is modified according to ICM42688ReadFIFO()
  * @param  hspi   : pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @param dataBuffer : large buffer to read all sensors a single time, size should be according to the modified FIFO packet
  * @retval HAL status
  */
HAL_StatusTypeDef ReadFIFOs(SPI_HandleTypeDef *hspi, uint8_t *dataBuffer){
	for (uint8_t currentSensor = 0; currentSensor < NUM_SENSORS; currentSensor++) {
		HAL_StatusTypeDef status = ICM42688ReadFIFO(currentSensor, hspi, dataBuffer + (currentSensor * FIFO_PACKET_SIZE_MODIFIED));  //Inputs pointer to start of the space for the data of the current sensor
		if (status == HAL_ERROR) {
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

/**
 * @brief  flushes the FIFOs of all sensors
 * @param  hspi   : pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
 * @retval HAL status
 */
HAL_StatusTypeDef FlushFIFOs(SPI_HandleTypeDef *hspi){
	for (uint8_t currentSensor = 0; currentSensor < NUM_SENSORS; currentSensor++) {
		HAL_StatusTypeDef status = ICM42688FlushFIFO(currentSensor, hspi);  //Inputs pointer to start of the space for the data of the current sensor
		if (status == HAL_ERROR) {
			return HAL_ERROR;
		}
	}
	return HAL_OK;
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
