/*
 * sensor_array.h
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */

#ifndef INC_SENSOR_ARRAY_H_
#define INC_SENSOR_ARRAY_H_
#include "icm_42688_p_spi.h"
#include "stm32h5xx_hal.h"

/************************* Defines etc. ************************/
#define NUM_SENSORS 				1
#define DATA_BUFFER_MAX_PACKAGES	1000

typedef struct {
    GPIO_TypeDef *Port;
    uint16_t Pin;
} SensorCS;

/************************* Variables etc. ************************/
extern uint8_t spi_rx_single[2];

extern const SensorCS sensorCSPin[NUM_SENSORS];

/************************* Functions ************************/
HAL_StatusTypeDef SetupSensors();
HAL_StatusTypeDef CheckWhoAmI(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ReadFIFOs(SPI_HandleTypeDef *hspi, uint8_t *dataBuffer);
void SetCSStartup();

#endif /* INC_SENSOR_ARRAY_H_ */
