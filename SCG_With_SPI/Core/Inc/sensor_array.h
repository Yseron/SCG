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

//Defines etc.
#define NUM_SENSORS 1

typedef struct {
    GPIO_TypeDef *Port;
    uint16_t Pin;
} SensorCS;

//Variables etc.
extern uint8_t spi_rx[2];

extern const SensorCS sensorCSPin[NUM_SENSORS];

//Functions
HAL_StatusTypeDef setup_sensors();
void set_CS_startup();
#endif /* INC_SENSOR_ARRAY_H_ */
