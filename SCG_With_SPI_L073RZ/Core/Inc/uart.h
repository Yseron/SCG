/*
 * uart.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Yseron
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32l0xx_hal.h"

/************************* Defines etc. ************************/

/************************* Variables etc. ************************/
extern volatile uint8_t uartComplete;

/************************* Functions ************************/
void uartSetup(UART_HandleTypeDef *huart);
void uartSendSensorData(UART_HandleTypeDef *huart, uint8_t *dataPacket, uint8_t *readyData);
void uartChangeFormat(int16_t data, uint8_t *dataText, uint8_t sensorType);
void uartSendBuffer(UART_HandleTypeDef *huart, uint8_t *dataBuffer, uint16_t *dataBufferCtrl);
void uartSendMeasurement(UART_HandleTypeDef *huart, uint8_t *dataBuffer);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_H_ */
