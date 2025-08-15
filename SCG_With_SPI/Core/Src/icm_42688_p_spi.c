/*
 * icm_42688_p_sensor.c
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */
#include "icm_42688_p_spi.h"
#include "stm32h5xx_hal.h"
#include "sensor_array.h"

//Defines
//Register Banks
#define REG_BANK_SEL 				0x76		//Register for selecting register bank
#define REGISTER_BANK_0 			(0 << 0)
#define REGISTER_BANK_1 			(1 << 0)
#define REGISTER_BANK_2 			(2 << 0)
#define REGISTER_BANK_3 			(3 << 0)
#define REGISTER_BANK_4 			(4 << 0)

//Bank 0
#define DRIVE_CONFIG 				0x13
	#define SPI_SLEW_RATE			(0 << 0)	//SPI SLEW RATE
#define TEMP_DATA1					0x1D		//1st of 2 bytes of Temp data
#define TEMP_DATA0					0x1E
#define ACCEL_DATA_X1				0x1F		//1st of 12 bytes for accel & gyro data
#define GYRO_CONFIG0 				0x4F
	#define GYRO_FS_SEL_15_625 		(7 << 5)	//Gyro max dps +-15.625
	#define GYRO_ODR_1000 			(6 << 0)	//Gyro ODR 1kHz
	#define GYRO_ODR_100 			(8 << 0)	//Gyro ODR 100Hz
	#define GYRO_ODR_12_5 			(11 << 0)	//Gyro ODR 12.5Hz
#define ACCEL_CONFIG0 				0x50
	#define ACCEL_FS_SEL_2			(3 << 5)	//Accel max g +-2
	#define ACCEL_ODR_1000 			(6 << 0)	//Accel ODR 1kHz
	#define ACCEL_ODR_100 			(8 << 0)	//Accel ODR 100Hz
	#define ACCEL_ODR_12_5 			(11 << 0)	//Accel ODR 12.5Hz
#define INTF_CONFIG1				0x4D
	#define INTF_CONFIG1_RESERVED	(9 << 4)	//Reserved set to reset value(probably irrelevant)
	#define RTC_MODE				(1 << 2)	//RTC clock input required
	#define CLKSEL 					(1 << 0)	//Select PLL if available(default)
#define WHO_AM_I					0x75

//Bank 1
#define INTF_CONFIG5				0x7B
	#define PIN9_FUNCTION				(2 << 1)//Pin 9 functions as Clock Input

//Variables

//Functions
HAL_StatusTypeDef ICM42688ReadSingle(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t *pRxData){
	HAL_StatusTypeDef status;
	const uint8_t TxData[2] = {sensorRegister | 0x80 , 0x00};
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)TxData, (uint8_t*)pRxData, 2, 1000);
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET);
	return status;
}
HAL_StatusTypeDef ICM42688Write(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t data){
	HAL_StatusTypeDef status;
	const uint8_t TxData[2] = {sensorRegister, data};
	uint8_t RxData[2] = {0x00, 0x00};
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)TxData, (uint8_t*)RxData, 2, 1000);
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET);
	return status;
}
HAL_StatusTypeDef ICM42688Setup(uint8_t sensorNumber, SPI_HandleTypeDef *hspi){

	return HAL_OK;
}
