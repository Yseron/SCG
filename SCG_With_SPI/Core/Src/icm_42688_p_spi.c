/*
 * icm_42688_p_spi.c
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */
#include "icm_42688_p_spi.h"
#include "stm32h5xx_hal.h"
#include "sensor_array.h"

/************************* Defines etc. ************************/
#define DISABLE_CLKIN 1 // 1 to disable CLKIN, 0 to enable CLKIN
#define DISABLE_FIFO 1 // 1 to disable FIFO, 0 to enable FIFO

//Register Banks
#define REG_BANK_SEL 					0x76		//Register for selecting register bank
#define REGISTER_BANK_0 				(0 << 0)
#define REGISTER_BANK_1 				(1 << 0)
#define REGISTER_BANK_2 				(2 << 0)
#define REGISTER_BANK_3 				(3 << 0)
#define REGISTER_BANK_4 				(4 << 0)

//Bank 0
#define DRIVE_CONFIG 					0x13
	#define SPI_SLEW_RATE				(0 << 0)	//SPI SLEW RATE
#define FIFO_CONFIG 					0x16
	#define FIFO_MODE_STOP_ON_FULL		(2 << 6)	//FIFO stops if full
#define TEMP_DATA1						0x1D		//1st of 2 bytes of Temp data
#define ACCEL_DATA_X1					0x1F		//1st of 12 bytes of accel & gyro data
#define FIFO_COUNTH						0x2E		//1st of 2 bytes of FIFO count
#define FIFO_COUNTL						0x2F		//2nd of 2 bytes of FIFO count
#define FIFO_DATA						0x30		//FIFO data port
#define INTF_CONFIG0					0x4C
	#define FIFO_COUNT_REC_RECORDS		(1 << 6)	//FIFO count is given in number of whole records
	#define FIFO_COUNT_ENDIAN_BIG		(1 << 5)	//FIFO count is in big endian form(default)
	#define SENSOR_DATA_ENDIAN_BIG		(1 << 4)	//Sensor data is in big endian form(default)
#define INTF_CONFIG1					0x4D
	#define INTF_CONFIG1_RESERVED		(9 << 4)	//Reserved set to reset value(probably irrelevant, default)
	#define RTC_MODE					(1 << 2)	//RTC clock input required
	#define CLKSEL 						(1 << 0)	//Select PLL if available(default)
#define PWR_MGMT0						0x4E
	#define IDLE						(1 << 4)	//Activates idle mode
	#define GYRO_MODE_LOW_NOISE			(3 << 2)	//Sets gyro to low noise mode
	#define ACCEL_MODE_LOW_NOISE		(3 << 0)	//Sets accel to low noise mode
#define GYRO_CONFIG0 					0x4F
	#define GYRO_FS_SEL_15_625 			(7 << 5)	//Gyro max dps +-15.625
	#define GYRO_ODR_1000 				(6 << 0)	//Gyro ODR 1kHz
	#define GYRO_ODR_100 				(8 << 0)	//Gyro ODR 100Hz
	#define GYRO_ODR_12_5 				(11 << 0)	//Gyro ODR 12.5Hz
#define ACCEL_CONFIG0 					0x50
	#define ACCEL_FS_SEL_2				(3 << 5)	//Accel max g +-2
	#define ACCEL_ODR_1000 				(6 << 0)	//Accel ODR 1kHz
	#define ACCEL_ODR_100 				(8 << 0)	//Accel ODR 100Hz
	#define ACCEL_ODR_12_5 				(11 << 0)	//Accel ODR 12.5Hz
#define TMST_CONFIG						0x54
	#define TMST_CONFIG_RESERVED		(1 << 4)	//Reserved set to reset value(probably irrelevant, default)
	#define TMST_RES_RTC				(1 << 3)	//Timestamp resolution is 1 per clock cycle
	#define TMST_FSYNC_EN_DISABLE		(0 << 1)	//FSYNC disabled
	#define TMST_EN						(1 << 0)	//Timestamps are enabled(default)
#define FIFO_CONFIG1					0x5F
	#define FIFO_RESUME_PARTIAL_RD_TRUE	(1 << 6) 	//FIFO can resume partial read
	#define FIFO_TEMP_EN				(1 << 2) 	//Enables temperature packets in FIFO
	#define FIFO_GYRO_EN				(1 << 1)	//Enables gyro packets in FIFO
	#define FIFO_ACCEL_EN				(1 << 0)	//Enables accelerometer packets in FIFO
#define WHO_AM_I						0x75

//Bank 1
#define INTF_CONFIG5					0x7B
	#define PIN9_FUNCTION_CLKIN			(2 << 1)	//Pin 9 functions as Clock Input

/************************* Variables etc. ************************/

/************************* Functions ************************/
/**
  * @brief  receive a single byte in blocking mode
  * @param  sensorNumber: number of sensor up to NUM_SENSORS. sensorCSPin has to be setup first
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @param  sensorRegister: register on the currently used bank
  * @param  pRxData: pointer to reception data buffer
  * @retval HAL status
  */
HAL_StatusTypeDef ICM42688ReadSingle(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t *pRxData){
	const uint8_t TxData[2] = {sensorRegister | 0x80 , 0x00}; // set Tx register and set first bit to 1(read)
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)TxData, (uint8_t*)pRxData, 2, 1000); //read 1 byte from register
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to high
	return status;
}

/**
  * @brief  read the current data of the sensors
  * @param  sensorNumber: number of sensor up to NUM_SENSORS. sensorCSPin has to be setup first
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @param  pRxData: pointer to reception data buffer
  * @retval HAL status
  */
HAL_StatusTypeDef ICM42688ReadIMUs(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t *pRxData){
	const uint8_t TxData[13] = {ACCEL_DATA_X1 | 0x80, 0}; // set Tx register and set first bit to 1(read)
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)TxData, (uint8_t*)pRxData, 13, 1000); //read 1 byte from register
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to low
	return status;
}

HAL_StatusTypeDef ICM42688ReadFIFO(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t *pRxData){
	uint8_t BytesInFIFO[2] = {0, 0};
	ICM42688ReadSingle(sensorNumber, hspi, FIFO_COUNTL, (uint8_t*)BytesInFIFO);
	if (16 * BytesInFIFO[1] <= (FIFO_MAX_SIZE - FIFO_PACKET_SIZE)){
		return HAL_ERROR;
	}else if (BytesInFIFO[1] != 0) {
		uint16_t BytesToRead = 16 * BytesInFIFO[1] + 1;
		const uint8_t TxFIFO[BytesToRead] = {FIFO_DATA | 0x80, 0};
		uint8_t RxFIFO[BytesToRead] = {0};
		HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)TxFIFO, (uint8_t*)RxFIFO, BytesToRead, 1000); //read all bytes from FIFO_DATA register
		HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to low
		if (status != HAL_OK) {
			return HAL_ERROR;
		}
		for (int i = 0; i < BytesInFIFO[1]; i++) {

		}
	}
	return HAL_OK;
}

/**
  * @brief  write a single byte in blocking mode
  * @param  sensorNumber: number of sensor up to NUM_SENSORS. sensorCSPin has to be setup first
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @param  sensorRegister: register on the currently used bank
  * @param  data: byte to be sent to the sensor
  * @retval HAL status
  */
HAL_StatusTypeDef ICM42688Write(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t sensorRegister, uint8_t data){
	const uint8_t TxData[2] = {sensorRegister, data}; //set Tx register
	uint8_t RxData[2] = {0x00, 0x00};
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
	HAL_SPI_StateTypeDef status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)TxData, (uint8_t*)RxData, 2, 1000); //transmit 2 bytes
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to high
	return status;
}

/**
  * @brief  sets up one sensor for SPI communication with CLKIN for a specific ODR & full scale range
  * @param  sensorNumber: number of sensor up to NUM_SENSORS. sensorCSPin has to be setup first
  * @param  hspi   : pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef ICM42688Setup(uint8_t sensorNumber, SPI_HandleTypeDef *hspi){
	if(ICM42688Write(sensorNumber, hspi, PWR_MGMT0, IDLE | GYRO_MODE_LOW_NOISE | ACCEL_MODE_LOW_NOISE) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(1);
	if(ICM42688Write(sensorNumber, hspi, DRIVE_CONFIG, SPI_SLEW_RATE) != HAL_OK){
		return HAL_ERROR;
	}
	if(ICM42688Write(sensorNumber, hspi, GYRO_CONFIG0, GYRO_FS_SEL_15_625 | GYRO_ODR_12_5) != HAL_OK){
		return HAL_ERROR;
	}
	if(ICM42688Write(sensorNumber, hspi, ACCEL_CONFIG0, ACCEL_FS_SEL_2 | ACCEL_ODR_12_5) != HAL_OK){
		return HAL_ERROR;
	}
	if(DISABLE_CLKIN){ //Skips setup of CLKIN if define is set to 1
		if(ICM42688Write(sensorNumber, hspi, INTF_CONFIG1, INTF_CONFIG1_RESERVED | RTC_MODE | CLKSEL) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, REG_BANK_SEL, REGISTER_BANK_1) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, INTF_CONFIG5, PIN9_FUNCTION_CLKIN) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, REG_BANK_SEL, REGISTER_BANK_0) != HAL_OK){
			return HAL_ERROR;
		}
	}
	if(DISABLE_FIFO){
		if(ICM42688Write(sensorNumber, hspi, INTF_CONFIG0, FIFO_COUNT_REC_RECORDS | FIFO_COUNT_ENDIAN_BIG | SENSOR_DATA_ENDIAN_BIG) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, FIFO_CONFIG, FIFO_MODE_STOP_ON_FULL) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, FIFO_CONFIG1, FIFO_RESUME_PARTIAL_RD_TRUE | FIFO_TEMP_EN | FIFO_GYRO_EN | FIFO_ACCEL_EN) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, TMST_CONFIG, TMST_CONFIG_RESERVED | TMST_RES_RTC | TMST_FSYNC_EN_DISABLE | TMST_EN) != HAL_OK){
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

/**
  * @brief  reads WhoAmI register of a sensor
  * @param  sensorNumber: number of sensor up to NUM_SENSORS. sensorCSPin has to be setup first
  * @param  hspi   : pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef ICM42688CheckWhoAmI(uint8_t sensorNumber, SPI_HandleTypeDef *hspi){
	if(ICM42688ReadSingle(sensorNumber, hspi, WHO_AM_I, spi_rx_single) != HAL_OK){
		return HAL_ERROR;
	}
	return HAL_OK;
}
