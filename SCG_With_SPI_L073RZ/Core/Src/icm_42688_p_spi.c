/*
 * icm_42688_p_spi.c
 *
 *  Created on: Aug 15, 2025
 *      Author: Yseron
 */
#include "icm_42688_p_spi.h"
#include "stm32l0xx_hal.h"
#include "sensor_array.h"

/************************* Defines etc. ************************/
//Debug
#define ENABLE_CLKIN 0 		 	// 1 to disable CLKIN, 0 to enable CLKIN
#define ENABLE_FIFO 1 			// 1 to disable FIFO, 0 to enable FIFO
#define ENABLE_FILTER 0		 	// 1 to disable Filters, 0 to enable Filters, NOT WORKING
#define DISABLE_WRITE_CHECK	0	// 1 to disable read on just written register, checking if value is equal

//Register Banks
#define REG_BANK_SEL 					0x76		//Register for selecting register bank
#define REGISTER_BANK_0 				(0 << 0)
#define REGISTER_BANK_1 				(1 << 0)
#define REGISTER_BANK_2 				(2 << 0)
#define REGISTER_BANK_3 				(3 << 0)
#define REGISTER_BANK_4 				(4 << 0)

//Bank 0
#define DEVICE_CONFIG					0x11
	#define SOFT_RESET_CONFIG			(1 << 0)	//Enables soft reset, wait 1 ms
#define DRIVE_CONFIG 					0x13
	#define SPI_SLEW_RATE				(0 << 0)	//SPI SLEW RATE
#define FIFO_CONFIG 					0x16
	#define FIFO_MODE_STREAM_TO_FIFO	(1 << 6)	//Streams to FIFO
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
#define GYRO_CONFIG_STATIC2				0xA0
	#define GYRO_AAF_DIS				(1 << 1)	//Disables AA filter for gyro
	#define GYRO_NF_DIS					(1 << 0)	//Disables notch filter for gyro

//Bank 2
#define ACCEL__CONFIG_STATIC2 			0x03
	#define ACCEL_AAF_DIS				(1 << 0)	//Disables AA filter for accelerometer

/************************* Variables etc. ************************/
uint8_t RxFIFO[FIFO_PACKET_SIZE + 1];

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
	const uint8_t TxData = sensorRegister | 0x80; // set Tx register and set first bit to 1(read)
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &TxData, 1, 1000); //send read address
	if (status == HAL_OK) {
		status = HAL_SPI_Receive(hspi, (uint8_t*)pRxData, 1, 1000); //receive 1 byte from register
	}
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
HAL_StatusTypeDef ICM42688ReadIMU(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t *pRxData){
	const uint8_t TxData[DIRECT_DATA_SIZE + 1] = {ACCEL_DATA_X1 | 0x80, 0}; // set Tx register and set first bit to 1(read) ACCEL_DATA_X1
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, TxData, pRxData, DIRECT_DATA_SIZE + 1, 1000); //read 12 bytes from register
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to low
	pRxData[0] = sensorNumber + 1; //sets first byte to current sensor number
	return status;
}

///**
//  * @brief  read the current FIFO content and modifies it and saves it to dataBuffer at position dataBufferPosition
//  * @param  sensorNumber: number of sensor up to NUM_SENSORS. sensorCSPin has to be setup first
//  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
//  * @param  dataBuffer: array in which the FIFO is saved
//  * @param	dataBufferPosition: position where the data should be saved in dataBuffer
//  * @retval HAL status
//  */
//HAL_StatusTypeDef ICM42688ReadFIFO(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t *dataBuffer, uint16_t *dataBufferPosition){
//	//Read number of packets in FIFO
//	uint8_t PacketsInFIFO = 0;
//	ICM42688ReadSingle(sensorNumber, hspi, FIFO_COUNTL, &PacketsInFIFO); // reads only lower byte as not more than 256 packets can be in the FIFO for the 3rd packet type
//	//Check if
//	if (16 * PacketsInFIFO > (FIFO_MAX_SIZE - FIFO_PACKET_SIZE)){
//		return HAL_ERROR;
//	}else if (PacketsInFIFO != 0) {
//		//Enough space in dataBuffer
//		const uint8_t TxFIFO = FIFO_DATA | 0x80;
//		uint8_t	RxFIFO[FIFO_PACKET_SIZE];
//		for (int i = 0; i < PacketsInFIFO; i++) {
//			HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
//			HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &TxFIFO, 1, 1000);
//			if (status == HAL_OK) {
//				status = HAL_SPI_Receive(hspi, (uint8_t*)RxFIFO, FIFO_PACKET_SIZE, 1000); //receive bytes from register
//			}
//			HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to low
//			if (status != HAL_OK) {
//				return HAL_ERROR;
//			}
//			uint16_t offset = (*dataBufferPosition * NUM_SENSORS * FIFO_PACKET_SIZE_MODIFIED) + (FIFO_PACKET_SIZE_MODIFIED * sensorNumber);
//			//Modifies read data and saves it to dataBuffer
//			//New packet: 	1 byte sensor number
//			//				6 bytes accel data
//			//				6 bytes gyro data
//			//				2 bytes timestamp
//			//Header
//			dataBuffer[offset] = sensorNumber;
//			//Accel
//			dataBuffer[offset + 1] = RxFIFO[2];
//			dataBuffer[offset + 2] = RxFIFO[3];
//			dataBuffer[offset + 3] = RxFIFO[4];
//			dataBuffer[offset + 4] = RxFIFO[5];
//			dataBuffer[offset + 5] = RxFIFO[6];
//			dataBuffer[offset + 6] = RxFIFO[7];
//			//Gyro
//			dataBuffer[offset + 7] = RxFIFO[8];
//			dataBuffer[offset + 8] = RxFIFO[9];
//			dataBuffer[offset + 9] = RxFIFO[10];
//			dataBuffer[offset + 10] = RxFIFO[11];
//			dataBuffer[offset + 11] = RxFIFO[12];
//			dataBuffer[offset + 12] = RxFIFO[13];
//			//Timestamp
//			dataBuffer[offset + 13] = RxFIFO[15];
//			dataBuffer[offset + 14] = RxFIFO[16];
//
//			//Moves dataBufferPosition by 1 new packet
//			if (*dataBufferPosition < 259) {
//				(*dataBufferPosition)++;
//			}else {
//				*dataBufferPosition = 0;
//			}
//		}
//	}
//	return HAL_OK;
//}

/**
  * @brief  read the current FIFO content and modifies it and saves it to dataBuffer at position dataBufferPosition
  * @param  sensorNumber: number of sensor up to NUM_SENSORS. sensorCSPin has to be setup first
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @param  dataBuffer: array in which the FIFO is saved
  * @param	dataBufferPosition: position where the data should be saved in dataBuffer
  * @retval HAL status
  */
HAL_StatusTypeDef ICM42688ReadFIFO(uint8_t sensorNumber, SPI_HandleTypeDef *hspi, uint8_t *dataBuffer){
	//Read number of packets in FIFO
//	uint8_t PacketsInFIFORx[3];
//	uint16_t PacketsInFIFO;
//	do{
//		const uint8_t TxData[3] = {FIFO_COUNTH | 0x80}; // set Tx register and set first bit to 1(read) ACCEL_DATA_X1
//		HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
//		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, TxData, PacketsInFIFORx, 3, 1000); //read 1 byte from register
//		HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to low
//		PacketsInFIFO = (PacketsInFIFORx[1] << 8) | PacketsInFIFORx[2];
//		//Check if
//	}while (PacketsInFIFO == 0);
//	if (PacketsInFIFO > 130) {
//		return HAL_ERROR;
//	}
	const uint8_t TxFIFO[1] = {FIFO_DATA | 0x80};
	uint8_t	RxFIFO[16];
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, TxFIFO, 1, 1000);
	if (status == HAL_OK) {
		status = HAL_SPI_Receive(hspi, RxFIFO, 16, 1000); //receive bytes from register
	}
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to low

//	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
//	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, TxFIFO, RxFIFO, FIFO_PACKET_SIZE + 1, 1000);
//	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to low
	if (status != HAL_OK) {
		return HAL_ERROR;
	}
	//Modifies read data and saves it to dataBuffer
	//New packet: 	1 byte sensor number
	//				6 bytes accel data
	//				6 bytes gyro data
	//				2 bytes timestamp
	//Header
	dataBuffer[0] = sensorNumber + 1;
	//Accel
	dataBuffer[1] = RxFIFO[2];
	dataBuffer[2] = RxFIFO[3];
	dataBuffer[3] = RxFIFO[4];
	dataBuffer[4] = RxFIFO[5];
	dataBuffer[5] = RxFIFO[6];
	dataBuffer[6] = RxFIFO[7];
	//Gyro
	dataBuffer[7] = RxFIFO[8];
	dataBuffer[8] = RxFIFO[9];
	dataBuffer[9] = RxFIFO[10];
	dataBuffer[10] = RxFIFO[11];
	dataBuffer[11] = RxFIFO[12];
	dataBuffer[12] = RxFIFO[13];
	//Timestamp
	dataBuffer[13] = RxFIFO[15];
	dataBuffer[14] = RxFIFO[16];
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
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_RESET); //set CS pin of sensor to low
	HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &sensorRegister, 1, 1000); //transmit write command and register value
	if (status == HAL_OK) {
			status = HAL_SPI_Transmit(hspi, &data, 1, 1000); //transmit data to register
		}
	HAL_GPIO_WritePin(sensorCSPin[sensorNumber].Port, sensorCSPin[sensorNumber].Pin, GPIO_PIN_SET); //set CS pin of sensor to high
	if (DISABLE_WRITE_CHECK == 0 && sensorRegister != 0x11) { //doesn't check device_config but also 0x11 register on all other banks, TODO: fix
		uint8_t RxDataCheck = 0x00;
		ICM42688ReadSingle(sensorNumber, hspi, sensorRegister, &RxDataCheck);
		if (RxDataCheck != data) { //Checks if written value is as it should be
			return HAL_ERROR;
		}
	}
	return status;
}

/**
  * @brief  sets up one sensor for SPI communication with CLKIN for a specific ODR & full scale range
  * @param  sensorNumber: number of sensor up to NUM_SENSORS. sensorCSPin has to be setup first
  * @param  hspi   : pointer to a SPI_HandleTypeDef structure that contains the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef ICM42688Setup(uint8_t sensorNumber, SPI_HandleTypeDef *hspi){
	if(ICM42688Write(sensorNumber, hspi, DEVICE_CONFIG, SOFT_RESET_CONFIG) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(1);
	if(ICM42688Write(sensorNumber, hspi, DEVICE_CONFIG, SOFT_RESET_CONFIG) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(1);
	if(ICM42688Write(sensorNumber, hspi, PWR_MGMT0, IDLE | GYRO_MODE_LOW_NOISE | ACCEL_MODE_LOW_NOISE) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(1);
	if(ICM42688Write(sensorNumber, hspi, DRIVE_CONFIG, 0x03) != HAL_OK){ // 0x05 doesnt work, 0x00 only for 4 MHz and below, not 8 MHz
		return HAL_ERROR;
	}
	HAL_Delay(1);
	if(ICM42688Write(sensorNumber, hspi, GYRO_CONFIG0, GYRO_FS_SEL_15_625 | GYRO_ODR_12_5) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(1);
	if(ICM42688Write(sensorNumber, hspi, ACCEL_CONFIG0, ACCEL_FS_SEL_2 | ACCEL_ODR_12_5) != HAL_OK){
		return HAL_ERROR;
	}
	HAL_Delay(1);
	if(ENABLE_CLKIN){ //Skips setup of CLKIN if define is set to 1
		if(ICM42688Write(sensorNumber, hspi, INTF_CONFIG1, INTF_CONFIG1_RESERVED | RTC_MODE | CLKSEL) != HAL_OK){
			return HAL_ERROR;
			HAL_Delay(1);
		}
		if(ICM42688Write(sensorNumber, hspi, REG_BANK_SEL, REGISTER_BANK_1) != HAL_OK){
			return HAL_ERROR;
			HAL_Delay(1);
		}
		if(ICM42688Write(sensorNumber, hspi, INTF_CONFIG5, PIN9_FUNCTION_CLKIN) != HAL_OK){
			return HAL_ERROR;
			HAL_Delay(1);
		}
		if(ICM42688Write(sensorNumber, hspi, REG_BANK_SEL, REGISTER_BANK_0) != HAL_OK){
			return HAL_ERROR;
			HAL_Delay(1);
		}
	}
	if(ENABLE_FIFO){ // Skips setup of FIFO if set to 0
		if(ICM42688Write(sensorNumber, hspi, FIFO_CONFIG1, FIFO_RESUME_PARTIAL_RD_TRUE | FIFO_TEMP_EN | FIFO_GYRO_EN | FIFO_ACCEL_EN) != HAL_OK){
			return HAL_ERROR;
			HAL_Delay(1);
		}
//		if(ICM42688Write(sensorNumber, hspi, INTF_CONFIG0, FIFO_COUNT_REC_RECORDS | FIFO_COUNT_ENDIAN_BIG | SENSOR_DATA_ENDIAN_BIG) != HAL_OK){
//			return HAL_ERROR;
//			HAL_Delay(1);
//		}
		if(ICM42688Write(sensorNumber, hspi, FIFO_CONFIG, FIFO_MODE_STREAM_TO_FIFO) != HAL_OK){
			return HAL_ERROR;
			HAL_Delay(1);
		}
//		if(ICM42688Write(sensorNumber, hspi, TMST_CONFIG, TMST_CONFIG_RESERVED | TMST_RES_RTC | TMST_FSYNC_EN_DISABLE | TMST_EN) != HAL_OK){
//			return HAL_ERROR;
//			HAL_Delay(1);
//		}
	}
	if(ENABLE_FILTER){ //Enables filters, doesnt work
		if(ICM42688Write(sensorNumber, hspi, REG_BANK_SEL, REGISTER_BANK_1) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, GYRO_CONFIG_STATIC2, GYRO_AAF_DIS | GYRO_NF_DIS) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, REG_BANK_SEL, REGISTER_BANK_2) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, ACCEL__CONFIG_STATIC2, ACCEL_AAF_DIS) != HAL_OK){
			return HAL_ERROR;
		}
		if(ICM42688Write(sensorNumber, hspi, REG_BANK_SEL, REGISTER_BANK_0) != HAL_OK){
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

