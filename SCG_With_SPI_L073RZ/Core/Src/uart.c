#include "stm32l0xx_hal.h"
#include "uart.h"
#include "icm_42688_p_spi.h"
#include "sensor_array.h"

uint8_t spacer[] = "-----------------------------------------------------------------------\r\n";
uint8_t newline[] = "\r\n";
volatile uint8_t uartComplete = 0;

void uartSetup(UART_HandleTypeDef *huart){
	  uint8_t header[] = "Sensor    |Accel X   Accel Y   Accel Z   Gyro X    Gyro Y    Gyro Z    \r\n";
	  HAL_UART_Transmit(huart, header, sizeof(header) - 1, 1000);

	  HAL_UART_Transmit(huart, spacer, sizeof(spacer) - 1, 1000);

}

void uartSendSensorData(UART_HandleTypeDef *huart, uint8_t *dataPacket, uint8_t *readyData){
	readyData[0] = '0' + (dataPacket[0] / 10);
	readyData[1] = '0' + (dataPacket[0] % 10);
	uint8_t dataText[] = "          ";
	for (int i = 0; i < 6; i++) {
		int16_t data = (int16_t)((dataPacket[i + i + 1] << 8) | dataPacket[i + i + 2]);
		if (i <= 2) {
			uartChangeFormat(data, dataText, 0);
		}else{
			uartChangeFormat(data, dataText, 1);
		}
		for (int j = 0; j < 10; j++) {
			readyData[j + (i * 10) + 11] = dataText[j];
		}
	}
}

//void uartSendMeasurement(UART_HandleTypeDef *huart, uint8_t *dataBuffer, uint16_t *dataBufferCtrl){
//	while(uartComplete == 0){
//	}
//	uartComplete = 0;
//	uint8_t readyData[] =
//			"          |                                                            \r\n"
//			"          |                                                            \r\n"
//			"          |                                                            \r\n"
//			"          |                                                            \r\n"
//			"          |                                                            \r\n"
//			"          |                                                            \r\n"
//			"          |                                                            \r\n"
//			"          |                                                            \r\n"
//			"-----------------------------------------------------------------------\r\n";
//	uint16_t offset = dataBufferCtrl[NUM_SENSORS + 1] * NUM_SENSORS * FIFO_PACKET_SIZE_MODIFIED;
//	for (int currentSensor = 0; currentSensor < NUM_SENSORS; currentSensor++) {
//		uartSendSensorData(huart, dataBuffer + offset + (currentSensor * FIFO_PACKET_SIZE_MODIFIED), readyData + (sizeof(spacer) * currentSensor));
//	}
//	HAL_UART_Transmit_DMA(huart, readyData, sizeof(readyData));
//	dataBufferCtrl[NUM_SENSORS + 1]++;
//}

void uartSendMeasurement(UART_HandleTypeDef *huart, uint8_t *dataBuffer){
	while(uartComplete == 0){
	}
	uartComplete = 0;
	uint8_t readyData[] =
			"          |                                                            \r\n"
			"          |                                                            \r\n"
			"          |                                                            \r\n"
			"          |                                                            \r\n"
			"          |                                                            \r\n"
			"          |                                                            \r\n"
			"          |                                                            \r\n"
			"          |                                                            \r\n"
			"-----------------------------------------------------------------------\r\n";
	for (int currentSensor = 0; currentSensor < NUM_SENSORS; currentSensor++) {
		uartSendSensorData(huart, dataBuffer + (currentSensor * FIFO_PACKET_SIZE_MODIFIED), readyData + (sizeof(spacer) * currentSensor));
	}
	HAL_UART_Transmit_DMA(huart, readyData, sizeof(readyData));
}

void uartChangeFormat(int16_t data, uint8_t *dataText, uint8_t sensorType){
	if (sensorType == 0) { //Accelerometer with +-2g
		float fracValue  = (float)data / 16384.0f;
		if (fracValue < 0) {
			dataText[0] = '-';
		}else{
			dataText[0] = ' ';
		}
		int intValue = (int)fracValue;
		if (intValue < 0) {
			intValue = -intValue;
		}
		dataText[1] = '0' + intValue;
		dataText[2] = '.';
		intValue = (int)((fracValue - (int)fracValue) * 100000 + 0.5f);
		if (intValue < 0) {
			intValue = -intValue;
		}
	    dataText[3] = '0' + (intValue / 10000);
	    dataText[4] = '0' + ((intValue / 1000) % 10);
	    dataText[5] = '0' + ((intValue / 100) % 10);
	    dataText[6] = '0' + ((intValue / 10) % 10);
	    dataText[7] = '0' + (intValue % 10);
	}
	if (sensorType == 1) { //Gyroscope with +-15.625dps
		float fracValue  = (float)data / 2097.2f;
		if (fracValue < 0) {
			dataText[0] = '-';
		}else{
			dataText[0] = ' ';
		}
		int intValue = (int)fracValue;
		if (intValue < 0) {
			intValue = -intValue;
		}
		dataText[1] = '0' + (intValue / 10);
		dataText[2] = '0' + (intValue % 10);
		dataText[3] = '.';
		intValue = (int)((fracValue - (int)fracValue) * 10000 + 0.5f);
		if (intValue < 0) {
			intValue = -intValue;
		}
	    dataText[4] = '0' + (intValue / 1000);
	    dataText[5] = '0' + ((intValue / 100) % 10);
	    dataText[6] = '0' + ((intValue / 10) % 10);
	    dataText[7] = '0' + (intValue % 10);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
			uartComplete = 1;
	}
}
