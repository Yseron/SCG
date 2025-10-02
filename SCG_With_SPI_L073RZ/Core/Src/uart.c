#include "stm32l0xx_hal.h"
#include "uart.h"
#include "icm_42688_p_spi.h"
#include "sensor_array.h"

/************************* Defines etc. ************************/

/************************* Variables etc. ************************/
uint8_t spacer[] = "-------------------------------------------------------------\r\n";
uint8_t newline[] = "\r\n";
volatile uint8_t uartComplete = 1;

/************************* Functions ************************/
/**
  * @brief  prints the header line and a separating line over uart in blocking mode
  * @param  huart   : pointer to a UART_HandleTypeDef structure that contains the configuration information for the UART module.
  * @retval none
  */
void uartSetup(UART_HandleTypeDef *huart){
	  uint8_t header[] = "Sensor|Accel X   Accel Y   Accel Z   Gyro X    Gyro Y    Gyro Z      \r\n";
	  HAL_UART_Transmit(huart, header, sizeof(header) - 1, 1000); //sends header
	  HAL_UART_Transmit(huart, spacer, sizeof(spacer) - 1, 1000); //sends horizontal line
}

/**
  * @brief  converts input data from two's complement to digitwise float representation for a whole sensor measurement
  * @param  huart   : pointer to a UART_HandleTypeDef structure that contains the configuration information for the UART module.
  * @param  dataPacket   : pointer to the beginning of a modified packet
  * @param  readyData   : pointer to a the beginning of the corresponding line in the empty table
  * @retval none
  */
void uartSendSensorData(UART_HandleTypeDef *huart, uint8_t *dataPacket, uint8_t *readyData){
	readyData[0] = '0' + (dataPacket[0] / 10); //Puts sensor number in first 2 spots
	readyData[1] = '0' + (dataPacket[0] % 10);
	uint8_t dataText[] = "         "; //Template for every changed value
	for (int i = 0; i < 6; i++) { //Iterates over every sensor measurement, i.e. accel x,y,z & gyro x,y,z
		int16_t data = (int16_t)((dataPacket[i + i + 1] << 8) | dataPacket[i + i + 2]); //Creates two's complement from both data bytes
		if (i <= 2) { //If data is for accelerometer
			uartChangeFormat(data, dataText, 0);
		}else{ //If data is from gyroscope
			uartChangeFormat(data, dataText, 1);
		}
		for (int j = 0; j < 9; j++) {
			readyData[j + (i * 9) + 7] = dataText[j]; //Copies data row into the table
		}
	}
}

//void uartSendMeasurement(UART_HandleTypeDef *huart, uint8_t *dataBuffer, uint16_t *dataBufferCtrl){
//	while(uartComplete == 0){
//	}
//	uartComplete = 0;
//	uint8_t readyData[] =
//			"         |                                                            \r\n"
//			"         |                                                            \r\n"
//			"         |                                                            \r\n"
//			"         |                                                            \r\n"
//			"         |                                                            \r\n"
//			"         |                                                            \r\n"
//			"         |                                                            \r\n"
//			"         |                                                            \r\n"
//			"-----------------------------------------------------------------------\r\n";
//	uint16_t offset = dataBufferCtrl[NUM_SENSORS + 1] * NUM_SENSORS * FIFO_PACKET_SIZE_MODIFIED;
//	for (int currentSensor = 0; currentSensor < NUM_SENSORS; currentSensor++) {
//		uartSendSensorData(huart, dataBuffer + offset + (currentSensor * FIFO_PACKET_SIZE_MODIFIED), readyData + (sizeof(spacer) * currentSensor));
//	}
//	HAL_UART_Transmit_DMA(huart, readyData, sizeof(readyData));
//	dataBufferCtrl[NUM_SENSORS + 1]++;
//}

/**
  * @brief  after uartComplete flag has been set: creates empty table for a single measurement from every sensor and sends it via UART in DMA mode which sets the uartComplete flag when done
  * @param  huart   : pointer to a UART_HandleTypeDef structure that contains the configuration information for the UART module.
  * @param  dataPacket   : pointer to an array with modified packets from each sensor
  * @retval none
  */
void uartSendMeasurement(UART_HandleTypeDef *huart, uint8_t *dataBuffer){
	while(uartComplete != 1){ //checks uartComplete flag
	}
	uartComplete = 0;
	 static uint8_t readyData[] = //Creates empty table
			"      |                                                      \r\n"
			"      |                                                      \r\n"
			"      |                                                      \r\n"
			"      |                                                      \r\n"
			"      |                                                      \r\n"
			"      |                                                      \r\n"
			"      |                                                      \r\n"
			"      |                                                      \r\n"
			"\r\n";
	for (int currentSensor = 0; currentSensor < NUM_SENSORS; currentSensor++) {
		uartSendSensorData(huart, dataBuffer + (currentSensor * FIFO_PACKET_SIZE_MODIFIED), readyData + ((sizeof(spacer) - 1) * currentSensor)); //Fills every row with sensor data, pointers are set to the beginning of the respective sensor data
	}
	HAL_UART_Transmit_DMA(huart, readyData, sizeof(readyData) - 1); //Sends whole table
}

/**
  * @brief  converts a single two's complement value to an array of the digits in float, only for +-2g and 15.625 dps/s
  * @param  data   : two's complement of a gyroscope or accelerometer value
  * @param  dataText   : pointer to an 10 element array filled with spaces
  * @param  sensorType   : defines if the data is to be interpreted as accelerometer or gyroscope, 0 for accel, 1 for gyro
  * @retval none
  */
void uartChangeFormat(int16_t data, uint8_t *dataText, uint8_t sensorType){
	if (sensorType == 0) { //Accelerometer with +-2g
		float fracValue  = (float)data / 16384.0f; //Creates float
		if (fracValue < 0) { //Sets sign
			dataText[0] = '-';
		}else{
			dataText[0] = ' ';
		}
		int intValue = (int)fracValue; //Value before decimal places
		if (intValue < 0) {
			intValue = -intValue; //Sets to positive
		}
		dataText[1] = '0' + intValue;
		dataText[2] = '.';
		intValue = (int)((fracValue - (int)fracValue) * 100000 + 0.5f); //Value after decimal places for 4 digits, rounded
		if (intValue < 0) {
			intValue = -intValue; //Sets to positive
		}
	    dataText[3] = '0' + (intValue / 10000); 		//1st after decimal
	    dataText[4] = '0' + ((intValue / 1000) % 10);	//2nd after decimal
	    dataText[5] = '0' + ((intValue / 100) % 10); 	//3rd after decimal
	    dataText[6] = '0' + ((intValue / 10) % 10);		//4th after decimal
	    dataText[7] = '0' + (intValue % 10);
	}
	if (sensorType == 1) { //Gyroscope with +-15.625dps
		float fracValue  = (float)data / 2097.2f; //Creates float
		if (fracValue < 0) { //Sets sign
			dataText[0] = '-';
		}else{
			dataText[0] = ' ';
		}
		int intValue = (int)fracValue; //Value before decimal places
		if (intValue < 0) {
			intValue = -intValue; //Sets to positive
		}
		dataText[1] = '0' + (intValue / 10);
		dataText[2] = '0' + (intValue % 10);
		dataText[3] = '.';
		intValue = (int)((fracValue - (int)fracValue) * 10000 + 0.5f); //Value after decimal places for 3 digits, rounded
		if (intValue < 0) {
			intValue = -intValue; //Sets to positive
		}
	    dataText[4] = '0' + (intValue / 1000);			//1st after decimal
	    dataText[5] = '0' + ((intValue / 100) % 10);	//2nd after decimal
	    dataText[6] = '0' + ((intValue / 10) % 10);		//3rd after decimal
	    dataText[7] = '0' + (intValue % 10);			//4th after decimal
	}
}

/**
  * @brief  set uartComplete flag, when a DMA transfer is done
  * @param  huart   : pointer to a UART_HandleTypeDef structure that contains the configuration information for the UART module.
  * @retval none
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
			uartComplete = 1; //sets flag
	}
}
