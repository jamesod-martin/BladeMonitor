#include "MPU6500.h"


//Initializes the MPU6500.
#define InitRegNum 7
uint8_t MPU6500Init(){
	uint8_t numErrors;
	if(!bspI2cSelect(0x00, MPU6500_I2C_ADD1)){
		numErrors = 99;
		return numErrors;
	}
	uint8_t i = 0;
	uint8_t Init_Data[InitRegNum][2] = {
		{PWR_MGMT_1, 0b10000000},     // Reset Device. Do once on initialization only.
		{PWR_MGMT_1, 0b00000000},     // Clock Source using internal 20MHz oscillator
		{PWR_MGMT_2, 0b00000111},     // Enable Acc only 0b00000111 = no wake up frequecy set, all accelerometers on, all gyros off
		{ACCEL_CONFIG, 0b00000000},   // Set Accelerometer full scale to +-2g
		{ACCEL_CONFIG2, 0b00000100}, // Set Acc Data Rate to 1kHz, with 460 Hz BW and 1.94ms delay
		{INT_PIN_CFG, 0b00110000},    // Sets the interrupt pin to be active high and reset on any read
		{INT_ENABLE, 0b00000001}  //Enables the interrupt pin whe raw sensor data is ready
	};
	//Write initialization array
	for (i = 0; i<InitRegNum; i++){
		uint8_t txDataBuff[2] = {0};
		uint8_t rxDataBuff[2] = {0};
		txDataBuff[0] = Init_Data[i][0];
		txDataBuff[1] = Init_Data[i][1];
		bspI2cWrite(txDataBuff, 2);
		if (i == 0){
			delay_ms(10);
		}
		bspI2cWriteRead(&txDataBuff[0], 1, rxDataBuff, 1);
		//if (rxDataBuff !=
	}
	bspI2cDeselect();
	return numErrors;
}

//GetAccI2c
uint8_t GetAccelerometerData(int16_t * data){
	uint8_t buff[6] = {0};
	uint8_t success = 0;
	uint8_t address = ACCEL_XOUT_H;
	success = bspI2cWriteRead(&address, 1, buff, 6);
	data[0] = (int16_t)((int16_t)(buff[0] << 8) | buff[1]);
	data[1] = (int16_t)((int16_t)(buff[2] << 8) | buff[3]);
	data[2] = (int16_t)((int16_t)(buff[4] << 8) | buff[5]);

	return success;
}
