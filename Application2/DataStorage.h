/*
 * DataStorage.h
 *
 *  Created on: Sep 15, 2016
 *      Author: jamesod
 */

#ifndef APPLICATION2_DATASTORAGE_H_
#define APPLICATION2_DATASTORAGE_H_
#include <xdc/std.h>

//struct for holding all values that will be stored in array
typedef struct{
	uint32_t timeStamp;		// the time the sample was recorded
	float offsetAngle;		// the calculated offset angle at the timestamp
	float batteryVoltage;	// batteryVoltage at timeStamp
	float temperature;		//
} sampleStruct;


uint32_t StoreData(sampleStruct newData);  // stores one new data struct in the array and pushes out the old one.
sampleStruct GetData(uint8_t userIndex);  // gets the sample at the index position
float GetBatteryVoltage(uint8_t userIndex); // gets the batteryVoltage at the index position
float GetOffsetAngle(uint8_t userIndex); // gets the offsetAngle at the index position
float GetTemperature(uint8_t userIndex); // gets the temperature at the index position
uint32_t GetTimeStamp(uint8_t userIndex); // gets the timeStamp at the index position
float CalculateAverageAngleChange(); 	// goes through the last 100 samples and determines the average change in angle per day.
float CalculateAverageVoltageChange(); 	// goes through the last 100 samples and determines the average change in battery voltage per day.
float EstimateBladeLife(float offsetAngle);
float EstimateBatteryLife(float batteryVoltage);

#endif /* APPLICATION2_DATASTORAGE_H_ */
