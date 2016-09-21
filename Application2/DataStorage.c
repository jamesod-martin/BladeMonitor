/*
 * DataStorage.c
 *
 *  Created on: Sep 15, 2016
 *      Author: jamesod
 */

#include "DataStorage.h"

static const float secondsInDay = 86400;
static const float endAngle = 90.0;
static const float endVoltage = 1.8;

sampleStruct allData[100] = {0};
uint8_t index = 0;
uint32_t totalSamples = 0;

/***********************************************************
 * This will store a new struct in the allData array and point the index at the last value added
 */
uint32_t StoreData(sampleStruct newData){
	allData[index] = newData;
	index++;
	totalSamples++;
	if(index > 99){
		index = 0;
	}
	return totalSamples;
}

/***********************************************************
 * Returns sample from indexed value
 */
sampleStruct GetData(uint8_t userIndex){
	return allData[userIndex];
}
/***********************************************************
 * Returns the average change in offsetAngle per day
 */
float CalculateAverageAngleChange(){
	uint8_t i;
	uint8_t numValues = 0;
	float total = 0;
	for (i = 1; i < 100; i++){
		if (allData[i].offsetAngle){
			numValues++;
			float changeInAngle = allData[i-1].offsetAngle - allData[i].offsetAngle;
			float changeInDays = (allData[i-1].timeStamp - allData[i-1].timeStamp) / secondsInDay;
			total += changeInAngle / changeInDays;			//this is the change per day of each sample
		}
	}
	return (total / (float) numValues);  //this is the average change per day of all samples
}
/***********************************************************
 * Returns the average change in batteryVoltage per day
 */
float CalculateAverageVoltageChange(){
	uint8_t i;
	uint8_t numValues = 0;
	float total = 0;
	for (i = 1; i < 100; i++){
		if (allData[i].batteryVoltage){
			numValues++;
			float changeInVoltage = allData[i-1].batteryVoltage - allData[i].batteryVoltage;
			float changeInDays = (allData[i-1].timeStamp - allData[i-1].timeStamp) / secondsInDay;
			total +=  changeInVoltage / changeInDays; 		//this is the change per day of each sample
		}
	}
	return (total / (float) numValues); //this is the average change per day of all samples
}
/***********************************************************
 * Returns the estimated life of the blade in days
 */
float EstimateBladeLife(float offsetAngle){
	float angleLeft = endAngle - offsetAngle;
	float angleChange = CalculateAverageAngleChange();
	return angleLeft / angleChange;
}
/***********************************************************
 * Returns the estimated life of the battery in days
 */
float EstimateBatteryLife(float batteryVoltage){
	float voltageLeft = endVoltage - batteryVoltage;
	float voltageChange = CalculateAverageVoltageChange();
	return voltageLeft / voltageChange;
}
