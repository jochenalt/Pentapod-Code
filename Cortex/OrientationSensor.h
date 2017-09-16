/*
 * IMU.h
 *
 *  Created on: 04.06.2017
 *      Author: SuperJochenAlt
 */

#ifndef ORIENTATIONSENSOR_H_
#define ORIENTATIONSENSOR_H_

#include "Adafruit_BNO055.h"
#include "i2c_t3-v9.1.h"

struct OrientationSensorData {
	uint8_t sensorID;
	adafruit_bno055_offsets_t calib;
	float nullX;
	float nullY;

	void print();
	void clear();
	void setDefault();
};

class OrientationSensor {
public:
	OrientationSensor();

	void setup(i2c_t3* wireLine);
	void reset();
	void updateCalibration();
	void loop(uint32_t now);
	void nullify();
	void saveCalibration();
	void readCalibrationFromEprom();
	void logSensorCalibration();
	bool isSetup();
	bool isFullyCalibrated();
	bool getData(float &x, float &y, float &z, uint8_t &newSystem, uint8_t &newGyro, uint8_t &newAcc, uint8_t& newMag);
	void printData();
	bool ok();
private:
	uint32_t setupTime;
	bool calibrationRead;

	Adafruit_BNO055* bno;
	uint8_t systemCalibStatus;
	uint8_t gyroCalibStatus;
	uint8_t accelCalibStatus;
	uint8_t magCalibStatus;
	sensors_event_t event;
	bool setupOk;
	bool useMagnetometer;
};

extern OrientationSensor orientationSensor;

#endif /* ORIENTATIONSENSOR_H_ */
