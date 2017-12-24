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
#include "TimePassedBy.h"

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
	void fetchData();
	void setCurrentOrientationAsOffset();
	void saveCalibration();
	void readCalibrationFromEprom();
	void logSensorCalibration();
	bool isSetup();
	bool isFullyCalibrated();
	bool getData(float &xAngle, float &yAngle, uint8_t &newSystem, uint8_t &newGyro, uint8_t &newAcc);
	void printData();
	bool ok();
	void setDueTime(uint32_t dueTime);
	uint32_t getAvrSensorReadingTime_ms();
private:
	void computeNullOffsetCorrection();
	uint32_t setupTime;
	bool calibrationRead;

	Adafruit_BNO055* bno;
	uint8_t systemCalibStatus;
	uint8_t gyroCalibStatus;
	uint8_t accelCalibStatus;
	TimePassedBy sensorTimer;
	TimePassedBy upgradeCalibrationTimer;
	imu::Quaternion nullOffset;

	uint32_t averageSensorReadingTime_ms;
	float recentXValue;
	float recentYValue;

	bool setupOk;
};

extern OrientationSensor orientationSensor;

#endif /* ORIENTATIONSENSOR_H_ */
