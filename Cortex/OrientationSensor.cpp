/*
 * IMU.cpp
 *
 *  Created on: 04.06.2017
 *      Author: JochenAlt
 */

#include <Arduino.h>
#include <OrientationSensor.h>
#include "i2c_t3-v9.1.h"
#include "pins.h"
#include "core.h"
#include "BotMemory.h"

OrientationSensor orientationSensor;
const uint8_t glbSensorID = 55;

void OrientationSensorData::print() {
	logger->print("IMU(");
	logger->print(sensorID);
	logger->print("):");
	logger->print(calib.accel_offset_x);
	logger->print(' ');
	logger->print(calib.accel_offset_y);
	logger->print(' ');
	logger->print(calib.accel_offset_z);
	logger->print(' ');
	logger->print(calib.gyro_offset_x);
	logger->print(' ');
	logger->print(calib.gyro_offset_y);
	logger->print(' ');
	logger->print(calib.gyro_offset_z);
	logger->print(' ');
	logger->print(calib.mag_offset_x);
	logger->print(' ');
	logger->print(calib.mag_offset_y);
	logger->print(' ');
	logger->print(calib.mag_offset_z);
	logger->print(' ');
	logger->print(calib.accel_radius);
	logger->print(' ');
	logger->print(calib.mag_radius);
	logger->println();
	logger->print("null=(");
	logger->print(nullX);
	logger->print(',');
	logger->print(nullY);
	logger->println(")");

}

void OrientationSensorData::setDefault () {
	sensorID= glbSensorID;
	calib.accel_offset_x = 0;
	calib.accel_offset_y = 0;
	calib.accel_offset_z = 0;
	calib.gyro_offset_x = 65535;
	calib.gyro_offset_y = 65535;
	calib.gyro_offset_z = 0;
	calib.mag_offset_x = 0;
	calib.mag_offset_y = 0;
	calib.mag_offset_z = 2;
	calib.accel_radius = 1000;
	calib.mag_radius = 645;

	nullX = -1.4;
	nullY = -10.4;
}

void OrientationSensorData::clear () {
	sensorID= glbSensorID;
	calib.accel_offset_x = 0;
	calib.accel_offset_y = 0;
	calib.accel_offset_z = 0;
	calib.gyro_offset_x = 0;
	calib.gyro_offset_y = 0;
	calib.gyro_offset_z = 0;
	calib.mag_offset_x = 0;
	calib.mag_offset_y = 0;
	calib.mag_offset_z = 0;
	calib.accel_radius = 0;
	calib.mag_radius = 0;
	nullX = 0;
	nullY = 0;
}

OrientationSensor::OrientationSensor() {
	bno = NULL;
	systemCalibStatus = 0;
	gyroCalibStatus = 0;
	magCalibStatus = 0;
	accelCalibStatus = 0;
	setupOk = false;
}

void OrientationSensor::reset() {
	// reset sensor
	pinMode(IMU_RESET_PIN, OUTPUT);
	digitalWrite(IMU_RESET_PIN, LOW);
	delay(5);
	digitalWrite(IMU_RESET_PIN, HIGH);
	delay(5);
	digitalWrite(IMU_RESET_PIN, LOW);
}

void OrientationSensor::setup(i2c_t3* newWireline) {
	if (bno == NULL)
		bno = new Adafruit_BNO055(newWireline, glbSensorID);


	// Initialise the sensor
	if(!bno->begin())
	{
		// There was a problem detecting the BNO055 ... check the connections
		setError(ErrorCodeType::IMU_NOT_DETECTED);
		logger->println("no IMU found!");
		setupOk = false;
		return;
	}
	setupOk = true;

	bno->setExtCrystalUse(true);

	setupTime = millis();
	calibrationRead = false;

	event.orientation.x = 0;
	event.orientation.y = 0;
	event.orientation.z = 0;
}


float  normDegree(float x) {
	if (x > 180.0)
		return normDegree(x-360.0);
	if (x < -180.0)
		return normDegree(x+360.0);
	return x;
}
bool OrientationSensor::getData(float &newX, float &newY, float &newZ, uint8_t &newSystem, uint8_t &newGyro, uint8_t &newAcc, uint8_t& newMag) {
	newSystem = systemCalibStatus;
	newGyro = gyroCalibStatus;
	newAcc = accelCalibStatus;
	newMag = magCalibStatus;

	// turn the data according to the position of the IMU
	newX = normDegree(-event.orientation.y) + memory.persMem.imuCalib.nullX;
	newY = normDegree(event.orientation.z-180.0) + memory.persMem.imuCalib.nullY;
	newZ = normDegree(-event.orientation.x);

	return (newSystem > 0) || (newGyro > 0) || (newMag > 0) || (newAcc > 0);

}

void OrientationSensor::nullify() {
	memory.persMem.imuCalib.nullX = -normDegree(-event.orientation.y);
	memory.persMem.imuCalib.nullY = -normDegree(event.orientation.z-180.0) ;
}

void OrientationSensor::updateCalibration()
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  bno->getCalibration(&systemCalibStatus, &gyroCalibStatus, &accelCalibStatus, &magCalibStatus);

  /* The data should be ignored until the system calibration is > 0 */

  if (!calibrationRead) {
	  readCalibrationFromEprom();
	  calibrationRead = true;
  }

  // if not yet in epprom but we are fully calibrated, store calibration
  if ((memory.persMem.imuCalib.sensorID != glbSensorID) && bno->isFullyCalibrated()) {
	  saveCalibration();
  }
}


bool OrientationSensor::isFullyCalibrated() {
	return bno->isFullyCalibrated();
}

void OrientationSensor::saveCalibration() {
	OrientationSensorData buffer;
	buffer.clear();
	bool ok = bno->getSensorOffsets(buffer.calib);
	buffer.sensorID = glbSensorID;

	if (ok) {
		logger->println("save calibration");
		buffer.print();
		logger->println();

		uint8_t* bufferRead =(uint8_t*)&buffer;
		uint8_t* persBuffer =(uint8_t*)&memory.persMem.imuCalib;
		for (uint8_t i = 0;i<sizeof(OrientationSensorData);i++)
			persBuffer[i] = bufferRead[i];

		memory.delayedSave();
		logSensorCalibration();
	}
}

void OrientationSensor::logSensorCalibration() {
	OrientationSensorData buffer;
	bool calibrated = bno->getSensorOffsets(buffer.calib);
	if (calibrated) {
		logger->println("actual IMU calibration");
		buffer.print();
	}
	logger->println("EEPROM calibration");
	memory.persMem.imuCalib.print();
}

void OrientationSensor::readCalibrationFromEprom() {
	if (!calibrationRead && (memory.persMem.imuCalib.sensorID == glbSensorID)) {
		if (memory.persMem.logServo) {
			logger->println("read IMU calibration");
			memory.persMem.imuCalib.print();
			logger->println();
		}
		bno->setSensorOffsets(memory.persMem.imuCalib.calib);
		if (memory.persMem.logServo)
			logSensorCalibration();
		calibrationRead = true;
	} else {
		if (memory.persMem.logServo)
			logger->println("IMU calibration not yet saved");
	}
}

bool OrientationSensor::ok() {
	return (systemCalibStatus > 0) || (gyroCalibStatus> 0) || (magCalibStatus> 0) || (accelCalibStatus> 0);
}

bool OrientationSensor::isSetup() {
	return setupOk && calibrationRead;
}

void OrientationSensor::loop(uint32_t now) {
	// first reading should no happen before one 300s

	static TimePassedBy timer;
	if (setupOk && timer.isDue_ms(1000, now)) {
		updateCalibration();
	}

	static TimePassedBy sensorTimer;
	if (setupOk && sensorTimer.isDue_ms(CORTEX_SAMPLE_RATE, now)) {
		/* Get a new sensor event */
		event.orientation.x = 0;
		event.orientation.y = 0;
		event.orientation.z = 0;

		bno->getEvent(&event);
	}
}

void OrientationSensor::printData() {

	float imuX,imuY,z;
	uint8_t newSystem, newGyro, newAcc, newMag;
	orientationSensor.getData(imuX, imuY, z, newSystem, newGyro, newAcc, newMag);
	int imuStatus = newSystem*1000 + newGyro*100 + newAcc*10 + newMag;


	/* Display the floating point data */
	logger->print("Orientation=(");
	logger->print(imuX, 1);
	logger->print(",");
	logger->print(imuY, 1);
	logger->print(",");
	logger->print(z, 1);
	logger->print(") calib(Sys,Gyro,Acc,Mag)=(");
	logger->print(imuStatus);
}
