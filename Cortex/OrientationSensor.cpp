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
#include "utilities.h"

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

	nullX = 2.19;
	nullY = -9.31;
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
	accelCalibStatus = 0;
	setupOk = false;
}

void OrientationSensor::reset() {
	// reset IMU but putting LO/HI/LO on reset PIN
	pinMode(IMU_RESET_PIN, OUTPUT);
	digitalWrite(IMU_RESET_PIN, LOW);
	delay(1);
	digitalWrite(IMU_RESET_PIN, HIGH);
	delay(1);
	digitalWrite(IMU_RESET_PIN, LOW);
	delay(1);
	pinMode(IMU_RESET_PIN, INPUT);
}

void OrientationSensor::setup(i2c_t3* newWireline) {
	if (bno == NULL)
		bno = new Adafruit_BNO055(newWireline, glbSensorID);


	// Initialise the sensor
	bool ok = bno->begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
	if(!ok)
	{
		// There was a problem detecting the BNO055 ... check the connections
		setError(ErrorCodeType::IMU_NOT_DETECTED);
		logger->println("IMU:no IMU found!");
		setupOk = false;
		return;
	}

	setupOk = true;

	bno->setExtCrystalUse(true);

	setupTime = millis();
	calibrationRead = false;

	orientationEvent.orientation.x = 0;
	orientationEvent.orientation.y = 0;
	orientationEvent.orientation.z = 0;
	accelerationEvent.orientation.x = 0;
	accelerationEvent.orientation.y = 0;
	accelerationEvent.orientation.z = 0;

}


float  normDegree(float x) {
	if (x > 180.0)
		return normDegree(x-360.0);
	if (x < -180.0)
		return normDegree(x+360.0);
	return x;
}

bool OrientationSensor::getData(float &newXAngle, float &newYAngle, float &newZAccel, uint8_t &newSystem, uint8_t &newGyro, uint8_t &newAcc) {
	newSystem = systemCalibStatus;
	newGyro = gyroCalibStatus;
	newAcc = accelCalibStatus;

	if (setupOk) {
		// turn the data according to the position of the IMU
		newXAngle = normDegree(orientationEvent.orientation.y - memory.persMem.imuCalib.nullX) 		 ;
		newYAngle = normDegree(180.0-orientationEvent.orientation.z - memory.persMem.imuCalib.nullY) ;
		if (newYAngle < -150)
			newYAngle += 180;
		if (newYAngle > 150)
			newYAngle -= 180;

		newZAccel = getZAccel();

		// plausibility check, maybe bot is on its back or IMU delivers rubbish
		if ((abs(newXAngle) > 30) || (abs(newYAngle)>30)) {
			logger->print("IMU switched off due to xy=(");
			logger->print(newXAngle);
			logger->print(',');
			logger->print(newYAngle);
			logger->print(")");

			newXAngle = 0;
			newYAngle = 0;
			setupOk = false;
		}
	} else {
		// if IMU is not working, return 0
		newXAngle = 0;
		newYAngle = 0;
		newZAccel = 0;
	}
	return (newSystem > 0) || (newGyro > 0) || (newAcc > 0);

}

void OrientationSensor::nullify() {
	memory.persMem.imuCalib.nullX = normDegree(orientationEvent.orientation.y);
	memory.persMem.imuCalib.nullY = normDegree(180.0-orientationEvent.orientation.z) ;
}

void OrientationSensor::updateCalibration()
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */

  uint8_t magCalibStatus;
  bno->getCalibration(&systemCalibStatus, &gyroCalibStatus, &accelCalibStatus, &magCalibStatus);

  /* The data should be ignored until the system calibration is > 0 */

  // when not using NDOF mode, system and magnetometer calibration is always 0
  // in order to not confuse the caller, set it to max
  systemCalibStatus = 3;

  if (!calibrationRead) {
	  logger->println("IMU: update IMU calibration by eeprom");

	  readCalibrationFromEprom();
	  calibrationRead = true;
  }

  // if not yet in epprom but we are fully calibrated, store calibration
  if ((memory.persMem.imuCalib.sensorID != glbSensorID) && bno->isFullyCalibrated()) {
	  logger->println("IMU: save calibration to eeprom");
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
		logger->println("IMU: save calibration");
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
		logger->println("IMU: actual IMU calibration");
		buffer.print();
	}
	logger->println("IMU: EEPROM calibration");
	memory.persMem.imuCalib.print();
}

void OrientationSensor::readCalibrationFromEprom() {
	if (!calibrationRead && (memory.persMem.imuCalib.sensorID == glbSensorID)) {
		if (memory.persMem.logServo) {
			logger->println("IMU: read calibration");
			memory.persMem.imuCalib.print();
			logger->println();
		}
		bno->setSensorOffsets(memory.persMem.imuCalib.calib);
		if (memory.persMem.logServo)
			logSensorCalibration();
		calibrationRead = true;
	} else {
		if (memory.persMem.logServo)
			logger->println("IMU: calibration not yet saved");
	}
}

bool OrientationSensor::ok() {
	return (systemCalibStatus > 0) || (gyroCalibStatus> 0)  || (accelCalibStatus> 0);
}

bool OrientationSensor::isSetup() {
	return setupOk && calibrationRead;
}

float OrientationSensor::getZAccel() {
	return currZAcceleration;
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
		orientationEvent.orientation.x = 0;
		orientationEvent.orientation.y = 0;
		orientationEvent.orientation.z = 0;

		accelerationEvent.acceleration.x = 0;
		accelerationEvent.acceleration.y = 0;
		accelerationEvent.acceleration.z = 0;

		bno->getOrientationEvent(&orientationEvent);
		// bno->getAccelerationEvent(&accelerationEvent);

		/*

		logger->print("event =[");
		logger->print(normDegree(orientationEvent.orientation.y));
		logger->print(",");
		logger->print(normDegree(180.0-orientationEvent.orientation.z));
		logger->print("]");
		// logger->println();

		// low pass z-aceleration to remove the offset (which is gravity)
		float accel = accelerationEvent.acceleration.x;

		static LowPassFilter lowpass(0.5);
		static TimePassedBy sampler;
		float dT = sampler.dT();
		float absVelocity = accel*dT;
		float lp = lowpass.get();
		float relVelocity = absVelocity - lp;


		currZAcceleration +=  relVelocity; // integrate a*dT to get differential velocity without gravity
		lowpass.set(absVelocity);


		logger->print("accel=");
		logger->print(accel,1);

		logger->print("absvel=");
		logger->print(absVelocity,3);
		logger->print("relvel=");
		logger->print(relVelocity,3);

		logger->print("lowpasZ");
		logger->print(lp,3);
		logger->print(",curr=");
		logger->print(currZAcceleration,3);
		logger->println();

		*/
	}
}

void OrientationSensor::printData() {

	float imuX,imuY,z;
	uint8_t newSystem, newGyro, newAcc;
	orientationSensor.getData(imuX, imuY, z, newSystem, newGyro, newAcc);
	int imuStatus = newSystem*100 + newGyro*10 + newAcc;

	/* Display the floating point data */
	logger->print("Orientation=(");
	logger->print(imuX, 1);
	logger->print(",");
	logger->print(imuY, 1);
	logger->print(",");
	logger->print(z, 1);
	logger->print(") calib(Sys,Gyro,Acc)=(");
	logger->print(imuStatus);
}
