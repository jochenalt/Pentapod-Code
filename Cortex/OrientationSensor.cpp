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
#include "utilities/Adafruit_BNO055/utility/quaternion.h"

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
	clear();
	sensorID = glbSensorID;
	calib.accel_offset_x = 0;
	calib.accel_offset_y = 0;
	calib.accel_offset_z = 0;
	calib.gyro_offset_x  = 65535;
	calib.gyro_offset_y  = 65535;
	calib.gyro_offset_z  = 0;
	calib.mag_offset_x   = 0;
	calib.mag_offset_y   = 0;
	calib.mag_offset_z   = 2;
	calib.accel_radius   = 1000;
	calib.mag_radius     = 645;

	// null values of IMU
	nullX = 0.0;
	nullY = 1.3;
}

void OrientationSensorData::clear () {
	sensorID = glbSensorID;
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

	nullX = 0.0;
	nullY = 1.3;
}

OrientationSensor::OrientationSensor() {
	bno = NULL;
	systemCalibStatus = 0;
	gyroCalibStatus = 0;
	accelCalibStatus = 0;
	setupOk = false;
}

void OrientationSensor::reset() {
	// reset BNO055 but putting LO/HI/LO on reset PIN
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

	sensorTimer.setRate(CORTEX_SAMPLE_RATE);
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

	bno->set2GRange();
	bno->setExtCrystalUse(true);

	setupTime = millis();
	calibrationRead = false;

	// most recent values
	recentXValue = 0;
	recentYValue = 0;

	// null value set with with nullOfset quaternions
	computeNullOffsetCorrection();

	// when entering the loop immediately read the calibration
	upgradeCalibrationTimer.setRate(5000);
	upgradeCalibrationTimer.setDueTime(millis()-5000);

	// I measured that
	averageSensorReadingTime_ms = 3;
}

// return a value between -180.0° and +180°
float  normDegree(float x) {
	if (x > 180.0)
		return normDegree(x-360.0);
	if (x < -180.0)
		return normDegree(x+360.0);
	return x;
}

bool OrientationSensor::getData(float &newXAngle, float &newYAngle, uint8_t &newSystem, uint8_t &newGyro, uint8_t &newAcc) {
	newSystem = systemCalibStatus;
	newGyro = gyroCalibStatus;
	newAcc = accelCalibStatus;

	if (setupOk) {
		// turn the data according to the position of the IMU
		newXAngle = normDegree(recentXValue);
		newYAngle = normDegree(recentYValue);

		// sometimes the BNO055 is confused and turns the coordinate system
		if (newYAngle < -150)
			newYAngle += 180;
		if (newYAngle > 150)
			newYAngle -= 180;

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
		} else {
			/*
			logger->print("I(");
			logger->print(newXAngle);
			logger->print(",");
			logger->print(newYAngle);
			logger->println(")");
*/
		}
	} else {
		// if IMU is not working, return 0
		newXAngle = 0;
		newYAngle = 0;
	}
	return (newSystem > 0) || (newGyro > 0) || (newAcc > 0);

}

void OrientationSensor::setCurrentOrientationAsOffset() {
	memory.persMem.imuCalib.nullX -= recentXValue;
	memory.persMem.imuCalib.nullY -= recentYValue;

	computeNullOffsetCorrection();
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
  if ((memory.persMem.imuCalib.sensorID != glbSensorID) && (gyroCalibStatus == 3) && (accelCalibStatus == 3)) {
	  logger->println("IMU: save calibration to eeprom");
	  saveCalibration();
  }
}


bool OrientationSensor::isFullyCalibrated() {
  uint8_t magCalibStatus;
  bno->getCalibration(&systemCalibStatus, &gyroCalibStatus, &accelCalibStatus, &magCalibStatus);
  systemCalibStatus = 3;
  printData();
  return ( (systemCalibStatus == 3) && (gyroCalibStatus == 3) && (accelCalibStatus == 3));
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

void OrientationSensor::setDueTime(uint32_t dueTime) {
	sensorTimer.setDueTime(dueTime);
}

// return average time of sensor reading
uint32_t OrientationSensor::getAvrSensorReadingTime_ms() {
	return averageSensorReadingTime_ms;
}

void OrientationSensor::computeNullOffsetCorrection() {
	nullOffset.fromEuler( M_PI-radians(memory.persMem.imuCalib.nullX),+radians(memory.persMem.imuCalib.nullY),-M_PI/2);
}

void OrientationSensor::fetchData() {
	if (setupOk) {
		// for synchronisation with odroid we need the average time to fetch data from IMU
		uint32_t start = millis();

		//  Get a new sensor event
		sensors_event_t orientationEvent;
		bno->getOrientationEvent(&orientationEvent);
		float imuXRaw, imuYRaw;

		// the following takes 2ms, maybe we should move that to the ODroid
		imu::Quaternion normalized = bno->getQuat() * nullOffset;
		normalized.toEuler(imuXRaw,  imuYRaw);
		recentXValue = degrees(-imuXRaw);
		recentYValue = degrees(-imuYRaw);
		uint32_t end = millis();
		averageSensorReadingTime_ms = (averageSensorReadingTime_ms + (end - start))/2;
		/*
		logger->print("Q(");
		logger->print(imuX);
		logger->print(",");
		logger->print(imuY);
		logger->println(")");
		*/

		// set the timer such that a following loop will not call IMU before next due cycle
		sensorTimer.setDueTime(end + sensorTimer.getRate());
	}
}

void OrientationSensor::loop(uint32_t now) {
	// check for reading calibration every 5s
	if (setupOk && upgradeCalibrationTimer.isDue_ms(5000, now)) {
	 	updateCalibration();
	}

	if (setupOk && sensorTimer.isDue_ms(CORTEX_SAMPLE_RATE, now)) {
		fetchData();
	}
}

void OrientationSensor::printData() {

	float imuX,imuY;
	uint8_t newSystem, newGyro, newAcc;
	orientationSensor.getData(imuX, imuY, newSystem, newGyro, newAcc);
	int imuStatus = newSystem*100 + newGyro*10 + newAcc;

	/* Display the floating point data */
	logger->print("Orientation=(");
	logger->print(imuX, 1);
	logger->print(",");
	logger->print(imuY, 1);
	logger->print(") calib(Sys,Gyro,Acc)=(");
	logger->print(imuStatus);
}
