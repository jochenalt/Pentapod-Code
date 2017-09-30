/*
 * Leg.cpp
 *
 *  Created on: 01.05.2017
 *      Author: JochenAlt
 */

#include <Leg.h>
#include <Adafruit_VL6180X.h>
#include "BotMemory.h"
#include "HerkulexServoDrive.h"

Leg::Leg() {
}


void Leg::setup(int newId, uint8_t serialId) {
	// initialize result variables of distance sensor
	distance = 0;
	distanceStatus = VL6180X_ERROR_SYSERR_1; // no value read yet

	// initialize one serial line per leg
	legId = newId;
	herkulexSerial = &Serial1;
	switch (serialId) {
		case 1: herkulexSerial = &Serial1;break;
		case 2: herkulexSerial = &Serial2;break;
		case 3: herkulexSerial = &Serial3;break;
		case 4: herkulexSerial = &Serial4;break;
		case 5: herkulexSerial = &Serial5;break;
		case 6: herkulexSerial = &Serial6;break;
		default:
			logger->print("leg ");
			logger->print(legId);
			logger->print(" has invalid Serial");
			logger->println(serialId);
	}

	// initialize herkulex manager for this serial line
	herkulexMgr.beginSerial(herkulexSerial,HERKULEX_BAUD_RATE_HIGH); // default baud rate of Herkulex.
	herkulexMgr.initialize();

	// check if all servos work with that baud rate by calling stat
	bool allServosReact = true;
	for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
		int status = herkulexMgr.stat(HerkulexServoDrive::getHerkulexId(legId,limbNo));
		if (status!= H_STATUS_OK)
			allServosReact = false;
	}

	// if not all servos react, this could be a wrong baud rate setting.
	// Check with the default baud rate, set the servo's EEP to the high baud rate
	// and restart.
	if (!allServosReact) {
		if (memory.persMem.logSetup) {
			logger->print(F("servos do not react on high baud rate"));
		}

		// switch back to low speeed
		herkulexMgr.beginSerial(herkulexSerial,HERKULEX_BAUD_RATE_DEFAULT); // default baud rate of Herkulex.
		herkulexMgr.initialize();										// initialize all motors within scope of that manager

		// change baud rate in all servos
		for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
			int servoID = HerkulexServoDrive::getHerkulexId(legId,limbNo);
			herkulexMgr.changeBaudRate(servoID, HERKULEX_BAUD_RATE_HIGH);// change baud rate to high rate
		}

		// servo still works with low baud rate, it requires a
		// reboot to activate the new baud rate
		for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
			int servoID = HerkulexServoDrive::getHerkulexId(legId,limbNo);
			herkulexMgr.reboot(servoID);
		}

		// reinitialize herkulex Manager with high baud rate
		herkulexMgr.beginSerial(herkulexSerial,HERKULEX_BAUD_RATE_HIGH); // increased baud rate that requires changed setting in servo
		herkulexMgr.initialize();										 // initialize all motors within scope of that manager

		for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
			int servoID = HerkulexServoDrive::getHerkulexId(legId,limbNo);
			int status = herkulexMgr.stat(legId*10 + limbNo+1);
			if (status == H_STATUS_OK) {
				if (memory.persMem.logSetup) {
					logger->print("servo ");
					logger->print(servoID);
					logger->println(" ok");
				}
			}
			else {
				if (memory.persMem.logSetup) {
					logger->print("servo ");
					logger->print(servoID);
					logger->print("stat=");
					logger->print(status);
					logger->println(" fail!");
				}
			}
		}
	}

	for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
		if (memory.persMem.logSetup) {
			logger->print(F("setup "));
			logger->print(legId);
			logger->print("/");
			logger->print(limbNo);
			logger->print(" ");

		}
		servos[limbNo].setup( &(memory.persMem.legs[legId].limbs[limbNo]), &herkulexMgr);
	}
}

ServoStatusType Leg::getStatus(int limbID) {
	return servos[limbID].stat();
}

void Leg::logStatus() {
	logger->print(legId);
	logger->print(F("="));
	for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
		uint8_t s = servos[limbNo].stat();
		if (s<16)
			cmdSerial->print(F("0"));
		logger->print(s, HEX);
	}
}


// check distance sensor
void Leg::fetchDistance() {

	// distance might be negative if communication issue
	distance = herkulexMgr.requestDistance(SENSOR_HERKULEX_SERVO_ID, distanceStatus);

	// communmication issues?
	if ((distanceStatus == VL6180X_ERROR_NONE) && (distance < 0)) {
		distanceStatus = VL6180X_ERROR_COMM;
	}

	// correct null value
	distance -= memory.persMem.legs[legId].nullDistance;
}

int Leg::getDistance() {
	if (distanceStatus == VL6180X_ERROR_NONE)
		return distance;
	return 200 + distanceStatus;
}

bool Leg::distanceAvailable() {
	return (distanceStatus == VL6180X_ERROR_NONE);
}

void Leg::setAngles(LimbAnglesType angles, uint16_t duration_ms) {
	for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
		servos[limbNo].setUserAngle(angles[limbNo], duration_ms);
	}
}
