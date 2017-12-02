#include "Arduino.h"
#include "BotMemory.h"
#include "utilities.h"
#include "watchdog.h"
#include "core.h"
#include "pins.h"
#include "HerkulexServoDrive.h"

// the herkulex id has been set upfront with the Herkulex Manager. The convention is as follows
// Hip  (drs 0101) = legId*10 + 1 = [01,   11,  12,  13,  14 ]
// Thigh(drs 0401) = legId + 100  = [100, 101, 102, 103, 104 ]
// Knee (drs 0101) = legId*10 + 3 = [03,   13,  23,  33,  43 ]
// Foot (drs 0201) = legId*10 + 4 = [04,   14,  24,  34,  44 ]

int HerkulexServoDrive::getHerkulexId(int legId, int limbId) {
	switch (limbId) {
		case HIP:   return legId*10 + 1;
		case THIGH: return 100 + legId;
		case KNEE:  return legId*10 + 4;
		case FOOT:  return legId*10 + 2;
	}
	return -1;
};


void HerkulexServoDrive::setup(LimbConfigType* newConfigData, HerkulexClass* newHerkulexMgr) {
	// voltage is measured within low-prio loop
	voltage = 0;

	configData = newConfigData;
	herkulexMgr = newHerkulexMgr;
	if (memory.persMem.logSetup) {
		logger->print(F("   "));
		configData->print();
	}
	movement.setNull();
	
	// switch off torque, wait for real action until enable is called
	herkulexMgr->setAccelerationMax(configData->herkulexMotorId, 0);   // rectangle speed profile
	herkulexMgr->setAccelerationRatio(configData->herkulexMotorId, 0); // rectangle speed profile
	herkulexMgr->torqueOFF(configData->herkulexMotorId);

	herkulexMgr->setLed(configData->herkulexMotorId, LED_BLUE); // on hold, disabled

	// get stat
	status = herkulexMgr->stat(configData->herkulexMotorId);
	connected = false;
	if (status == H_STATUS_OK) {
		connected = true;

		// fetch current angle of servo
		bool error;
		float rawHerkulexAngle = herkulexMgr->getAngle(configData->type, configData->herkulexMotorId, error);
		currentUserAngle = convertHerkulexAngle2UserAngle(rawHerkulexAngle);

		if (memory.persMem.logSetup) {
			logger->print(" type=");
			logger->print(configData->type);
			logger->print(" herkAngle=");
			logger->print(rawHerkulexAngle,1);
			logger->print(" userAngle=");
			logger->print(currentUserAngle,1);
			logger->print(" error=");
			logger->print(error);
		}

		// switch off torque
		herkulexMgr->torqueOFF(configData->herkulexMotorId);

		if (memory.persMem.logSetup) {
			logger->println();
		}
	} else {
		if (memory.persMem.logSetup) {
			logger->print(F("    stat="));
			logger->print(status,HEX);
			logger->print(F("    servo angle="));
			logger->print(currentUserAngle);

			logFatal(F(" connection failed"));
		}
		setError(HERKULEX_STATUS_FAILED);
	}

} //setup

void HerkulexServoDrive::disable() {
	if (connected) {
		herkulexMgr->setLed(configData->herkulexMotorId, LED_BLUE); // on hold, disabled
		herkulexMgr->torqueOFF(configData->herkulexMotorId);
		enabled = false;
	}
}

bool HerkulexServoDrive::isEnabled() {
	return enabled;
}

void HerkulexServoDrive::enable() {
	if (!connected )
		return;

	if (memory.persMem.logServo) {
		logger->print(F("enable"));
		logger->println();
	}

	herkulexMgr->setLed(configData->herkulexMotorId, LED_GREEN); // active

	// fetch current angle of servo
	currentUserAngle = readCurrentAngle();

	if (memory.persMem.logServo) {
		logger->print(F("angle["));
		logger->print(configData->herkulexMotorId);
		logger->print(F("]="));
		logger->println(currentUserAngle);
	}

	// switch on torque
	herkulexMgr->torqueON(configData->herkulexMotorId);

	// now servo is in a valid angle range. Set this angle as starting point
	setUserAngle(currentUserAngle,CORTEX_SAMPLE_RATE);

	enabled = true;
}

void HerkulexServoDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
	if (memory.persMem.logServo) {
		logger->print(F("Herkulex.changeAngle("));
		logger->print(pAngleChange);
		logger->print(F(" duration="));
		logger->print(pAngleTargetDuration);
		logger->println(") ");
	}
	
	// this methods works even when no current Angle has been measured
	movement.set(getCurrentAngle(), getCurrentAngle()+pAngleChange, millis(), pAngleTargetDuration);
}

void HerkulexServoDrive::setUserAngle(float pUserAngle,uint32_t pAngleTargetDuration) {
	uint32_t now = millis();
	
	// limit angle
	pUserAngle = constrain(pUserAngle, configData->minAngle,configData->maxAngle);

	if (abs(lastAngle-pUserAngle) > 0.1) {
		if (memory.persMem.logServo) {		
			logger->print(F("Herkulex.setAngle("));
			logger->print(pUserAngle);
			logger->print(F(" duration="));
			logger->print(pAngleTargetDuration);
			logger->println(") ");
		}
		lastAngle = pUserAngle;
	}

	movement.set(movement.getCurrentAngle(now), pUserAngle, now, pAngleTargetDuration);
}

void HerkulexServoDrive::setNullAngle(float pRawAngle /* uncalibrated */) {
	if (configData)
		configData->nullAngle = pRawAngle;
}


float HerkulexServoDrive::convertUserAngle2HerkulexAngle(float userAngle) {
	float herkulexAngle = ((userAngle + configData->nullAngle)*configData->gearRatio)*(configData->direction());
	return herkulexAngle;
}

float HerkulexServoDrive::convertHerkulexAngle2UserAngle(float herkulexAngle) {
	float userAngle = herkulexAngle/configData->direction() / configData->gearRatio - configData->nullAngle;
	return userAngle;
}


void HerkulexServoDrive::moveToAngle(float pUserAngle, uint16_t pDuration_ms) {
	if (connected && enabled) {
		if (memory.persMem.logServo) {
			float actualAngle = readCurrentAngle();
			if (status == H_STATUS_OK) {
				if (abs(lastAngle-pUserAngle)>0.1)
				{
					logger->print(F("servo("));
					logLimbName(configData->id),
					logger->print(F(") ang="));
					logger->print(pUserAngle);
					logger->print("°,");
					logger->print(pDuration_ms);
					logger->print(",");
					logger->print(actualAngle);
					logger->print("° ");
				}
			}
		}
		pUserAngle = constrain(pUserAngle, configData->minAngle+angleLimitOffset,configData->maxAngle-angleLimitOffset) ;

		float herkulexAngle = convertUserAngle2HerkulexAngle(pUserAngle);

		// add one sample slot to the time, otherwise the servo does not run smooth but in steps
		herkulexMgr->moveOneAngle(configData->type, configData->herkulexMotorId,herkulexAngle, pDuration_ms, LED_GREEN);
	}
}

// read the status, store it such that stat() can return the result
void HerkulexServoDrive::readStatus() {
	if (!connected)
		status = SERVO_STAT_NO_COMM;
	else {
		if (readStatusCounter++ % 2 == 0) {
			// check if stat is ok
			status = herkulexMgr->stat(configData->herkulexMotorId);
		}
		else {
			// check if stat is ok
			voltage = herkulexMgr->getVoltage(configData->type, configData->herkulexMotorId);
		}
	}
}

// return the servo status in error enum
ServoStatusType HerkulexServoDrive::stat() {
	switch (status) {
		case H_STATUS_OK: return SERVO_STAT_OK;
		case H_ERROR_INPUT_VOLTAGE: return SERVO_STAT_VOLTAGE;
		case H_ERROR_POS_LIMIT: return SERVO_STAT_LIMIT;
		case H_ERROR_TEMPERATURE_LIMIT: return SERVO_STAT_TEMP;
		case H_ERROR_INVALID_PKT: return SERVO_STAT_INV_PKT;
		case H_ERROR_OVERLOAD: return SERVO_STAT_OVERLOAD;
		case H_ERROR_DRIVER_FAULT: return SERVO_STAT_DRIVER_FAULT;
		case H_ERROR_EEPREG_DISTORT: return SERVO_STAT_EEPREG_DISTORT;
		default:
			return SERVO_STAT_NO_COMM;
	};
}


float HerkulexServoDrive::getCurrentAngle() {
	return currentUserAngle;
}

float HerkulexServoDrive::readCurrentAngle() {
	bool error;
	int count = 0;
	float herkulexAngle;
	do {
		herkulexAngle = herkulexMgr->getAngle(configData->type, configData->herkulexMotorId, error);
	}
	while ((error == true) && (count++ < 3));

	float userAngle = convertHerkulexAngle2UserAngle(herkulexAngle);
	return userAngle;
}

void HerkulexServoDrive::loop(uint32_t now) {
	if (connected && enabled && !movement.isNull()) {
		// compute where the servo is supposed to be
		float toBeAngle = movement.getCurrentAngle(now+(int)CORTEX_SAMPLE_RATE);

		currentUserAngle = toBeAngle;
		moveToAngle(toBeAngle, 1 /*HERKULEX_MIN_SAMPLE */);
	} else {
		currentUserAngle = readCurrentAngle();
	}

	// run the low level loop of 1Hz returning the status of the servo
	// take care that we request the status equally distributed
	if (statusReadTimer.isDue()) {
		readStatus();
	}
}

void HerkulexServoDrive::syncStatusTimer(uint32_t now) {
	int legId = configData->leg;
	int limbId = configData->id;
	statusReadTimer.setRate(CORTEX_SAMPLE_RATE*20);
	statusReadTimer.setDueTime(now + (legId*5 + limbId) * CORTEX_SAMPLE_RATE);
}


float HerkulexServoDrive::readServoTorque(){
	int16_t pwm = herkulexMgr->getPWM(configData->herkulexMotorId); // pwm is proportional to torque
	float torque = float (pwm);
	return torque;
}
