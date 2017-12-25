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
// Foot (drs 0201) = legId*10 + 4 = [02,   12,  22,  32,  42 ]

// This method returns the herkulex ID of one sepcific limb
int HerkulexServoDrive::getHerkulexId(int legId /* start with 0 */, int limbId /* hip is 0 */) {
	switch (limbId) {
		case HIP:   return legId*10 + 1;
		case THIGH: return 100 + legId;
		case KNEE:  return legId*10 + 4;
		case FOOT:  return legId*10 + 2;
	}
	return -1; // should never happen
};

// setup a servo, to be called before using it
void HerkulexServoDrive::setup(LimbConfigType* newConfigData, HerkulexClass* newHerkulexMgr) {
	// voltage is measured within low-prio loop
	// but at this stage, we do not have a voltage yet.
	voltage = 0;

	// print memory content
	configData = newConfigData;
	herkulexMgr = newHerkulexMgr;
	if (memory.persMem.logSetup) {
		logger->print(F("   "));
		configData->print();
	}

	// no current movement
	movement.setNull();
	
	// make servos stiffer by increasing P and I value of PI controller
	// (dont dare to set D, all my trials ended up in jerks and vibration)
	switch (newConfigData->id) {
	// Thigh and Foot must be very reactive to IMU tilt
	case THIGH:
		herkulexMgr->setPositionKi(configData->herkulexMotorId, 450);
		herkulexMgr->setPositionKp(configData->herkulexMotorId, 600);
		break;
	case FOOT:
		herkulexMgr->setPositionKi(configData->herkulexMotorId, 250);
		herkulexMgr->setPositionKp(configData->herkulexMotorId, 300);
		break;
	// Knee and HIP can be softer
	case KNEE:
	case HIP:
		herkulexMgr->setPositionKi(configData->herkulexMotorId, 200);
		herkulexMgr->setPositionKp(configData->herkulexMotorId, 100);
		break;
	}

	// do not use trapezoid but rectangular speed profile in order to move
	// without vibrations. Otherwise the trapezoid profile would permanently stop and start within one loop
	herkulexMgr->setAccelerationRatio(configData->herkulexMotorId, 0); // ratio of acceleration within in one loop (default: 25[%])

	// start without torque, will be turn on by enable command
	herkulexMgr->torqueOFF(configData->herkulexMotorId);

	// these LED's can't be seen in the leg. Whatever.
	herkulexMgr->setLed(configData->herkulexMotorId, LED_BLUE); // on hold, disabled

	// get/check status of servo
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

// switch off the motor ( it turns freely afterwards)
void HerkulexServoDrive::disable() {
	if (connected) {
		herkulexMgr->setLed(configData->herkulexMotorId, LED_BLUE); // on hold, disabled
		herkulexMgr->torqueOFF(configData->herkulexMotorId);
		enabled = false;
	}
}

// enable a servo, i.e. switch on the motor. The servo does not move, upfront it
// reads the current angle and holds thr servo at that angle
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
	setUserAngle(currentUserAngle,CORTEX_SAMPLE_RATE+1);

	enabled = true;
}


// define the user angle of the servo (but do not yet send it to the servo, this happens in loop() )
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

// conversion from user angle to servo angle (take care of gear ratio and null angle)
float HerkulexServoDrive::convertUserAngle2HerkulexAngle(float userAngle) {
	float herkulexAngle = ((userAngle + configData->nullAngle)*configData->gearRatio)*(configData->direction());
	return herkulexAngle;
}

// conversion from serv angle to user angle (take care of gear ratio and null angle)
float HerkulexServoDrive::convertHerkulexAngle2UserAngle(float herkulexAngle) {
	float userAngle = herkulexAngle/configData->direction() / configData->gearRatio - configData->nullAngle;
	return userAngle;
}

// send a move command to the servo.
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

// read the status of the servo, stores it such that stat() can return the result
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

// return the cached servo status in an error enum
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


// returns the last angle of the servo, asked via readCurrentAngle
float HerkulexServoDrive::getCurrentAngle() {
	return currentUserAngle;
}


// asks the current angle from the servo
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

// main loop, to be called every CORTEX_SAMPLE_RATE ms.
// sends a move command to the servo.
void HerkulexServoDrive::loop(uint32_t now) {
	if (connected && enabled && !movement.isNull()) {
		// compute where the servo is supposed to be
		float toBeAngle = movement.getCurrentAngle(now+(int)CORTEX_SAMPLE_RATE);

		currentUserAngle = toBeAngle;

		// send the change of angle of one loop to the servo
		moveToAngle(toBeAngle, HERKULEX_MIN_SAMPLE);
	} else {
		currentUserAngle = readCurrentAngle();
	}

	// run the low level loop of 1Hz returning the status of the servo
	// take care that we request the status equally distributed
	/*
	if (statusReadTimer.isDue()) {
		logger->print("stat(");
		int legId = configData->leg;
		int limbId = configData->id;
		logger->print(legId);
		logger->print("/");
		logger->println(limbId);
		readStatus();
	}
*/
}

// Each servo has a timer that takes care when the status of this servo is to be fetched.
// Within each controller loop, one servo is asked for its status. This timer synchronizes
// that, it is initialized with a frequency of loop-frequency*20 and fires at a point in time
// that ensures that witrhin each controller only one of all servo-timers fires.
void HerkulexServoDrive::syncStatusTimer(uint32_t now) {
	int legId = configData->leg;
	int limbId = configData->id;
	statusReadTimer.setRate(CORTEX_SAMPLE_RATE*20);
	statusReadTimer.setDueTime(now + (legId*5 + limbId) * CORTEX_SAMPLE_RATE);
}


// return the torque in terms of PWM power that is sent to the servo
float HerkulexServoDrive::readServoTorque(){
	int16_t pwm = herkulexMgr->getPWM(configData->herkulexMotorId); // pwm is proportional to torque
	float torque = float (pwm);
	return torque;
}
