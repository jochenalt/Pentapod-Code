#include <Config.h>
#include "Arduino.h"
#include "BotMemory.h"
#include "utilities.h"
#include "pins.h"
#include "Config.h"
#include "HerkulexServoDrive.h"


void LegConfigType::setDefaults() {

	for (int legId = 0;legId<NUMBER_OF_LEGS;legId++) {
		LegConfigType& leg = memory.persMem.legs[legId];
		leg.legId = legId;

		// the stops are given by the construction
		//                  limb-id 		herkulex-id 	clkwse null min max
		leg.limbs[HIP].set(HIP, HERKULEX_DRS_0101, HerkulexServoDrive::getHerkulexId(legId, HIP),
									false, 0,
									degrees(actuatorConfigType[HIP].minAngle),
									degrees(actuatorConfigType[HIP].maxAngle), 1.0);
		leg.limbs[THIGH].set(THIGH, HERKULEX_DRS_0401, HerkulexServoDrive::getHerkulexId(legId, THIGH),
				                    true, 0,
									degrees(actuatorConfigType[THIGH].minAngle),
									degrees(actuatorConfigType[THIGH].maxAngle), 1.0);
		leg.limbs[KNEE].set(KNEE, 	HERKULEX_DRS_0101, HerkulexServoDrive::getHerkulexId(legId, KNEE),
								    false, 0,
									degrees(actuatorConfigType[KNEE].minAngle),
									degrees(actuatorConfigType[KNEE].maxAngle), 34.0/21.0);
		leg.limbs[FOOT].set(FOOT, 	HERKULEX_DRS_0201, HerkulexServoDrive::getHerkulexId(legId, FOOT),
									false, 0,
									degrees(actuatorConfigType[FOOT].minAngle),
									degrees(actuatorConfigType[FOOT].maxAngle), 1.0);

	}

	// these null values have been measured by setting all legs to
	// a middle position (when the leg is completely stretched)
	// and calling >get all. Then all the actual angles have been taken
	// and set here, which sets the correct null position.
	// hip: align it from top view
	// thigh: stretch one leg, set it to null, then bend all thighs to its stop and align all legs
	// knee: there's a line on top of the knee where the null position can be seen
	// foot: put it flat on the ground
	memory.persMem.legs[0].limbs[HIP].nullAngle		= 2.6;
	memory.persMem.legs[0].limbs[THIGH].nullAngle	= 2.6;
	memory.persMem.legs[0].limbs[KNEE].nullAngle	= 2.0;
	memory.persMem.legs[0].limbs[FOOT].nullAngle	= -3.9;

	memory.persMem.legs[1].limbs[HIP].nullAngle		=1.0;
	memory.persMem.legs[1].limbs[THIGH].nullAngle	=2.3;
	memory.persMem.legs[1].limbs[KNEE].nullAngle	=10.4;
	memory.persMem.legs[1].limbs[FOOT].nullAngle	=-13.3;

	memory.persMem.legs[2].limbs[HIP].nullAngle		=4.2;
	memory.persMem.legs[2].limbs[THIGH].nullAngle	= 6.0;
	memory.persMem.legs[2].limbs[KNEE].nullAngle	=-4.8;
	memory.persMem.legs[2].limbs[FOOT].nullAngle	= 4.8;

	memory.persMem.legs[3].limbs[HIP].nullAngle		= -1.3;
	memory.persMem.legs[3].limbs[THIGH].nullAngle	= -28.2;
	memory.persMem.legs[3].limbs[KNEE].nullAngle	= -4.2;
	memory.persMem.legs[3].limbs[FOOT].nullAngle	= 4.9;

	memory.persMem.legs[4].limbs[HIP].nullAngle		= -11.0;
	memory.persMem.legs[4].limbs[THIGH].nullAngle	= 4.1;
	memory.persMem.legs[4].limbs[KNEE].nullAngle	=1.2;
	memory.persMem.legs[4].limbs[FOOT].nullAngle	=0.0;

	// each legs gets one serial line.
	// Teensy has 6 uarts. Serial0 is USB, Serial1 is used as CLI interface2.
	// The following is from the schematics
	memory.persMem.legs[0].serialId = 6; // leg 0 uses Serial2
	memory.persMem.legs[1].serialId = 1; // leg 1 uses Serial3
	memory.persMem.legs[2].serialId = 4; // leg 2 uses Serial4
	memory.persMem.legs[3].serialId = 3; // leg 3 uses Serial1
	memory.persMem.legs[4].serialId = 2; // leg 4 uses Serial6

	// null values of the distance sensors
	memory.persMem.legs[0].nullDistance = 18;
	memory.persMem.legs[1].nullDistance = 30;
	memory.persMem.legs[2].nullDistance = 31;
	memory.persMem.legs[3].nullDistance = 26;
	memory.persMem.legs[4].nullDistance = 32;
}


void LegConfigType::print() {
	logger->print(F("leg "));
	logLegName(legId);
	logger->print(" nullDistance=");
	logger->println(nullDistance);
	for (int i= 0;i<NumberOfLimbs;i++) {
		logger->print("   ");
		limbs[i].println();
	}
}


void LimbConfigType::print() {
	logger->print(F("Limb "));
	logLimbName(id);
	logger->print(F("{"));
				
	logger->print(F(" herkulexMotorId="));
	logger->print(herkulexMotorId);
	logger->print(" clockwise=");
	logger->print(clockwise);
	logger->print(" null=");
	logger->print(nullAngle);
	logger->print(" min=");
	logger->print(minAngle);
	logger->print(" max=");
	logger->print(maxAngle);
	logger->print(" gear=");
	logger->print(gearRatio);

	logger->print(F("}"));
}

void LimbConfigType::println() {
	print();
	logger->println();
}

void logLimbName(LimbIdentifier id) {
	switch(id) {
		case HIP:	logger->print(F("hip"));break;
		case THIGH:	logger->print(F("thigh"));break;
		case KNEE:	logger->print(F("knee"));break;
		case FOOT:	logger->print(F("foot"));break;
	default:
		logger->print(id);
		logFatal(F("invalid actuator"));
		break;
	}
	logger->print(F("("));
	logger->print(id);
	logger->print(F(")"));
}

void logLegName(int legId) {
	switch(legId) {
		case 0:	logger->print(F("left-back"));break;
		case 1:	logger->print(F("left-front"));break;
		case 2:	logger->print(F("front"));break;
		case 3:	logger->print(F("right-front"));break;
		case 4:	logger->print(F("right-back"));break;
		default:
			logger->print(legId);
			logFatal(F("invalid leg"));
			break;
	}
	logger->print(F("("));
	logger->print(legId);
	logger->print(F(")"));
}


void logFatal(const __FlashStringHelper *ifsh) {
	logger->print(F("FATAL:"));
	logger->println(ifsh);
}

void logError(const __FlashStringHelper *ifsh) {
	logger->print(F("ERROR:"));
	logger->println(ifsh);
}


void logPin(uint8_t pin) {
	logger->print(F("pin("));
	logger->print(pin);
	logger->print(F(")"));
}


