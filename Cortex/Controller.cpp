/* 
* Motors.cpp
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/

#include "Arduino.h"
#include "utilities.h"
#include "BotMemory.h"
#include "Controller.h"
#include <I2CPortScanner.h>
#include "watchdog.h"
#include "core.h"
#include "limits.h"

Controller controller;

Controller::Controller()
{
	setuped= false;						// flag to indicate a finished setup (used in stepperloop())
	enabled = false;					// motors are disabled until explicitly enabled
	loopTime_ms = 0;					// we measure the time per loop
}

void Controller::enable() {
	resetError();

	for (int legsNo = 0;legsNo<NumberOfLegs;legsNo++) {
		// do not overload power supply by switching on all servos simulatenously
		delay(20);
		for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
			legs[legsNo].servos[limbNo].enable();
		}
	}
	enabled = true;
}

void Controller::disable() {
	resetError();

	for (int i = 0;i<NumberOfLegs;i++) {
		for (int j = 0;j<NumberOfLimbs;j++) {
			legs[i].servos[j].disable();
		}
	}
	enabled = false;
}


void Controller::logConfiguration() {
	memory.println();
}

bool Controller::setup() {
	servoLoopTimer.setRate(CORTEX_SAMPLE_RATE);

	if (memory.persMem.logSetup) {
		logger->println(F("--- setup controller "));
	}

	// reset any remains
	resetError();
	disable();

	if (memory.persMem.logSetup) {
		logger->println(F("--- servos disabled"));
	}

	for (int i = 0;i<NUMBER_OF_LEGS;i++) {
		int serialId = memory.persMem.legs[i].serialId;

		if (memory.persMem.logSetup) {
			logger->print(F("--- setup leg "));
			logger->print(i);

			logger->print(" on Serial");
			logger->print(serialId);
			logger->println();
		}
		legs[i].setup(i, serialId);
	}
	for (int legId = 0;legId<NUMBER_OF_LEGS;legId++) {
		for (int limbId = 0;limbId<NumberOfLimbs;limbId++) {
			legs[legId].servos[limbId].syncStatusTimer();
		}
	}


	if (memory.persMem.logSetup) {
		logger->println(F("--- setup done"));
	}

	setuped= true;
	return !isError();
}

TimePassedBy& Controller::getTimer() {
	return servoLoopTimer;
}


void Controller::loop(uint32_t now) {
	// update the servo position
	if (servoLoopTimer.isDue_ms(CORTEX_SAMPLE_RATE,now))
	{

		// static uint32_t lastCall = 0;

		uint32_t start = millis();
		for (int leg = 0;leg<NUMBER_OF_LEGS;leg++) {
			legs[leg].fetchDistance();
		}

		// important: iterate over all legs limb-wise, such
		// that all serial lines are sending simultaneously. (start with all hips, then all thighs,...)
		// Each loop just sends a fire-and-forget command with the to-be position,
		// we do not wait for a reply, which takes around 10ms for 25 servos
		// additionally, there's a low level loop running with 1 Hz requesting the servos status
		for (int limb = 0;limb<NumberOfLimbs;limb++) {
			uint32_t now = millis();
			for (int leg = 0;leg<NUMBER_OF_LEGS;leg++) { // one leg, one serial line
				legs[leg].servos[limb].loop(now);
			}
		}

		uint32_t end = millis();
		loopTime_ms = (end - start);

		/*
		cmdSerial->print("t=(");
		cmdSerial->print(loopTime_ms);
		cmdSerial->print(",");
		cmdSerial->print(start - lastCall);
		cmdSerial->println(")");
		lastCall = start;

*/

	}
}


void Controller::adaptSynchronisation() {
	// Synchronize receiving commands and loop
	// regardless how punctual this request comes in,
	// in the long run it has a constant frequency.
	// But we need to ensure, that the cortex sends the command
	// in a precice frequency to the servos.
	// So, measure the time the requests come in
	// and try to send the commands to the servos
	// right in the middle of two requests.
	// filter the measurements to have a constant frequency.

	uint32_t now = millis();


	// get the time when the controller will fire the next loop
	uint32_t asIsDueTime = servoLoopTimer.getDueTime();

	// the ideal timing is when the command comes in right between two move commands
	uint32_t toBeDueTime = now + servoLoopTimer.getRate()/2;

	// if the as-is time is not ok, i.e. not in the middle 50% of two requests,
	// adapt the controller fire time accordingly.
	// by this, the time will typcially never adapted but only when it moves out of
	// the middle window between two requests.
	if (asIsDueTime > toBeDueTime + servoLoopTimer.getRate()/4 ) {
		servoLoopTimer.delayNextFire(-1);
	}
	if (asIsDueTime  < toBeDueTime - servoLoopTimer.getRate()/4) {
		servoLoopTimer.delayNextFire(+1);
	}
	/*
	cmdSerial->print("ctrl due t=");
	cmdSerial->print( now);
	cmdSerial->print(" ");
	cmdSerial->print( asIsDueTime);
	cmdSerial->print("vs.");
	cmdSerial->print(toBeDueTime);
	cmdSerial->print("adapt");
	cmdSerial->print(servoLoopTimer.getDueTime());

	cmdSerial->print(" ctrl due time=");

	cmdSerial->print(servoLoopTimer.getDueTime()-now);
	cmdSerial->println("ms");
	*/
}