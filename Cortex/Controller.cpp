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
	setuped= false;
	enabled = false;					// motors are disabled until explicitly enabled
	loopTime_ms = 0;					// we measure the time per loop, just to check that everyhing is running finde
}

void Controller::enable() {
	resetError();

	for (int legsNo = 0;legsNo<NumberOfLegs;legsNo++) {
		// do not overload power supply by switching on all servos simulatenously
		for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
			legs[legsNo].servos[limbNo].enable();
			delay(20);
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
	loopTime_ms = 0;
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

	uint32_t syncTime = millis();
	for (int legId = 0;legId<NUMBER_OF_LEGS;legId++) {
		for (int limbId = 0;limbId<NumberOfLimbs;limbId++) {
			legs[legId].servos[limbId].syncStatusTimer(syncTime);
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


void Controller::sendCommandToServos() {
	uint32_t now = millis();

	// important: iterate over all legs limb-wise, such
	// that all serial lines are sending simultaneously. (start with all hips, then all thighs,...)
	// Each loop just sends a fire-and-forget command with the to-be position,
	// we do not wait for a reply, which takes around 10ms for 25 servos
	// send command to Thigh, Hip, Foot, Knee in order to be more reactive
	for (int limb = 0;limb<NumberOfLimbs;limb++) {
		int actLimb = limb;
		switch (actLimb) {
			case 0: actLimb = THIGH;break;
			case 1: actLimb = HIP;break;
			case 2: actLimb = FOOT;break;
			case 3: actLimb = KNEE;break;
		}
		for (int leg = 0;leg<NUMBER_OF_LEGS;leg++) { // one leg, one serial line
			legs[leg].servos[actLimb].loop(now);
		}
	}

	uint32_t middle = millis();

	for (int leg = 0;leg<NUMBER_OF_LEGS;leg++) {
		legs[leg].fetchDistance();
	}
	uint32_t end = millis();

	static uint32_t distanceTime = 0;
	static TimePassedBy logTimer(5000);

	distanceTime = (end - middle + distanceTime)/2;
	loopTime_ms = ((end - now) + loopTime_ms)/2;

	if (logTimer.isDue()) {
		cmdSerial->print("TIME(");
		cmdSerial->print(loopTime_ms);
		cmdSerial->print("(");
		cmdSerial->print(distanceTime);
		cmdSerial->println(")ms");
	}

	// set timer such that next loop happens with designated rate unless another I2C request kicks in
	servoLoopTimer.setDueTime(now + servoLoopTimer.getRate());
}

void Controller::loop(uint32_t now) {
	// update the servo position
	if (servoLoopTimer.isDue_ms(CORTEX_SAMPLE_RATE,now))
	{
		// send commands to all servos via 5 serial lines
		sendCommandToServos();

		// right after sending the command, fetch data from IMU to leverage the time in between two I2C command best
		orientationSensor.fetchData();
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

	// the ideal timing is when the command comes in right between two move commands. Substract the runtime of a loop
	uint32_t toBeDueTime = now + servoLoopTimer.getRate()/2 - 16/2;

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
