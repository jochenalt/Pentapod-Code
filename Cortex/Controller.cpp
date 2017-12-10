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
#include "PowerVoltage.h"

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
	round = 0;

	communicationDuration_ms = 0;
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
	// logger->print("fire=");
	// logger->println(now);

	round++;
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

	uint32_t servoendtime = millis();
	/*
	for (int leg = 0;leg<NumberOfLegs;leg++) {
		legs[leg].fetchDistance();
	}
	*/
	// grab the current distance coming from laser distance sensors
	// apply the same trick for performance: send the request to all serial lines
	// and collect the answers in a second round
	for (int leg = 0;leg<NumberOfLegs;leg++) {
		legs[leg].fetchDistanceRequest(); // send request to serial line
	}
	for (int leg = 0;leg<NumberOfLegs;leg++) {
		legs[leg].fetchDistanceResponse(); // collect distance from previous round
	}

	uint32_t distanceendtime = millis();

	// run low level loop that asks one servo per loop for its status
	// ( results in a 1.6 Hz loop per servo )
	for (int limb = 0;limb<NumberOfLimbs;limb++) {
		for (int leg = 0;leg<NUMBER_OF_LEGS;leg++) { // one leg, one serial line
			int servoNumber = leg*NumberOfLimbs + limb;
			if (round % NumberOfServos == servoNumber)
				legs[leg].servos[limb].readStatus();
		}
	}

	uint32_t end = millis();

	static uint32_t distanceTime_ms = 0;
	distanceTime_ms = (distanceendtime - servoendtime)/2;
	static uint32_t statusTime_ms = 0;
	statusTime_ms = (end - distanceendtime)/2;
	static uint32_t servoTime_ms = 0;
	servoTime_ms = (end - servoendtime)/2;
	loopTime_ms = ((end - now) + loopTime_ms)/2;

	static TimePassedBy logTimer(5000);
	if (logTimer.isDue()) {
		cmdSerial->print("TIME(loop=");
		cmdSerial->print(loopTime_ms);
		cmdSerial->print("ms ");
		cmdSerial->print( " servo=");
		cmdSerial->print(servoTime_ms);
		cmdSerial->print( "ms stat=");
		cmdSerial->print(statusTime_ms);
		cmdSerial->print( "ms dist=");
		cmdSerial->print(distanceTime_ms);
		cmdSerial->print("ms comm=");
		cmdSerial->print(controller.getCommunicationDuration_ms());
		cmdSerial->println(" ms)");
	}
}

void Controller::loop(uint32_t now) {
	// update the servo position
	if (servoLoopTimer.isDue_ms(CORTEX_SAMPLE_RATE,now))
	{
		// send commands to all servos via 5 serial lines
		sendCommandToServos();

		// right after sending the command, fetch data from IMU to leverage the time in between two I2C command best
		orientationSensor.fetchData();

		voltage.loop(now);			// check the voltage with 1Hz
	}
}


void Controller::adaptSynchronisation(uint32_t now) {
	// Synchronize receiving commands and loop
	// regardless how punctual this request comes in,
	// in the long run it has a constant frequency.
	// But we need to ensure, that the cortex sends the command
	// in a precice frequency to the servos.
	// So, measure the time the requests come in
	// and try to send the commands to the servos
	// right in the middle of two requests.
	// filter the measurements to have a constant frequency.


	// get the time when the controller will fire the next loop
	uint32_t asIsDueTime = servoLoopTimer.getDueTime();

	// the ideal timing is when the command comes in right between two move commands. Substract the runtime of a loop
	uint32_t toBeDueTime = now + servoLoopTimer.getRate()/2 - loopDuration_ms()/2;

	// if the as-is time is not ok, i.e. not in the middle 50% of two requests,
	// adapt the controller fire time accordingly.
	// by this, the time will typcially never adapted but only when it moves out of
	// the middle window between two requests.
	int difference = toBeDueTime-asIsDueTime;

	if (asIsDueTime > toBeDueTime + servoLoopTimer.getRate()/6 ) {
		servoLoopTimer.delayNextFire(difference);
		/*
		logger->print("adapt t ");
		logger->print(difference);
		logger->print("now=");
		logger->print(now);
		logger->print( "asis=");
		logger->print(asIsDueTime);
		logger->print( "new asis=");
		logger->print(servoLoopTimer.getDueTime());
		logger->print(" tobe=");
		logger->print(toBeDueTime);
		logger->print(" limit=");
		logger->print(toBeDueTime - servoLoopTimer.getRate()/6 );
		logger->print("|");
		logger->print(toBeDueTime + servoLoopTimer.getRate()/6 );
		logger->println();
		*/
	}
	if (asIsDueTime  < toBeDueTime - servoLoopTimer.getRate()/6) {
		servoLoopTimer.delayNextFire(difference);
		/*
		logger->print("adapt t ");
		logger->print(difference);
		logger->print("now=");
		logger->print(now);
		logger->print(" asis=");
		logger->print(asIsDueTime);
		logger->print( "new asis=");
		logger->print(servoLoopTimer.getDueTime());
		logger->print(" tobe=");
		logger->print(toBeDueTime);
		logger->print(" limit=");
		logger->print(toBeDueTime - servoLoopTimer.getRate()/6 );
		logger->print("|");
		logger->print(toBeDueTime + servoLoopTimer.getRate()/6 );
		logger->println();
		*/
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
