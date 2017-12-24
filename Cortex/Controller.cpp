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
#include "CortexComPackage.h"
#include "I2CSlave.h"

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
	uint32_t start = millis();

	round++;

	uint32_t distancestarttime = start;
	// grab the current distance coming from distance sensors.
	// first, send requests to all serial lines to save time
	// and collect the answers in a second round
	for (int leg = 0;leg<NumberOfLegs;leg++) {
		legs[leg].fetchDistanceRequest();
	}

	// now collect all replies
	for (int leg = 0;leg<NumberOfLegs;leg++) {
		legs[leg].fetchDistanceResponse();
	}
	uint32_t distanceendtime = millis();

	// iterate over all legs limb-wise, such
	// that all serial lines are sending simultaneously. (start with all hips, then all thighs,...)
	// Each loop just sends a fire-and-forget command with the to-be position,
	// we do not wait for a reply, which takes around 10ms for 25 servos
	// send command to Thigh, Hip, Foot, Knee in order to be more reactive
	uint32_t servostarttime = millis();
	for (int limb = 0;limb<NumberOfLimbs;limb++) {
		int actLimb = limb;
		switch (actLimb) {
			case 0: actLimb = THIGH;break;
			case 1: actLimb = HIP;break;
			case 2: actLimb = FOOT;break;
			case 3: actLimb = KNEE;break;
		}
		for (int leg = 0;leg<NUMBER_OF_LEGS;leg++) { // one leg, one serial line
			legs[leg].servos[actLimb].loop(servostarttime);
		}
	}
	uint32_t servoendtime = millis();

	// run low level loop that asks one servo per loop for its status
	// ( results in a 2.2 Hz loop per servo )
	uint32_t statstarttime = millis();

	for (int limb = 0;limb<NumberOfLimbs;limb++) {
		for (int leg = 0;leg<NUMBER_OF_LEGS;leg++) { // one leg, one serial line
			int servoNumber = leg*NumberOfLimbs + limb;
			if (round % NumberOfServos == servoNumber)
				legs[leg].servos[limb].readStatus();
		}
	}
	uint32_t statendtime = millis();
	uint32_t end = statendtime;

	// low pass all measurements. These measurements are used to adapt the synchronisation
	// between communication from Cortex via I2C and communication to servos
	static uint32_t distanceTime_ms = 0;
	distanceTime_ms = (distanceendtime - distancestarttime + distanceTime_ms)/2;
	static uint32_t statusTime_ms = 0;
	statusTime_ms = (statendtime - statstarttime + statusTime_ms)/2;
	static uint32_t servoTime_ms = 0;
	servoTime_ms = (servoendtime - servostarttime  + servoTime_ms)/2;
	loopTime_ms = ((end - start) + loopTime_ms)/2;

	static TimePassedBy logTimer(5000);
	if (logTimer.isDue()) {
		cmdSerial->print("TIME(loop=");
		cmdSerial->print(loopTime_ms);
		cmdSerial->print("ms servo");
		cmdSerial->print(servoTime_ms);
		cmdSerial->print( "ms stat=");
		cmdSerial->print(statusTime_ms);
		cmdSerial->print( "ms dist=");
		cmdSerial->print(distanceTime_ms);
		cmdSerial->print("ms comm=");
		cmdSerial->print(controller.getCommunicationDuration_ms());
		cmdSerial->print("ms imu=");
		cmdSerial->print(orientationSensor.getAvrSensorReadingTime_ms());
		cmdSerial->println("ms)");
	}
}

void Controller::loop(uint32_t now) {
	// update the servo position
	if (servoLoopTimer.isDue_ms(CORTEX_SAMPLE_RATE,now))
	{
		// send commands to all servos via 5 serial lines
		sendCommandToServos();

		// low priority jobs
		memory.loop(now);			// check if something has to be written to EEPROM
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
	// additionally, we need to add the time for the response of the I2C call,  which takes 2ms
	// during the i2c communication, we avoid any other interrupts

	// final timing scheme:
	// 1. communication comes in 2ms
	// 2. process communication and prepare reply 1ms
	// 3. wait for request of reply and send reply 3ms
	// 4. time buffer 2ms
	// 5. send command to server, get distance and get imu 12ms
	// 6. time buffer 2ms
	// cortex i2c has 400KHz, requires approx 9 bits per byte, + 1 is for rounding, the road and the address/ack stuff
	const uint32_t cortexI2CFrequency = 400000;
	const uint32_t cortexCommunicationResponseDuration_ms = cortexI2CFrequency/1000/9/Cortex::ResponsePackageData::Size + 1;

	// final computation to have equal gaps between communication and duty. Give 1ms buffer for clock stretching
	uint32_t toBeDueTime = now + (servoLoopTimer.getRate()  - loopDuration_ms())/2 + cortexCommunicationResponseDuration_ms;

	// if the as-is time is not ok, i.e. not in the middle 50% of two requests,
	// adapt the controller fire time accordingly.
	// by this, the time will typically never adapted but only when it moves out of
	// the middle window between two requests.
	int difference = toBeDueTime-asIsDueTime;

	if (abs(difference) > 0) {
		servoLoopTimer.delayNextFire(difference);
		/*
		logger->print("synch(");
		logger->print(servoLoopTimer.getRate());
		logger->print(",");
		logger->print(millis());
		logger->print(")=");

		logger->print(difference);
		logger->println("ms");
		*/
	}
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


	// integrate the error if error is reasonable
	/*
	differenceInt = (differenceInt + difference*10)/2;
	if ((abs(differenceInt) > 5) && (abs(differenceInt) < 40)) {
		servoLoopTimer.setRate(CORTEX_SAMPLE_RATE + sgn(differenceInt));
		logger->print("ratio(");
		logger->print(servoLoopTimer.getRate());
		logger->print(",");
		logger->print(differenceInt);
		logger->println(")");
	}
	*/
}
