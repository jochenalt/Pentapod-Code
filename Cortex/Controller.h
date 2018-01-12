/* 
* Controller.h
*
* Controller runs the main loop for servos, the IMU and the distance sensors in all legs
*
* Author: JochenAlt
*/


#ifndef __MOTORS_H__
#define __MOTORS_H__


#include "Arduino.h"
#include "HerkulexServoDrive.h"

#include "TimePassedBy.h"
#include "Leg.h"

class Controller {
	public:
		Controller();

		bool setup();
		bool isSetup() { return setuped;};
		bool isEnabled() { return enabled;}
		void enable();
		void disable();

		// show configuration in logfile
		void logConfiguration();

		void loop(uint32_t now);

		Leg& getLeg(int i) { return legs[i]; };
		uint32_t loopDuration_ms() { return loopTime_ms; };
		TimePassedBy& getTimer();
		void adaptSynchronisation(uint32_t now);
		void sendCommandToServos();
		uint32_t& getCommunicationDuration_us() { return communicationDuration_us; };


	private:
		int round;
		Leg legs[NUMBER_OF_LEGS];
		bool setuped = false;
		bool enabled = false;
		uint32_t loopTime_ms;
		TimePassedBy servoLoopTimer;
		uint32_t communicationDuration_us;
};

extern Controller controller;

#endif //__MOTORS_H__
