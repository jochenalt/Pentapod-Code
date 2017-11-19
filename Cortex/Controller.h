/* 
* Controller.h
*
* Controller running the main loop for steppers, encoders and servos.
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
		uint32_t looptime() { return loopTime_ms; };
		TimePassedBy& getTimer();
		void adaptSynchronisation();
		void sendCommandToServos();


	private:

		Leg legs[NUMBER_OF_LEGS];
		bool setuped = false;
		bool enabled = false;
		uint32_t loopTime_ms;
		TimePassedBy servoLoopTimer;
};

extern Controller controller;

#endif //__MOTORS_H__
