/* 
* MotorDriver.h
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#ifndef __MOTORDRIVER_HERKULEX_IMPL_H__
#define __MOTORDRIVER_HERKULEX_IMPL_H__


#include <Arduino.h>
#include "core.h"
#include "Config.h"
#include "HerkuleX.h"
#include "MotorBase.h"
#include "TimePassedBy.h"



class HerkulexServoDrive: public MotorBase
{
//functions
public:
	HerkulexServoDrive(): MotorBase (){
		configData = NULL;
		currentUserAngle = 0;
		overloadDetected = false;
		anyHerkulexError = false;
		lastAngle = 0;
		connected = false;
		enabled = false;
		status = SERVO_STAT_NO_COMM;
	}
	
	// establish communication, set null and limits, make the servo ready to be enabled
	void setup(LimbConfigType* config, HerkulexClass* herkulexMgr);

	// sends move command to the servo
	void loop(uint32_t now);

	// define movement for one loop that is to be executed in loop()
	void setUserAngle(float userAngle,uint32_t pDuration_ms);

	// return cached recently read user angle
	float getCurrentAngle();

	// read user angle from servo
	float readCurrentAngle();
	
	// switch on torque
	void enable();

	// switch off torque
	void disable();

	// returns true, if servo has been setup'ed and reacts
	bool isConnected() { return connected; };

	void  readStatus();

	// return the recently read status
	ServoStatusType stat();

	// return the recently read voltage
	float  getVoltage() { return voltage; };

	// get the herkulex ID of one specific servo
	static int getHerkulexId(int legId, int limbId);

	// to be called at the beginning for all servos at the same time.
	void syncStatusTimer(uint32_t now);

private:
	float readServoTorque();
	float convertUserAngle2HerkulexAngle(float herkulexAngle);
	float convertHerkulexAngle2UserAngle(float userAngle);

	void moveToAngle(float angle, uint16_t pDuration_ms);
	float currentUserAngle;              // current user angle (the angle after null values and gear ratio has been considered)
	boolean overloadDetected;			 // used to store servo feedback, true of too much load on the servo
	boolean anyHerkulexError;			 // true, if an error happened
	LimbConfigType* configData;			 // defines null angles, limits etc.
	float lastAngle;					 // angle of previous run
	
	bool connected;
	bool enabled;
	float voltage;
	HerkulexClass* herkulexMgr;
	uint8_t status;
	TimePassedBy statusReadTimer;
	int readStatusCounter;
}; //MotorDriver

#endif //__MOTORDRIVER_HERKULEX_IMPL_H__
