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
		beforeFirstMove = true;
		currentUserAngle = 0;
		overloadDetected = false;
		anyHerkulexError = false;
		lastAngle = 0;
		connected = false;
		enabled = false;
		status = SERVO_STAT_NO_COMM;
	}
	void setUserAngle(float userAngle,uint32_t pDuration_ms);
	void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration);
	
	void setup(LimbConfigType* config, HerkulexClass* herkulexMgr);
	void loop(uint32_t now);
	float getCurrentAngle();
	float readCurrentAngle();

	void setNullAngle(float pAngle);
	
	LimbConfigType& getConfig() { return *configData;}
	void enable();
	void disable();
	bool isEnabled();
	bool isConnected() { return connected; };

	ServoStatusType stat();
	bool statusOK() { return status == H_STATUS_OK; };

	static int getHerkulexId(int legId, int limbId) { return legId*10 + limbId + 1; };

	void syncStatusTimer();
private:	
	void readStatus();
	float readServoTorque();
	float convertUserAngle2HerkulexAngle(float herkulexAngle);
	float convertHerkulexAngle2UserAngle(float userAngle);

	void moveToAngle(float angle, uint16_t pDuration_ms);
	bool beforeFirstMove;

	float currentUserAngle;
	boolean overloadDetected;			// used to store servo feedback, true of too much load on the servo
	boolean anyHerkulexError;
	LimbConfigType* configData;
	float lastAngle;					 // angle of previous run
	
	bool connected;
	bool enabled;
	HerkulexClass* herkulexMgr;
	uint8_t status;
	TimePassedBy statusReadTimer;
}; //MotorDriver

#endif //__MOTORDRIVER_HERKULEX_IMPL_H__
