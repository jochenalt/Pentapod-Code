/*
 * MotorBase.h
 *
 * abstract base class of stepper and servos. Used by controller to access a generic motor.
 *
 * Author: JochenAlt
 */ 


#ifndef __MOTOR_BASE_H__
#define __MOTOR_BASE_H__

#include "Space.h"

class MotorBase {
	public:
	MotorBase () {movement.setNull();};
	MotorBase (MotorBase& base) {movement = base.movement;};
	virtual ~MotorBase () {};
		
	// the following methods are redefined in GearedStepperDrive and HerkulexServoDrive
	virtual void setUserAngle(float pAngle,uint32_t pAngleTargetDuration_ms) = 0;
	virtual void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration_ms) = 0;
	virtual void loop(uint32_t now_ms) = 0;
	virtual float getCurrentAngle() = 0;
	virtual void enable() = 0;
	virtual void disable() = 0;
	virtual bool isEnabled() = 0;
	
	AngleMovement movement;
};


#endif
