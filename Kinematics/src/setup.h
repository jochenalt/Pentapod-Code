/*
 * Types and constants used for kinematics
 *
 * Author: JochenAlt
 */

#ifndef SETUP_H_
#define SETUP_H_

#include <valarray>
#include <stdint.h>

#include "basics/types.h"

// logging switches
// #define KINEMATICS_LOGGING

enum GeneralEngineModeType { BeingAsleep, LiftBody, WalkingMode, TerrainMode, FallASleep};
enum LegGaitPhase { LegMovesUp = 0, LegMovesDown = 1, LegOnGround = 2};
enum GaitModeType { OneLegInTheAir, TwoLegsInTheAir, ThreeLegsInTheAir, SexyWalk, Tripod, Auto, FourLegWalk, None };


// maximum speed a leg uses to settle down when no movement happens
const realnum legSettleDownSpeed = 50.0/1000.0; // [mm/ms]

// maximum and minimum body height
const realnum maxBodyHeight = 290.0;
const realnum minBodyHeight = 35.0;

// min/max radius of ground touch points
const realnum minFootTouchPointRadius = 240.0;
const realnum maxFootTouchPointRadius = 270.0;
const realnum sleepingFootTouchPointRadius = 250;

// possible acceleration of angualar speed
const realnum maxAngularSpeedAcceleration = 0.1; //  [rad/s^2]
const realnum maxAngularSpeed = 20.0; 			 //  [rad/s]

// walking direction cannot be changed immediately but with that angular speed per speed*t
const realnum maxAngularSpeedPerSpeed = 0.3;

// minimum foot speed used in gait á place to reset legs
const realnum minGaitFrequency = 0.2; 				// [Hz]
const realnum maxGaitFrequency = 1.0; 				// [Hz]

// maximum speed of virtual foot ref point
const realnum maxGaitRefPointSpeed = 50.0; 			// [mm/s}

// maximum speed of a foot that is in the air during a gait
const realnum maxFootSpeed = 400; 					// [mm/s]

// maximum speed of a foot that is in the air during a gait
const realnum maxStartupAngleSpeed = 0.3; 			// [RAD/s]

// breathing configuration
const realnum minBreathingFrequency = 0.3; 			// [Hz]
const realnum maxBreathingFrequency = 3.0; 			// [Hz]
const realnum maxBreathingFromFootSpeed = 200.0; 	// [mm/s]
const realnum minBreathingFromFootSpeed = 50;		// [mm/s]
const realnum breathingAmplitude = 3.0;


// max acceleration of a toe
const realnum maxAcceleration = 8.0; 				// [mm/s^2]

#endif /* SETUP_H_ */
