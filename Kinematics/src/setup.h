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
enum GaitModeType { OneLegInTheAir, TwoLegsInTheAir, SexyWalk, Auto, FourLegWalk, None };

enum ShutDownModeType { NoShutDownActive, Initiate, FallAsleep, Done };
enum NavigationStatusType { NavPending=0, 	 	// The goal has yet to be processed by the action server
							NavActive = 1, 		// The goal is currently being processed by the action server
							NavPreempted = 2, 	// The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
							NavSucceeded = 3, 	// The goal was achieved successfully by the action server (Terminal State)
							NavAborted = 4, 	// The goal was aborted during execution by the action server due to some failure (Terminal State)
							NavRejected = 5, 	// The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
							NavPreempting = 6, 	// The goal received a cancel request after it started executing and has not yet completed execution
							NavRecalled = 8, 	//  The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
							NavLost = 9 		// An action client can determine that a goal is LOST. This should not be sent over the wire by an action server
};

// maximum speed a leg uses to settle down when no movement happens
const realnum legSettleDownSpeed = 50.0/1000.0; // [mm/ms]

// maximum and minimum body height
const realnum maxBodyHeight = 260.0;			// maximum height of the body (goes along with minFootTouchPointRadius)
const realnum minBodyHeight = 50.0;				// minimum height of the body (goes along with maxFootTouchPointRadius)
const realnum standardBodyHeigh = 120.0;		// default height when walking

// min/max radius of ground touch points
const realnum minFootTouchPointRadius = 220.0;
const realnum maxFootTouchPointRadius = 340.0;
const realnum sleepingFootTouchPointRadius = 305;

// max acceleration
const realnum maxAcceleration = 70.0; 		     // [mm/s^2]

// maximum speed
const realnum maxSpeed = 130.0; 				// [mm/s]

// possible acceleration of angular speed
const realnum maxAngularSpeedAcceleration = 0.6; //  [rad/s^2]
const realnum maxAngularSpeed = maxSpeed/sleepingFootTouchPointRadius; //  [rad/s]

// walking direction cannot be changed immediately but with that angular speed per speed*t
const realnum maxAngularSpeedPerSpeed = 0.8;	 // [rad/s /  (mm/s)] = [rad/mm]

// limits of gait frequency (for beautiness of the gait mainly)
const realnum minGaitFrequency = 0.2; 				// [Hz]
const realnum maxGaitFrequency = 3.0; 				// [Hz]

// maximum speed of virtual foot ref point
const realnum maxGaitRefPointSpeed = 50.0; 			// [mm/s}

// maximum speed of a foot that is in the air during a gait
const realnum maxFootSpeed = 500; 					// [mm/s]

// maximum speed of a foot that is in the air during a gait
const realnum maxStartupAngleSpeed = 0.3; 			// [RAD/s]


// below that distance a toe is already moving with the ground
// (prevents that a movement is blocked due to the weight of the bot, such that a leg touches the ground before its computed touch point)
const realnum moveWithGroundBelowThisGroundDistance = 10.0; // [mm]

// Typically, the top point of the knee in one gait is in the
// middle of the touch point and the point when the toe leaves the ground.
// This has the consequence, that during the touch point the leg is bent against the walking direction
// which leads to a slight breaking effect that reduces the smoothness of a gait.
// The following factor allows to move this knee-zenit-point towards the walking direction
const realnum kneeZenitPointOffset = 0.5;			// [0.0..1.0]
const realnum kneeZenitPointFactor = 1.0;					// [0..1]


#endif /* SETUP_H_ */
