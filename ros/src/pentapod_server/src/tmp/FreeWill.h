/*
 * FreeWill.h
 *
 * Class implementing the free will, i.e.
 * - The body pose is adapted according to the distance to any wall
 *
 *  Created on: Dec 12, 2017
 *      Author: JochenAlt
 */

#ifndef PENTAPOD_SERVER_SRC_FREEWILL_H_
#define PENTAPOD_SERVER_SRC_FREEWILL_H_

#include <stdio.h>
#include <string>
#include "setup.h"
#include <Map.h>
#include <EngineState.h>

class FreeWill {
public:
	FreeWill();

	// tell me all structures I need. These parameters are not copied but referred internally.
	void setup(const EngineState& state);

	// compute the body pose depending on the distance to walls: When a wall gets close, the body pose goes to upright position. In the free space, body pose is in standard position
	Pose getAutonomousBodyPose();

	// turn on/off the free will
	void turnAutonomousMode(bool ok) { turnOn  = ok; };

	// check if a new goal has to be identified, compute it, and publish it to navigation stack
	void publish();

	static FreeWill& getInstance() { static FreeWill will; return will; };

private:
	// compute the next navigation goal depending on current position and history of visits
	Pose computeNavigationGoal();


	const Map* slamMap = NULL;
	const Map* globalCostMap= NULL;
	const Map* localCostMap= NULL;
	const Pose* odomFrame = NULL;
	const Pose* pose = NULL;
	const EngineState* engineState= NULL;
	bool turnOn = false;
	bool latchGoalReachable = false;
	vector<StampedPose> lastVisits;
};

#endif /* PENTAPOD_SERVER_SRC_FREEWILL_H_ */
