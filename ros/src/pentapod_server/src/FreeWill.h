/*
 * FreeWill.h
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

	void setup(const Map& slamMap, const Map& globalCostMap, const Map& localCostMap, const Pose& odomFrame, const Pose& pose, const EngineState& state);
	Pose getAutonomousBodyPose();

private:

	const Map* slamMap = NULL;
	const Map* globalCostMap= NULL;
	const Map* localCostMap= NULL;
	const Pose* odomFrame= NULL;
	const Pose* pose= NULL;
	const EngineState* engineState= NULL;

};

#endif /* PENTAPOD_SERVER_SRC_FREEWILL_H_ */
