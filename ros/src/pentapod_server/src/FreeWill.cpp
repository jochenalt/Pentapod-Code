/*
 * FreeWill.cpp
 *
 *  Created on: Dec 12, 2017
 *      Author: JochenAlt
 */

#include "FreeWill.h"

FreeWill::FreeWill() {

}


void FreeWill::setup(const Map& newSlamMap, const Map& newGlobalCostMap, const Map& newLocalCostMap, const Pose& newOdomFrame, const Pose& newPose, const EngineState& state) {
	slamMap =  (Map*)&newSlamMap;
	localCostMap =   (Map*)&newLocalCostMap;
	globalCostMap=  (Map*)&newGlobalCostMap;
	odomFrame = (Pose*) &newOdomFrame;
	pose = (Pose*) &newOdomFrame;
	engineState = (EngineState*) &state;
}


Pose FreeWill::getAutonomousBodyPose() {
	const realnum standUpRightScariness = 0.3;
	const realnum sitDownScariness = 0.2;

	Pose result = engineState->currentBodyPose;
	if (engineState->currentScaryness > standUpRightScariness) {
		result.position.z = uprightBodyHeigh;
	}
	if (engineState->currentScaryness < sitDownScariness) {
		result.position.z = standardBodyHeigh;
	}
	return result;
}
