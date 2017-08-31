/*
 * EngineState.h
 *
 * Represents the state of the pentapod that allows to display it in an UI, i.e. odometry, all positions and angles
 *
 *  Created on: 07.05.2017
 *      Author: JochenAlt
 */

#ifndef ENGINESTATE_H_
#define ENGINESTATE_H_

#include "basics/stringhelper.h"

#include "setup.h"
#include "spatial.h"
#include "GaitController.h"

class EngineState {
public:
	EngineState () {
		isTurnedOn = false;
		currentSpeed = 0;
		currentAngularSpeed = 0;
		currentWalkingDirection = 0;
		currentNoseOrientation = 0;
		currentGaitMode = None;
		engineMode = BeingAsleep;
		for (int i = 0;i < NumberOfLegs;i++) {
			footAngle[i] = 0;
			footOnGroundFlag[i] = 0;
		}
	};

	~EngineState () {};

	void operator=(const EngineState& state) {
		currentBodyPose = state.currentBodyPose;
		legAngles = state.legAngles;
		frontLegPose = state.frontLegPose;
		isTurnedOn = state.isTurnedOn;
		currentSpeed = state.currentSpeed;
		currentAngularSpeed = state.currentAngularSpeed;
		currentWalkingDirection = state.currentWalkingDirection;
		currentNoseOrientation = state.currentNoseOrientation;
		currentOdomPose = state.currentOdomPose;
		currentFusedPose = state.currentFusedPose;
		currentMapPose = state.currentMapPose;
		currentGaitMode = state.currentGaitMode;
		engineMode = state.engineMode;
		currentGaitRefPoints = state.currentGaitRefPoints;
		for (int i = 0;i < NumberOfLegs;i++) {
			legPhase[i] = state.legPhase[i];
			footAngle[i] = state.footAngle[i];
			footOnGroundFlag[i] = state.footOnGroundFlag[i];
		}

		// this is used for helper-indications in the UI
		groundPoints = state.groundPoints;
		hipPoseWorld = state.hipPoseWorld;
		toePointsWorld = state.toePointsWorld;
	}
	bool operator!=(const EngineState& state) {
		return !((*this) ==  state);
	}
	bool operator==(const EngineState& state) {
		return ((currentBodyPose == state.currentBodyPose) &&
				(legAngles == state.legAngles) &&
				(frontLegPose == state.frontLegPose) &&
				(isTurnedOn == state.isTurnedOn) &&
				(currentSpeed == state.currentSpeed) &&
				(currentAngularSpeed == state.currentAngularSpeed) &&
				(currentWalkingDirection == state.currentWalkingDirection) &&
				(currentNoseOrientation == state.currentNoseOrientation) &&
				(currentOdomPose == state.currentOdomPose) &&
				(currentGaitMode == state.currentGaitMode) &&
				(toePointsWorld == state.toePointsWorld) &&
				(engineMode == state.engineMode) &&
				(currentGaitRefPoints == state.currentGaitRefPoints) &&
				(groundPoints == state.groundPoints));
	}
	Pose currentBodyPose;
	LegAnglesType legAngles;
	LegPose frontLegPose;
	bool isTurnedOn;
	mmPerSecond currentSpeed;
	realnum currentAngularSpeed;
	realnum currentWalkingDirection;
	angle_rad currentNoseOrientation;
	Pose currentOdomPose;
	Pose currentFusedPose;
	Pose currentMapPose;
	GaitModeType currentGaitMode;
	GeneralEngineModeType engineMode;
	FootOnGroundFlagType footOnGroundFlag;
	PentaPointType currentGaitRefPoints;
	LegGaitPhase legPhase[NumberOfLegs];
	realnum footAngle[NumberOfLegs];
	PentaPointType toePointsWorld;

	// this is used for helper-indications in the UI
	PentaPointType groundPoints;
	PentaPoseType hipPoseWorld;

	std::ostream& serialize(std::ostream &out) const;
	std::istream& deserialize(std::istream &in);
};
#endif