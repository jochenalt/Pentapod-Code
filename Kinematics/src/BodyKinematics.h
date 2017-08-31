/*
 * BodyKinematics.h
 *
 *  Created on: 12.04.2017
 *      Author: JochenAlt
 */

#ifndef BODYKINEMATICS_H_
#define BODYKINEMATICS_H_

#include "spatial.h"
#include "LegKinematics.h"
#include "core.h"

class Engine;

/*
 * BodyKinematics computes the angles of all legs depending on a given
 * looking-direction and belly pose. The legs are arranged in this manner:
 *
 *
 *   1.leg      o 2. leg                              y
 *   o         /                                      ^
 *    \       /                                       |     body coordinate system
 *     \   --/                                        |
 *      \/    \                                       |
 *      |   C  |------o 3. leg / front leg           zC------> x
 *      /\    /
 *     /   --\
 *    /       \
 *   o         \
 *  5. leg      o  4. leg
 *
 * All points and poses are given in the "belly coordinate system" which origin is the ground right below the belly button.
 * The body kinematics is not aware of topics like gait, speed, or absolute coordinates when moving, it can just deliver the angles
 * of legs and poses of the hips.
 */

class BodyKinematics {
public:
	BodyKinematics();
	virtual ~BodyKinematics() {};

	// call me upfront before to anything
	void setup(Engine& pMainController);

	void computeForwardKinematics(const LegAnglesType& allLegsAngles, const Rotation &IMUorientation, PentaPointType& footPoints, Pose& bodyPose);

	// compute the kinematics of all legs and return poses of hips, angles, and ground points
	bool computeKinematics(const Pose& bellyPose, const PentaPointType& footTouchPoint, PentaPointType& hipsWorld, LegAnglesType& legAngles, PentaPointType& groundWorld);

	void setBodyPose(const Pose& bellyPose);

	// compute the body pose by given foot points (used during start up)
	void computeBodyPoseOutOfFootPoints(const PentaPointType& footPoints);

	// return current pose of the body
	const Pose& getCurrentBellyPose() const { return currentBellyPose; };

	// return poses of all hips in world coordinates
	const PentaPoseType&  getHipPose();

	// return kinematics of one leg
	LegKinematics& getLeg(int legNo) { return legKinematic[legNo]; };

	// return the ground distance correction of one leg. Returns a factor which is used to compute the height of a feet when
	// multiplied with the measured distance
	angle_deg getFootAngle(int legNo) { return footAngle[legNo]; };

	// the foot is not a point but has dampener with diameter of 20mm. When the foot
	// touches the ground at a certain angle, we need to virtually adapt the ground height
	// in order to compensate this.
	realnum getFatFootCorrectionHeight(int legNo) const ;

	// the nose-orientation defines the direction the bot is looking to
	// (not necessarily the same like the walking direction)
	angle_rad getCurrentNoseOrientation() { return  noseOrientation; };
	void setCurrentNoseOrientation(angle_rad newNoseDirection) { noseOrientation = newNoseDirection; };

	// during startup phase we move slowly and try to cope with invalid position kinematicswise
	void startupPhase(bool onOff) { duringStartup = onOff; };

private:
	// kinematic computation per leg
	LegKinematics legKinematic[NumberOfLegs];

	// hip pose in world coordinates
	PentaPoseType hipPoseWorld;

	// transformation matrix from origin to belly button pose
	HomMatrix origin2Belly;

	// pose of the belly within the body
	Pose currentBellyPose;

	// the bot's nose goes to that direction
	angle_rad noseOrientation;

	// angle used to compensate the non-perpendicular orientation of a leg.
	angle_deg footAngle[NumberOfLegs];

	// my main controller
	Engine *mainController;

	// during startup we ave a different hip offsetAngle
	bool duringStartup = false;
};

#endif /* BODYKINEMATICS_H_ */
