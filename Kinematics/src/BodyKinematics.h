/*
 * BodyKinematics.h
 *
 *  Created on: 12.04.2017
 *      Author: JochenAlt
 */

#ifndef BODYKINEMATICS_H_
#define BODYKINEMATICS_H_

#include "basics/spatial.h"
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
	BodyKinematics() {};
	~BodyKinematics() {};

	// call me upfront before to anything
	void setup(Engine& pMainController);

	// compute bodyPose and toe points out of leg angles and IMU orientation
	void computeForwardKinematics(const PentaLegAngleType& allLegsAngles, const Rotation &IMUorientation, PentaPointType& toePoints, Pose& bodyPose);

	// compute poses of hips, angles, and ground points out of toe points
	bool computeKinematics(const Pose& bellyPose, const PentaPointType& toePoints, const PentaPointType& walkingTouchPoint, PentaPointType& hipsWorld, PentaLegAngleType& legAngles, PentaPointType& groundWorld);

	// body pose is is the relative position of the belly above the origin right below the bot
	void setBodyPose(const Pose& bellyPose);

	// return current pose of the body
	const Pose& getCurrentBellyPose() const { return currentBellyPose; };

	// return poses of all hips in world coordinates
	const PentaPoseType&  getHipPose();

	// return kinematics of one leg
	LegKinematics& getLeg(int legNo) { return legKinematic[legNo]; };

	// return the angle of a foot wrt to the z-axis.
	angle_deg getFootsDeviationAngleFromZ(int legNo) { return angleDeviationFromZ[legNo]; };

	// a toe is not a point but has dampener with a diameter of 20mm. When the foot
	// touches the ground at a certain angle, we need to virtually adapt the ground height
	// in order to compensate this.
	// Returns the factor we have to correct the height due to that situation, which is sin(angle)*radius of dampener
	realnum getFatFootCorrectionHeight(int legNo) const ;

	// the nose-orientation defines the direction the bot's noe (i.e. its front leg) is looking to
	// (not necessarily the same like the walking direction)
	angle_rad& getCurrentNoseOrientation() { return  noseOrientation; };

	// during startup phase we move slowly and try to cope with invalid positions and singularities
	void startupPhase(bool onOff) { duringStartup = onOff; };

private:
	// kinematic computation per leg
	LegKinematics legKinematic[NumberOfLegs];

	// hip pose in world coordinates
	PentaPoseType hipPoseWorld;

	// transformation matrix from origin to belly button pose
	HomogeneousMatrix origin2Belly;

	// pose of the belly within the body
	Pose currentBellyPose;

	// the bot's nose goes to that direction
	angle_rad noseOrientation;

	// angle used to compensate the non-perpendicular orientation of a leg.
	angle_deg angleDeviationFromZ[NumberOfLegs];

	// my main controller
	Engine *mainController;

	// during startup we compensate weired leg angles
	bool duringStartup = false;
};

#endif /* BODYKINEMATICS_H_ */
