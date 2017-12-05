/*
 * BodyKinematics.cpp
 *
 *  Created on: 12.04.2017
 *      Author: JochenAlt
 */

#include "DenavitHardenbergParam.h"
#include "BodyKinematics.h"
#include "core.h"
#include "Engine.h"

BodyKinematics::BodyKinematics() {
}

void BodyKinematics::setup(Engine& pMainController) {
	mainController = &pMainController;
	// set a default position of te belly button
	currentBellyPose.position = Point(3, {0.0,0.0,140.0});

	// define default tranformation matrix to belly button (will be turned by body position and orientation later on)
	origin2Belly = HomMatrix(4,4,
				{ 1, 	0,  	0,  	0,
				  0, 	1, 		0,	 	0,
				  0,	0,		1,		0,
				  0,	0,		0,		1});

	// null out the leg orientation compensation of the ground distance
	for (int i = 0;i<NumberOfLegs;i++)
		footAngle[i] = 0.0;

	// looking direction in in z-axis (3rd leg is at the front)
	noseOrientation = 0;

	DenavitHardenbergParams dh(DenavitHardenbergParams::THETA,0,0,cos(radians(CAD::HipNickAngle))*(CAD::HipLength + CAD::HipCentreDistance),0.0);

	// arrange the legs in a circle of equal angles
	// if the number of legs is odd, have the middle leg at the front.
	realnum startAngle;
	if (NumberOfLegs % 2 == 0)
		startAngle = 360.0/NumberOfLegs*(float(NumberOfLegs/2)-0.5);
	else
		startAngle = 360.0/NumberOfLegs*(float(NumberOfLegs/2));

	// order of legs is starting from left side last leg, turning clockwise
	for (int i = 0;i<NumberOfLegs;i++) {
		hipPoseWorld[i].orientation.z = radians(startAngle);
		startAngle -= 360.0/NumberOfLegs;

		hipPoseWorld[i].position.z = cos(radians(CAD::HipNickAngle))*(CAD::HipLength + CAD::HipCentreDistance);
		legKinematic[i].setupLegPosition(dh, hipPoseWorld[i].orientation.z);
		legKinematic[i].setup(pMainController);
	}

	duringStartup = true;
}


// called during startup. Compute forward kinematics out of the angles
void BodyKinematics::computeForwardKinematics(const LegAnglesType& allLegsAngles, const Rotation &IMUorientation, PentaPointType& footPoints, Pose& bodyPose) {
	// get orientation from IMU
	currentBellyPose.orientation = IMUorientation;

	// but guess the lowest position (we assume the bot is on the ground)
	currentBellyPose.position.z = minBodyHeight;
	setBodyPose(currentBellyPose);
	bodyPose = currentBellyPose;
	for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
		LegKinematics& kin = legKinematic[legNo];
		LegPose legPose;
		legPose.angles = allLegsAngles[legNo];
		kin.computeForwardKinematics(legPose);
		Point footPoint = kin.convertHipToBodyCoord(legPose.position);
		footPoints[legNo] = footPoint;
	}
	realnum avrZ = 0;
	for (int legNo = 0;legNo< NumberOfLegs;legNo++)
		avrZ += footPoints[legNo].z;

	avrZ /= NumberOfLegs;
	for (int legNo = 0;legNo< NumberOfLegs;legNo++)
		footPoints[legNo].z -= avrZ;

	bodyPose.position.z -= avrZ;

	// ToDO: take orientation out of IMU value
}

void BodyKinematics::setBodyPose(const Pose& bellyPose) {
	currentBellyPose = bellyPose;

	// translate from origin to the belly button
	HomMatrix current = HomMatrix(4,4,
					{ 1, 	0,  	0,  	bellyPose.position.x,
					  0, 	1, 		0,	 	bellyPose.position.y,
					  0,	0,		1,		bellyPose.position.z,
					  0,	0,		0,		1});

	// transform body centre to rotated body centre
	HomMatrix rotateBody;
	createRotationMatrix(bellyPose.orientation, rotateBody);
	current *= rotateBody;

	for (int i = 0;i<NumberOfLegs;i++) {
		LegKinematics& kin = legKinematic[i];
		kin.setBodyTransformation(current); // compute transformation matrix per hip
	}
}


// kinematics out of body pose and toe points
bool BodyKinematics::computeKinematics(
		const Pose& bellyPose, const PentaPointType& toeWorld, // in params
		const PentaPointType& walkingTouchPoint,
		PentaPointType& hipsWorld, LegAnglesType& legAngles, PentaPointType& groundWorld) {

	setBodyPose(bellyPose);

	bool totalOk = true;
	for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
		Point toePointWorld = toeWorld[legNo];
		LegKinematics& kin = legKinematic[legNo];

		// compute hips in world coordinates
		hipsWorld[legNo] = kin.getHipPoseWorld();

		// compute foot touch point from hips perspective
		LegPose toeHipCoord;
		toeHipCoord.position = kin.convertWorldToHipCoord(toePointWorld);
		toeHipCoord.angles = legAngles[legNo];
		Point walkingTouchPointHipCoord = kin.convertWorldToHipCoord(walkingTouchPoint[legNo]);

		// compute inverse kinematics per leg
		bool ok;

		// compute angle 0
		if (duringStartup) {
			// during startup phase control that care that angle acceleration is not exceeded
			// since we might be outside the area of angle0
			realnum givenAngle0 = legAngles[legNo][0];
			realnum toBeAngle0 = atan2(toeHipCoord.position[Y], toeHipCoord.position[X])*0.4;
			if (legAngles[legNo].isNull()) {
				givenAngle0 = toBeAngle0;
			}

			// during startup reduce angle speed
			realnum maxAngleDiff = maxStartupAngleSpeed * (CORTEX_SAMPLE_RATE/1000.0) ;
			realnum  angleDiff = toBeAngle0 - givenAngle0;
			if (abs(angleDiff) > maxAngleDiff)
				angleDiff = sgn(angleDiff)* maxAngleDiff;
			realnum angle0 = givenAngle0 + angleDiff;
			ok = kin.computeInverseKinematics(toeHipCoord, angle0);
			if (!ok) {
				// ok, did not work, so try the original angle.
				ok = kin.computeInverseKinematics(toeHipCoord, givenAngle0);
				if (!ok) {
					ROS_ERROR_STREAM("kinematics of leg " << legNo << " with " << toeHipCoord << " and " << givenAngle0 << "/" << toBeAngle0 << "/" << angleDiff << " during startup could not be found");
				}
			}
		} else {
			// during normal walking the knee should be in a position such that
			// when the leg goes down it touchs the ground as perpendicular as possible
			// in order to have less walking by unstiff motor and legs3

			// walkingTouchPointHipCoord is the position right above the point
			// where the leg will touch the ground. Compute the to-be knee position

			realnum toBeAngle0;
			// that is the middle point of the toePoint and this point
			Point knee = toeHipCoord.position*(1.0-kneeZenitPointOffset) +  walkingTouchPointHipCoord*kneeZenitPointOffset;
			toBeAngle0 = atan2(knee.y, knee.x) * kneeZenitPointFactor;

			// take the dampener into account which is not a point but a fat toe with a significant diameter
			// @TODO check if this is working with terrain mode
			// toeHipCoord.position.z += getFatFootCorrectionHeight(legNo);

			ok = kin.computeInverseKinematics(toeHipCoord,toBeAngle0);
			if (!ok) {
				ROS_ERROR_STREAM("kinematics of leg " << legNo << " with toe " << toeHipCoord << " could not be found");
			}
		}

		// check
		LegPose toeCopy (toeHipCoord);
		toeCopy.position.null();
		kin.computeForwardKinematics(toeCopy);

		// compute angles of the leg from hips perspective
		if (ok) {
			// return the angles of that leg
			legAngles[legNo] = toeHipCoord.angles;

			LegPose ftp;
			ftp.angles = toeHipCoord.angles;
			ftp.position = toeWorld[legNo];

			// compute ground height correction induced by sensor value
			Point groundPointBody;
			footAngle[legNo]= kin.computeFootAngle(ftp, groundPointBody);
			groundWorld[legNo] = groundPointBody.getRotatedAroundZ(noseOrientation);
		}
		totalOk = totalOk && ok;
	}

	return totalOk;
}


const PentaPoseType& BodyKinematics::getHipPose() {
	return hipPoseWorld;
}

realnum BodyKinematics::getFatFootCorrectionHeight(int legNo) const {
	// this is not really correct but an approximation that is mostly ok
	return sin(abs(footAngle[legNo])) * CAD::FootDampenerDiameter/2.0;
}

