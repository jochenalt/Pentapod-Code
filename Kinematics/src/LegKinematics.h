/*
 * LegKinematics.h
 *
 * This is the tough one. Computes forward and inverse kinematics of a leg.
 * By reading code only, this is hard to understand.
 * Try kinematics.xlsx or the kinematics documentation if you need to understand what is behind the code
 *
 * Author: JochenAlt
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "basics/spatial.h"
#include "DenavitHardenbergParam.h"
#include "core.h"


class Engine; // required to resolve a cyclic dependency. The engine contains 5 instances of LegKinematics
/**
 * Leg kinematics can compute the kinematics of a single leg. It works in the coordinate
 * system which origin is in the hip. The joints are called hip/thigh/knee/foot.
 *
 *   side view
 *              O Foot
 *             / \
 *       Knee O   \
 *  Hip      /     \
 *   O------O       \
 *        Thigh      \
 *                    X  Toe Point
 *
 *
 */
class LegKinematics {
	friend class BodyKinematics;
public:
	LegKinematics();
	virtual ~LegKinematics(){};

	Point getDefaultToePoint();

	// call me upfront
	void setup(Engine&);

	// compute a pose out of joint angles
	void computeForwardKinematics(LegPose& pose);

	// inverse kinematics, compute angles out of pose with given angle0
	bool computeInverseKinematics(LegPose& pose, realnum angle0);

	// inverse kinematics, compute angles out of pose
	bool computeInverseKinematics(LegPose& pose);

	// compute the distance to the ground depending on the measurement of the distance sensor in the foot.
	// Compensate the orientation of the foot
	// realnum computeRealGroundDistance(Pose& pose, realnum measuredGroundDistance,  Point &measuredPoint );
	realnum computeFootAngle(const LegPose& pose, Point &measuredPoint );

	// check if inverse kinematics is withing boundaries
	bool isInBoundaries(const LimbAngles& angles, int& actuatorOutOfBounds);

	// hip offset is the angle added to the hip angle used to change gait formation
	// (especially going from 5-leg walk to 4-leg walk. In that case, the 4 legs are
	// walking shaped in a square instead of a 5-sided polygon
	void setHipOffset(realnum newHipOffset) { hipOffset = newHipOffset; }

	float performanceTest();
	bool selftest();
private:
	// compute euler angles out of transformation matrix
	void computeEulerAngles(const HomMatrix &current, realnum &alpha, realnum &beta, realnum &gamma);
	// compute full leg pose and show off the resulting transformation matrix
	void computeForwardKinematics(LegPose& pose, HomMatrix &current);

	void setupLegPosition(const DenavitHardenbergParams& dh, realnum pTheta);
	void setBodyTransformation(const HomMatrix& origin2Body);
	Point convertWorldToHipCoord(const Point& footPoseWorld);
	Point convertHipToBodyCoord(const Point& hipCoord);
	Point convertBodyToWorldCoord(const Point& body);

	Point getHipPoseWorld();

	// compute the inverse of a transformation matrix.
	void computeInverseTransformationMatrix(HomMatrix m, HomMatrix& inv);

	// the foot is not a point but has dampener with diameter of 20mm. When the foot
	// touches the ground at a certain angle, we need to virtually adapt the ground height
	// in order to compensate this.
	realnum getFatFootCorrectionHeight(const LegPose &ftp) ;

	DenavitHardenbergParams origin2Hip;					// DH from origin to Hip
	HomMatrix origin2HipTransformation;					// transformation from body to Hip

	HomMatrix origin2PosedHipTransformation;			// vector transformation from origin to hip which (including body pose)
	HomMatrix origin2PosedHipTransformationInv;			// coord transformation from origin to hip which (including body pose)

	DenavitHardenbergParams DHParams[NumberOfLimbs]; 	// DH params of leg

	realnum hipOffset;

	const Engine *mainController;
};


#endif /* KINEMATICS_H_ */
