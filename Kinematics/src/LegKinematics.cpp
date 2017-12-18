#include <exception>

#include "core.h"
#include "basics/logger.h"
#include "basics/util.h"
#include "basics/types.h"

#include "setup.h"
#include "LegKinematics.h"


LegKinematics::LegKinematics() {}

// setup denavit hardenberg parameters and set the
// rotation matrixes of the gripper to view coord.
void LegKinematics::setup(Engine& pMainController) {
	mainController = &pMainController;

	// define and compute Denavit Hardenberg Parameter
	// check Kinematics.xls for explantation
	DHParams[LimbConfiguration::HIP] =		DenavitHardenbergParams(DenavitHardenbergParams::TURN_AROUND_Z, radians(90.0), 0, CAD::HipJointLength, 				0);
	DHParams[LimbConfiguration::THIGH] = 	DenavitHardenbergParams(DenavitHardenbergParams::TURN_AROUND_Z, radians(90.0), 0, CAD::ThighLength+CAD::ThighKneeGapLength + CAD::KneeJointLength,	0);
	DHParams[LimbConfiguration::KNEE] =		DenavitHardenbergParams(DenavitHardenbergParams::TURN_AROUND_X, radians(90.0), 0, 0, 0);
	DHParams[LimbConfiguration::LOWERLEG] =	DenavitHardenbergParams(DenavitHardenbergParams::TURN_AROUND_Z, 0, 0, CAD::FootLength + CAD::DampenerLength ,0);
	hipOffset = 0;
}

Point LegKinematics::getDefaultToePoint() {
	LegPose pose;
	pose.angles = LimbAngles::getDefaultPosition();
	computeForwardKinematics(pose);
	return pose.position;
}

// use DenavitHardenberg parameter and compute the Dh-Transformation matrix with a given joint angle (theta)
void LegKinematics::setupLegPosition(const DenavitHardenbergParams& dh, realnum pTheta) {

	HomogeneousMatrix origin2Hip;

	dh.computeDHMatrix(pTheta,origin2Hip);

	HomogeneousMatrix nick;
	Rotation r(0, radians(CAD::HipNickAngle),0);
	createRotationMatrix(r, nick);

	origin2HipTransformation = origin2Hip*nick;
}

void LegKinematics::setBodyTransformation(const HomogeneousMatrix& origin2Body) {
	origin2PosedHipTransformation = origin2Body * origin2HipTransformation ;

	// now create transformation to move coordinate system instead of points
	computeInverseTransformationMatrix(origin2PosedHipTransformation,origin2PosedHipTransformationInv);
}


void LegKinematics::computeInverseTransformationMatrix(HomogeneousMatrix m, HomogeneousMatrix& inverse) {
	inverse = HomogeneousMatrix(4,4,
			{ 	m[0][0], m[1][0], m[2][0],- m[0][0]*m[0][3] - m[1][0]*m[1][3] - m[2][0]*m[2][3],
				m[0][1], m[1][1], m[2][1],- m[0][1]*m[0][3] - m[1][1]*m[1][3] - m[2][1]*m[2][3],
				m[0][2], m[1][2], m[2][2],- m[0][2]*m[0][3] - m[1][2]*m[1][3] - m[2][2]*m[2][3],
				0,			0		,0		,	1});

}

Point LegKinematics::convertWorldToHipCoord(const Point& world) {

	HomogeneousVector worldHom = {
			world.x,
			world.y,
			world.z,
					1.0 };

	Point BodyCoord = origin2PosedHipTransformationInv * worldHom;
	return BodyCoord;
}

Point LegKinematics::convertHipToBodyCoord(const Point& hipCoord) {

	HomogeneousVector hipHom = {
			hipCoord.x,
			hipCoord.y,
			hipCoord.z,
					1.0 };

	Point worldCoord= origin2PosedHipTransformation* hipHom;
	return worldCoord;
}


Point LegKinematics::getHipPoseWorld() {
	return  { origin2PosedHipTransformation[0][3], origin2PosedHipTransformation[1][3], origin2PosedHipTransformation[2][3] };
}


void LegKinematics::computeForwardKinematics(LegPose& pose) {
	HomogeneousMatrix current;
	computeForwardKinematics(pose, current);
}

// compute forward kinematics, i.e. out of given joint angles compute the
// position and orientation of the toe
void LegKinematics::computeForwardKinematics(LegPose& pose, HomogeneousMatrix& current) {
	// convert angles to intern offsets where required (angle 1)
	realnum angle[NumberOfLimbs] = {
			pose.angles[0],pose.angles[1],pose.angles[2],pose.angles[3] };

	// compute final position by multiplying all DH transformation matrixes
	HomogeneousMatrix currDHMatrix;
	DHParams[LimbConfiguration::HIP].computeDHMatrix(angle[LimbConfiguration::HIP], current);

	DHParams[LimbConfiguration::THIGH].computeDHMatrix(angle[LimbConfiguration::THIGH], currDHMatrix);
	current *= currDHMatrix;

	DHParams[LimbConfiguration::KNEE].computeDHMatrix(angle[LimbConfiguration::KNEE], currDHMatrix);
	current *= currDHMatrix;

	DHParams[LimbConfiguration::LOWERLEG].computeDHMatrix(angle[LimbConfiguration::LOWERLEG], currDHMatrix);
	current *= currDHMatrix;

	// position of hand is given by last row of transformation matrix
	pose.position = current.column(3);
}


angle_deg LegKinematics::computeFootAngle(const LegPose& pose, Point &measuredPoint ) {

	realnum measuredGroundDistance = 100.0;
	realnum angle[NumberOfLimbs] = {
			pose.angles[0],pose.angles[1],pose.angles[2],pose.angles[3] };

	// start with transformation from belly button to hip

	HomogeneousMatrix current = origin2PosedHipTransformation;

	// compute final position by multiplying all DH transformation matrixes
	HomogeneousMatrix currDHMatrix;
	DHParams[LimbConfiguration::HIP].computeDHMatrix(angle[LimbConfiguration::HIP], currDHMatrix);

	current *= currDHMatrix;
	DHParams[LimbConfiguration::THIGH].computeDHMatrix(angle[LimbConfiguration::THIGH], currDHMatrix);
	current *= currDHMatrix;

	DHParams[LimbConfiguration::KNEE].computeDHMatrix(angle[LimbConfiguration::KNEE], currDHMatrix);
	current *= currDHMatrix;

	DHParams[LimbConfiguration::LOWERLEG].computeDHMatrix(angle[LimbConfiguration::LOWERLEG], currDHMatrix);

	// prolong the foot by the measured distance.
	DenavitHardenbergParams prologedFoot =	DenavitHardenbergParams(DenavitHardenbergParams::TURN_AROUND_Z, 0, 0, CAD::FootLength + CAD::DampenerLength + measuredGroundDistance,0);
	prologedFoot.computeDHMatrix(angle[LimbConfiguration::LOWERLEG], currDHMatrix);

	current *= currDHMatrix;

	// position of prolonged foot is given by last row of transformation matrix
	// measuredPoint is the point where the foot touches the ground considering
	// not the real foot length but the foot length plus the measured distance
	measuredPoint = current.column(3);

	// return the perpendicular height to the ground
	realnum c = measuredGroundDistance; // hypothenusis
	realnum a = sqrt(sqr(measuredPoint.x-pose.position.x) + sqr(measuredPoint.y-pose.position.y)); // distance current position|measured point

	return asin(a/c);
}


// compute euler angles out of rotation matrix that is included in transformation matrix current
// (check wikipedia)
void LegKinematics::computeEulerAngles(const HomogeneousMatrix &current, realnum &alpha, realnum &beta, realnum &gamma) {
	beta = atan2(-current[2][0], sqrt(current[0][0]*current[0][0] + current[1][0]*current[1][0]));
	gamma = 0;
	alpha = 0;
	if (almostEqual(beta, (M_PI/2.0), floatPrecision)) {
		alpha = 0;
		gamma = atan2(current[0][1], current[1][1]);
	} else {
		if (almostEqual(beta, -(M_PI/2.0),floatPrecision)) {
			alpha = 0;
			gamma = -atan2(current[0][1], current[1][1]);
		} else {
			alpha = atan2(current[1][0],current[0][0]);
			gamma = atan2(current[2][1], current[2][2]);
		}
	}
}


// true if solution is within min/max values per actuator
bool LegKinematics::isInBoundaries(const LimbAngles& angles, int& actuatorOutOfBounds) {
	bool ok = true;
	for (int i = 0;i<NumberOfLimbs;i++) {
		realnum angle = angles[i];
		if ((angle < (actuatorConfigType[i].minAngle-floatPrecision)) ||
			(angle > (actuatorConfigType[i].maxAngle+floatPrecision))) {
			actuatorOutOfBounds = i;
			ok = false;
		}
	}
	return ok;
}

// inverse kinematics, compute angles out of pose. Start with
// arbitrary computation of angle0 which can be set freely.
bool LegKinematics::computeInverseKinematics(LegPose& pose) {
	// arbitrary definition of bisectional angle to foot touch point
	realnum angle0;
	angle0 = arctanApprox(pose.position[Y] / pose.position[X])*0.3 + hipOffset;
	return computeInverseKinematics(pose, angle0);
}

// inverse kinematics with a given angle0
bool LegKinematics::computeInverseKinematics(LegPose& pose, realnum angle0) {
	realnum d0 = DHParams[LimbConfiguration::HIP].getXTranslation();
	realnum d1 = DHParams[LimbConfiguration::THIGH].getXTranslation();
	realnum d2 = DHParams[LimbConfiguration::KNEE].getXTranslation();
	realnum d3 = DHParams[LimbConfiguration::LOWERLEG].getXTranslation();

	// toe point
	Point& Toe = pose.position;
	realnum s0 = sin(angle0);
	realnum c0 = cos(angle0);

	// ***** compute angle1 *****
	// - consider triangle A B C
	// - compute height and touch point of height to c
	// - construct circle with centre H and radius of previous height
	// - intersect this circle with possible points of C(depending from angle1)
	// -> angle1
	Point Hip( d0*cos(angle0),
			 d0*sin(angle0),
			 DHParams[LimbConfiguration::HIP].getZTranslation());

	// length of H.ypothenuse of orthogonal triangle of knee/lower leg
	// (used later on to construct height to c)
	realnum a = sqrt(d3*d3+d2*d2); // length of H.ypothenusis of triangle d3, d2 and a
	realnum c = Toe.distance(Hip);

	// compute height of triangle FTP B H
	// (used as radius of circle)
	realnum height_c = triangleHeightToC (a,d1,c);
	realnum height_c_pow_2 = height_c*height_c;
	// compute length of FTP to H (pythagoras), equal to hc1
	// (used to construct H)
	realnum length_ToeH = sqrt(d3*d3+d2*d2 - height_c_pow_2);

	// construct H B.y starting from A and moving towards B with length length_ToeH
	realnum H_normate_length = (length_ToeH/c);
	Point H = Toe*(1.0-H_normate_length) + Hip*H_normate_length;

	// compute BH as normal of circle
	Point BH = H - Toe;

	// circle is defined as H + sin(alpha)*S + cos(alpha)*T with S and T orthogonal
	// vector to each other and to BH.
	realnum Sx = height_c/(sqrt( sqr(BH.x/BH.y) + 1.0));
	Point S(	Sx,
				sqrt(height_c_pow_2 - Sx*Sx),
				0 );

	if (sgn(BH.y) == sgn(BH.x))
		S.x = - S.x;

	// compute T orthogonal to S and BH
	Point T( 	S.y*BH.z,
				-S.x*BH.z,
				S.x*BH.y - S.y*BH.x);

	// normalize the length of T
	realnum len = T.length();
	if (len > floatPrecision)
		T *= height_c/len;

	// solve the circle equation for alpha B.y using sinusoid, returning two solutions
	realnum alpha1 = qnan;
	realnum alpha2 = qnan;
	bool chooseAngle1AsYouLike; // true if there is a infinite number of solutions, i.e. choose angle1 as you like
	solveTrgLinearCombinationWithEqualPhase(s0*S.x -c0*S.y, s0*T.x-c0*T.y, c0*H.y - s0*H.x, alpha1, alpha2, chooseAngle1AsYouLike);
	// solveTrgLinearCombinationWithEqualPhase(s0*S.x -c0*S.y, s0*T.x-c0*T.y, c0*H.y - s0*H.x, alpha1, chooseAngle1AsYouLike);

	realnum angle1_1 = qnan;
	realnum angle1_2 = qnan;
	realnum s1_1 = qnan; // = sin(angle1_1)
	realnum c1_1 = qnan; // = cos(angle1_1)
	realnum s1_2 = qnan; // = sin(angle1_2)
	realnum c1_2 = qnan; // = cos(angle1_2)
	if (!chooseAngle1AsYouLike) {
		// alpha is defined now, construct C in order to derive angle1 B.y transformation matrix
		// as long as the circle is not in the xy pane, it is sufficient to evaluate one coordinate only.
		// We take z.
		Point C1 = H + S*sin(alpha1) + T*cos(alpha1);
		s1_1 = C1.z / d1;		// coming from transformation matrix
		angle1_1 = asin(s1_1);
		c1_1 = cos(angle1_1);

		if (alpha2 != qnan) {
			Point C2 = H + S*sin(alpha2) + T*cos(alpha2);
			s1_2 = C2.z / d1;	// coming from transformation matrix
			angle1_2 = asin(s1_2);
			c1_2 = cos(angle1_2);
		}
		else {
			angle1_2 = angle1_1;
			s1_2 = s1_1;
			c1_2 = c1_1;
		}
	} else {
		// if we have infinite solutions, the knee is outstretched. Compute out of z coordinate of toe only
		s1_1 = Toe.z/(d1+d2+d3);
		angle1_1 = asin(s1_1);
		c1_1 = cos(angle1_1);

		angle1_2 = angle1_1;
		s1_2 = s1_1;
		c1_2 = c1_1;
	}


	// *** compute angle3 ***
	realnum b = d1;
	realnum angle3 =  0;

	// assume for angle 2 the current angle (which will become true when
	// the leg is outstretched, and the triangle a,b,c is a line)
	realnum s2_1 = sin(pose.angles[2] + M_PI/2.0);
	realnum s2_2 = s2_1;
	realnum s2_3 = s2_1;
	realnum s2_4 = s2_1;

	// compute via cosine law (2 solutions)
	if (c < a + b - floatPrecision)
		angle3 = M_PI - triangleGamma(a,b,c); // otherwise gamma remains 0, the triangle is no triangle
	else {
		// leg is outstretched, no triangle but a line only, so we are free to choose angle3
		// we take the same angle that we already have.
		angle3 = pose.angles[3];
	}

	realnum s3 = sin(angle3);
	realnum c3 = cos(angle3);

	// *** compute angle 2 via transformation matrix. We get two arcsin
	// solutions, which are combined with two solutions of angle 1 (ends up in 4 solutions)
	if (c1_1*s3 > floatPrecision)
		s2_1 = (Toe.z-d1*s1_1 - s1_1*d3*c3) / (-c1_1*d3*s3);

	// for numerical reasons s2_1 could be slightly above 1.0, limit that in order to call arcsin afterwards
	if (abs(s2_1) >= 1.0)
		s2_1 = sgn(s2_1);

	// compute angle2 with both solutions
	realnum angle2_1 = asin(s2_1);
	realnum angle2_2 = M_PI - angle2_1;
	realnum c2_1 = cos(angle2_1);
	realnum c2_2 = -c2_1;
	s2_2 = s2_1;


	// do the same for the second solution of angle1
	if (c1_2*s3 > floatPrecision)
		s2_3 = (Toe.z-d1*s1_2 - s1_2*d3*c3) / (-c1_2*d3*s3);
	if (abs(s2_3) >= 1.0)
		s2_3 = sgn(s2_3);
	realnum angle2_3 = asin(s2_3);
	realnum angle2_4 = M_PI - angle2_3;
	s2_4 = s2_3;
	realnum c2_3 = cos(angle2_3);
	realnum c2_4 = -c2_3;

	// collect all possible solutions and decide which one is to be used
	const int NumberOfSolutions = 4;
	LimbAngles solutions[NumberOfSolutions] =
		{ { angle0, angle1_1,  angle2_1 - (M_PI_2), angle3 },
		  { angle0, angle1_1,  angle2_2 - (M_PI_2), angle3 },
		  { angle0, angle1_2,  angle2_3 - (M_PI_2), angle3 },
		  { angle0, angle1_2,  angle2_4 - (M_PI_2), angle3 }};

	realnum solutionsSinCos[NumberOfSolutions][4] =
		{ { s1_1, c1_1, s2_1, c2_1},
		  { s1_1, c1_1, s2_2, c2_2},
		  { s1_2, c1_2, s2_3, c2_3},
		  { s1_2, c1_2, s2_4, c2_4}};

	// if we have several solutions, now choose one
	// we take the one which is valid and where the knee and thigh moves the least
	realnum bestSolutionAngleDistance;
	int bestSolutionIdx = -1; // -1 = no solution yet
	for (int i = 0;i<NumberOfSolutions;i++) {
		realnum s1 = solutionsSinCos[i][0];
		realnum c1 = solutionsSinCos[i][1];
		realnum s2 = solutionsSinCos[i][2];
		realnum c2 = solutionsSinCos[i][3];

		// compute toe point out of angles to check if this solution is valid
		Point checkToePoint(c0*c1*d3*c3 + ( s0*c2+c0*s1*s2) * d3*s3 + c0*d1*c1 + d0*c0,
							s0*c1*d3*c3 + (-c0*c2+s0*s1*s2) * d3*s3 + s0*d1*c1 + d0*s0,
							s1*d3*c3 - c1*s2*d3*s3 + d1*s1);

		int actuatorOutOfBounds;
		bool valid = 	almostEqual(checkToePoint, Toe, floatPrecisionSqrt) &&
						isInBoundaries(solutions[i], actuatorOutOfBounds);

		// if multiple solutions are valid, take the one with the least different to the current angle
		realnum angleDistance = sqr(solutions[i][0]-pose.angles[0]) + sqr(solutions[i][2]-pose.angles[2]) + sqr(solutions[i][3]-pose.angles[3]);
		if (valid &&
			((bestSolutionIdx == -1) ||
			 ((bestSolutionIdx != -1) && (angleDistance < bestSolutionAngleDistance)))) {
			bestSolutionIdx = i;
			bestSolutionAngleDistance = angleDistance;
		}
	}
	if (bestSolutionIdx == -1)
		return false;

	// set the paramter only when result is ok
	pose.angles = solutions[bestSolutionIdx];
	return true;
}


float LegKinematics::performanceTest() {
		LegPose p;
		p.angles[0] = radians(10);
		p.angles[1] = radians(12);
		p.angles[2] = radians(13);
		p.angles[3] = radians(14);

		computeForwardKinematics(p);

		uint32_t start = millis();
		int count = 100000;
		for (int i = 0;i<count;i++) {
			LegPose tmp = p;
			computeInverseKinematics(tmp);
		}
		uint32_t end = millis();
		float duration = float(end-start)/float(count)/1000.0;
		return duration;
}

bool LegKinematics::selftest() {
	realnum a0 = radians(-50);
	realnum a1 = radians(-50);
	realnum a2 = radians(-50);
	realnum a3 = radians(-50);
	/*
	Point t(167.66833802467517, -41.284647885139833, 	19.795718772657366);
	Pose tt;
	tt.position = t;
	ok = computeInverseKinematics(tt, -0.096540198738167715);

	Pose p;

	p.angles[0] = a0;
	p.angles[1] = a1;
	p.angles[2] = a2;
	p.angles[3] = a3;

	computeForwardKinematics(p);

	ok = computeInverseKinematics(p, a0);
	if (!ok ||
		(abs(a0 - p.angles[0]) > 0.1) ||
		(abs(a1 - p.angles[1]) > 0.1) ||
		(abs(a2 - p.angles[2]) > 0.1) ||
		(abs(a3 - p.angles[3]) > 0.1)) {
		LOG(ERROR) << setprecision(5) << "error a=(" << degrees(a0) << "," << degrees(a1) << "," << degrees(a2) << "," << degrees(a3) << ") p=(" << p.position.x << "," << p.position.y << "," << p.position.z << ") IK=(" << degrees(p.angles[0]) << "," << degrees(p.angles[1]) << "," << degrees(p.angles[2]) << "," << degrees(p.angles[3]) << ")";
	}

*/
	for ( a0 = radians(-60.0);a0< radians(60); a0 += radians(5.0)) {
		for ( a1 = radians(-60.0);a1< radians(60); a1 += radians(5.0)) {
			for ( a2 = radians(-60.0);a2< radians(60); a2 += radians(5.0)) {
				for (a3 = radians(-25);a3< radians(130); a3 += radians(5.0)) {
					{
						if (abs(a0) < floatPrecision)
							a0 = 0;
						if (abs(a1) < floatPrecision)
							a1 = 0;
						if (abs(a2) < floatPrecision)
							a2 = 0;
						if (abs(a3) < floatPrecision)
							a3 = 0;

						LegPose p;
						p.angles[0] = a0;
						p.angles[1] = a1;
						p.angles[2] = a2;
						p.angles[3] = a3;

						computeForwardKinematics(p);

						bool ok = computeInverseKinematics(p, a0);
						LegPose target;
						target.angles = p.angles;
						computeForwardKinematics(target);
						if (!ok || p.position.distance(target.position) > 0.1) {
							ROS_ERROR_STREAM(setprecision(5) << "error a=(" << degrees(a0) << "," << degrees(a1) << "," << degrees(a2) << "," << degrees(a3) << ") p=(" << p.position.x << "," << p.position.y << "," << p.position.z << ") IK=(" << degrees(p.angles[0]) << "," << degrees(p.angles[1]) << "," << degrees(p.angles[2]) << "," << degrees(p.angles[3]) << ")");
						}
						/*
							if (!ok ||
									(abs(a0 - p.angles[0]) > 0.1) ||
									(abs(a1 - p.angles[1]) > 0.1) ||
									(abs(a2 - p.angles[2]) > 0.1) ||
									(abs(a3 - p.angles[3]) > 0.1)) {
								LOG(ERROR) << setprecision(5) << "error a=(" << degrees(a0) << "," << degrees(a1) << "," << degrees(a2) << "," << degrees(a3) << ") p=(" << p.position.x << "," << p.position.y << "," << p.position.z << ") IK=(" << degrees(p.angles[0]) << "," << degrees(p.angles[1]) << "," << degrees(p.angles[2]) << "," << degrees(p.angles[3]) << ")";
								// return false;
							};
							*/
					}
				}
			}
		}
	}
	return true;
}


