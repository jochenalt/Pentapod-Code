/*
 * GaitControl.cpp
 *
 *  Created on: 15.04.2017
 */

#include "core.h"
#include "GaitController.h"
#include "BodyKinematics.h"
#include "Engine.h"


GaitController::GaitController() {
}

void GaitController::setup(Engine& pMainController) {

	mainController = &pMainController;
	targetGaitType = TwoLegsInTheAir;

	for (int i = 0;i<NumberOfLegs;i++) {
		lastPhase[i] = LegGaitDuty; 				// start with stay-on-the-ground-phase
		currentGroundPercentage[i] = getFootOnTheGroundRatio(0.0);
		feetOnGround[i] = true;
		perpendicularGroundHeight[i] = 0; // distance sensors havent delivery anything yet
	}

	currentAbsWalkingDirection = 0;
	currentAngularSpeed = 0;
	currentSpeed = 0;	// initial speed is zero
	globalGaitBeat = 0;
	includeFrontLeg = true; // include front leg into the gait movement
	adaptToGaitRefPointType = ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE; 		// stop gait when not moving but perform gait when ref points differ a lot from toe position

	gaitRefPointRadius = 300;

	setTargetGaitRefPointsRadius(gaitRefPointRadius, 0,0); // happens in addition to ctor to make setup idempotent when changing basic parameters
	toePoints = currentGaitRefPoints;
	for (int i = 0;i<NumberOfLegs;i++) {
		lastPhasePositions[i] = toePoints[i];
	}

	currentWalkingTouchPoints = targetGaitRefPoints;

	// during setup, ignore the moderated movement towards the target foot ref points
	currentGaitRefPoints = targetGaitRefPoints;
	setTargetGaitMode(GaitModeType(TwoLegsInTheAir));

	globalGaitBeat = 0;
	gaitHeight = 0; // will be overwritten in MainController shortly

	// start with automated gait selection
	targetGaitType = TwoLegsInTheAir;
	fastestFootSpeed = 0;

	frontLegPosition = Point(280,1,140);

	// initialize the leg's gaits (used for sensor computation)
	for (int legNo = 0;legNo<NumberOfLegs;legNo++)
		legPhase[legNo] = LegGaitDuty;
}

void GaitController::setFrontLegWorld(const Point& x) {
	frontLegPosition = x.getRotatedAroundZ(- mainController->getBodyKinematics().getCurrentNoseOrientation());
};

Point GaitController::getFrontLegWorld() {
	return frontLegPosition.getRotatedAroundZ(mainController->getBodyKinematics().getCurrentNoseOrientation());
};

void GaitController::setTargetGaitRefPoint(int legNo, const Point& newGaitRefPoint) {
	targetGaitRefPoints[legNo] = newGaitRefPoint;
}


void GaitController::setTargetGaitRefPointsRadius (realnum radius, realnum spiderModeRatio, realnum fourLegsModeRatio) {
	gaitRefPointRadius = radius;
	realnum startAngle;
	if (NumberOfLegs % 2 == 0)
		startAngle = 360.0/NumberOfLegs*(float(NumberOfLegs/2)-0.5);
	else
		startAngle = 360.0/NumberOfLegs*(float(NumberOfLegs/2));

	realnum angleAdaption4LegsMode = 0;
	realnum angleAdaptionSpiderMode = 0;

	for (int i = 0;i< NumberOfLegs;i++) {
		angleAdaption4LegsMode = 0;

		// in case of 4-legs walk sort out legs in a square
		if (i == 0)
			angleAdaption4LegsMode = -(360.0/4 - 360.0/NumberOfLegs);
		if (i == 1)
			angleAdaption4LegsMode = -(360.0/4 - 360.0/NumberOfLegs);
		if (i == NumberOfLegs -2)
			angleAdaption4LegsMode = (360.0/4 - 360.0/NumberOfLegs);
		if (i == NumberOfLegs -1)
			angleAdaption4LegsMode = (360.0/4 - 360.0/NumberOfLegs);

		angleAdaptionSpiderMode = 0;
		if (i == 0)
			angleAdaptionSpiderMode = -(360.0/7 - 360.0/NumberOfLegs);
		if (i == 1)
			angleAdaptionSpiderMode = (360.0/7 - 360.0/NumberOfLegs);
		if (i == NumberOfLegs -2)
			angleAdaptionSpiderMode = -(360.0/7 - 360.0/NumberOfLegs);
		if (i == NumberOfLegs -1)
			angleAdaptionSpiderMode = (360.0/7 - 360.0/NumberOfLegs);

		realnum angle = startAngle + fourLegsModeRatio*angleAdaption4LegsMode + spiderModeRatio*angleAdaptionSpiderMode;
		realnum currentRadius = gaitRefPointRadius;

		realnum zCoordOverGround = perpendicularGroundHeight[i];
		zCoordOverGround += mainController->getBodyKinematics().getFatFootCorrectionHeight(i);

		// limit z-coord
		zCoordOverGround = constrain(zCoordOverGround,
				mainController->getBodyKinematics().getCurrentBellyPose().position.z - maxBodyHeight,
				mainController->getBodyKinematics().getCurrentBellyPose().position.z - minBodyHeight);

		// add small amount to avoid singularity at 0,0
		Point newGaitRefPoint = Point(	0.1 + currentRadius*cos(radians(angle)),
										0.1 + currentRadius*sin(radians(angle)),
										zCoordOverGround);

		targetGaitRefPoints[i] = newGaitRefPoint;

		startAngle -= 360.0/NumberOfLegs;
	}
}


// compute position of passed point after the passed duration
Point GaitController::getNextToePoint(const Point& currentPoint, seconds dT) {
	// what angle are we rotating in one loop?
	realnum loopAngle = currentAngularSpeed*dT;

	// rotate by delta angle around origin
	Point newPosition = currentPoint.getRotatedAroundZ(-loopAngle);

	// and add the regular speed vector (from perspective of the bot, the ground moves backwards)
	newPosition -= Point(speedX, speedY,0)*dT;

	return newPosition;
}


// compute next point of leg with legNo
Point GaitController::interpolateLegMotion(
		int legNo, 						/* number of leg, starting with 0 */
		const Point& currentToePoint,  	/* current toe point of considered leg */
		const Point& diffToePoint,	   	/* next toe point in time dT of considered leg */
		millimeter dDistance, 			/* length of diffToePoint */
		const Point& gaitRefPoint,		/* gait reference point, i.e. point on xy-plane that moves forward and backward */
		millimeter fullStepLength_mm, 	/* length on the ground of the full gait */
		realnum setGroundPercentage, 	/* i.e. ground-touch-ratio of that leg 0..1 */
		realnum gaitProgress,			/* continous number indicating the progress of the gait, one full gait has a length of 1 */
		seconds dT						/* time between now and next call */
		) {


	// normalize gaitProcess such that it is between 0 and 1
	gaitProgress = fmod(gaitProgress, 1.0);

	// determine current phase
	realnum currGroundPercentage = currentGroundPercentage[legNo];

	realnum onePhaseBeat = (1.0-currGroundPercentage)/2.0;

	// during normal gait take care that only one leg leaves or touches the ground at a time
	// to keep the bot horizontally . Without that, the IMU has a hard time to compensate
	// warping of legs.
	if ((targetGaitType == SpiderWalk) || (targetGaitType == TwoLegsInTheAir))
		onePhaseBeat = (1.0-currGroundPercentage*1.1)/2.0;

	realnum localPhaseBeat = fmod(gaitProgress,onePhaseBeat)/onePhaseBeat;

	// if we do not move but sort out the legs only, use small gait height only;
	if (fullStepLength_mm < floatPrecision)
		gaitHeight = sortOutLegsGaitHeight;
	else
		gaitHeight = walkingGaitHeight;

	// fetch last position of the previous point2point movement
	Point& lastPhasePosition = lastPhasePositions[legNo];

	int phaseCounter = int(gaitProgress/onePhaseBeat);

	LegGaitPhase phase = (LegGaitPhase)(int)(gaitProgress/onePhaseBeat);
	if ((int)phase > LegGaitDuty)
		phase = LegGaitDuty;

	// if we are on the ground for any reason, move with the ground until we move up
	if ((feetOnGround[legNo] == true) && (phase != LegGaitUp))
		phase = LegGaitDuty;

	bool newPhase = (phase != lastPhase[legNo]);
	if (newPhase) {
		lastPhasePositions[legNo] = currentToePoint;
		lastPhase[legNo] = phase;

		// when we start a gait we may change the ground percentage without harming the current movement.
		if (phaseCounter == 0) {
			currentGroundPercentage[legNo] = setGroundPercentage;
		}
	}

	legPhase[legNo] = phase;

	Point result(currentToePoint);
	Point groundProjection(currentToePoint);
	groundProjection.z = 0;

	// We move when
	// - speed is != 0
	// - our current ref point from to-be ref point is very different or the leg is up
	// - gait mode is switched
	realnum moveLength = diffToePoint.length();

	bool doMove =  (moveLength > floatPrecision)
				   || !feetOnGround[legNo]
 				   || (adaptToGaitRefPointType == ADAPT_TO_GAIT_POINT)
                   || ((moveLength < floatPrecision) && ((sqr(gaitRefPoint.x-groundProjection.x) + sqr(gaitRefPoint.y-groundProjection.y)) > sqr(moveToeWhenDistanceGreaterThan)) && (adaptToGaitRefPointType != DO_NOT_ADAPT_GAIT_POINT));

	/*
	LOG(DEBUG) << "[" << legNo << "] move=" << doMove << " ml=" << moveLength << "|" << (moveLength > floatPrecision) << " fog" << !feetOnGround[legNo]
	           << "grd" << (sqr(gaitRefPoint.x-groundProjection.x) + sqr(gaitRefPoint.y-groundProjection.y))
	           << " gaitProgress" << gaitProgress << " LPB" << localPhaseBeat << " phase=" << (int)phase;
*/
	// cout << "doMove[" << legNo << "]" << doMove << "ml=" << moveLength << "tpd" << footTouchPoint.distanceSqr(groundProjection) << endl;
	// if we do not move, lower all feet slowly to the ground
	if (doMove) {
		switch (phase) {
			case LegGaitUp: {
				// next touch point is gait ref point + half a step forward
				Point nextTouchPoint (gaitRefPoint);
				if (abs(dDistance) > floatPrecision) {
					// we could take fullsteplength, but this does not yet consider the
					// last mm when the toes goes with the ground already. For that, slightly increase the
					// step length (following the computed step length this is not THAT important)
					nextTouchPoint -= diffToePoint*(fullStepLength_mm*(0.5 + moveWithGroundBelowThisGroundDistance/gaitHeight)/dDistance);
				}

				Point prevTouchPoint = lastPhasePosition;

				// if we enter this phase the first time in that gait, define the bezier curve
				if (newPhase) {
					Point leftSupportPoint = prevTouchPoint;
					leftSupportPoint.z = gaitHeight + gaitRefPoint.z;
					Point rightSupportPoint = nextTouchPoint;
					rightSupportPoint.z = gaitHeight + gaitRefPoint.z;

					bezier[legNo].set(prevTouchPoint, leftSupportPoint, nextTouchPoint, rightSupportPoint);

				}

				// get next point within bezier curve. Apply moderation at the
				// starting point to achieve an soft start up
				result = bezier[legNo].getCurrent((1.0-moderate(1.0-localPhaseBeat, 1.0))*postponeZenith);

				// if toe is close to the ground move leg with the ground during the first mm
				if (result.z < gaitRefPoint.z + moveWithGroundBelowThisGroundDistance/3) {
					Point nextToe = getNextToePoint(currentToePoint, dT);
					result.x = nextToe.x;
					result.y = nextToe.y;
					// don't overwrite z-coordinate
				}

				if (result.z > gaitRefPoint.z + floatPrecision) {
					feetOnGround[legNo] = false;
				}
				break;
			}
			case LegGaitDown: {
				// maybe the toe touches the ground already, then move with the ground
				if (currentToePoint.z < floatPrecision + gaitRefPoint.z ) {
					result = currentToePoint;
					result += diffToePoint;
					result.z = gaitRefPoint.z;
					feetOnGround[legNo] = true;
				} else {
					// target position is the gait ref point + half a step ahead
					Point target = gaitRefPoint;
					if (abs(dDistance) > floatPrecision)
						target -= diffToePoint*(fullStepLength_mm*0.5/dDistance);

					// get next point on bezier curve. Apply moderation to achieve
					// a soft slow-down before touching the ground
					Point bezierPoint = bezier[legNo].getCurrent(moderate(localPhaseBeat, 2.0)*(1.0-postponeZenith)+postponeZenith);

					Point bezierTouchPoint = bezier[legNo].getEnd();
					Point touchPointDifference = target-bezierTouchPoint;
					result = (touchPointDifference*localPhaseBeat) + bezierPoint;

					// check if the toe touches the ground (used elsewhere)
					if (result.z < floatPrecision + gaitRefPoint.z)
						feetOnGround[legNo] = false;

					// if toe is close to the ground move leg with the ground already when doing the last mm's
					if (result.z < gaitRefPoint.z + moveWithGroundBelowThisGroundDistance) {
						Point nextToe = getNextToePoint(currentToePoint, dT);
						result.x = nextToe.x;
						result.y = nextToe.y;
						// don't overwrite z-coordinate
					}
				}
				break;
			}
			case LegGaitDuty:
			default:
			{
				// move with the ground
				result = getNextToePoint(currentToePoint, dT);
				feetOnGround[legNo] = true;
				break;
			}
		} // switch
	}
	return result;
}

GaitModeType GaitController::getActualGaitMode(realnum footSpeed) {
	if ((targetGaitType == Auto) || (targetGaitType == None)) {
		if (footSpeed < 10.0)
			return OneLegInTheAir;
		if (footSpeed < 20.0)
			return TwoLegsInTheAir;
		else
			return TwoLegsInTheAir;
	}
	return targetGaitType;
}

realnum GaitController::getLegAddOn(realnum globalGaitRatio, realnum footSpeed,int leg, GaitModeType gaitMode) {
	switch (gaitMode) {
		case FourLegWalk:
			if (NumberOfLegs == 5) {
				realnum legSequence[5] = { 0,1,0,2,3 };
				return legSequence[leg];
			}
			if (NumberOfLegs == 6) {
				realnum legSequence[6] = { 0,1,0, 0, 2,3 };
				return legSequence[leg];
			}
			else if (NumberOfLegs == 7) {
				realnum legSequence[7] = { 0,1,0,0,0,2,3 };
				return legSequence[leg];
			}
		case OneLegInTheAir:
		case SpiderWalk:
		case TwoLegsInTheAir:
		{
			if (NumberOfLegs == 5) {
				realnum legSequence[5] = { 0,3,1,4,2 };
				return legSequence[leg];
			}
			if (NumberOfLegs == 6) {
				realnum legSequence[6] = { 0,4,2,1,5,3 };
				return legSequence[leg];
			}
			else if (NumberOfLegs == 7) {
				realnum legSequence[7] = { 0,4,1,5,2,6,3 };
				return legSequence[leg];
			}
		}

		case Auto:
		case None:
			return getLegAddOn(globalGaitRatio, footSpeed, leg, getActualGaitMode(footSpeed));
	}
	return getLegAddOn(globalGaitRatio, footSpeed, leg, OneLegInTheAir);
}

realnum GaitController::getFootOnTheGroundRatio(realnum footSpeed, GaitModeType gm) {
	switch (gm) {
		case FourLegWalk: return 3.0/4.0;
		case OneLegInTheAir: return float(NumberOfLegs-1)/float(NumberOfLegs);
		case TwoLegsInTheAir:
		case SpiderWalk:
			return float(NumberOfLegs-2)/float(NumberOfLegs);
		case None:
		case Auto:
			return getFootOnTheGroundRatio(footSpeed, getActualGaitMode(footSpeed));
	}
	return getFootOnTheGroundRatio(getActualGaitMode(footSpeed));
}

void GaitController::loop() {
	seconds dT = gaitLoopSample.dT();
	if (dT != 0) {
		realnum maxFootDistance = 0;
		realnum loopFootDistance[NumberOfLegs];
		PentaPointType loopFootMoveVector;

		int numberOfActiveLegs = NumberOfLegs;
		if (targetGaitType == FourLegWalk)
			numberOfActiveLegs = 4;

		realnum maxRefPointDistance = 0;
		for (int i = 0;i<NumberOfLegs;i++) {
			// compute move vector of this foot that happens in one loop
			Point& gaitRefPoint = currentGaitRefPoints[i];
			Point& loopFootMove = loopFootMoveVector[i];
			Point nextPoint = getNextToePoint(gaitRefPoint, dT);
			realnum moveX = nextPoint.x - gaitRefPoint.x;
			realnum moveY = nextPoint.y - gaitRefPoint.y;
			loopFootMove.x = moveX;
			loopFootMove.y = moveY;

			loopFootDistance[i] = loopFootMove.length();

			realnum totalFootDistance = loopFootDistance[i];
			realnum refPointDistance = gaitRefPoint.distance(toePoints[i]);
			if (totalFootDistance > maxFootDistance)
				maxFootDistance = totalFootDistance;
			if (refPointDistance > maxRefPointDistance)
				maxRefPointDistance = refPointDistance;
		}
		biggestRefPointDistance = maxRefPointDistance;
		fastestFootSpeed = maxFootDistance/dT;

		realnum footOntheGroundPercentage = getFootOnTheGroundRatio(fastestFootSpeed);
		realnum gaitDuration = 1.0/gaitSpeed; // duration in s per gait, might be infinite
		if (abs(gaitSpeed) < floatPrecision)
			gaitDuration = 1.0/floatPrecision;

		// increase global gait counter, one full gait represents 1.0
		globalGaitBeat += dT*gaitSpeed;

		// now move each foot
		for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
			// leave out the front leg if we are in four-leg more, or
			// if we just switch to four leg mode
			bool omitThisLeg = ((targetGaitType == FourLegWalk) && (legNo > 1) && (legNo < NumberOfLegs-2)) ||
								((legNo == NumberOfLegs/2) && !includeFrontLeg);

			// define gait pattern
			Point newFootPosition;

			if (!omitThisLeg) {
				realnum legSequenceNo = getLegAddOn(globalGaitBeat, fastestFootSpeed, legNo);
				realnum legGaitBeat = globalGaitBeat+legSequenceNo/float(numberOfActiveLegs);
				millimeter fullGaitStepLength  = ((gaitDuration * footOntheGroundPercentage)/dT) * loopFootDistance[legNo];
				newFootPosition = interpolateLegMotion(legNo,  toePoints[legNo], loopFootMoveVector[legNo], loopFootDistance[legNo], currentGaitRefPoints[legNo],
													  fullGaitStepLength,footOntheGroundPercentage, legGaitBeat, dT);

			}
			else {
				// special position for the omitted leg. Take passed position.
				newFootPosition = frontLegPosition;
				feetOnGround[legNo] = false; // 				(newFootPosition.position.z == distanceSqr(currentGaitRefPoints[legNo]) < floatPrecision);
				if (feetOnGround[legNo]){
					lastPhase[legNo] = LegGaitDuty;
				}
				targetGaitRefPoints[legNo] = newFootPosition;
			}
			toePoints[legNo] = newFootPosition;

			// move target foot point
			currentGaitRefPoints[legNo].moveTo(targetGaitRefPoints[legNo], dT, maxGaitRefPointSpeed);

			// compute point where foot will touch the ground
			// target = currentGaitrefPoint - diffToePoint*(fullStepLength_mm*0.5/dDistance);
			// target = currentGaitrefPoint - loopFootMoveVector[legNo]*(((gaitDuration * footOntheGroundPercentage)/dT) *0.5);

			currentWalkingTouchPoints[legNo] = currentGaitRefPoints[legNo] - loopFootMoveVector[legNo] * gaitDuration * footOntheGroundPercentage/dT *0.5;
		}

		// adapt speed vector accordingly
		angle_rad noseOrientation = mainController->getCurrentNoseOrientation();
		speedX = currentSpeed*cos(currentAbsWalkingDirection-noseOrientation);
		speedY = currentSpeed*sin(currentAbsWalkingDirection-noseOrientation);

		// move current position
		currPosWorld.x += currentSpeed*cos(currentAbsWalkingDirection )*dT;
		currPosWorld.y += currentSpeed*sin(currentAbsWalkingDirection )*dT;
	}
};

void GaitController::imposeFootPointsWorld(const PentaPointType& footPointsWorld) {
	toePoints = footPointsWorld.getRotatedAroundZ(-mainController->getBodyKinematics().getCurrentNoseOrientation());
}

PentaPointType GaitController::getGaitRefPointsWorld() {
	return currentGaitRefPoints.getRotatedAroundZ(mainController->getBodyKinematics().getCurrentNoseOrientation());
};

Point GaitController::getGaitRefPointsWorld(int legNo) {
	return currentGaitRefPoints[legNo].getRotatedAroundZ(mainController->getBodyKinematics().getCurrentNoseOrientation());
};

PentaPointType GaitController::getToePointsWorld() {
	return toePoints.getRotatedAroundZ(mainController->getBodyKinematics().getCurrentNoseOrientation());
};

Point GaitController::getToePointsWorld(int legNo) {
	return toePoints[legNo].getRotatedAroundZ(mainController->getBodyKinematics().getCurrentNoseOrientation());
};
