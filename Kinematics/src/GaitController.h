/*
 * GaitControl.h
 *
 *  Created on: 15.04.2017
 */

#ifndef GAITCONTROL_H_
#define GAITCONTROL_H_

#include "core.h"
#include "basics/logger.h"

#include "spatial.h"
#include "BezierCurve.h"

class Engine;

typedef bool FootOnGroundFlagType[NumberOfLegs];

enum AdaptToGaitRefPointType { DO_NOT_ADAPT_GAIT_POINT, ADAPT_TO_GAIT_POINT, ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE };
class GaitController {
public:

	GaitController();

	// call me upfront
	void setup(Engine& pMainController);

	// call me to compute the next iteration of the gait
	void loop();

	// return current speed (not neccessarily the target speed set by setTargetMovement)
	mmPerSecond getCurrentSpeedX() { return speedX; };
	mmPerSecond getCurrentSpeedY() { return speedY; };
	mmPerSecond& getCurrentSpeed() { return currentSpeed; };
	radPerSecond& getCurrentAngularSpeed() { return currentAngularSpeed; };
	angle_rad& getCurrentAbsWalkingDirection() { return currentAbsWalkingDirection; };

	// return gait ratio which is a number from 0..1 representing the current state within the gait
	realnum getGaitRatio() { return globalGaitBeat; };

	// change gait mode
	void setTargetGaitMode(GaitModeType gaitType) { targetGaitType = gaitType; };

	// returns the gait mode which has been set by setTargetGaitMode
	GaitModeType getTargetGaitMode() { return targetGaitType; };

	// return the gait phase of a specified leg
	LegGaitPhase getLegsGaitPhase(int legNo) { return legPhase[legNo]; };
	LegGaitPhase getLastGaitPhase(int legNo) { return lastPhase[legNo]; };


	// returns the speed of the fastest foot. One foot has a higher speed than the others when the bot rotates
	realnum getFastestFootSpeed() { return fastestFootSpeed; };

	// get the biggest distance from a toe to its gait ref point.
	realnum getBiggestRefPointDistance() { return biggestRefPointDistance; };

	const FootOnGroundFlagType& getFeetOnGround() { return feetOnGround; };
	void setFeetOnGround(const FootOnGroundFlagType& newFeetOnGround) { for (int i = 0;i<NumberOfLegs;i++) feetOnGround[i] = newFeetOnGround[i]; };

	int getFeetOnTheGround() {
		int legsOnTheGround = 0;
		for (int i = 0;i<NumberOfLegs;i++)
			if (getFeetOnGround()[i])
				legsOnTheGround++;
		return legsOnTheGround;
	}

	realnum distanceToGaitRefPoints() {
		realnum sum = 0;
		for (int i = 0;i<NumberOfLegs;i++) {
			Point distance (currentGaitRefPoints[i]);
			distance -= toePoints[i];
			distance.z = 0;
			sum += distance.length();
		}
		return sum/NumberOfLegs;
	}

	PentaPointType getCurrentWalkingTouchPoints() { return currentWalkingTouchPoints; };

	PentaPointType getGaitRefPointsWorld() ;
	Point getGaitRefPointsWorld(int legNo) ;

	Point getToePointsWorld(int legNo) ;
	PentaPointType getToePointsWorld() ;

	// A gait ref point is the point middle point on the ground . The toe moves around this point
	const PentaPointType& getGaitRefPoints() { return currentGaitRefPoints; };

	const PentaPointType& getToePoints() { return toePoints; };

	realnum getFootOnTheGroundRatio(realnum footSpeed, GaitModeType gm = None);

	// the Ref Points are used as orientation points for a gait. It is the point at the ground in the middle of a gait.
	void setTargetGaitRefPointsRadius(realnum rad, realnum spiderAdaptionRatio, realnum fourLegsModeRatio);
	realnum getGaitRefPointsRadius() { return gaitRefPointRadius; };
	void setGaitSpeed(realnum newGaitSpeed) { gaitSpeed = newGaitSpeed; };
	realnum getGaitSpeed() { return gaitSpeed; };

	void setGaitHeight(realnum newWalkGaitHeight, realnum newAdaptLegsGaitHeight) { walkingGaitHeight = newWalkGaitHeight; sortOutLegsGaitHeight = newAdaptLegsGaitHeight;};

	void setFrontLegWorld(const Point& x);
	Point getFrontLegWorld();

	// define whether the front leg should be included in the gait movement
	void setIncludeFrontLeg (bool ok) { includeFrontLeg = ok;	}

	// force a gait movement even when we do not move. Used during gait changes
	void adaptToGaitRefPoint(AdaptToGaitRefPointType ok) { adaptToGaitRefPointType = ok; };

	Point& getCurrentPositionWorld() { return currPosWorld; };
	void setCurrentPositionWorld(const Point& p) { currPosWorld = p; };

	void setAbsoluteGroundHeight(int legno, realnum newGroundCorrection) { perpendicularGroundHeight[legno] = newGroundCorrection; };

	realnum getAbsoluteGroundHeight(int legno) { return perpendicularGroundHeight[legno]; };
	realnum getAvrPerpendicularGroundHeight() { realnum s = 0;for (int i = 0;i<NumberOfLegs;i++) s += perpendicularGroundHeight[i]; return s/NumberOfLegs; };

	realnum getPerpendicularDistanceToGround(int legNo) { return toePoints[legNo].z - getAbsoluteGroundHeight(legNo); };
	realnum getMinGaitRefPoint() { realnum min = 1000.0;for (int i = 0;i<NumberOfLegs;i++) if (currentGaitRefPoints[i].z < min) min=currentGaitRefPoints[i].z ; return min; };

	// impose current feet points (used when initializing). In walking mode, foot points are computed internally
	void imposeFootPointsWorld(const PentaPointType& footPoints);

	// set one foot touch point (and update the same in world coordinates)
	void setTargetGaitRefPoint(int legNo, const Point& footTouchPoint);
private:

	realnum getLegAddOn(realnum globalGaitRatio, realnum footSpeed,int leg, GaitModeType gaitMode = None);

	GaitModeType getActualGaitMode(realnum footSpeed);
	Point getNextToePoint(const Point& currentToePoint, seconds dT);
	Point interpolateLegMotion(int legNo, const Point& currentPoint,  const Point& loopFootMoveVector, realnum loopDistance, const Point& gaitRefPoint,
							   millimeter stepLength, realnum groundPercentage, realnum ratio,
							   seconds duration_s);

	PentaPointType currentGaitRefPoints;
	PentaPointType targetGaitRefPoints;
	PentaPointType toePoints;
	PentaPointType currentWalkingTouchPoints; // the position where a leg touches the ground

	Point currPosWorld;

	FootOnGroundFlagType feetOnGround;
	mmPerSecond speedX;					// current speed in x direction
	mmPerSecond speedY;					// current speed in x direction
	radPerSecond currentAngularSpeed;
	angle_rad currentAbsWalkingDirection;
	realnum currentSpeed; 				// current speed, slowly accelerating to totalSpeed

	realnum gaitSpeed;					// [gaits/s] number of full gaits per second

	realnum globalGaitBeat;				// global heartbeat indicating the position within a gait. One beat happens between [x..x+1]
	realnum walkingGaitHeight;			// maximum height of a leg during walking
	realnum sortOutLegsGaitHeight;	 // maximum heigh of a leg when sorting out legs during startup
	realnum gaitHeight;
	bool stretchingLegsMode;			// mode that stretches the legs (used during wakeup)

	GaitModeType targetGaitType;		// target gait type (the current gait is switched coordinatedly, so this is not even to currentGaitType)

	realnum gaitRefPointRadius;			// gait ref points (middle ground point of each leg in a gait) are arranged in a circle

	realnum fastestFootSpeed;
	realnum biggestRefPointDistance;	// biggest current difference of a toe to its gait point
	bool includeFrontLeg;			 	// true, if the front leg shall be included in the gait (used during gait switching)
	AdaptToGaitRefPointType adaptToGaitRefPointType;	// do gait even when speed is zero. Used during switching gait

	Point frontLegPosition;

	realnum perpendicularGroundHeight[NumberOfLegs];

	LegGaitPhase legPhase[NumberOfLegs];
	LegGaitPhase lastPhase[NumberOfLegs];

	realnum currentGroundPercentage[NumberOfLegs];
	Point lastPhasePositions[NumberOfLegs];

	Engine *mainController;
	TimeSamplerStatic gaitLoopSample;

	BezierCurve bezier[NumberOfLegs];
};


#endif /* GAITCONTROL_H_ */
