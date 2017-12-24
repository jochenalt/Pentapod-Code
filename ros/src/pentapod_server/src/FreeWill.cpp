/*
 * FreeWill.cpp
 *
 *  Created on: Dec 12, 2017
 *      Author: JochenAlt
 */

#include "Dispatcher.h"
#include "FreeWill.h"
#include "IntoDarkness.h"

FreeWill::FreeWill() {

}


void FreeWill::setup() {
	slamMap =  (Map*)&Dispatcher::getInstance().getSlamMap();;
	localCostMap  =  (Map*)&Navigator::getInstance().getLocalCostmap();
	globalCostMap =  (Map*)&Navigator::getInstance().getGlobalCostmap();

	odomFrame = (Pose*) &Dispatcher::getInstance().getOdomFrame();
	baseLink = (Pose*) &Dispatcher::getInstance().getBaselink();
	engineState = (EngineState*) &Dispatcher::getInstance().getEngineState();

	  // needs to be turned on to do anything
	turnOn = false;
}


Pose FreeWill::getAutonomousBodyPose() {
	const realnum standUpRightScariness = 0.30;
	const realnum sitDownScariness = 0.2;

	Pose result = engineState->moderatedBodyPose;
	if (engineState->currentScaryness > standUpRightScariness) {
		result.position.z = uprightBodyHeigh;
	}
	if (engineState->currentScaryness < sitDownScariness) {
		result.position.z = standardBodyHeigh;
	}
	return result;
}

Pose FreeWill::computeNavigationGoal() {
	// select a dark hole that has not been visited in the last
	// couple of times, that is in the near range and that is reachable
	const realnum distanceWeight = 1.0;
	const realnum optimumDistance = 3.0; // [m]
	const realnum avoidRepeatedVisitsWeight = 1.0;

	std::vector<Point> holes;

	milliseconds now = millis();
	IntoDarkness::getInstance().getDarkScaryHoles(holes);
	StampedPose result;
	realnum maxScore = -1;
	for (int i = 0;i<holes.size();i++) {
		Point hole = holes[i];

		// get the weighted average distance to any hole that has been visited already
		// the more recent a visit took place the more relevant it is
		realnum lastVisitDistance = 0;
		realnum weightSum = 0;
		for (int j = 0;j<lastVisits.size();j++) {
			realnum visitWeight = (now - lastVisits[i].timestamp)/now;
			lastVisitDistance += lastVisits[i].pose.position.distance(hole) * visitWeight;
			weightSum += visitWeight;
		}
		lastVisitDistance /= weightSum;

		// scoring of distance to the current position and distance to last recently visited holes
		realnum score = distanceWeight * abs(hole.distance(baseLink->position) - optimumDistance) +
				        avoidRepeatedVisitsWeight * lastVisitDistance;
		if (score > maxScore) {
			score = maxScore;
			result.pose.position = hole;
			result.timestamp = now;
		}
	}

	// add to list of last visits
	if (maxScore > -1) {
		lastVisits.insert(lastVisits.end(),result);
		return result.pose.position;
	}

	// no place to go found
	return Point();
}

void FreeWill::publish() {
	static bool lastTurnedOn = false;
	if (turnOn == lastTurnedOn) {
		if (turnOn) {
			// check if navigation is finished
			if (Navigator::getInstance().getNavigationGoalStatus().isDone()) {

				if (latchGoalReachable) {
					// goal could not be reached, but this does not make a difference here, we select the next one
				}

				// find a new dark scary hole
				Pose nextGoal = computeNavigationGoal();
				if (nextGoal.isNull()) {
					Navigator::getInstance().setNavigationGoal(nextGoal, true);
					latchGoalReachable = true; // check asynchronously if goal can be reached.
				}
			} else {
				if (latchGoalReachable && (Navigator::getInstance().getNavigationGoalStatus() == actionlib::SimpleClientGoalState::ACTIVE)) {
					latchGoalReachable = false; // goal can be reached
				}
			}
		}
	} else {
		if (!turnOn) { // free will has been switched off, cancel the current goal
			Navigator::getInstance().cancelNavigationGoal();

		}
	}

	lastTurnedOn = turnOn;
}
