/*
 * MainController.cpp
 *
 *  Created on: 07.05.2017
 *      Author: JochenAlt
 */

#include "core.h"
#include "basics/stringhelper.h"
#include "Engine.h"

Engine::Engine() {
	isSetup = false;
}


bool Engine::setupProduction(string i2cport, int i2cadr, string cortextSerialPort, int cortexSerialBaudRate) {
	ROS_DEBUG_STREAM("Engin::setupProduction");

	bool ok = setupCommon();
	legController.setupCortexCommunication(i2cport, i2cadr, cortextSerialPort, cortexSerialBaudRate);
	ok = legController.isCortexCommunicationOk();

	// if cortex is up and running
	if (ok) {
		ROS_DEBUG_STREAM("fetch angles");

		// fetch the angles via cortex to initialize the feet with it
		ok = legController.fetchAngles(legAngles);
		Rotation imuOrientation = legController.getIMUOrientation();
		imuOrientation.z = 0;

		// compute leg pose out of angles, estimate body pose
		PentaPointType footPoints;
		Pose bodyPose;
		bodyKinematics.computeForwardKinematics(legAngles, imuOrientation, footPoints, bodyPose);
		setTargetBodyPose(bodyPose, true);

		// impose the footpoint into state
		ROS_DEBUG_STREAM("impose foot points");
		imposeFootPointsWorld(footPoints);
	}

	ROS_DEBUG_STREAM("Engin::setupProduction done");
	isSetup = true;
	return ok;
}

bool Engine::setupSimulation() {
	ROS_DEBUG_STREAM("Engine::setupSimulation");

	bool ok = setupCommon();
	ROS_DEBUG_STREAM("Engine::setupSimulation done");

	isSetup = true;

	return ok;
}

bool Engine::setupCommon() {
	kinematics.setup(*this);
	bodyKinematics.setup(*this);
	gaitControl.setup(*this);
	legController.setup();

    frontLegPose.position = Point(280,1,80);

    // assume that touches is on the ground
	inputBodyPose.position.z = minBodyHeight;
	inputBodyPose.orientation.null();
	moderatedBodyPose = inputBodyPose;
	currentBodyPose = inputBodyPose;

	// before walking, we need to lift to standard walking height
	inputBodyPose.position.z = standardBodyHeigh;

	fourWalkLegRatio = 0; // we start with 5 legs mode. 0 % of 4 legs mode
	spiderWalkLegRatio = 0; // we start with 5 legs mode.

	gaitControl.setTargetGaitMode(OneLegInTheAir);
	targetGaitMode = OneLegInTheAir;
	humpsCompensation = 0;

	targetWalkingDirection = 0;
	targetSpeed = 0;
	targetAngularSpeed = 0;

	// we start with assuming trhat all legs are on the ground
	for (int i = 0;i<NumberOfLegs;i++)
		lastFeetOnGround[i] = true;


	// impose random foot points. Should be overwritten by first-time sensor read
	PentaPointType footPoints;
	for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
		realnum turnAngle = legNo*M_PI*2.0/NumberOfLegs; // randomFloat(-M_PI,M_PI);
		Point deviation(70,0,0);
		deviation = deviation.getRotatedAroundZ(turnAngle);
		footPoints[legNo] = getGaitRefPointWorld(legNo) + deviation;
	}
	imposeFootPointsWorld(footPoints);

	generalMode = BeingAsleep;
	shutdownMode = NoShutDownActive;
	turnedOn = false;

	imuPID.reset();

	return true;
}

void Engine::wakeUp() {
	ROS_DEBUG_STREAM("wake up");
	if ((generalMode != WalkingMode) && (generalMode != TerrainMode) && (generalMode != LiftBody)){
		generalMode = LiftBody;
		bodyKinematics.startupPhase(true);
		gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
	}
}

void Engine::fallAsleep() {
	ROS_DEBUG_STREAM("fall asleep");
	generalMode = FallASleep;
	inputBodyPose.position = Point(0,0,minBodyHeight);
	inputBodyPose.orientation = Rotation(0,0,0);
	targetAngularSpeed = 0;
	targetSpeed = 0;
}

void Engine::terrainMode(bool terrainModeOn) {
	ROS_DEBUG_STREAM("terrainMode(" << terrainModeOn <<")");
	if (terrainModeOn) {
		generalMode = TerrainMode;
	}
	else
		generalMode = WalkingMode;
}

void Engine::turnOn() {
	ROS_DEBUG_STREAM("turnON()");

	setupCommon();

	if (legController.isCortexCommunicationOk()) {

		CriticalBlock block(loopMutex);

		// setup bot and fetch the angles
		bool ok = legController.setupBot();
		ok = ok && legController.fetchAngles(legAngles);
		Rotation imuOrientation = legController.getIMUOrientation();
		imuOrientation.z = 0;

		if (ok) {
			// compute the footpoints out of angles
			PentaPointType footPoints;
			Pose bodyPose;
			bodyKinematics.computeForwardKinematics(legAngles, imuOrientation, footPoints, bodyPose);
			setTargetBodyPose(bodyPose, true);

			// impose the footpoints
			imposeFootPointsWorld(footPoints);

			PentaPointType tmpHipPoints;
			LegAnglesType allLegAngles = legAngles;
			ok = bodyKinematics.computeKinematics(
										bodyPose,
										gaitControl.getToePoints(),
										gaitControl.getCurrentWalkingTouchPoints(),
										tmpHipPoints,
										allLegAngles,
										groundPoints);
			if (ok) {
				ok = ok && legController.enableBot();
				ok = ok && legController.moveSync (allLegAngles, 1000);

				// in the last second, gait went on, so impose the old state to not hiccup within the loop
				imposeFootPointsWorld(footPoints);
				turnedOn = true;
			}
		}
	}

	turnedOn = true;
}

void Engine::turnOff() {
	ROS_DEBUG_STREAM("turnOff");
	legController.disableBot();
	turnedOn = false;
}


void Engine::computeSingleLegAdHoc(LegPose& newSingleLegPoseWorld) {

	LegPose pose;
	pose.position = newSingleLegPoseWorld.position;
	bool ok = Engine::getLegKinematics().computeInverseKinematics(pose);
	if (ok)
		newSingleLegPoseWorld = pose;
}

void Engine::setTargetFrontLegPose(const Point& newFrontLegPoseWorld) {
	frontLegPose.position = newFrontLegPoseWorld.getRotatedAroundZ(-getCurrentNoseOrientation());
}

LegPose Engine::getFrontLegPoseWorld() {
	LegPose pose = frontLegPose;
	pose.position.getRotatedAroundZ(getCurrentNoseOrientation());
	return pose;
}


void Engine::setTargetBodyPose(const Pose& newBodyPose, bool immediately) {
	// do not accept any more commands when in danger
	if (legController.betterShutMeDown())
		return;
	// ROS_DEBUG_STREAM("setTargetBodyPose(" << newBodyPose << "," << immediately <<")");

	// dont take this if we actually wake up
	if ((generalMode == WalkingMode) || (generalMode == TerrainMode) || (generalMode == LiftBody) || (generalMode == FallASleep))
		inputBodyPose = newBodyPose;
	if (immediately) {
		currentBodyPose = newBodyPose;
		moderatedBodyPose = newBodyPose;
	}
}

LegAnglesType Engine::getLegAngles() {
	return legAngles;
};

PentaPointType Engine::getHipPoints() {
	return hipPoints;
}


bool Engine::ratedloop() {
	if (mainLoopTimeSample.isDue(CORTEX_SAMPLE_RATE)) {
		CriticalBlock criticalBlock(loopMutex);
		loop();
		return true;
	}
	return false;
}

void Engine::loop() {
	// send last commmand to legController, return
	// state and distance to grounds
	// do this first to have a precise timing
	legController.loop();

	PentaPointType tmpHipPoints;
	LegAnglesType allLegAngles = legAngles;

	// grab last measurement of foot sensors
	if (legController.isCortexCommunicationOk()) {
		realnum distance[NumberOfLegs];
		legController.getDistanceSensor(distance);

		// LOG(DEBUG) << "Distance" << distance[0]*cos(bodyKinematics.getFootAngle(0)) << " " << distance[1]*cos(bodyKinematics.getFootAngle(1)) << " "<< distance[2]*cos(bodyKinematics.getFootAngle(2)) << " " << distance[3]*cos(bodyKinematics.getFootAngle(3)) << " " << distance[4]*cos(bodyKinematics.getFootAngle(4));
		// LOG(DEBUG) << "GROUND-z" << gaitControl.getAbsoluteGroundHeight(0) << " " << gaitControl.getAbsoluteGroundHeight(1) << " "<< gaitControl.getAbsoluteGroundHeight(2) << " "<< gaitControl.getAbsoluteGroundHeight(3) << " "<< gaitControl.getAbsoluteGroundHeight(4);
		// LOG(DEBUG) << "Toe Points" << gaitControl.getToePoints()[0].z << " " << gaitControl.getToePoints()[1].z << " "<< gaitControl.getToePoints()[2].z << " "<< gaitControl.getToePoints()[3].z << " "<< gaitControl.getToePoints()[4].z;

		processDistanceSensors(distance);

		// if not turned on, fetch the angles and update the pose
		if (!turnedOn) {
			bool ok = legController.fetchAngles(allLegAngles);
			Rotation imuOrientation = legController.getIMUOrientation();
			imuOrientation.z = 0;
			if (ok) {
				// compute leg pose out of angles, estimate body pose
				PentaPointType footPoints;
				Pose bodyPose;
				bodyKinematics.computeForwardKinematics(allLegAngles, imuOrientation, footPoints, bodyPose);
				setTargetBodyPose(bodyPose, true);

				// set leg angles to real measured once to pass that information to all following IK computations
				legAngles = allLegAngles;

				// impose the footpoint into state
				imposeFootPointsWorld(footPoints);
			}
		}
	}

	if (isTurnedOn()) {
		computeBodyPose();
		computeGaitRefPointRadius();
		computeGaitSpeed();
		computeGaitHeight();
		computeGaitMode();
		computeFrontLeg();
		computeWakeUpProcedure();
		computeAcceleration();
		gaitControl.loop();
	}

	bool ok = bodyKinematics.computeKinematics(
										currentBodyPose,
										gaitControl.getToePoints(),
										gaitControl.getCurrentWalkingTouchPoints(),
										tmpHipPoints,
										allLegAngles,
										groundPoints);
	if (ok) {
		for (int i = 0;i<NumberOfLegs;i++) {
			hipPoints[i] = tmpHipPoints[i];
			legAngles[i] = allLegAngles[i];
		}
		if (legController.isCortexCommunicationOk()) {
			legController.setMovement (allLegAngles, CORTEX_SAMPLE_RATE);
		}
	}

	// did a fatal error occur? Then fall asleep and shutdown
	if (legController.betterShutMeDown()) {
		switch (shutdownMode) {
			case NoShutDownActive: {
				ROS_ERROR_STREAM("Better shut me down due to fatal error. Go down and stop moving.");
				shutdownMode = Initiate;
				targetSpeed = 0;
				targetAngularSpeed = 0;
				inputBodyPose = Pose(Point(0,0,standardBodyHeigh), Rotation(0,0,0));
			}
			case Initiate: {
				if ((currentBodyPose.position.z - standardBodyHeigh < floatPrecision)
					 && (getCurrentSpeed() < floatPrecision)) {
					ROS_ERROR_STREAM("Better shut me down due to fatal error. Fall asleep.");
					fallAsleep();
					shutdownMode = FallAsleep;
				}
			}
			case FallAsleep: {
				if (generalMode == BeingAsleep) {
					ROS_ERROR_STREAM("Better shut me down due to fatal error. Turn off.");

					turnOff();
					shutdownMode = Done;
				}
			}
			case Done:
				break;

		}
	}
}

mmPerSecond& Engine::getTargetSpeed () {
	return targetSpeed;
};


mmPerSecond Engine::getTargetSpeedLimited() {
	realnum targetSpeedByRotation = targetAngularSpeed*gaitControl.getGaitRefPointsRadius()/(2.0*M_PI);
	realnum totalTargetSpeed = abs(targetSpeed) + abs(targetSpeedByRotation);
	realnum allowedFootSpeedRatio = totalTargetSpeed/maxSpeed;
	realnum limitedTargetSpeed = targetSpeed;
	if (allowedFootSpeedRatio > 1.0)
		limitedTargetSpeed /= allowedFootSpeedRatio;

	return limitedTargetSpeed;
};

bool Engine::wakeUpIfNecessary() {

	if (!isSetup)
		return false;

	if (!isListeningToMovements()) {
		if (!isTurnedOn()) {
			ROS_DEBUG_STREAM("Engine::wakeUpIfNecessary:turnOn");
			turnOn();
		}
		if (generalMode == BeingAsleep) {
			ROS_DEBUG_STREAM("Engine::wakeUpIfNecessary:turnwakeUp");
			wakeUp();
		}
	}

	return ((generalMode == WalkingMode) || (generalMode == TerrainMode));
}

void Engine::setTargetSpeed (mmPerSecond newSpeed /* mm/s */) {
	// do not accept any more commands when in danger
	if (legController.betterShutMeDown())
		return;

	targetSpeed = newSpeed;
};

void Engine::setTargetAngularSpeed(radPerSecond rotateZ) {
	// do not accept any more commands when in danger
	if (legController.betterShutMeDown())
		return;

	rotateZ = constrain(rotateZ, -maxAngularSpeed, maxAngularSpeed);
	targetAngularSpeed = rotateZ;
};

radPerSecond& Engine::getTargetAngularSpeed() {
	return targetAngularSpeed;
};

radPerSecond Engine::getTargetAngularSpeedLimited() {
	realnum targetSpeedByRotation = targetAngularSpeed*gaitControl.getGaitRefPointsRadius()/(2.0*M_PI);
	realnum totalTargetSpeed = abs(targetSpeed) + abs(targetSpeedByRotation);
	realnum allowedFootSpeedRatio = totalTargetSpeed/maxSpeed;
	realnum limitedTargetAngularSpeed= targetAngularSpeed;
	if (allowedFootSpeedRatio > 1.0)
		limitedTargetAngularSpeed /= allowedFootSpeedRatio;

	return limitedTargetAngularSpeed;
};


void Engine::setTargetWalkingDirection(angle_rad newWalkingDirection) {
	// do not accept any more commands when in danger
	if (legController.betterShutMeDown())
		return;

	wakeUpIfNecessary();
	targetWalkingDirection = newWalkingDirection;
};


angle_rad& Engine::getTargetWalkingDirection() {
	return targetWalkingDirection;
};

angle_rad Engine::getCurrentWalkingDirection() {
	angle_rad currentWalkingDirection = gaitControl.getCurrentAbsWalkingDirection() - getCurrentNoseOrientation();
	if (currentWalkingDirection > M_PI)
		currentWalkingDirection -= 2.0*M_PI;
	if (currentWalkingDirection < -M_PI)
		currentWalkingDirection += 2.0*M_PI;
	return currentWalkingDirection;
};


const PentaPoseType& Engine::getHipPoseWorld() {
	return bodyKinematics.getHipPose(); // static position of all hips relative to belly
}

GaitModeType Engine::getGaitMode() {
	return gaitControl.getTargetGaitMode();
}

PentaPointType Engine::getGaitRefPoints() {

	return gaitControl.getGaitRefPointsWorld();
}

const FootOnGroundFlagType& Engine::getFootOnGround() {

	return gaitControl.getFeetOnGround();
}

void Engine::computeBodyPose() {
	// maximum speed the body moves its position or orientation
	const realnum maxLiftBodyPositionSpeed = 80.0; 	// [mm/s]
	const realnum maxBodyPositionSpeed = 40.0; 	// [mm/s]

	const realnum maxBodyOrientationSpeed = 0.4; 	// [RAD/s]

	realnum dT = bodyPoseSampler.dT();
	if (dT > floatPrecision) {

		// add the height that came out of hump compensation.
		// this moves the body up if a hump is underneath
		Pose input(inputBodyPose);
		input.position.z += humpsCompensation;

		// limit the new bodypose
		input.position.z = constrain(input.position.z, minBodyHeight, maxBodyHeight);

		// move towards the target body pose
		// but: if we are in lift or fall asleep mode, wait until all legs are on the ground
		if (((generalMode != LiftBody) &&  (generalMode != FallASleep))
			|| ((generalMode == LiftBody) && (gaitControl.getFeetOnTheGround() == NumberOfLegs) && (gaitControl.distanceToGaitRefPoints() < standUpWhenDistanceSmallerThan))
			|| ((generalMode == FallASleep) && (gaitControl.getFeetOnTheGround() == NumberOfLegs))) {

				realnum bodySpeed = maxBodyPositionSpeed;
			if ((generalMode == LiftBody) || (generalMode == FallASleep))
				bodySpeed = maxLiftBodyPositionSpeed;
			moderatedBodyPose.moveTo(inputBodyPose, dT, bodySpeed, maxBodyOrientationSpeed);
		}

		Pose toBePose = moderatedBodyPose;

		// compensate according to IMU measurement (only when walking)
		Rotation imu = legController.getIMUOrientation();
		imu.z = 0;
		Pose imuCompensation;
		Rotation error;
		if (legController.isIMUValueValid() && ((generalMode == WalkingMode) || (generalMode == TerrainMode))) {
			// PID controller on orientation of x/y axis only, z is not used
			Rotation maxError (radians(20.0), radians(20.0), radians(0.0));
			error = toBePose.orientation - imu ;
			imuCompensation.orientation = imuPID.getPID(error, 0.8, 8.0, 0.023, maxError);

		} else {
			// in any other mode than walking keep the IMU in a reset state
			imuPID.reset();
		}
		currentBodyPose = toBePose;
		currentBodyPose.orientation += imuCompensation.orientation;
		ROS_DEBUG_STREAM("IMU error=("<< std::setprecision(3) << degrees(imu.x) << "," << degrees(imu.y)
				         << "), PID=(" << degrees(imuCompensation.orientation.x) << "," << degrees(imuCompensation.orientation.y) << ")"
				         << ") after=(" << degrees(currentBodyPose.orientation.x) << "," << degrees(currentBodyPose.orientation.y) << ")");
	}
}


void Engine::computeGaitMode() {
	realnum dT = gaitModeSampler.dT();

	int numberOfLegsOnTheGround = gaitControl.getFeetOnTheGround();
	// turning the legs to FourLegGait-Position takes the time of moving one leg
	realnum switchGaitDuration = 1.0/gaitControl.getGaitSpeed()/NumberOfLegs; // wait one and a half step

	bool legJustWentDown [NumberOfLegs];
	bool legJustWentUp [NumberOfLegs];

	// find out which legs touch or leave the ground at this very moment
	const FootOnGroundFlagType& legOnGround = gaitControl.getFeetOnGround();
	for (int i = 0;i<NumberOfLegs;i++) {
		legJustWentDown[i] = (lastFeetOnGround[i] == false) && (legOnGround[i] == true);
		legJustWentUp[i]   = (lastFeetOnGround[i] == true)  && (legOnGround[i] == false);
	}

	// cout << "dn" << legWentDown[0] << "|" << legWentDown[1]<< "|" << legWentDown[2] << "|" << legWentDown[3] << "|" << legWentDown[4]
	// 	 << "   up" << legWentUp[0] << "|" << legWentUp[1]<< "|" << legWentUp[2] << "|" << legWentUp[3] << "|" << legWentUp[4]
	//  	 << "   on" << legOnGround[0] << "|" << legOnGround[1]<< "|" << legOnGround[2] << "|" << legOnGround[3] << "|" << legOnGround[4] << endl;

	for (int i = 0;i<NumberOfLegs;i++) {
		lastFeetOnGround[i] = legOnGround[i];
	}

	// if we are not in FourLegMode but havent fully reached it, work on it
	if (targetGaitMode == gaitControl.getTargetGaitMode() && (gaitControl.getTargetGaitMode() != FourLegWalk) && (fourWalkLegRatio > floatPrecision)) {
		fourWalkLegRatio -= dT/switchGaitDuration;
		gaitControl.setForceGait(ADAPT_TO_GAIT_POINT);
		if (fourWalkLegRatio <= 0.0) {
			fourWalkLegRatio = 0.0;
			gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
		}
		gaitControl.setFourWalkModeRatio(speedUpAndDown(fourWalkLegRatio));
	}

	// if we are not in SpiderMode but havent fully reached it, work on it
	if (targetGaitMode == gaitControl.getTargetGaitMode() && (gaitControl.getTargetGaitMode() != SpiderWalk) && (spiderWalkLegRatio > floatPrecision)) {
		spiderWalkLegRatio -= dT/switchGaitDuration;
		gaitControl.setForceGait(ADAPT_TO_GAIT_POINT);
		if (spiderWalkLegRatio <= 0.0) {
			spiderWalkLegRatio = 0.0;
			gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
		}
		gaitControl.setSpiderModeRatio(speedUpAndDown(spiderWalkLegRatio));
	}

	// find out if we switched to OneLeg mode, but are not yet there. Then wait for next time.
	if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) && (numberOfLegsOnTheGround < NumberOfLegs - 1)){
		return;
	}

	// if we want to change the mode, carry out specialized procedures
	if (targetGaitMode != gaitControl.getTargetGaitMode()) {
		if (targetGaitMode == FourLegWalk) {
			// switching to four leg mode happens in phase. First switch to OneLegInTheAir if not there already
			if (gaitControl.getTargetGaitMode() == TwoLegsInTheAir) {
				// no sync point necessary, switch immediately and wait until mode is complete (see above)
				gaitControl.setTargetGaitMode(OneLegInTheAir);
				gaitControl.setForceGait(ADAPT_TO_GAIT_POINT);	// start gait switch, force a gait even when not moving to carry out switch
				return;
			}

			// if in full OneLegInTheAir mode, and Four-Leg-position is not yet reached, move towards it.
			// when finished, switch to FourLeg mode
			if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) &&
				(numberOfLegsOnTheGround >=  NumberOfLegs - 1)) {
				if (((fourWalkLegRatio < floatPrecision) && legJustWentUp[NumberOfLegs/2]) ||
					((fourWalkLegRatio > floatPrecision) && !legOnGround[NumberOfLegs/2]))	{
					fourWalkLegRatio += dT/switchGaitDuration; // this is time critical, switch must happen within one leg movement
					gaitControl.setForceGait(ADAPT_TO_GAIT_POINT);
					gaitControl.setIncludeFrontLeg(false);
					if (fourWalkLegRatio >= 1.0) {
						gaitControl.setTargetGaitMode(FourLegWalk);
						gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
					}
				}

				gaitControl.setFourWalkModeRatio(speedUpAndDown(fourWalkLegRatio));
				return;
			}


			return;
		}
		if (targetGaitMode == SpiderWalk) {
			// if in full OneLegInTheAir mode, and Spider-Leg-position is not yet reached, move towards it.
			// when finished, switch to FourLeg mode
			if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) &&
				(numberOfLegsOnTheGround >=  NumberOfLegs - 1)) {
				if (((spiderWalkLegRatio < floatPrecision) && legJustWentUp[NumberOfLegs/2]) ||
					((spiderWalkLegRatio > floatPrecision) && !legOnGround[NumberOfLegs/2]))	{
					spiderWalkLegRatio += dT/switchGaitDuration; // this is time critical, switch must happen within one leg movement
					gaitControl.setForceGait(ADAPT_TO_GAIT_POINT);
					gaitControl.setIncludeFrontLeg(true);
					if (spiderWalkLegRatio >= 1.0) {
						gaitControl.setTargetGaitMode(SpiderWalk);
						gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
					}
				}

				gaitControl.setSpiderModeRatio(speedUpAndDown(spiderWalkLegRatio));
				return;
			}

			if ((gaitControl.getTargetGaitMode() == TwoLegsInTheAir)) {
				spiderWalkLegRatio += dT/switchGaitDuration; // this is time critical, switch must happen within one leg movement
				gaitControl.setForceGait(ADAPT_TO_GAIT_POINT);
				gaitControl.setIncludeFrontLeg(true);
				if (spiderWalkLegRatio >= 1.0) {
					gaitControl.setTargetGaitMode(SpiderWalk);
					gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
				}

				gaitControl.setSpiderModeRatio(speedUpAndDown(spiderWalkLegRatio));
				return;
			}


			return;
		}
		if (targetGaitMode == OneLegInTheAir) {
			if (gaitControl.getTargetGaitMode() == TwoLegsInTheAir) {
				// no need to wait for sync point, switch immediately
				gaitControl.setTargetGaitMode(targetGaitMode);
				gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
				return;
			}
			if ((gaitControl.getTargetGaitMode() == FourLegWalk) || (gaitControl.getTargetGaitMode() == SpiderWalk)) {
				if ((gaitControl.getToePointsWorld()[NumberOfLegs/2].distanceSqr(gaitControl.getGaitRefPointsWorld()[NumberOfLegs/2]) < sqr(maxFootSpeed*dT)) &&
					(legJustWentDown[0] || legJustWentDown[1] || legJustWentDown[3] || legJustWentDown[4] || (numberOfLegsOnTheGround == 4))) {
					gaitControl.setTargetGaitMode(targetGaitMode);
					gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
					gaitControl.setIncludeFrontLeg(true);
				}
			}
			return;
		}
		if (targetGaitMode == TwoLegsInTheAir) {
			if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) || (gaitControl.getTargetGaitMode() == SpiderWalk)) {
				// no need to wait for sync point, switch immediately
				gaitControl.setTargetGaitMode(targetGaitMode);
				gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
				return;
			}
			if ((gaitControl.getTargetGaitMode() == FourLegWalk)) {
				if ((gaitControl.getToePointsWorld()[NumberOfLegs/2].distanceSqr(gaitControl.getGaitRefPointsWorld()[NumberOfLegs/2]) < sqr(maxFootSpeed*dT)) &&
					(legJustWentDown[0] || legJustWentDown[1] || legJustWentDown[3] || legJustWentDown[4] || (numberOfLegsOnTheGround == 4))) {
					gaitControl.setTargetGaitMode(targetGaitMode);
					gaitControl.setIncludeFrontLeg(true);
					gaitControl.setForceGait(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore

					return;
				}
			}
			return;
		}
	}
}

void Engine::computeGaitRefPointRadius() {
	switch (generalMode) {
		case FallASleep:
		case BeingAsleep:
			gaitControl.setTargetGaitRefPointsRadius (sleepingFootTouchPointRadius);
			inputBodyPose.orientation = Rotation(0,0,0);
			inputBodyPose.position.z = constrain(inputBodyPose.position.z, minBodyHeight, maxBodyHeight);
			break;
		default: {
				if ((generalMode != LiftBody) && (generalMode != FallASleep)) {
					realnum heightOverGround = moderatedBodyPose.position.z  - gaitControl.getAvrPerpendicularGroundHeight();
					heightOverGround = constrain(heightOverGround, minBodyHeight, maxBodyHeight);
					realnum radius = minFootTouchPointRadius + (maxBodyHeight - heightOverGround-minBodyHeight)/(maxBodyHeight-minBodyHeight)*(maxFootTouchPointRadius-minFootTouchPointRadius);
					gaitControl.setTargetGaitRefPointsRadius (radius);
				} else {
					gaitControl.setTargetGaitRefPointsRadius (sleepingFootTouchPointRadius);
					inputBodyPose.orientation = Rotation(0,0,0);
				}
				if (fourWalkLegRatio > 0) {
					// Hip offset is set in order to reflect the 5-leg polygon walk resp. the 4-leg gait shaped as a square
					// fourWalkLegRatio is a factor going from 0 to 1 used to have a smooth migration between both modes
					realnum hipOffset = fourWalkLegRatio*radians(360/NumberOfLegs - 360/4);

					// change only other legs than the front leg
					bodyKinematics.getLeg(0).setHipOffset(hipOffset);
					bodyKinematics.getLeg(1).setHipOffset(hipOffset);
					bodyKinematics.getLeg(NumberOfLegs-1).setHipOffset(-hipOffset);
					bodyKinematics.getLeg(NumberOfLegs-2).setHipOffset(-hipOffset);
				}
				if (spiderWalkLegRatio > 0) {
					// Hip offset is set in order to reflect the 5-leg polygon walk resp. the 4-leg gait shaped as a square
					// fourWalkLegRatio is a factor going from 0 to 1 used to have a smooth migration between both modes
					realnum hipOffset = spiderWalkLegRatio*radians(360/NumberOfLegs - 360/7);

					// change only other legs than the front leg
					bodyKinematics.getLeg(0).setHipOffset(-hipOffset);
					bodyKinematics.getLeg(1).setHipOffset(hipOffset);
					bodyKinematics.getLeg(NumberOfLegs-1).setHipOffset(hipOffset);
					bodyKinematics.getLeg(NumberOfLegs-2).setHipOffset(-hipOffset);
				}

			}
			break;
	}
}

void Engine::computeGaitSpeed() {
	// gait speed is computed by defining a to-be gait length. In order to not
	// let it stumble when big changes occur in terms of speed, angular speed,
	// or body speed, reduce the gait step length accordingly

	// but conmpute acceleration of speed and angular speed first
	realnum dT = gaitSpeedSampler.dT();
	if (dT > 0.0) {
		realnum speedAcc = (getTargetSpeedLimited() - getCurrentSpeed())/dT;
		speedAcc = constrain(speedAcc, -maxAcceleration, maxAcceleration);
		realnum angularSpeedAcc = (getTargetAngularSpeedLimited() - getCurrentAngularSpeed())/dT;
		angularSpeedAcc = constrain(angularSpeedAcc, -maxAngularSpeedAcceleration, maxAngularSpeedAcceleration);

		// gait length is 130mm in normal conditions, but is reduced if any disturbance happens
		// later on, this leads to small steps as long as the disturbance takes
		realnum gaitStepLength =  130.0
								  - 40.0*(moderatedBodyPose.position.z - minBodyHeight)/(maxBodyHeight - minBodyHeight)
								  - 50.0*abs(angularSpeedAcc)/maxAngularSpeedAcceleration
								  - 50.0*abs(speedAcc)/maxAcceleration;
		gaitStepLength =  constrain(gaitStepLength, 40.0, 130.0); // take care that there is a minimum gait step length

		realnum gaitStepLengthDiff = gaitStepLength - lastGaitStepLength;
		gaitStepLengthDiff = constrain(gaitStepLengthDiff, -4.0, 4.0);
		gaitStepLength = lastGaitStepLength + gaitStepLengthDiff;

		lastGaitStepLength = gaitStepLength;
		const realnum gaitSpeedPerBodySpeed = 1.0/80; // [gaits/(mm/s)] one gait per 80mm/s speed of body

		// compute foot speed and body speed
		realnum fastestFootSpeed = gaitControl.getFastestFootSpeed();

		// moderated body is following the input body position, but limited by max speed
		realnum bodySpeed = 0;
		if (dT > 0)
			bodySpeed = moderatedBodyPose.distance(lastModeratedBodyPose)/dT;
		lastModeratedBodyPose = moderatedBodyPose;

		// gait comes out of biggest speed of any foot or the body.
		realnum footOntheGroundPercentage = gaitControl.getFootOnTheGroundRatio(fastestFootSpeed);
		realnum gaitSpeed = max(footOntheGroundPercentage*fastestFootSpeed/gaitStepLength,bodySpeed*gaitSpeedPerBodySpeed);

		// limit the gait frequency mainly for optical reasons.
		gaitSpeed = constrain(gaitSpeed,minGaitFrequency, maxGaitFrequency);

		// during startup procedure, move slow
		if ((generalMode == LiftBody) || (generalMode == FallASleep)) {
			gaitSpeed = minGaitFrequency;
		}

	gaitControl.setGaitSpeed(gaitSpeed);
	}
}

void Engine::computeWakeUpProcedure() {
	if ((generalMode == BeingAsleep) ||
		(generalMode == LiftBody) ||
		(generalMode == FallASleep)) {
		targetGaitMode = OneLegInTheAir;
		gaitControl.setTargetGaitMode(targetGaitMode);
	}

	// After lifting the body, when close to the target position, stop with wake-up-Procedure
	if ((generalMode == LiftBody)) {
		if ((moderatedBodyPose.position.distanceSqr(inputBodyPose.position) < sqr(2.0)) &&
			(gaitControl.getFeetOnTheGround() == NumberOfLegs))  {
			targetGaitMode = TwoLegsInTheAir;
			generalMode = WalkingMode;
			bodyKinematics.startupPhase(false);
		}
	}

	if (generalMode == FallASleep) {
		if (gaitControl.getFeetOnTheGround() == NumberOfLegs) {
			gaitControl.setForceGait(DO_NOT_ADAPT_GAIT_POINT);
			if (abs(moderatedBodyPose.position.z-minBodyHeight) < 1.0) {
				// falling asleep is done
				generalMode = BeingAsleep;
			};

		}
	}
}


void Engine::setTargetGaitMode(GaitModeType newGaitType) {
	targetGaitMode = newGaitType;
};


void Engine::computeGaitHeight() {

	// above a constant height, moderate the gait height depending on body height
	// At minHeight, gaitHeight is 60, at max height, gaitheight is 90mm
	// realnum currentHeight = moderatedBodyPose.position.z;
	realnum gaitHeight = 70; // + 20 * moderate( (currentHeight - minBodyHeight)/(maxBodyHeight-minBodyHeight), 1.0);

	gaitControl.setGaitHeight(gaitHeight, 55);
}

void Engine::computeFrontLeg() {
	realnum dT = frontLegSampler.dT();
	Point currentPosition = gaitControl.getToePointsWorld(NumberOfLegs/2);
	Point currentTouchPoint = gaitControl.getGaitRefPointsWorld(NumberOfLegs/2);

	// if we switch to Four-Leg Gait, move slowly to pose of front leg once
	if (targetGaitMode == FourLegWalk) {
		if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) && (fourWalkLegRatio > floatPrecision)) {
			currentPosition.moveTo(frontLegPose.position,dT, maxFootSpeed);
		}
	}

	// if we switch back to normal, move slowly to touch point
	if (targetGaitMode != FourLegWalk) {
		if (gaitControl.getTargetGaitMode() == FourLegWalk) {
			currentPosition.moveTo(currentTouchPoint,dT, maxFootSpeed);
		}
	};

	// if we are in four leg gait, move the front leg
	if ((targetGaitMode == FourLegWalk) && (gaitControl.getTargetGaitMode() == FourLegWalk))
		currentPosition.moveTo(frontLegPose.position,dT, maxFootSpeed);

	gaitControl.setFrontLegWorld(currentPosition);
}


void Engine::imposeDistanceSensors(realnum distance[NumberOfLegs]) {
	processDistanceSensors(distance);
}

void Engine::processDistanceSensors(realnum distance[NumberOfLegs]) {

	if (generalMode == TerrainMode) {
		realnum groundHeight[NumberOfLegs];
		for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
			if ((distance[legNo] >= 0) && (distance[legNo] < 200)) {

				// if we are not in the right leg phase, do not change the ground height correction
				groundHeight[legNo] = gaitControl.getAbsoluteGroundHeight(legNo);
				realnum toeZ = gaitControl.getToePointsWorld(legNo).z;
				bool newPhase = gaitControl.getLegsGaitPhase(legNo) != gaitControl.getLastGaitPhase(legNo);
				switch (gaitControl.getLegsGaitPhase(legNo)) {
						break;
					case LegMovesDown: {
						if (newPhase) {
							gaitControl.setAbsoluteGroundHeight(legNo,0);
						}
						// compute perpendicular distance to ground by angle of lower leg
						realnum currGroundHeight = toeZ - distance[legNo] * cos(bodyKinematics.getFootAngle(legNo));
						groundHeight[legNo] = currGroundHeight;
						break;
					}
					case LegMovesUp: {
						if (newPhase)
							gaitControl.setAbsoluteGroundHeight(legNo,0);
						realnum currGroundHeight = -toeZ + distance[legNo] * cos(bodyKinematics.getFootAngle(legNo));
						groundHeight[legNo] = currGroundHeight;
						break;
					}
					case LegOnGround:
						// reset the ground height
						// gaitControl.setAbsoluteGroundHeight(legNo,0);
						break;
				}
			}
		}

		// level ground height of all legs such that average is 0
		// this has an influence on the legs in the air only.
		// legs in stance do not react on AbsoluteGroundHeight within gaitController
		realnum totalGroundHeight = 0;
		for (int legNo = 0;legNo< NumberOfLegs;legNo++)
			totalGroundHeight += groundHeight[legNo];
		totalGroundHeight /= NumberOfLegs;

		// tell the gait controller the absolute height of the ground
		// such that the average of all corrections is zero
		for (int legNo = 0;legNo< NumberOfLegs;legNo++) {
			groundHeight[legNo] -= totalGroundHeight;

			gaitControl.setAbsoluteGroundHeight(legNo, groundHeight[legNo]);
		}
	}
}


void Engine::computeSensoredDistance() {
	// average all differences between assumed and measured ground
	realnum totalGroundHeight = 0;
	for (int i = 0;i<NumberOfLegs;i++) {
		realnum measuredGroundHeight = gaitControl.getAbsoluteGroundHeight(i);

		totalGroundHeight += (measuredGroundHeight);
	}
	totalGroundHeight /= NumberOfLegs;

	// compute and lowpass (1s reaction time) hump compensation to get a leveled walk on bumpy ground
	realnum dT = humpCompensationFilterSampler.dT();
	humpsCompensation = lowpass(humpsCompensation, totalGroundHeight, 1.0, dT);
}



// pass and impose current feet points. Used during startup procedure to tell the initial position
void Engine::imposeFootPointsWorld(const PentaPointType& footPointsWorld) {
	// pass to gait control first to get the foot points in local coord systems
	gaitControl.imposeFootPointsWorld(footPointsWorld);
	PentaPointType footPoints = gaitControl.getToePoints();
	PentaPointType walkingTouchPoints = gaitControl.getCurrentWalkingTouchPoints();

	// compute kinematics to get the angle of all feet
	PentaPointType tmpHipPoints;
	bool ok = bodyKinematics.computeKinematics(
									currentBodyPose,
									footPoints,
									walkingTouchPoints,
									tmpHipPoints,
									legAngles,
									groundPoints);

	if (ok) {
		gaitControl.imposeFootPointsWorld(footPointsWorld);
	}
};

void Engine::getState(EngineState &data) {
	data.currentBodyPose = currentBodyPose;
	data.legAngles = legAngles;
	data.isTurnedOn = isTurnedOn();
	data.currentNoseOrientation = getCurrentNoseOrientation();
	data.currentSpeed = getCurrentSpeed();
	data.currentAngularSpeed = getCurrentAngularSpeed();
	data.currentWalkingDirection = getCurrentWalkingDirection();
	data.currentGaitMode = getGaitMode();
	data.engineMode = getGeneralMode();
	data.currentGaitRefPoints = getGaitRefPoints();
	data.frontLegPose = getFrontLegPoseWorld();
	data.currentOdomPose = getOdomPose();
	data.toePointsWorld = gaitControl.getToePointsWorld();
	data.hipPoseWorld = getHipPoseWorld();
	data.groundPoints = getGroundPoints();
	data.currentBaselinkPose = getOdomPose();

	for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
		data.footOnGroundFlag[legNo] = getFootOnGround()[legNo];
		data.legPhase[legNo] = gaitControl.getLegsGaitPhase(legNo);
		data.footAngle[legNo] = getFootAngle(legNo);
	}
}


void Engine::computeAcceleration() {
	realnum dT = movementSample.dT();

	// either we have a changed initiated by the user or we fall asleep and need to decellerate
	if (isListeningToMovements() || (generalMode == FallASleep)) {

		// limit the target speed and target angular speed in case bot is walking and rotating, which might exceed maximum speed
		realnum limitedTargetSpeed = getTargetSpeedLimited();
		realnum limitedTargetAngularSpeed= getTargetAngularSpeedLimited();

		// accelerate to limitedTargetSpeed
		realnum newSpeed = getCurrentSpeed();
		if (abs(getCurrentSpeed() - limitedTargetSpeed) > floatPrecision) {
			realnum speedAcc= (limitedTargetSpeed - getCurrentSpeed())/dT;
			realnum allowedAcceleration = maxAcceleration;
			if (sgn(speedAcc) != sgn(getCurrentSpeed()))
				allowedAcceleration *= 4.0;
			speedAcc = constrain(speedAcc, -allowedAcceleration, allowedAcceleration);
			newSpeed += speedAcc*dT;
			newSpeed = constrain(newSpeed, -maxSpeed, maxSpeed);

		}

		// move towards limitedTargetAngularSpeed
		realnum newAngularSpeed = gaitControl.getCurrentAngularSpeed();
		if (abs(newAngularSpeed - limitedTargetAngularSpeed)> floatPrecision) {
			realnum angularSpeedAcc= (limitedTargetAngularSpeed - getCurrentAngularSpeed())/dT;
			angularSpeedAcc = constrain(angularSpeedAcc, -maxAngularSpeedAcceleration, maxAngularSpeedAcceleration);
			newAngularSpeed += angularSpeedAcc*dT;
			newAngularSpeed = constrain(newAngularSpeed, -maxAngularSpeed, +maxAngularSpeed);
		}

		// really set the new values
		gaitControl.getCurrentSpeed() = newSpeed;
		gaitControl.getCurrentAngularSpeed() = newAngularSpeed;

		// move towards target walking direction
		angle_rad absTargetWalkingDirection = getTargetWalkingDirection() + getCurrentNoseOrientation();
		if (abs(absTargetWalkingDirection - gaitControl.getCurrentAbsWalkingDirection()) > floatPrecision) {
			realnum speedDirectionAcce= (absTargetWalkingDirection - gaitControl.getCurrentAbsWalkingDirection())*maxAngularSpeedPerSpeed/dT;
			speedDirectionAcce = constrain(speedDirectionAcce, -maxAngularSpeedPerSpeed, maxAngularSpeedPerSpeed);
			gaitControl.getCurrentAbsWalkingDirection() += speedDirectionAcce*dT;
		}

		// rotate speed direction according to rotation speed
		angle_rad currentWalkingDirection = gaitControl.getCurrentAbsWalkingDirection();
		currentWalkingDirection += getCurrentAngularSpeed()*dT;

		if (currentWalkingDirection > M_PI)
			currentWalkingDirection -= 2.0*M_PI;
		if (currentWalkingDirection < -M_PI)
			currentWalkingDirection += 2.0*M_PI;

		gaitControl.getCurrentAbsWalkingDirection() = currentWalkingDirection;

		// rotate nose along the angular speed
		angle_rad noseOrientation = getCurrentNoseOrientation();
		noseOrientation += getCurrentAngularSpeed()*dT;
		if (noseOrientation > M_PI)
			noseOrientation -= 2*M_PI;
		if (noseOrientation < -M_PI)
			noseOrientation += 2*M_PI;

		getBodyKinematics().setCurrentNoseOrientation(noseOrientation);
	}
}
