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
	cortex.setupCortexCommunication(i2cport, i2cadr, cortextSerialPort, cortexSerialBaudRate);
	ok = cortex.isCortexCommunicationOk();

	// if cortex is up and running
	if (ok) {
		ROS_DEBUG_STREAM("fetch angles");

		// fetch the angles via cortex to initialize the feet with it
		ok = cortex.fetchAngles(legAngles);
		Rotation imuOrientation = cortex.getIMUOrientation();
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
	cortex.setup();

    frontLegPose.position = Point(280,1,80);

    // assume that touches is on the ground
	inputBodyPose.position.z = minBodyHeight;
	inputBodyPose.orientation.null();
	moderatedBodyPose = inputBodyPose;
	currentBodyPose = inputBodyPose;

	fourWalkLegRatio = 0; // we start with 5 legs mode. 0 % of 4 legs mode
	spiderWalkLegRatio = 0; // we start with 5 legs mode.

	gaitControl.setTargetGaitMode(OneLegInTheAir);
	currentGaitMode = OneLegInTheAir;
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
		inputBodyPose.position.z = standardBodyHeigh;
		bodyKinematics.startupPhase(true);
		gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
	}
}

void Engine::fallAsleep() {
	ROS_DEBUG_STREAM("fall asleep");
	generalMode = FallASleep;
	inputBodyPose.position = Point(0,0,minBodyHeight);
	inputBodyPose.orientation = Rotation(0,0,0);
	targetAngularSpeed = 0;
	targetSpeed = 0;
	gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
}

void Engine::terrainMode(bool terrainModeOn) {
	ROS_DEBUG_STREAM("terrainMode(" << terrainModeOn <<")");
	if (terrainModeOn) {
		generalMode = TerrainMode;
	}
	else
		generalMode = WalkingMode;

	gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
}

void Engine::turnOn() {
	ROS_DEBUG_STREAM("turnON()");

	setupCommon();

	if (cortex.isCortexCommunicationOk()) {

		CriticalBlock block(loopMutex);

		// setup bot and fetch the angles
		bool ok = cortex.setupBot();
		ok = ok && cortex.fetchAngles(legAngles);
		Rotation imuOrientation = cortex.getIMUOrientation();
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
			PentaLegAngleType allLegAngles = legAngles;
			ok = bodyKinematics.computeKinematics(
										bodyPose,
										gaitControl.getToePoints(),
										gaitControl.getCurrentWalkingTouchPoints(),
										tmpHipPoints,
										allLegAngles,
										groundPoints);
			if (ok) {
				ok = ok && cortex.enableBot();
				ok = ok && cortex.moveSync (allLegAngles, 1000);

				// in the last second, gait went on, so impose the old state to not hiccup within the loop
				imposeFootPointsWorld(footPoints);
				turnedOn = true;
			}
		}
	}

	gaitControl.reset();
	gaitSpeedSampler.reset();
	turnedOn = true;
}

void Engine::turnOff() {
	ROS_DEBUG_STREAM("turnOff");
	if (cortex.isCortexCommunicationOk())
		cortex.disableBot();
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
	if (cortex.betterShutMeDown())
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

PentaLegAngleType Engine::getLegAngles() {
	return legAngles;
};

PentaPointType Engine::getHipPoints() {
	return hipPoints;
}


bool Engine::ratedloop() {
	if (mainLoopTimeSample.isDue(CORTEX_SAMPLE_RATE)) {
		// CriticalBlock criticalBlock(loopMutex);
		loop();
		return true;
	}
	return false;
}

void Engine::loop() {
	// send last commmand to legController, return
	// state and distance to grounds
	// do this first to have a precise timing
	cortex.loop();

	PentaPointType tmpHipPoints;
	PentaLegAngleType allLegAngles = legAngles;

	// grab last measurement of foot sensors
	if (cortex.isCortexCommunicationOk()) {
		realnum distance[NumberOfLegs];
		cortex.getDistanceSensor(distance);

		// LOG(DEBUG) << "GROUND-z" << gaitControl.getAbsoluteGroundHeight(0) << " " << gaitControl.getAbsoluteGroundHeight(1) << " "<< gaitControl.getAbsoluteGroundHeight(2) << " "<< gaitControl.getAbsoluteGroundHeight(3) << " "<< gaitControl.getAbsoluteGroundHeight(4);
		// LOG(DEBUG) << "Toe Points" << gaitControl.getToePoints()[0].z << " " << gaitControl.getToePoints()[1].z << " "<< gaitControl.getToePoints()[2].z << " "<< gaitControl.getToePoints()[3].z << " "<< gaitControl.getToePoints()[4].z;

		processDistanceSensors(distance);

		// if not turned on, fetch the angles and update the pose
		if (!turnedOn) {
			bool ok = cortex.fetchAngles(allLegAngles);
			Rotation imuOrientation = cortex.getIMUOrientation();
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
		computeGaitCircleRadius();
		computeBodyPose();
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
		if (cortex.isCortexCommunicationOk()) {
			cortex.setMovement (allLegAngles, CORTEX_SAMPLE_RATE);
		}
	}

	// did a fatal error occur? Then fall asleep and shutdown
	if (cortex.betterShutMeDown()) {
		switch (shutdownMode) {
			case NoShutDownActive: {
				ROS_ERROR_STREAM("Better shut me down due to fatal error. Go down and stop moving.");
				shutdownMode = InitiateShutDown;
				targetSpeed = 0;
				targetAngularSpeed = 0;
				inputBodyPose = Pose(Point(0,0,standardBodyHeigh), Rotation(0,0,0));
			}
			case InitiateShutDown: {
				if ((currentBodyPose.position.z - standardBodyHeigh < floatPrecision)
					 && (getCurrentSpeed() < floatPrecision)) {
					ROS_ERROR_STREAM("Better shut me down due to fatal error. Fall asleep.");
					fallAsleep();
					shutdownMode = ExecuteShutdown;
				}
			}
			case ExecuteShutdown: {
				if (generalMode == BeingAsleep) {
					ROS_ERROR_STREAM("Better shut me down due to fatal error. Turn off.");

					turnOff();
					shutdownMode = ShutdownDone;
				}
			}
			case ShutdownDone:
				break;

		}
	}
}

mmPerSecond& Engine::getTargetSpeed () {
	return targetSpeed;
};


mmPerSecond Engine::getTargetSpeedLimited() {
	realnum targetSpeedByRotation = targetAngularSpeed*gaitControl.getGaitRefCircleRadius()/(2.0*M_PI);
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
	if (cortex.betterShutMeDown())
		return;

	targetSpeed = newSpeed;
};

void Engine::setTargetAngularSpeed(radPerSecond rotateZ) {
	// do not accept any more commands when in danger
	if (cortex.betterShutMeDown())
		return;

	rotateZ = constrain(rotateZ, -maxAngularSpeed, maxAngularSpeed);
	targetAngularSpeed = rotateZ;
};

radPerSecond& Engine::getTargetAngularSpeed() {
	return targetAngularSpeed;
};

radPerSecond Engine::getTargetAngularSpeedLimited() {
	realnum targetSpeedByRotation = targetAngularSpeed*gaitControl.getGaitRefCircleRadius()/(2.0*M_PI);
	realnum totalTargetSpeed = abs(targetSpeed) + abs(targetSpeedByRotation);
	realnum allowedFootSpeedRatio = totalTargetSpeed/maxSpeed;
	realnum limitedTargetAngularSpeed= targetAngularSpeed;
	if (allowedFootSpeedRatio > 1.0)
		limitedTargetAngularSpeed /= allowedFootSpeedRatio;

	return limitedTargetAngularSpeed;
};


void Engine::setTargetWalkingDirection(angle_rad newWalkingDirection) {
	// do not accept any more commands when in danger
	if (cortex.betterShutMeDown())
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

	const realnum maxBodyOrientationSpeed = 0.8; 	// [RAD/s]

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
		bool readyForLifting = (gaitControl.getFeetOnTheGround() == NumberOfLegs)
				               && ((gaitControl.getAdaptionTypeToGaitRefPoint() == DO_NOT_ADAPT_GAIT_POINT) || (gaitControl.distanceToGaitRefPoints() < standUpWhenDistanceSmallerThan))
							   && (gaitControl.getCurrentSpeed() < floatPrecision);
		if (((generalMode != LiftBody) && (generalMode != FallASleep))
			|| ((generalMode == LiftBody)  && readyForLifting)
			|| ((generalMode == FallASleep) && readyForLifting)) {
			realnum bodySpeed = maxBodyHeightSpeed;
;

			if ((generalMode != WalkingMode) && (generalMode != TerrainMode))
				gaitControl.setAdaptionTypeToGaitRefPoint(DO_NOT_ADAPT_GAIT_POINT);
			if ((generalMode == LiftBody) || (generalMode == FallASleep))
				bodySpeed = maxLiftBodyPositionSpeed;
			moderatedBodyPose.moveTo(inputBodyPose, dT, bodySpeed, maxBodyOrientationSpeed);
		}

		Pose toBePose = moderatedBodyPose;

		// compensate according to IMU measurement (only when walking)
		Rotation imu = cortex.getIMUOrientation();
		imu.z = 0;
		Pose imuCompensation;
		Rotation error;
		Rotation maxError (radians(15.0), radians(15.0), radians(0.0));

		bool modeIsIMUAware = (generalMode == WalkingMode) || (generalMode == TerrainMode);
		if (modeIsIMUAware) {
			if (cortex.isIMUValueValid()) {
				// PID controller on orientation of x/y axis only, z is not used
				error = toBePose.orientation - imu ;
				imuCompensation.orientation = imuPID.getPID(error, 0.50, 7.0, 0.0, maxError);
			}
			else {
				if (cortex.isCortexCommunicationOk())
					ROS_WARN_STREAM("IMU value is invalid");
			}
		} else {
			// in any other mode than walking keep the IMU in a reset state, so slowly reset it
			imuCompensation.orientation = imuPID.getPID(Rotation(), 0.50, 7.0, 0.0, maxError);
		}
		currentBodyPose = toBePose;
		currentBodyPose.orientation += imuCompensation.orientation;
		ROS_DEBUG_STREAM("IMU(" << std::setprecision(2) << degrees(imu.x) << "," << degrees(imu.y) << "|" << degrees(imuPID.getErrorIntegral().x) << "," << degrees(imuPID.getErrorIntegral().y) << ") error=("<< std::setprecision(3) << degrees(imu.x) << "," << degrees(imu.y)
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

	for (int i = 0;i<NumberOfLegs;i++) {
		lastFeetOnGround[i] = legOnGround[i];
	}

	// if we are not in FourLegMode but havent fully reached it, work on it
	if (currentGaitMode == gaitControl.getTargetGaitMode() && (gaitControl.getTargetGaitMode() != FourLegWalk) && (fourWalkLegRatio > floatPrecision)) {
		fourWalkLegRatio -= dT/switchGaitDuration;
		gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT);
		if (fourWalkLegRatio <= 0.0) {
			fourWalkLegRatio = 0.0;
			gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
		}
	}

	// if we are not in SpiderMode but havent fully reached it, work on it
	if (currentGaitMode == gaitControl.getTargetGaitMode() && (gaitControl.getTargetGaitMode() != SpiderWalk) && (spiderWalkLegRatio > floatPrecision)) {
		spiderWalkLegRatio -= dT/switchGaitDuration;
		gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT);
		if (spiderWalkLegRatio <= 0.0) {
			spiderWalkLegRatio = 0.0;
			gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
		}
	}

	// find out if we switched to OneLeg mode, but are not yet there. Then wait for next time.
	if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) && (numberOfLegsOnTheGround < NumberOfLegs - 1)){
		return;
	}

	// if we want to change the mode, carry out specialized procedures
	if (currentGaitMode != gaitControl.getTargetGaitMode()) {
		if (currentGaitMode == FourLegWalk) {
			// switching to four leg mode happens in phase. First switch to OneLegInTheAir if not there already
			if (gaitControl.getTargetGaitMode() == TwoLegsInTheAir) {
				// no sync point necessary, switch immediately and wait until mode is complete (see above)
				gaitControl.setTargetGaitMode(OneLegInTheAir);
				gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT);	// start gait switch, force a gait even when not moving to carry out switch
				return;
			}

			// if in full OneLegInTheAir mode, and Four-Leg-position is not yet reached, move towards it.
			// when finished, switch to FourLeg mode
			if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) &&
				(numberOfLegsOnTheGround >=  NumberOfLegs - 1)) {
				if (((fourWalkLegRatio < floatPrecision) && legJustWentUp[NumberOfLegs/2]) ||
					((fourWalkLegRatio > floatPrecision) && !legOnGround[NumberOfLegs/2]))	{
					fourWalkLegRatio += dT/switchGaitDuration; // this is time critical, switch must happen within one leg movement
					gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT);
					gaitControl.setIncludeFrontLeg(false);
					if (fourWalkLegRatio >= 1.0) {
						gaitControl.setTargetGaitMode(FourLegWalk);
						gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
					}
				}
				return;
			}
			return;
		}
		if (currentGaitMode == SpiderWalk) {
			// if in full OneLegInTheAir mode, and Spider-Leg-position is not yet reached, move towards it.
			// when finished, switch to FourLeg mode
			if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) &&
				(numberOfLegsOnTheGround >=  NumberOfLegs - 1)) {
				if (((spiderWalkLegRatio < floatPrecision) && legJustWentUp[NumberOfLegs/2]) ||
					((spiderWalkLegRatio > floatPrecision) && !legOnGround[NumberOfLegs/2]))	{
					spiderWalkLegRatio += dT/switchGaitDuration; // this is time critical, switch must happen within one leg movement
					gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT);
					gaitControl.setIncludeFrontLeg(true);
					if (spiderWalkLegRatio >= 1.0) {
						gaitControl.setTargetGaitMode(SpiderWalk);
						gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
					}
				}
				return;
			}

			if ((gaitControl.getTargetGaitMode() == TwoLegsInTheAir)) {
				spiderWalkLegRatio += dT/switchGaitDuration; // this is time critical, switch must happen within one leg movement
				gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT);
				gaitControl.setIncludeFrontLeg(true);
				if (spiderWalkLegRatio >= 1.0) {
					gaitControl.setTargetGaitMode(SpiderWalk);
					gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
				}
				return;
			}
			return;
		}
		if (currentGaitMode == OneLegInTheAir) {
			if (gaitControl.getTargetGaitMode() == TwoLegsInTheAir) {
				// no need to wait for sync point, switch immediately
				gaitControl.setTargetGaitMode(currentGaitMode);
				gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
				return;
			}
			if ((gaitControl.getTargetGaitMode() == FourLegWalk) || (gaitControl.getTargetGaitMode() == SpiderWalk)) {
				if ((gaitControl.getToePointsWorld()[NumberOfLegs/2].distanceSqr(gaitControl.getGaitRefPointsWorld()[NumberOfLegs/2]) < sqr(maxFootSpeed*dT)) &&
					(legJustWentDown[0] || legJustWentDown[1] || legJustWentDown[3] || legJustWentDown[4] || (numberOfLegsOnTheGround == 4))) {
					gaitControl.setTargetGaitMode(currentGaitMode);
					gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
					gaitControl.setIncludeFrontLeg(true);
				}
			}
			return;
		}
		if (currentGaitMode == TwoLegsInTheAir) {
			if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) || (gaitControl.getTargetGaitMode() == SpiderWalk)) {
				// no need to wait for sync point, switch immediately
				gaitControl.setTargetGaitMode(currentGaitMode);
				gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore
				return;
			}
			if ((gaitControl.getTargetGaitMode() == FourLegWalk)) {
				if ((gaitControl.getToePointsWorld()[NumberOfLegs/2].distanceSqr(gaitControl.getGaitRefPointsWorld()[NumberOfLegs/2]) < sqr(maxFootSpeed*dT)) &&
					(legJustWentDown[0] || legJustWentDown[1] || legJustWentDown[3] || legJustWentDown[4] || (numberOfLegsOnTheGround == 4))) {
					gaitControl.setTargetGaitMode(currentGaitMode);
					gaitControl.setIncludeFrontLeg(true);
					gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE); // switching done, do not force gait anymore

					return;
				}
			}
			return;
		}
	}
}


void Engine::computeGaitCircleRadius() {
	switch (generalMode) {
		case FallASleep:
			if (moderatedBodyPose.position.z < standardBodyHeigh)
				gaitControl.setTargetGaitCircleRadius (sitDownTouchPointRadius, spiderWalkLegRatio, fourWalkLegRatio);
			else
				gaitControl.setTargetGaitCircleRadius (standUpFootTouchPointRadius, spiderWalkLegRatio, fourWalkLegRatio);

			gaitControl.assignTargetGaitCirclePoints();
			inputBodyPose.orientation = Rotation(0,0,0);
			inputBodyPose.position.z = constrain(inputBodyPose.position.z, minBodyHeight, maxBodyHeight);

			break;
		case BeingAsleep:
			gaitControl.setTargetGaitCircleRadius(standUpFootTouchPointRadius, spiderWalkLegRatio, fourWalkLegRatio);
			gaitControl.assignTargetGaitCirclePoints();
			inputBodyPose.orientation = Rotation(0,0,0);
			inputBodyPose.position.z = constrain(inputBodyPose.position.z, minBodyHeight, maxBodyHeight);
			break;
		case LiftBody:
			gaitControl.setTargetGaitCircleRadius (standUpFootTouchPointRadius, spiderWalkLegRatio, fourWalkLegRatio);
			inputBodyPose.orientation = Rotation(0,0,0);
			break;
		default: {

		    /* the gait circle is computed by assuming that the foot is perpendicular to the ground, i.e. only
			 thigh is moving up. This holds true if the thigh is at least horizontal, when going down even more,
		     not only the thigh goes down but the foot moves to the outside

		     standard position    low body position   high body position
				  \    /                   \   / /\            \   /
			   \__/---|                 \_/-/  \            \_/\
		              |                                         \
		                                                         |
		                                                         |
		     */
			realnum heightOverGround = moderatedBodyPose.position.z  - gaitControl.getAvrPerpendicularGroundHeight();
			heightOverGround = constrain(heightOverGround, minBodyHeight, maxBodyHeight);

			static const realnum horicontalThighHeight = CAD::HipMountingPointOverBody - sin(CAD::HipNickAngle)*(CAD::HipJointLength + CAD::HipLength) + CAD::FootLength;
			static const realnum thighLength = (CAD::ThighKneeGapLength + CAD::ThighLength + CAD::KneeJointLength);
			realnum radius = 0;
			if (heightOverGround > horicontalThighHeight) {
				radius = cos(CAD::HipNickAngle)*(CAD::HipCentreDistance + CAD::HipJointLength + CAD::HipLength)  + sqrt(sqr(thighLength) - sqr((heightOverGround-horicontalThighHeight)));;
			}
			else {
				static const realnum distanceKneeToe = sqrt(sqr(thighLength) + sqr(CAD::FootLength));
				radius = cos(CAD::HipNickAngle)*(CAD::HipCentreDistance + CAD::HipJointLength + CAD::HipLength)  + sqrt(sqr(distanceKneeToe) - sqr(heightOverGround - sin(CAD::HipNickAngle)*(CAD::HipJointLength + CAD::HipLength)));;
			}
			// realnum regularLegLength = (CAD::ThighLength + CAD::HipJointLength + CAD::KneeJointLength  + CAD::FootLength )*0.85;
			// realnum radius = 0.85*sqrt(sqr(regularLegLength) - sqr(heightOverGround)) + CAD::HipCentreDistance  + CAD::HipLength;

			radius -= 30.0;
			gaitControl.setTargetGaitCircleRadius (radius, spiderWalkLegRatio, fourWalkLegRatio);

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
			break;
		}
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
		realnum bodyHeighSpeed = constrain((moderatedBodyPose.position.z - inputBodyPose.position.z)/dT, -maxBodyHeightSpeed, maxBodyHeightSpeed);

		// gait length is 130mm in normal conditions, but is reduced if any disturbance happens
		// later on, this leads to small steps as long as the disturbance takes
		realnum gaitStepLength =  130.0
								  - 20.0*(moderatedBodyPose.position.z - minBodyHeight)/(maxBodyHeight - minBodyHeight)		// the higher, the smaller the gait
								  - 20.0*abs(bodyHeighSpeed/maxBodyHeightSpeed)				  								// the faster the body height changes, the smaller the gait
								  - 80.0*sqr(minBodyHeight/moderatedBodyPose.position.z) 									// if close to the ground, reduce gait length
		                          - 30.0*sqr((moderatedBodyPose.position.z)/maxBodyHeight) 									// if close to the ground, reduce gait length
								  - 30.0*abs(angularSpeedAcc)/maxAngularSpeedAcceleration									// the more angular acceleration, the smaller the gait
								  - 30.0*abs(speedAcc)/maxAcceleration;														// the more acceleration, the smaller the gait
		gaitStepLength =  constrain(gaitStepLength, 40.0, 135.0); // take care that there is a minimum gait step length

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
	if ((generalMode == LiftBody) ||
		(generalMode == FallASleep)) {
		currentGaitMode = OneLegInTheAir;
		gaitControl.setTargetGaitMode(currentGaitMode);
	}

	// After lifting the body, when close to the target position, stop with wake-up-Procedure
	if ((generalMode == LiftBody)) {
		if ((moderatedBodyPose.position.distanceSqr(inputBodyPose.position) < sqr(2.0)) &&
			(gaitControl.getFeetOnTheGround() == NumberOfLegs))  {
			currentGaitMode = TwoLegsInTheAir;
			generalMode = WalkingMode;
			gaitControl.setAdaptionTypeToGaitRefPoint(DO_NOT_ADAPT_GAIT_POINT);
			bodyKinematics.startupPhase(false);
		}
	}


	// if bot moves from FallASleep Phase to BeeingAsleeep phase, all legs needs to be in the right position
	if (generalMode == FallASleep) {
		if ((gaitControl.getFeetOnTheGround() == NumberOfLegs) && (gaitControl.distanceToGaitRefPoints() < moveToeWhenDistanceGreaterThan)) {
			// go down without adapting to gait
			gaitControl.setAdaptionTypeToGaitRefPoint(DO_NOT_ADAPT_GAIT_POINT);
			if (abs(moderatedBodyPose.position.z-minBodyHeight) < floatPrecision) {
				// falling asleep is done
				generalMode = BeingAsleep;
				inputBodyPose.null();
				inputBodyPose.position.z = minBodyHeight;
				gaitControl.setAdaptionTypeToGaitRefPoint(ADAPT_TO_GAIT_POINT_WHERE_APPROPRIATE);
			};
		}
	}
}


void Engine::setGaitMode(GaitModeType newGaitType) {
	currentGaitMode = newGaitType;
};


void Engine::computeGaitHeight() {

	// above a constant height, moderate the gait height depending on body height
	// At minHeight, gaitHeight is 60, at max height, gaitheight is 90mm
	// realnum currentHeight = moderatedBodyPose.position.z;
	realnum gaitHeight = 70; // + 20 * moderate( (currentHeight - minBodyHeight)/(maxBodyHeight-minBodyHeight), 1.0);

	if (generalMode == TerrainMode)
		gaitHeight = 130;

	realnum currentGait = gaitControl.getGaitHeight();
	if (currentGait != gaitHeight) {
		gaitControl.setGaitHeight(gaitHeight, 55);
	}
}

void Engine::computeFrontLeg() {
	realnum dT = frontLegSampler.dT();
	Point currentPosition = gaitControl.getToePointsWorld(NumberOfLegs/2);
	Point currentTouchPoint = gaitControl.getGaitRefPointsWorld(NumberOfLegs/2);

	// if we switch to Four-Leg Gait, move slowly to pose of front leg once
	if (currentGaitMode == FourLegWalk) {
		if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) && (fourWalkLegRatio > floatPrecision)) {
			currentPosition.moveTo(frontLegPose.position,dT, maxFootSpeed);
		}
	}

	// if we switch back to normal, move slowly to touch point
	if (currentGaitMode != FourLegWalk) {
		if (gaitControl.getTargetGaitMode() == FourLegWalk) {
			currentPosition.moveTo(currentTouchPoint,dT, maxFootSpeed);
		}
	};

	// if we are in four leg gait, move the front leg
	if ((currentGaitMode == FourLegWalk) && (gaitControl.getTargetGaitMode() == FourLegWalk))
		currentPosition.moveTo(frontLegPose.position,dT, maxFootSpeed);

	gaitControl.setFrontLegWorld(currentPosition);
}


void Engine::imposeDistanceSensors(realnum distance[NumberOfLegs]) {
	processDistanceSensors(distance);
}

void Engine::processDistanceSensors(realnum distance[NumberOfLegs]) {


	ROS_DEBUG_STREAM("Distance (dist|toe|ground)" << std::fixed << std::setprecision(0)
			   << distance[0]*cos(bodyKinematics.getFootsDeviationAngleFromZ(0)) << ":" << gaitControl.getToePointsWorld(0).z << ":" << gaitControl.getAbsoluteGroundHeight(0) << ((gaitControl.getLegsGaitPhase(0)==LegGaitDown)?"D":"") << ((gaitControl.getLegsGaitPhase(0)==LegGaitUp)?"U":"")<< " "
			   << distance[1]*cos(bodyKinematics.getFootsDeviationAngleFromZ(1)) << ":" << gaitControl.getToePointsWorld(1).z << ":" << gaitControl.getAbsoluteGroundHeight(1) << ((gaitControl.getLegsGaitPhase(1)==LegGaitDown)?"D":"") << ((gaitControl.getLegsGaitPhase(1)==LegGaitUp)?"U":"")<< " "
			   << distance[2]*cos(bodyKinematics.getFootsDeviationAngleFromZ(2)) << ":" << gaitControl.getToePointsWorld(2).z << ":" << gaitControl.getAbsoluteGroundHeight(2) << ((gaitControl.getLegsGaitPhase(2)==LegGaitDown)?"D":"") << ((gaitControl.getLegsGaitPhase(2)==LegGaitUp)?"U":"")<< " "
			   << distance[3]*cos(bodyKinematics.getFootsDeviationAngleFromZ(3)) << ":" << gaitControl.getToePointsWorld(3).z << ":" << gaitControl.getAbsoluteGroundHeight(3) << ((gaitControl.getLegsGaitPhase(3)==LegGaitDown)?"D":"") << ((gaitControl.getLegsGaitPhase(3)==LegGaitUp)?"U":"")<< " "
			   << distance[4]*cos(bodyKinematics.getFootsDeviationAngleFromZ(4)) << ":" << gaitControl.getToePointsWorld(4).z << ":" << gaitControl.getAbsoluteGroundHeight(4) << ((gaitControl.getLegsGaitPhase(4)==LegGaitDown)?"D":"") << ((gaitControl.getLegsGaitPhase(4)==LegGaitUp)?"U":"")<< " ");

	if (generalMode == TerrainMode) {
		realnum groundHeight[NumberOfLegs];


		for (int legNo = 0;legNo<NumberOfLegs;legNo++) {

				// do only ground height correction for the legs that are currently moving
				groundHeight[legNo] = gaitControl.getAbsoluteGroundHeight(legNo);

				// toeZ is the expected height, if the ground was perfectly flat
				realnum toeZ = gaitControl.getToePointsWorld(legNo).z;

				// compute perpendicular distance to ground by angle of lower leg
				realnum perpendicularDistance = distance[legNo];
				bool validDistance = (perpendicularDistance >= 0) && (perpendicularDistance < 200);
				perpendicularDistance *= cos(bodyKinematics.getFootsDeviationAngleFromZ(legNo));

				switch (gaitControl.getLegsGaitPhase(legNo)) {
						break;
					case LegGaitDown: {
						// compare expected and measured distance from ground.
						// the difference is the new absolute ground height
						if (validDistance) {
							groundHeight[legNo] = lowpass (groundHeight[legNo], toeZ - perpendicularDistance, 50 /* ms */, CORTEX_SAMPLE_RATE);
						}
						break;
					}
					case LegGaitUp: {
						// if the toe leaves the ground, reset the absolute height to 0 to
						// avoid that the absolute ground height continously grows
						groundHeight[legNo] = 0;
					}
					case LegGaitDuty:
						// do not the absolute ground height if the toe is on the ground
						break;
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
			// groundHeight[legNo] -= totalGroundHeight;

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
	data.currentIMUAwareBodyPose = currentBodyPose;
	data.moderatedBodyPose = moderatedBodyPose;
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
	data.baseLinkInMapFrame = getOdomPose(); // will be overwritten later on, this asignment is only valid if odomFrame=0, used for simulation only

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

		getBodyKinematics().getCurrentNoseOrientation() = noseOrientation;
	}
}


void Engine::scriptState(milliseconds delay, bool continous) {
	targetScriptMilestoneDelay = delay;
	targetContinousScriptComputation = continous;
	if (!continousScriptComputation) {
		scriptMilestoneDelay = targetScriptMilestoneDelay;
		saveScriptMilestoneDelay = targetScriptMilestoneDelay;
	}
	continousScriptComputation = targetContinousScriptComputation;
}

void Engine::computeScript() {
	// wait until time of milestone is up
	if ((currentScript == Engine::ScriptType::NO_SCRIPT))
		return;

	if (scriptMilestoneDelay > 0) {
		uint32_t now = millis();
		if (lastScriptInvocation > 0) {
			uint32_t timeDiff = now - lastScriptInvocation;
			if (scriptMilestoneDelay > timeDiff) {
				scriptMilestoneDelay -= timeDiff;
				lastScriptInvocation = now;
			}
			else {
				scriptMilestoneDelay = 0;
				currentScriptMilestone++;
				continousScriptComputation = false;
			}
		}
		lastScriptInvocation = now;
	}

	// wait for next milestone?
	realnum continousComputationRatio = 1.0;
	if (scriptMilestoneDelay > 0) {
		continousComputationRatio = (float)(saveScriptMilestoneDelay-scriptMilestoneDelay)/(float)saveScriptMilestoneDelay;

		if (!continousScriptComputation)
			return;
	}

	// last delay passed, execute new milestone
 	switch (currentScript) {
	case WALL_APPEARS: {
		switch (currentScriptMilestone) {
			case 0:
				turnOn();
				wakeUp();
				setTargetSpeed(1);
				setTargetBodyPose(Pose(Point(0,0,100),Rotation(0,0,0)));
				scriptState(7000);
				break;
			case 1:
				setTargetSpeed(0);
				setTargetBodyPose(Pose(Point(0,0,140),Rotation(0,radians(-15),0)));
				scriptState(1000);
				break;
			case 2:
				setTargetBodyPose(Pose(Point(20,0,80),Rotation(0,radians(10),0)));
				scriptState(1000);
				break;
			case 3:
				setTargetBodyPose(Pose(Point(0,0,130),Rotation(0,radians(0),0)));
				scriptState(500);
				break;
			case 4:
				setTargetBodyPose(Pose(Point(0,0,100),Rotation(0,radians(0),0)));
				scriptState(500);
				break;
			case 5:
				setTargetAngularSpeed(M_PI/4.0);
				scriptState(2500);
				break;
			case 6:
				setTargetBodyPose(Pose(Point(0,20,80),Rotation(radians(5),radians(0),0)));
				setTargetAngularSpeed(0);
				setTargetSpeed(1);
				scriptState(800);
				break;
			case 7:
				setTargetBodyPose(Pose(Point(0,-20,80),Rotation(radians(-5),radians(0),0)));
				setTargetAngularSpeed(0);
				setTargetSpeed(1);
				scriptState(800);
				break;
			case 8:
				setTargetBodyPose(Pose(Point(0,20,80),Rotation(radians(5),radians(0),0)));
				setTargetAngularSpeed(0);
				setTargetSpeed(1);
				scriptState(800);
				break;
			case 9:
				setTargetBodyPose(Pose(Point(0,-20,80),Rotation(radians(-5),radians(0),0)));
				setTargetAngularSpeed(0);
				setTargetSpeed(1);
				scriptState(800);
				break;
			case 10:
				setTargetBodyPose(Pose(Point(0,20,80),Rotation(radians(5),radians(0),0)));
				setTargetAngularSpeed(0);
				setTargetSpeed(1);
				scriptState(800);
				break;
			case 11:
				setTargetBodyPose(Pose(Point(0,-20,80),Rotation(radians(-5),radians(0),0)));
				setTargetAngularSpeed(0);
				setTargetSpeed(1);
				scriptState(800);
				break;
			case 12:
				currentScript = NO_SCRIPT;
				break;
		}
		break;
	}
	case HILL_APPEARS: {
		switch (currentScriptMilestone) {
			case 0:
				turnOn();
				wakeUp();
				setTargetSpeed(0);
				setTargetBodyPose(Pose(Point(0,0,100),Rotation(0,0,0)));
				scriptState(5000);
				break;
			case 1:
				setGaitMode(FourLegWalk);
				setTargetFrontLegPose(Point(250,0,100));
				setTargetSpeed(0);
				setTargetBodyPose(Pose(Point(25,0,140),Rotation(radians(0),radians(-7),0)));
				scriptState(2000);
				break;
			case 2:
				setTargetSpeed(1);
				scriptState(500);
				break;
			case 3: {
				realnum duration = 10000;
				realnum alpha = continousComputationRatio*M_PI*2*duration/1000;
				Point p(sin(alpha)*40,cos(alpha),70);
				setTargetFrontLegPose(Point(300 + sin(alpha/2)*40,sin(alpha)*40,150+cos(alpha)*40));
				scriptState(duration, true);
				break;
			}
			case 4:
				setTargetFrontLegPose(Point(280,0,150));
				scriptState(500);
				break;
			case 5:
				currentScript = NO_SCRIPT;
		};
		break;
	}
	case BOX_APPEARS: {
		switch (currentScriptMilestone) {
		case 0:
			turnOn();
			wakeUp();
			setTargetSpeed(1);
			setTargetBodyPose(Pose(Point(0,0,70),Rotation(0,0,0)));
			scriptState(7000);
			break;
		case 1:
			setTargetSpeed(1);
			scriptState(2000);
			break;
		case 2:
			setGaitRadius(280);
			setTargetBodyPose(Pose(Point(0,0,70),Rotation(radians(0),radians(0),0)));
			scriptState(400);
			break;
		case 3: {
			realnum duration = 10000;
			realnum alpha = continousComputationRatio*M_PI*duration/1000;
			Point p(sin(alpha)*50,cos(alpha)*50,70);
			setTargetBodyPose(Pose(p,Rotation(radians(5)*cos(alpha),-radians(5)*sin(alpha),0)));
			scriptState(duration, true);
			break;
		}
		case 4:
			setTargetBodyPose(Pose(Point(0,0,90),Rotation(radians(0),radians(0),0)));
			scriptState(500);
			break;
		case 5:
			currentScript = NO_SCRIPT;
			break;
		default:
			break;
		};
		break;
	}
	case NO_SCRIPT:
		return;

	default:
		break;
	}
}

void Engine::executeScript(ScriptType script ) {
	currentScript = script;
	currentScriptMilestone = 0;
	scriptMilestoneDelay = 0;
	saveScriptMilestoneDelay = 0;
	lastScriptInvocation = 0;
	continousScriptComputation = false;
	targetContinousScriptComputation = false;
}

void Engine::getScript(ScriptType& script, int &milestonenumber) {
	script = currentScript;
	milestonenumber = currentScriptMilestone;

}
