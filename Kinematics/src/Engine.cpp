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
}


bool Engine::setupProduction(string i2cport, int i2cadr, string cortextSerialPort, int cortexSerialBaudRate) {
	ROS_DEBUG_STREAM("Engin::setupProduction");

	bool ok = setupCommon();
	legController.setupCortexCommunication(i2cport, i2cadr, cortextSerialPort, cortexSerialBaudRate);
	ok = legController.isCortexCommunicationOk();

	// if cortex is up and running
	if (ok) {
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
		imposeFootPointsWorld(footPoints);
	}

	return ok;
}

bool Engine::setupSimulation() {
	ROS_DEBUG_STREAM("Engin::setupSimulation");

	bool ok = setupCommon();
	return ok;
}

bool Engine::setupCommon() {

	kinematics.setup(*this);
	bodyKinematics.setup(*this);
	gaitControl.setup(*this);

    frontLegPose.position = Point(280,1,80);

	inputBodyPose.position.z = minBodyHeight;
	moderatedBodyPose = inputBodyPose;
	currentBodyPose = inputBodyPose;
	fourWalkLegRatio = 0; // we start with 5 legs mode. 0 % of 4 legs mode
	gaitControl.setTargetGaitMode(OneLegInTheAir);
	targetGaitMode = OneLegInTheAir;
	humpsCompensation = 0;

	targetWalkingDirection = 0;
	targetSpeed = 0;
	targetAngularSpeed = 0;

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
	turnedOn = false;

	return true;
}

void Engine::wakeUp() {
	ROS_DEBUG_STREAM("wake up");
	if (generalMode != WalkingMode) {
		generalMode = LiftBody;
		bodyKinematics.startupPhase(true);
	}
}

void Engine::fallAsleep() {
	ROS_DEBUG_STREAM("fall asleep");
	generalMode = FallASleep;
	inputBodyPose.position = Point(0,0,minBodyHeight);
	inputBodyPose.orientation = Rotation(0,0,0);
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

	if (legController.isCortexCommunicationOk()) {

		CriticalBlock block(loopMutex);

		// fetch the angles
		bool ok = legController.fetchAngles(legAngles);
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
	ROS_DEBUG_STREAM("setTargetBodyPose(" << newBodyPose << "," << immediately <<")");

	// dont take this if we actually wake up
	if ((generalMode == WalkingMode) || (generalMode == TerrainMode) || (generalMode == LiftBody))
		inputBodyPose = newBodyPose;
	if (immediately) {
		currentBodyPose = newBodyPose;
		moderatedBodyPose = newBodyPose - bodySwing;
	}
}

LegAnglesType Engine::getLegAngles() {
	return legAngles;
};

PentaPointType Engine::getHipPoints() {
	return hipPoints;
}


void Engine::ratedloop() {
	if (mainLoopTimeSample.isDue(CORTEX_SAMPLE_RATE)) {
		CriticalBlock criticalBlock(loopMutex);
		loop();
	}
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
		computeBodySwing();
		computeGaitRefPointRadius();
		computeGaitSpeed();
		computeGaitHeight();
		computeGaitMode();
		computeFrontLeg();
		computeWakeUpProcedure();
		computeMovement();
		gaitControl.loop();
	}

	bool ok = bodyKinematics.computeKinematics(
										currentBodyPose,
										gaitControl.getToePoints(),
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
}

mmPerSecond& Engine::getTargetSpeed () {
	return targetSpeed;
};

void Engine::setTargetSpeed (mmPerSecond newSpeed /* mm/s */) {
	targetSpeed = newSpeed;
};

void Engine::setTargetAngularSpeed(radPerSecond rotateZ) {
	rotateZ = constrain(rotateZ, -maxAngularSpeed, maxAngularSpeed);
	targetAngularSpeed = rotateZ;
};

radPerSecond& Engine::getTargetAngularSpeed() {
	return targetAngularSpeed;
};

void Engine::setTargetWalkingDirection(angle_rad newWalkingDirection) {
	targetWalkingDirection = newWalkingDirection;
};


angle_rad& Engine::getTargetWalkingDirection() {
	return targetWalkingDirection;
};

angle_rad Engine::getCurrentWalkingDirection() {
	angle_rad currentWalkingDirection = gaitControl.getCurrentAbsWalkingDirection() - getCurrentNoseOrientation();
	if (currentWalkingDirection > M_PI)
		currentWalkingDirection = 2*M_PI-currentWalkingDirection;
	if (currentWalkingDirection < -M_PI)
		currentWalkingDirection = currentWalkingDirection + 2*M_PI;
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
	const realnum maxBodyPositionSpeed = 20.0; 	// [mm/s]
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
		// but: if we are in lift mode, wait until all legs are on the ground
		if ((generalMode != LiftBody)
			|| ((generalMode == LiftBody) && (gaitControl.getFeetOnTheGround() == NumberOfLegs) && (gaitControl.distanceToGaitRefPoints() < 5.0)))
			moderatedBodyPose.moveTo(inputBodyPose, dT, maxBodyPositionSpeed, maxBodyOrientationSpeed);

		// add the swing and IMU correction
		Pose toBePose = moderatedBodyPose + bodySwing;

		// compensate according to IMU measurement (only when walking)
		Rotation imu = legController.getIMUOrientation();
		imu.z = 0; // z coordnate is not used
		Pose imuCompensation;
		if (legController.isIMUValueValid() && (generalMode == WalkingMode)) {
			// small PID controller
			Rotation maxError (radians(20.0), radians(20.0), radians(0.0));
			Rotation error = toBePose.orientation - imu ;
			imuCompensation.orientation = imuPID.getPID(error, .8, 2.0, 0.01, maxError);
			// cout << " tobe=" << moderatedBodyPose.orientation << " comp=" << imuCompensation.orientation ;

		} else {
			imuPID.reset();
		}
		currentBodyPose = toBePose  + imuCompensation;
		// cout << " curr=" << currentBodyPose << endl;
	}
}


void Engine::computeGaitMode() {
	realnum dT = gaitModeSampler.dT();

	int numberOfLegsOnTheGround = gaitControl.getFeetOnTheGround();
	static FootOnGroundFlagType lastFeetOnGround = {true,true,true,true,true};
	// turning the legs to FourLegGait-Position takes the time of moving one leg
	realnum switchGaitDuration = 1.0/gaitControl.getGaitSpeed()/NumberOfLegs; // wait one and a half step

	bool legWentDown [NumberOfLegs];
	bool legWentUp [NumberOfLegs];

	// find out which legs touch or leave the ground at this very moment
	const FootOnGroundFlagType& legOnGround = gaitControl.getFeetOnGround();
	for (int i = 0;i<NumberOfLegs;i++) {
		legWentDown[i] = (lastFeetOnGround[i] == false) && (legOnGround[i] == true);
		legWentUp[i]   = (lastFeetOnGround[i] == true)  && (legOnGround[i] == false);
	}

	// cout << "dn" << legWentDown[0] << "|" << legWentDown[1]<< "|" << legWentDown[2] << "|" << legWentDown[3] << "|" << legWentDown[4]
	// 	 << "   up" << legWentUp[0] << "|" << legWentUp[1]<< "|" << legWentUp[2] << "|" << legWentUp[3] << "|" << legWentUp[4]
	//  	 << "   on" << legOnGround[0] << "|" << legOnGround[1]<< "|" << legOnGround[2] << "|" << legOnGround[3] << "|" << legOnGround[4] << endl;

	for (int i = 0;i<NumberOfLegs;i++) {
		lastFeetOnGround[i] = legOnGround[i];
	}

	// if we are not in FourLegMode but havent fully left it, work on it
	if (targetGaitMode == gaitControl.getTargetGaitMode() && (gaitControl.getTargetGaitMode() != FourLegWalk) && (fourWalkLegRatio > floatPrecision)) {
		fourWalkLegRatio -= dT/switchGaitDuration;
		gaitControl.setForceGait(true);
		if (fourWalkLegRatio <= 0.0) {
			fourWalkLegRatio = 0.0;
			gaitControl.setForceGait(false);
		}
		gaitControl.setFourWalkModeRatio(speedUpAndDown(fourWalkLegRatio));
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
				gaitControl.setForceGait(true);	// start gait switch, force a gait even when not moving to carry out switch
				return;
			}

			// if in full OneLegInTheAir mode, and Four-Leg-position is not yet reached, move towards it.
			// when finished, switch to FourLeg mode
			if ((gaitControl.getTargetGaitMode() == OneLegInTheAir) &&
				(numberOfLegsOnTheGround >=  NumberOfLegs - 1)) {
				if (((fourWalkLegRatio < floatPrecision) && legWentUp[NumberOfLegs/2]) ||
					((fourWalkLegRatio > floatPrecision) && !legOnGround[NumberOfLegs/2]))	{
					fourWalkLegRatio += dT/switchGaitDuration; // this is time critical, switch must happen within one leg movement
					gaitControl.setForceGait(true);
					gaitControl.setIncludeFrontLeg(false);
					if (fourWalkLegRatio >= 1.0) {
						gaitControl.setTargetGaitMode(FourLegWalk);
						gaitControl.setForceGait(false); // switching done, do not force gait anymore
					}
				}

				gaitControl.setFourWalkModeRatio(speedUpAndDown(fourWalkLegRatio));
				return;
			}

			return;
		}
		if (targetGaitMode == OneLegInTheAir) {
			if (gaitControl.getTargetGaitMode() == TwoLegsInTheAir) {
				// no need to wait for sync point, switch immediately
				gaitControl.setTargetGaitMode(targetGaitMode);
				gaitControl.setForceGait(false); // switching done, do not force gait anymore
				return;
			}
			if (gaitControl.getTargetGaitMode() == FourLegWalk) {
				if ((gaitControl.getToePointsWorld()[NumberOfLegs/2].distanceSqr(gaitControl.getGaitRefPointsWorld()[NumberOfLegs/2]) < sqr(maxFootSpeed*dT)) &&
					(legWentDown[0] || legWentDown[1] || legWentDown[3] || legWentDown[4] || (numberOfLegsOnTheGround == 4))) {
					gaitControl.setTargetGaitMode(targetGaitMode);
					gaitControl.setForceGait(false); // switching done, do not force gait anymore
					gaitControl.setIncludeFrontLeg(true);
				}
			}
			return;
		}
		if (targetGaitMode == TwoLegsInTheAir) {
			if (gaitControl.getTargetGaitMode() == OneLegInTheAir) {
				// no need to wait for sync point, switch immediately
				gaitControl.setTargetGaitMode(targetGaitMode);
				gaitControl.setForceGait(false); // switching done, do not force gait anymore
				return;
			}
			if (gaitControl.getTargetGaitMode() == FourLegWalk) {
				if ((gaitControl.getToePointsWorld()[NumberOfLegs/2].distanceSqr(gaitControl.getGaitRefPointsWorld()[NumberOfLegs/2]) < sqr(maxFootSpeed*dT)) &&
					(legWentDown[0] || legWentDown[1] || legWentDown[3] || legWentDown[4] || (numberOfLegsOnTheGround == 4))) {
					gaitControl.setTargetGaitMode(targetGaitMode);
					gaitControl.setIncludeFrontLeg(true);
					gaitControl.setForceGait(false); // switching done, do not force gait anymore

					return;
				}
			}
			return;
		}
	}
}

void Engine::computeBodySwing() {
	realnum dT = bodySwingSampler.dT();
	Pose newBodySwing;
	realnum angle;

	realnum gaitRatio = gaitControl.getGaitRatio();
	realnum speed = getTargetSpeed();
	GaitModeType gaitMode = gaitControl.getCurrentGaitMode();

	if (NumberOfLegs % 2 == 0)
		angle = fmod(gaitRatio * (M_PI*2.0) + 1.25*M_PI, M_PI*2.0 );
	else
		angle = fmod(gaitRatio * (M_PI*2.0) + 1.25*M_PI + M_PI/float(NumberOfLegs), M_PI*2.0);

	if (gaitMode == SexyWalk) {
		newBodySwing.position.y = 15.0*cos(-angle);
		newBodySwing.orientation.x = radians(5.0)*sin(angle+M_PI*0.5);
	}

	if (gaitMode == Tripod) {
		realnum angle = fmod(gaitRatio * (M_PI*2.0) + 1.5*M_PI, M_PI*2.0 );
		newBodySwing.position.y = 10.0*sin(angle);
	}

	if ((gaitMode == FourLegWalk) || (targetGaitMode == FourLegWalk)) {
		realnum angle = fmod(gaitRatio * (M_PI*2.0) + 0*.5*M_PI, M_PI*2.0 );
		newBodySwing.position.y = 30.0*sin(angle);
		newBodySwing.position.x = 30.0*cos(angle);
		newBodySwing.position.z = 0; // dont breath, too hectic
	} else {
		// breathing with 0.3 Hz when not moving, and going up to 4 Hz when running
		// use a complementary filter that reacts after 5s when slowing down and 1s if accelerating

		if (false) {
			static realnum breathingFrequency = minBreathingFrequency;
			static realnum breathingAngle = 0;
			realnum breathingRatio = speed/(maxBreathingFromFootSpeed-minBreathingFromFootSpeed);
			realnum newBreathingFrequency = minBreathingFrequency + breathingRatio*(maxBreathingFrequency-minBreathingFrequency);

			realnum filterReactingTime = (newBreathingFrequency > breathingFrequency)?1.0:5.0;

			breathingFrequency = lowpass(breathingFrequency, newBreathingFrequency, filterReactingTime, dT);
			breathingAngle += dT*(breathingFrequency*(2.0*M_PI));
			if (generalMode == FallASleep)
				newBodySwing.position.z = breathingAmplitude*sin(breathingAngle) / 2.0;
			else
				newBodySwing.position.z = breathingAmplitude*sin(breathingAngle);
		}
	}

	// moderate the body swing
	const realnum maxBodySwingSpeed = 100.0; // [mm / gait]
	const realnum maxBodyRotateSpeed = 3.0; // [RAD / gait]

	// gaitSpeed is gaits/s
	bodySwing.moveTo(newBodySwing, dT, maxBodySwingSpeed*gaitControl.getGaitSpeed(), maxBodyRotateSpeed*gaitControl.getGaitSpeed());
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
				if (generalMode != LiftBody) {
					realnum heightOverGround = moderatedBodyPose.position.z  - gaitControl.getAvrPerpendicularGroundHeight();
					heightOverGround = constrain(heightOverGround, minBodyHeight, maxBodyHeight);
					realnum radius = minFootTouchPointRadius + (maxBodyHeight - heightOverGround-minBodyHeight)/(maxBodyHeight-minBodyHeight)*(maxFootTouchPointRadius-minFootTouchPointRadius);
					gaitControl.setTargetGaitRefPointsRadius (radius);
				} else {
					gaitControl.setTargetGaitRefPointsRadius (sleepingFootTouchPointRadius);
					inputBodyPose.orientation = Rotation(0,0,0);
				}
				// Hip offset is set in order to reflect the 5-leg polygon walk resp. the 4-leg gait shaped as a square
				// fourWalkLegRatio is a factor going from 0 to 1 used to have a smooth migration between both modes
				realnum hipOffset = fourWalkLegRatio*radians(360/NumberOfLegs - 360/4);

				// change only other legs than the front leg
				bodyKinematics.getLeg(0).setHipOffset(hipOffset);
				bodyKinematics.getLeg(1).setHipOffset(hipOffset);
				bodyKinematics.getLeg(NumberOfLegs-1).setHipOffset(-hipOffset);
				bodyKinematics.getLeg(NumberOfLegs-2).setHipOffset(-hipOffset);
			}
			break;
	}
}

void Engine::computeGaitSpeed() {
	// gait speed is computed by defining a gait length, i.e. the length
	// one leg is on the ground and compute the gait speed in terms of gaits
	// per second accordingly
	realnum gaitStepLength = 140.0  - 70.0*(moderatedBodyPose.position.z - minBodyHeight)/(maxBodyHeight - minBodyHeight) - 50.0*gaitControl.getCurrentAngularSpeed()/maxAngularSpeed;
	const realnum gaitSpeedPerBodySpeed = 1.0/80; // [gaits/(mm/s)] one gait per 80mm/s speed of body

	// compute foot speed and body speed
	realnum fastestFootSpeed = gaitControl.getFastestFootSpeed();

	// moderated body is following the input body position, but limited by max speed
	static Pose lastModeratedBodyPose = moderatedBodyPose;
	static TimeSamplerStatic gaitSpeedSampler;
	realnum bodySpeed = 0;
	realnum dT = gaitSpeedSampler.dT();
	if (dT > 0)
		bodySpeed = moderatedBodyPose.distance(lastModeratedBodyPose)/dT;
	lastModeratedBodyPose = moderatedBodyPose;

	// gait comes out of biggest speed of any foot or the body.
	realnum footOntheGroundPercentage = gaitControl.getFootOnTheGroundRatio(fastestFootSpeed);
	realnum gaitSpeed = max(footOntheGroundPercentage*fastestFootSpeed/gaitStepLength,bodySpeed*gaitSpeedPerBodySpeed);

	// there's an lower limit of gaitspeed, slow-motion looks weired
	gaitSpeed = constrain(gaitSpeed,minGaitFrequency, maxGaitFrequency);

	// during startup procedure, move slow
	if (generalMode == LiftBody) {
		gaitSpeed = minGaitFrequency;
	}

	gaitControl.setGaitSpeed(gaitSpeed);
}

void Engine::computeWakeUpProcedure() {
	if ((generalMode == BeingAsleep) ||
		(generalMode == LiftBody) ||
		(generalMode == FallASleep)) {
		targetGaitMode = OneLegInTheAir;
		gaitControl.setTargetGaitMode(targetGaitMode);
	}

	// After lifting the body, when close to the target position, stop with wake-up-Procedure
	if ((generalMode == LiftBody) &&
		(moderatedBodyPose.position.distanceSqr(inputBodyPose.position) < sqr(2.0)) &&
		(gaitControl.getFeetOnTheGround() == NumberOfLegs))  {
		targetGaitMode = TwoLegsInTheAir;
		generalMode = WalkingMode;
		bodyKinematics.startupPhase(false);
	}

	if (generalMode == FallASleep) {
		if ((gaitControl.getFeetOnTheGround() == NumberOfLegs) &&
			(abs(moderatedBodyPose.position.z-minBodyHeight) < 1.0))
			generalMode = BeingAsleep;
	}
}


void Engine::setTargetGaitMode(GaitModeType newGaitType) {
	targetGaitMode = newGaitType;
};


void Engine::computeGaitHeight() {
	realnum currentHeight = moderatedBodyPose.position.z;

	// above a constant height, moderate the gait height depending on body height

	realnum gaitHeight = 65 + 30 * moderate( (maxBodyHeight - currentHeight - minBodyHeight)/(maxBodyHeight-minBodyHeight), 1.0);

	gaitControl.setGaitHeight(gaitHeight, 40);
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

	if (generalMode  == TerrainMode) {
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

	// compute kinematics to get the angle of all feet
	PentaPointType tmpHipPoints;
	bool ok = bodyKinematics.computeKinematics(
									currentBodyPose,
									footPoints,
									tmpHipPoints,
									legAngles,
									groundPoints);

	if (ok) {
		// use the foot angles to correct the kinematics
		PentaPointType fpw (footPointsWorld);
		for (int i = 0;i<NumberOfLegs;i++)
			fpw[i].z += bodyKinematics.getFatFootCorrectionHeight(i);

		gaitControl.imposeFootPointsWorld(fpw);
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
	data.currentFusedPose = getOdomPose();

	for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
		data.footOnGroundFlag[legNo] = getFootOnGround()[legNo];
		data.legPhase[legNo] = gaitControl.getLegsGaitPhase(legNo);
		data.footAngle[legNo] = getFootAngle(legNo);
	}
}


void Engine::computeMovement() {
	realnum dT = movementSample.dT();

	if ((generalMode == WalkingMode) || (generalMode == TerrainMode)) {
		// accelerate to totalSpeed
		if (abs(getCurrentSpeed() - getTargetSpeed()) > floatPrecision) {
			realnum speedDiff = getTargetSpeed()-getCurrentSpeed();
			if (abs(speedDiff)> maxAcceleration*dT)
				speedDiff = sgn(speedDiff)*maxAcceleration*dT;
			gaitControl.getCurrentSpeed() += speedDiff;
		}

		// move towards target angular speed
		if (abs(getCurrentAngularSpeed() - getTargetAngularSpeed())> floatPrecision) {
			realnum angularSpeedAcc= (getTargetAngularSpeed() - getCurrentAngularSpeed())/dT;
			angularSpeedAcc = constrain(angularSpeedAcc, -maxAngularSpeedAcceleration, maxAngularSpeedAcceleration);
			gaitControl.getCurrentAngularSpeed() += angularSpeedAcc*dT;
		}

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
			currentWalkingDirection = 2*M_PI-currentWalkingDirection;
		if (currentWalkingDirection < -M_PI)
			currentWalkingDirection = currentWalkingDirection + 2*M_PI;

		gaitControl.getCurrentAbsWalkingDirection() = currentWalkingDirection;

		// rotate nose along the angular speed
		angle_rad noseOrientation = getCurrentNoseOrientation();
		noseOrientation += getCurrentAngularSpeed()*dT;
		if (noseOrientation > M_PI)
			noseOrientation = 2*M_PI-noseOrientation;
		if (noseOrientation < -M_PI)
				noseOrientation = noseOrientation + 2*M_PI;

		getBodyKinematics().setCurrentNoseOrientation(noseOrientation);
	}
}
