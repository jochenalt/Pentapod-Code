/*
 * Engine.h
 *
 * Starts, stops and runs the "engine" of the pentapod by controlling all legs.
 *
 *  Created on: 07.05.2017
 *      Author: JochenAlt
 */

#ifndef ENGINE_H_
#define ENGINE_H_

#include "BodyKinematics.h"
#include "GaitController.h"
#include "CortexClient.h"
#include "spatial.h"
#include "EngineState.h"

class Engine {
	friend class GaitController;
	friend class BodyKinematics;
public:

	Engine();
	virtual ~Engine() {};

	// call this upfront before calling loop. Returns true if everyhing ok
	bool setupProduction(string i2cport, int i2cadr,string cortextSerialPort, int cortexSerialBaudRate);

	// setup this engine in simulation-mode without actual cortex
	bool setupSimulation();

	// returns true if communication with cortex is o
	bool cortexCommunicationOk() { return legController.isCortexCommunicationOk(); };

	// call this as often as possible
	bool ratedloop();

	// actually runs a call to cortex
	void loop();

	// switch on/off the bot
	void turnOn();
	void turnOff();
	bool isTurnedOn() { return turnedOn; };

	// start wakeup and fallasleep procedure
	void wakeUp();

	// go down again
	void fallAsleep();

	// terrain mode adapts the gait to bumpy terrain
	void terrainMode(bool terrainModeOn);

	// set a new body pose the bot moves to smoothly
	void setTargetBodyPose(const Pose& newBodyPose, bool immediately = false);

	// set the pose of the front leg
	void setTargetFrontLegPose(const Point& pose);

	// used for ad-hoc computation of a single pose without having body kinematics. Used in the UI
	// to compute the view that does not show the entire bot, but one leg only
	void computeSingleLegAdHoc(LegPose& newSingleLegPoseWorld);

	// get/set target speed. current speed accelerates until this speed is reached
	void setTargetSpeed ( mmPerSecond newSpeed);
	mmPerSecond& getTargetSpeed ();
	mmPerSecond getTargetSpeedLimited(); // in case sum of speed and angular speed is too this is the reduced target speed

	// set walking direction
	void setTargetWalkingDirection(angle_rad walkingDirection);
	angle_rad& getTargetWalkingDirection();
	angle_rad getCurrentWalkingDirection();

	// set/get target  turn speed
	void setTargetAngularSpeed(radPerSecond rotateZ);
	radPerSecond& getTargetAngularSpeed();
	radPerSecond getTargetAngularSpeedLimited(); // in case sum of speed and angular speed is too this is the reduced angular target speed

	// switch to another gait type including a switching procedure, which might take some time
	void setTargetGaitMode(GaitModeType gaitType);

	angle_rad getCurrentNoseOrientation() { return bodyKinematics.getCurrentNoseOrientation(); };

	mmPerSecond getCurrentSpeedX() { return gaitControl.getCurrentSpeedX(); };
	mmPerSecond getCurrentSpeed() { return gaitControl.getCurrentSpeed(); };
	mmPerSecond getCurrentSpeedY() { return gaitControl.getCurrentSpeedY(); };
	radPerSecond& getCurrentAngularSpeed() { return gaitControl.getCurrentAngularSpeed(); };

	GaitModeType getGaitMode();

	// get the pose of all hips in world coordinates (relative to the belly buttons origin)
	const PentaPoseType&  getHipPoseWorld();

	// get all relevant data representing the current pose of the body and all legs
	LegAnglesType getLegAngles();
	PentaPointType getHipPoints();

	PentaPointType getGaitRefPoints();
	const PentaPointType& getGroundPoints() { return groundPoints; };
	const Pose& getCurrentBodyPose() { return currentBodyPose; }

	const FootOnGroundFlagType& getFootOnGround();

	// in case of 4-leg gait the front leg is not used in the gait and can be set explicitely.
	// The bot moves towards this position smoothly
	LegPose getFrontLegPoseWorld();

    Pose getOdomPose() { return Pose(gaitControl.getCurrentPositionWorld(), Rotation(0,0,getCurrentNoseOrientation())); };

    // return the mode in terms of sleeping, walking, terrain-walking,...
	GeneralEngineModeType getGeneralMode() { return generalMode; };
	bool isAwake() { return ((generalMode == WalkingMode)  || (generalMode == TerrainMode)); };

	// return true, if we can receive speed commands
	bool isListeningToMovements() { return (generalMode == GeneralEngineModeType::WalkingMode) || (generalMode == GeneralEngineModeType::TerrainMode); };

	// to be called when distance sensors have a new distance measured
	angle_deg getFootAngle(int legNo) { return bodyKinematics.getFootAngle(legNo); };
	Point getToePointsWorld(int legNo)  { return gaitControl.getToePointsWorld(legNo); };
	Point getGaitRefPointWorld(int legNo)  { return gaitControl.getGaitRefPointsWorld(legNo); };

	LegGaitPhase getLegsGaitPhase(int legNo) { return gaitControl.getLegsGaitPhase(legNo); };

	void imposeDistanceSensors(realnum distance[NumberOfLegs]);

	// get the full state of the bot
	void getState(EngineState &data);

private:

	bool wakeUpIfNecessary();
	bool setupCommon();

    // access internal structures
	BodyKinematics& getBodyKinematics() { return bodyKinematics; };
	GaitController& getGaitController() { return gaitControl; } ;

	// process the distance sensors and generate the corrections of the ground
	void processDistanceSensors(realnum distance[NumberOfLegs]);

	// pass and impose current feet points. Used during startup procedure to tell the initial position
	void imposeFootPointsWorld(const PentaPointType& footPoints);


	// returns true, if setup and enabled
	LegKinematics& getLegKinematics() { return kinematics; };

	void computeBodyPose();				// moderates the set body pose
	void computeBodySwing();			// computes breathing and the sexy walk
	void computeGaitRefPointRadius();	// compute the radius of the points where the feet touch the ground
	void computeGaitSpeed();			// compute the gait speed depending on the body speed
	void computeGaitHeight();			// computes and moderates the height of the body
	void computeGaitMode(); 			// takes care of slowly switching between 4-Legs-Mode and 5-Legs mode
	void computeCentreOfGravity(); 		// takes care of moving the as to centre of gravity in case of 4-legs mode
	void computeFrontLeg();				// in 4 leg walk, the front leg is treated specially
	void computeSensoredDistance();		// use distance sensor information to influence touch points
	void computeWakeUpProcedure();		// start up bot by sorting legs first and going up
	void computeAcceleration();				// compute acceleration, turning etc.
	void computeWarpCompensation();		// compute compensation of warping legs

	BodyKinematics bodyKinematics;		// compute kinematics of all legs and the body
	GaitController gaitControl;			// generates the gait
	LegKinematics kinematics;			// computes kinematics of a single leg
	CortexClient legController;			// proxy to cortex

	realnum fourWalkLegRatio;			// ratio between 0 and 1 switching slowly between 5 and 4 legs mode

	radPerSecond targetAngularSpeed;
	angle_rad targetWalkingDirection;
	realnum targetSpeed;				// set speed in mm/s
	FootOnGroundFlagType lastFeetOnGround; // feet on the ground flag of last loop

	Pose inputBodyPose;					// input pose, which is moderated afterwards and gets additional breathing
	GaitModeType targetGaitMode;		// gait mode (5-leg, 4-leg,...)
	Pose moderatedBodyPose;				// pose after limiting acceleration
	Pose currentBodyPose;				// current real pose including body swing
	Pose bodySwing;						// add on to bodypose simulating sexy walk and breathing
	realnum humpsCompensation;  		// if any humps are on the ground,  adapt the body height accordingly

	Point targetFrontLeg;				// target position of the front leg
	LegPose frontLegPose;				// current pose of the single front leg

	PentaPointType hipPoints;			// points of all hips in absolute world coordinates
	PentaPointType groundPoints;		// projection of the toe to the ground in absolute world coordinates
	PentaPointType warpCompensation;	// correction of all toe pointsprojection of the toe to the ground in absolute world coordinates

	LegAnglesType legAngles;			// current angles of all legs

	ExclusiveMutex loopMutex;			// loop is running in an own thread. Mutex to synchronize that with commands
	Pose lastModeratedBodyPose;
	TimeSamplerStatic mainLoopTimeSample;
	TimeSamplerStatic bodyPoseSampler;
	TimeSamplerStatic gaitModeSampler;
	TimeSamplerStatic bodySwingSampler;
	TimeSamplerStatic gaitSpeedSampler;
	TimeSamplerStatic warpingCompensationSampler;
	TimeSamplerStatic movementSample;


	TimeSamplerStatic frontLegSampler;
	TimeSamplerStatic humpCompensationFilterSampler;

	GeneralEngineModeType generalMode;
	ShutDownModeType shutdownMode;
	SpatialPID imuPID;
	bool turnedOn;
	bool isSetup;

};


#endif /* MAINCONTROLLER_H_ */
