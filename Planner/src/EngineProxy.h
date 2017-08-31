/*
 * KinematicsDispatcher.h
 *
 *  Created on: 21.06.2017
 *      Author: JochenAlt
 */

#ifndef ENGINEPROXY_H_
#define ENGINEPROXY_H_

#include "Engine.h"
#include "Map.h"
#include "LaserScan.h"

class EngineCaller {
public:
	void setup(string pHost, int pPort);

	bool httpGET(string path, string &responsestr, int timeout_ms);
	bool httpPOST(string path, string body, string &responsestr, int timeout_ms);

	std::string host;
	int port;
};

class EngineProxy {
	const int BotTrajectorySampleRate = 100; 	// [ms] update rate of fetching the kinematic
	const int UpdateMapSampleRate = 500;		// [ms] update rate of map fetching (if no map is there, nothing happens)
	const int UpdateLaserScanSampleRate = 500;	// [ms] update rate of laser scan
	const int UpdateTrajectorySampleRate = 500;	// [ms] update rate of laser scan

public:
	EngineProxy();
	virtual ~EngineProxy();

	static EngineProxy& getInstance();
	void setupRemoteEngine(string engineWebserverHost, int engineWebserverPort);
	void setupSimulatedEngine();

	void loop();

	// switch on/off the bot
	void turnOn();
	void turnOff();
	bool isTurnedOn();

	// start wakeup and fallasleep procedure
	// void wakeUp(const Pose& targetBodyPose);
	void wakeUp();
	void fallAsleep();
	void terrainMode(bool ok);

	// set a new body pose the bot moves to smoothly
	void setTargetBodyPose(const Pose& newBodyPose);

	// set the new speed and direction of the body
	void setTargetMovement ( mmPerSecond newSpeed,  realnum newRotateZ /* radians/s */,realnum direction );

	// in case of 4-leg gait the front leg is not used in the gait and can be set explicitely.
	// The bot moves towards this position smoothly
	void setTargetFrontLegPoseWorld(const Point& pose);

	// used to request the current state of the Bot
	void updateEngineState(EngineState & state);

	// switch to another gait type including a switching procedure, which might take some time
	void setGaitMode(GaitModeType gaitType);


	GaitModeType getGaitMode();

	// get the pose of all hips in world coordinates (relative to the belly buttons origin)
	const PentaPoseType& getHipPoseWorld();

	// get all relevant data representing the current pose of the body and all legs
	LegAnglesType getLegAngles();
	PentaPointType getGaitRefPoints();
	const PentaPointType& getGroundPoints();
	Pose getBodyPose();
	realnum getLiftHeight();
	angle_rad getNoseOrientation();
	mmPerSecond getCurrenSpeed();
	angle_rad getCurrenWalkingDirection();
	angle_rad getCurrenAngularSpeed();

	void getCurrentMovement ( angle_rad& orientation, mmPerSecond& newSpeed, realnum& rotateZ /* radians/s */, realnum& speedDirection);

	const FootOnGroundFlagType& getFootOnGround();

	LegPose getFrontLegPoseWorld();

	GeneralEngineModeType getGeneralMode();

	angle_deg getFootAngle(int legNo);

	Point getToePointsWorld(int legNo) ;

	Point getGaitRefPointWorld(int legNo);

	LegGaitPhase getLegsGaitPhase(int legNo);

	void imposeDistanceSensors(realnum distance[NumberOfLegs]);

	// return the occupancy grid
	Map& getMap();

	// return the last lidar scan
	LaserScan& getLaserScan();

	// return the passed path
	Trajectory& getTrajectory();


	// return the pose in the map
	const Pose& getMapPose();

	// return the pose in the map
	const Pose& getOdomPose();

	// take map pose for general pose and incrementally update via odom
	const Pose& getFusedPose();

	bool isBotDataAvailable();
	bool isMapDataAvailable();
	bool isLaserScanAvailable();
	bool isTrajectoryDataAvailable();

	bool isEstimatedPoseAvailable();

private:
	void updateLaserScan();
	void updateMap();
	void updateTrajectory();

	bool newLaserScanAvailable = false;
	bool newBotDataAvailable = false;
	bool newMapDataAvailable = false;
	bool newMapPoseDataAvailable = false;
	bool newTrajectoryDataAvailable = false;

	EngineCaller remoteEngine;
	Engine engine;
	EngineState data;
	EngineState lastData;

	bool callRemoteEngine;
	TimeSamplerStatic remoteEngineCallTimer;
	TimeSamplerStatic fetchMapTimer;
	TimeSamplerStatic fetchEstimatedPoseTimer;
	TimeSamplerStatic fetchLaserScanTimer;
	TimeSamplerStatic fetchTrajectoryTimer;

	Map map;
	Pose mapPose;
	LaserScan laserScan;
	Trajectory trajectory;
};

#endif /* KINEMATICSDISPATCHER_H_ */
