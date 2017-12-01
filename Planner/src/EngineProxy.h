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
#include "Trajectory.h"
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
	const int UpdateMapSampleRate = 1000;		// [ms] update rate of map fetching (if no map is there, nothing happens)
	const int UpdateLocalCostmapSampleRate = 1000;		// [ms] update rate of map fetching (if no map is there, nothing happens)
	const int UpdateLaserScanSampleRate = 1000;	// [ms] update rate of laser scan
	const int UpdateTrajectorySampleRate =1000;	// [ms] update rate of laser scan

public:
	EngineProxy();
	virtual ~EngineProxy();

	enum TrajectoryType { TRAJECTORY, GLOBAL_PLAN, LOCAL_PLAN };


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

	// get all current angles of all legs
	LegAnglesType getLegAngles();

	// get all gait ref points (the point in der middle of a gait when the foot is on the ground)
	PentaPointType getGaitRefPoints();

	// get all ground points that is the current toe point but projected to the ground
	const PentaPointType& getGroundPoints();

	// get current position and orientation of the body
	Pose getBodyPose();

	// get orientation of the front leg (=nose). Not necessarily the same like WalkingDirection. 0° is towards x-axis
	angle_rad getNoseOrientation();

	// get current speed in [mm/s] in the walking direction
	mmPerSecond getCurrenSpeed();

	// get the current walking direction, 0° is towards x-axis
	angle_rad getCurrenWalkingDirection();

	// get the current rotational speed in [°/s]
	radPerSecond getCurrenAngularSpeed();

	// conviniency method to get all of the above in one call
	void getCurrentMovement ( angle_rad& orientation, mmPerSecond& newSpeed, realnum& rotateZ /* radians/s */, realnum& speedDirection);

	// get flags which foot is currently touching the ground
	const FootOnGroundFlagType& getFootOnGround();

	// get the pose of the front leg in case we are in the 4-foot-gait mode where the front leg can be moved freely
	LegPose getFrontLegPoseWorld();

	// return current mode in terms of sleeping, walking, standing up,...
	GeneralEngineModeType getGeneralMode();

	// get perpendicular angle of a foot against the ground plane
	angle_deg getFootAngle(int legNo);

	// get toe point in world coordinates. LegNo= [0..4]
	Point getToePointsWorld(int legNo) ;

	// get gait ref point in world coordinates. LegNo= [0..4]
	Point getGaitRefPointWorld(int legNo);

	// get current gait phase of one leg in terms of moving up, moving down, on the ground
	LegGaitPhase getLegsGaitPhase(int legNo);

	void imposeDistanceSensors(realnum distance[NumberOfLegs]);

	// return the map with occupancy grid
	Map& getMap();

	// return local costmap on the basis of current laser scan
	Map& getLocalCostmap();

	// return the global costmap on basis of SLAM map
	Map& getGlobalCostmap();

	// return the last 360° lidar scan in one array
	LaserScan& getLaserScan();

	// return the current trajectory
	Trajectory& getTrajectory(TrajectoryType type);

	// return the pose in coordinates of the map (jumps descretly depending on SLAM)
	const Pose& getMapPose();

	// return the odom pose (continously moving)
	const Pose& getOdomPose();

	// take map pose for general pose and incrementally update via odom
	const Pose& getFusedPose();

	// set the new navigation goal
	void setNavigationGoal(const Pose& goal, bool latchOrientation = false);

	// get last navigation goal(gets nulled once it has been reached)
	Pose getCurrentNavigationGoal();

	// get last navigation goal(gets nulled once it has been reached)
	NavigationStatusType getCurrentNavigationStatus();

	vector<Point>& getDarkScaryHoles();

	// flags if new data from server is available
	bool isBotDataAvailable();
	bool isMapDataAvailable();
	bool isCostmapDataAvailable();

	bool isLaserScanAvailable();
	bool isTrajectoryDataAvailable();
	bool isEstimatedPoseAvailable();
	bool isNavigationStatusAvailable();

private:
	void updateLaserScan();
	void updateGlobalMaps();
	void updateLocalCostmap();
	void updateGlobalCostmap();

	void updateTrajectory();
	void updatePlan();
	void updateNavigation();


	bool newLaserScanAvailable = false;
	bool newBotDataAvailable = false;
	bool newMapDataAvailable = false;
	bool newLocalCostmapAvailable = false;
	bool newGlobalCostmapAvailable = false;
	bool newMapPoseDataAvailable = false;
	bool newTrajectoryDataAvailable = false;
	bool newNavigationStatusIsAvailable = false;

	EngineCaller remoteEngine;
	Engine engine;
	EngineState data;
	EngineState lastData;

	bool callRemoteEngine;
	TimeSamplerStatic remoteEngineCallTimer;
	TimeSamplerStatic fetchMapTimer;
	TimeSamplerStatic fetchLocalCostmapTimer;
	TimeSamplerStatic fetchGlobalCostmapTimer;

	TimeSamplerStatic fetchEstimatedPoseTimer;
	TimeSamplerStatic fetchLaserScanTimer;
	TimeSamplerStatic fetchTrajectoryTimer;

	Map localCostmap;
	Map globalCostmap;

	Map map;
	Pose mapPose;
	LaserScan laserScan;
	Trajectory trajectory;
	Trajectory localPlan;
	Trajectory globalPlan;
	vector<Point> darkScaryHoles;


	Pose navigationGoal;
	NavigationStatusType navigationStatus;
};

#endif /* KINEMATICSDISPATCHER_H_ */
