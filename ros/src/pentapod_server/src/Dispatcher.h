/*
 * CmdDispatcher.cpp
 *
 * Takes http requests and dispatches them to according functions
 *
 *      Author: JochenAlt
 */


#ifndef WEBSERVERAPI_H_
#define WEBSERVERAPI_H_

#include <vector>
#include <stdio.h>
#include <string>
#include "basics/spatial.h"

#include "setup.h"
#include "Engine.h"
#include <Map.h>
#include <Trajectory.h>

#include <LaserScan.h>

// common message types
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

// self-made message types
#include "pentapod_engine/engine_command_mode.h"

// publishing transformations
#include <tf/transform_broadcaster.h>

// move base action client
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// look for dark scary holes
#include "IntoDarkness.h"
#include "Navigator.h"

#include "FreeWill.h"

using namespace std;

class Dispatcher {
public:
	Dispatcher();

	void setup(ros::NodeHandle& handle);

	bool dispatch(string uri, string query, string body, string &response, bool &okOrNOk);

	void listenerOccupancyGrid (const nav_msgs::OccupancyGrid::ConstPtr& og );
	void listenerGlobalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og );
	void listenerLocalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og );
	void listenerLocalPlan(const nav_msgs::Path::ConstPtr& og );
	void listenerGlobalPlan(const nav_msgs::Path::ConstPtr& og );

	void listenToLaserScan (const sensor_msgs::LaserScan::ConstPtr& scanPtr );

	void listenerSLAMout (const geometry_msgs::PoseStamped::ConstPtr&  og );
	void listenerOdometry(const nav_msgs::Odometry::ConstPtr& odom);
	void listenerBotState(const std_msgs::String::ConstPtr& fullStateStr);
	void listenToTrajectory(const nav_msgs::Path::ConstPtr& path);

	void setNavigationGoal(const Pose& goalPose, bool setOrientationToPath = false);

	Pose getNavigationGoal();
	void cancelNavigationGoal();

	actionlib::SimpleClientGoalState getNavigationGoalStatus();

	void setupNavigationStackTopics(ros::NodeHandle& handle);

	// call a service to start/stop the motor of the lidar
	void startLidar(bool on);

	// call the move_base service to clear all costmaps.
	void clearCostmaps();


	void broadcastTransformationMapToOdom();
	NavigationStatusType getNavigationStatusType();
	void advertiseBodyPose();

	static Dispatcher& getInstance() { static Dispatcher dispatcher; return dispatcher; };

	Pose& getOdomFrame() { return odomFrame; };
	Pose& getOdomPose() { return odomPose; };
	Map& getSlamMap() { return slamMap; };
	Map& getGlobalCostmap() { return Navigator::getInstance().getGlobalCostmap(); };
	Map& getLocalCostmap() { return Navigator::getInstance().getLocalCostmap(); };
	EngineState& getEngineState() { return engineState; };
	// return the fused position of slam outcome and odometry
	Pose& getBaselink() { engineState.baseLinkInMapFrame; };

private:

	void advertiseBodyPoseToEngine(const Pose& bodyPose);
	tf::TransformBroadcaster broadcaster;
	std::string serializedLaserScan;

	Map slamMap;
	std::string serializedMap;
	// Map globalCostMap;
	// std::string globalCostMapSerialized;

	// Map localCostMap;
	// std::string localCostMapSerialized;
	// int localCostmapGenerationNumber;
	// int globalCostmapGenerationNumber;

	Trajectory localPlan;
	std::string localPlanSerialized;

	Trajectory globalPlan;
	std::string globalPlanSerialized;
	std::string serializedTrajectory;

	int mapGenerationNumber;

	int localPlanGenerationNumber;
	int globalPlanGenerationNumber;
	int trajectoryGenerationNumber;

	std::string serializedLaserData;

	EngineState engineState;
	Pose mapPose;
	Pose odomFrame;
	Pose odomPose;

	ros::Publisher cmdVel;
	ros::Publisher cmdBodyPose;
	ros::Publisher cmdModePub;
	ros::Publisher initalPosePub;

	ros::Subscriber occupancyGridSubscriber;
	ros::Subscriber globalCostmapSubscriber;
	ros::Subscriber localCostmapSubscriber;
	ros::Subscriber globalPathSubscriber;
	ros::Subscriber localPathSubscriber;
	ros::Subscriber laserScanSubscriber;

	ros::Subscriber estimatedSLAMPoseSubscriber;
	ros::Subscriber odomSubscriber;
	ros::Subscriber stateSubscriber;
	ros::Subscriber pathSubscriber;

	ros::ServiceClient startLidarService;
	ros::ServiceClient stopLidarService;
	ros::ServiceClient clearCostmapService;
	MoveBaseClient* moveBaseClient;
	Pose navigationGoal;
    Pose navigationGoal_world;
	bool lidarIsOn;
	bool lastLidarShouldBeOn;
	bool latchedGoalOrientationToPath;
};


#endif /* WEBSERVERAPI_H_ */
