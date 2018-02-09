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
#include "std_msgs/Int8.h"
#include <geometry_msgs/Point.h>
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

	// call a service to start/stop the motor of the lidar
	void startLidar(bool on);

	void broadcastTransformationMapToOdom();
	void advertiseBodyPose();

	static Dispatcher& getInstance() { static Dispatcher dispatcher; return dispatcher; };

	Pose& getOdomFrame() { return odomFrame; };
	Pose& getOdomPose() { return odomPose; };
	Map& getSlamMap() { return slamMap; };
	EngineState& getEngineState() { return engineState; };
	// return the fused position of slam outcome and odometry
	Pose& getBaselink() { engineState.baseLinkInMapFrame; };

private:
	void listenerSlamGrid (const nav_msgs::OccupancyGrid::ConstPtr& og );
	void listenToLaserScan (const sensor_msgs::LaserScan::ConstPtr& scanPtr );
	void listenerSLAMout (const geometry_msgs::PoseStamped::ConstPtr&  og );
	void listenerOdometry(const nav_msgs::Odometry::ConstPtr& odom);
	void listenerBotState(const std_msgs::String::ConstPtr& fullStateStr);
	void advertiseBodyPoseToEngine(const Pose& bodyPose);
	void advertiseGaitModeToEngine(GaitModeType gaitMode);
	void advertiseFrontLegToEngine(Point frontleg);

	tf::TransformBroadcaster broadcaster;
	std::string serializedLaserScan;

	Map slamMap;
	std::string serializedSlamMap;
	int slamMapGenerationNumber;
	ros::Subscriber slamMapSubscriber;
	ros::Subscriber slamPoseSubscriber;

	EngineState engineState;
	Pose mapPose;
	Pose odomFrame;
	Pose odomPose;

	ros::Publisher cmdVel;
	ros::Publisher cmdBodyPose;
	ros::Publisher cmdModePub;
	ros::Publisher initalPosePub;
	ros::Publisher cmdFrontLeg;
	ros::Publisher cmdGaitMode;

	ros::Subscriber laserScanSubscriber;
	std::string serializedLaserData;

	ros::Subscriber odomSubscriber;
	ros::Subscriber stateSubscriber;

	ros::ServiceClient startLidarService;
	ros::ServiceClient stopLidarService;
	bool lidarIsOn;
	bool lastLidarShouldBeOn;

	Pose advertisedAutonomousBodyPose;
};


#endif /* WEBSERVERAPI_H_ */
