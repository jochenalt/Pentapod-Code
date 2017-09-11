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
#include "setup.h"
#include "Engine.h"
#include "spatial.h"
#include <Map.h>
#include <LaserScan.h>

// common message types
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

// self-made message types
#include "pentapod_engine/engine_command_mode.h"

// publishing transformations
#include <tf/transform_broadcaster.h>

using namespace std;

class CommandDispatcher {
public:
	CommandDispatcher();

	void setup(ros::NodeHandle& handle);

	bool dispatch(string uri, string query, string body, string &response, bool &okOrNOk);

	void listenerOccupancyGrid (const nav_msgs::OccupancyGrid::ConstPtr& og );
	void setLaserScan (const sensor_msgs::LaserScan::ConstPtr& scanPtr );

	void listenerSLAMout (const geometry_msgs::PoseStamped::ConstPtr&  og );
	void listenerOdometry(const nav_msgs::Odometry::ConstPtr& odom);
	void listenerBotState(const std_msgs::String::ConstPtr& fullStateStr);
	void setTrajectory(const nav_msgs::Path::ConstPtr& path);


private:

	std::string serializedLaserScan;

	std::string serializedMap;
	int mapGenerationNumber;

	std::string serializedLaserData;
	std::string serializedTrajectory;
	EngineState engineState;
	Pose mapPose;
	Pose fusedMapOdomPose;
	Pose odomPose;

	ros::Publisher cmdVel;
	ros::Publisher cmdBodyPose;
	ros::Publisher cmdModePub;

	ros::Subscriber occupancyGridSubscriber;
	ros::Subscriber laserScanSubscriber;

	ros::Subscriber estimatedSLAMPoseSubscriber;
	ros::Subscriber odomSubscriber;
	ros::Subscriber stateSubscriber;
	ros::Subscriber pathSubscriber;
};


#endif /* WEBSERVERAPI_H_ */
