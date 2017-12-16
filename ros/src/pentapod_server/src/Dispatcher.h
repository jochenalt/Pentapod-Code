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
	static Dispatcher& getInstance() { static Dispatcher singleton; return singleton; };

	// initialize all listeners
	void setup(ros::NodeHandle& handle);

	// dispatch an incoming REST calls from mongoose webserver
	bool dispatch(string uri, string query, string body, string &response, bool &okOrNOk);

	void broadcastTransformationMapToOdom();
	void advertiseBodyPose();

	// return odom Frame, i.e. the difference between SLAM located map and origin of the odometry. Is amended everytime a new slam location takes place
	Pose& getOdomFrame() { return odomFrame; };

	// return odomPose, i.e. the continous odometry based on pentapods engine only, SLAM is not considered
	Pose& getOdomPose() { return odomPose; };

	// return the last SLAM map with Free/Occupied/Unknown grid cells
	Map& getSlamMap() { return slamMap; };

	// return comprehensive information on the engine
	EngineState& getEngineState() { return engineState; };

	// return the position of the bos in the frame of SLAM map (not on odomFrame!)
	Pose& getBaselink() { engineState.baseLinkInMapFrame; };

	// send a specific body pose to the engine
	void advertiseBodyPoseToEngine(const Pose& bodyPose);

private:
	// call a service to start/stop the motor of the lidar
	void startLidar(bool on);

	// various listeners to ROS topics
	void listenerOccupancyGrid (const nav_msgs::OccupancyGrid::ConstPtr& og );
	void listenToLaserScan (const sensor_msgs::LaserScan::ConstPtr& scanPtr );
	void listenerSLAMout (const geometry_msgs::PoseStamped::ConstPtr&  og );
	void listenerOdometry(const nav_msgs::Odometry::ConstPtr& odom);
	void listenerBotState(const std_msgs::String::ConstPtr& fullStateStr);

	// tf broadcaster used for publishing map->odom transformation
	tf::TransformBroadcaster broadcaster;

	// SLAM map
	Map slamMap;

	// serialized form the slamMap, is serialized right after update to serve the according REST call quickly
	std::string serializedMap;

	// number indicating the version of the map
	int mapGenerationNumber;

	// comprehensive information on the engine, including all legs and toes
	EngineState engineState;

	// last location as indicated by SLAM
	Pose mapPose;


	// difference of mapPose and odom origin
	Pose odomFrame;

	// odometry in the frame of odomFrame
	Pose odomPose;

	// various engine topics
	ros::Publisher cmdVel;
	ros::Publisher cmdBodyPose;
	ros::Publisher cmdModePub;
	ros::Publisher initalPosePub;


	// service for starting the lidar
	ros::ServiceClient startLidarService;

	// service for stoping the lidar
	ros::ServiceClient stopLidarService;

	// true if lidar has been turned on physically
	bool lidarIsOn;

	// true if lidar is supposed to be on. Can be different from lidarIsOn if the command has been delayed since the bot not yet upright.
	bool lastLidarShouldBeOn;

	// subscription to RPLidar sensor
	ros::Subscriber laserScanSubscriber;

	// serialized form the last laser scan, is serialized right after update to serve according RESt call quickly
	std::string serializedLaserScan;

};


#endif /* WEBSERVERAPI_H_ */
