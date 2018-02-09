/*
 * OdomPublisher.h
 *
 *  Created on: Aug 3, 2017
 *      Author: jochenalt
 */

#ifndef PENTAPOD_ENGINE_SRC_ODOMPUBLISHER_H_
#define PENTAPOD_ENGINE_SRC_ODOMPUBLISHER_H_


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// SLAM topics
#include <geometry_msgs/PoseStamped.h>

// bot topics
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int8.h"
#include <geometry_msgs/Point.h>

// ROS messages for Pentapod
#include "pentapod_engine/engine_command_mode.h"

#include "setup.h"
#include "Engine.h"

class OdomPublisher {
public:
	OdomPublisher();
	virtual ~OdomPublisher();

	void setup(ros::NodeHandle& handle, Engine& pEngine);
	void broadcastTransformation();
	void broadcastOdom();
	void broadcastState();

	void listenToSpeedCommand (const geometry_msgs::Twist::ConstPtr& vel_msg);
	void listenToBodyPose (const geometry_msgs::Twist::ConstPtr& bodypose_msg);
	void listenToMoveMode (const pentapod_engine::engine_command_mode::ConstPtr& mode_msg);
	void listenToFrontLeg(const geometry_msgs::Point::ConstPtr& frontleg_msg);
	void listenToGaitMode (const std_msgs::Int8 gait_mode_msg);

private:
	tf::TransformBroadcaster broadcaster;
	ros::Publisher  state_pub;
	ros::Subscriber cmd_vel;
	ros::Subscriber cmd_body_pose;
	ros::Subscriber cmd_mode;
	ros::Subscriber gait_mode;
	ros::Subscriber front_leg;

	ros::Publisher odom_pub;
	Engine* engine = NULL;
	ros::NodeHandle* handle = NULL;
};

#endif /* PENTAPOD_ENGINE_SRC_ODOMPUBLISHER_H_ */
