/*
 * OdomPublisher.cpp
 *
 *  Created on: Aug 3, 2017
 *      Author: jochenalt
 */

// Pentapod/Kinematics

#include "OdomPublisher.h"

#include "core.h"
#include "setup.h"
#include "Engine.h"

#include "OdomPublisher.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// messages and services
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// ROS messages for Pentapod
#include "pentapod_engine/engine_command_mode.h"

OdomPublisher::OdomPublisher() {
}

OdomPublisher::~OdomPublisher() {
}


void OdomPublisher::speedCommandSubscriber (const geometry_msgs::Twist::ConstPtr& vel_msg) {
	// the x/y coord frame is relative to the walking direction
	// this needs to be translated into the pentapod engine, that
	// has a nose-direction where the x/y frame is located and a walking-direction that defines a
	// deviation from the nose direction
	mmPerMillisecond fullspeed = sqrt(sqr(vel_msg->linear.x*1000.0) + sqr(vel_msg->linear.y*1000.0));
	angle_rad newWalkingDirectionDeviation = atan2(vel_msg->linear.y, vel_msg->linear.x);
	if (vel_msg->linear.x < 0) {
		fullspeed = -fullspeed;
		newWalkingDirectionDeviation += M_PI;

		if (newWalkingDirectionDeviation > M_PI)
			newWalkingDirectionDeviation = 2*M_PI-newWalkingDirectionDeviation;
		if (newWalkingDirectionDeviation < -M_PI)
			newWalkingDirectionDeviation = newWalkingDirectionDeviation + 2*M_PI;
	}

	angle_rad newWalkingDirection = newWalkingDirectionDeviation + engine->getCurrentWalkingDirection();

	engine->setTargetSpeed(fullspeed);
	engine->setTargetAngularSpeed(vel_msg->angular.z);
	engine->setTargetWalkingDirection(newWalkingDirection);
}


void OdomPublisher::bodyPoseCommandSubscriber (const geometry_msgs::Twist::ConstPtr& bodypose_msg) {
	Pose bodyPose;
	bodyPose.position.x = bodypose_msg->linear.x*1000.0;
	bodyPose.position.y = bodypose_msg->linear.y*1000.0;
	bodyPose.position.z = bodypose_msg->linear.z*1000.0;

	bodyPose.orientation.x = bodypose_msg->angular.x;
	bodyPose.orientation.y = bodypose_msg->angular.y;
	bodyPose.orientation.z = bodypose_msg->angular.z;

	engine->setTargetBodyPose(bodyPose);
}

void OdomPublisher::modeCommandSubscriber (const pentapod_engine::engine_command_mode::ConstPtr& mode_msg) {
	switch (mode_msg->command) {
		case pentapod_engine::engine_command_mode::POWER_MODE_ON_CMD:
			engine->turnOn();
			break;
		case pentapod_engine::engine_command_mode::POWER_MODE_OFF_CMD:
			engine->turnOff();
			break;
		case pentapod_engine::engine_command_mode::AWAKE_CMD:
			engine->wakeUp();
			break;
		case pentapod_engine::engine_command_mode::FALL_ASLEEP_CMD:
			engine->fallAsleep();
			break;
		case pentapod_engine::engine_command_mode::TERRAIN_MODE_CMD:
			engine->terrainMode(true);
			break;
		case pentapod_engine::engine_command_mode::WALKING_MODE_CMD:
			engine->terrainMode(false);
			break;
		default:
			break;
	}
}


void OdomPublisher::broadcastOdom() {

	// compute odom->base_link transformation and broadcast
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(engine->getCurrentNoseOrientation() + engine->getCurrentBodyPose().orientation.z);
	geometry_msgs::TransformStamped odom_trans;
	ros::Time current_time = ros::Time::now();

	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = engine->getOdomPose().position.x/1000.0;
	odom_trans.transform.translation.y = engine->getOdomPose().position.y/1000.0;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	broadcaster.sendTransform(odom_trans);

	// publish odom via ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.pose.pose.position.x = engine->getOdomPose().position.x/1000.0;
	odom.pose.pose.position.y = engine->getOdomPose().position.y/1000.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = engine->getCurrentSpeedX()/1000.0;
	odom.twist.twist.linear.y = engine->getCurrentSpeedY()/1000.0;
	odom.twist.twist.angular.z = engine->getCurrentAngularSpeed();
	odom_pub.publish(odom);
}

void OdomPublisher::broadcastState() {
	// publish serialized state via ROS
	EngineState state;
	engine->getState(state);
	std::stringstream out;
	state.serialize(out);

	string engineStateSerialized = out.str();
    std_msgs::String msg;
    msg.data = engineStateSerialized;

	state_pub.publish(msg);
}


void OdomPublisher::breadcastTransformation() {
	// send transformation from base to laser

	broadcaster.sendTransform(
		  tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1),
					      tf::Vector3(engine->getCurrentBodyPose().position.x*1000.0,
					    		      engine->getCurrentBodyPose().position.y*1000.0,
									  engine->getCurrentBodyPose().position.z*1000.0 + CAD::LaserSensorHeight)),
			ros::Time::now(),"base_link", "base_laser"));

	broadcaster.sendTransform(
		  tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1),
					      tf::Vector3(0,0,0)),
			ros::Time::now(),"map", "odom"));

	broadcaster.sendTransform(
		  tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1),
					      tf::Vector3(0,0,0)),
			ros::Time::now(),"odom", "base_footprint"));

	broadcaster.sendTransform(
		  tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1),
					      tf::Vector3(0,0,0)),
			ros::Time::now(),"base_footprint", "base_link"));

	broadcaster.sendTransform(
		  tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1),
					      tf::Vector3(0,0,0)),
			ros::Time::now(),"base_frame", "laser"));

	broadcaster.sendTransform(
		  tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1),
					      tf::Vector3(0,0,0)),
			ros::Time::now(),"base_frame", "nav"));
}


void OdomPublisher::setup(ros::NodeHandle& pHandle, Engine& pEngine) {
	handle = &pHandle;
	engine = &pEngine;

	// cmd_vel&odom are commonly used topics, so no namespace
	cmd_vel 		= handle->subscribe("/engine/cmd_vel" ,  10 , &OdomPublisher::speedCommandSubscriber, this);
	odom_pub 		= handle->advertise<nav_msgs::Odometry>("/engine/odom", 50);

	// these topcis are self_made and specific to the pentapod
	cmd_body_pose 	= handle->subscribe("/engine/cmd_pose" , 10 , &OdomPublisher::bodyPoseCommandSubscriber, this);
	cmd_mode 		= handle->subscribe("/engine/cmd_mode" , 10 , &OdomPublisher::modeCommandSubscriber, this);
	state_pub 	    = handle->advertise<std_msgs::String>("/engine/get_state", 50);
}
