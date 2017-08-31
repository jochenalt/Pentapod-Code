#ifndef LOGGER_H_
#define LOGGER_H_


#if defined(ROS_BUILD) || defined(ROS_BUILD_SHARED_LIBS) || defined(ROS_PACKAGE_NAME)
	// Nothing to do, use the ROS functionality
	#include <ros/ros.h>
#else
#include <thread>
#define _ELPP_THREAD_SAFE
#define ELPP_THREAD_SAFE
#ifdef _WIN32
	#define ELPP_DEFAULT_LOG_FILE "logs/manfred.log"
#else
	#define ELPP_DEFAULT_LOG_FILE "/var/log/manfred.log"
#endif

#include "easylogging++.h"

#define ROS_DEBUG_STREAM(args) LOG(DEBUG) << args;
#define ROS_INFO_STREAM(args)  LOG(INFO) << args;
#define ROS_ERROR_STREAM(args) LOG(ERROR) << args;
#define ROS_ASSERT assert

#endif
#endif
