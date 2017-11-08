#include "CmdDispatcher.h"

#include <vector>
#include "setup.h"

#include "core.h"
#include "Map.h"

#include "Util.h"

#include <move_base_msgs/MoveBaseAction.h>

using namespace std;


bool getURLParameter(vector<string> names, vector<string> values, string key, string &value) {
	for (int i = 0;i<(int)names.size();i++) {
		if (names[i].compare(key) == 0) {
			value = values[i];
			return true;
		}
	}
	return false;
}


void compileURLParameter(string uri, vector<string> &names, vector<string> &values) {
	names.clear();
	values.clear();

	std::istringstream iss(uri);
	std::string token;
	while (std::getline(iss, token, '&'))
	{
		// extract name and value of parameter
		int equalsIdx = token.find("=");
		if (equalsIdx > 0) {
			string name = token.substr(0,equalsIdx);
			string value = token.substr(equalsIdx+1);
			names.insert(names.end(), name);
			values.insert(values.end(), urlDecode(value));
		};
	}
}

CommandDispatcher::CommandDispatcher() {
	mapGenerationNumber = 0;
}



void CommandDispatcher::setup(ros::NodeHandle& handle) {

	//tell the action client that we want to spin a thread by default and wait until up and running
	moveBaseClient = new MoveBaseClient("move_base", true);

	// subscribe to the navigation stack topic that delivers the global costmap
	occupancyGridSubscriber = handle.subscribe("/move_base/global_costmap/costmap", 1000, &CommandDispatcher::listenerGlobalCostmap, this);

	// subscribe to the SLAM topic that delivers the occupancyGrid
	occupancyGridSubscriber = handle.subscribe("map", 1000, &CommandDispatcher::listenerOccupancyGrid, this);

	// subscribe to the laser scaner directly in order to display the nice red pointcloud
	laserScanSubscriber = handle.subscribe("scan", 1000, &CommandDispatcher::setLaserScan, this);

	// subscribe to the SLAM topic that deliveres the etimated position
	estimatedSLAMPoseSubscriber = handle.subscribe("slam_out_pose", 1000, &CommandDispatcher::listenerSLAMout,  this);

	// subscribe to the path
	pathSubscriber = handle.subscribe("/trajectory", 1000, &CommandDispatcher::listenToTrajectory,  this);

	// subscribe to the bots odom (without being fused with SLAM)
	odomSubscriber = handle.subscribe("odom", 1000, &CommandDispatcher::listenerOdometry, this);

	// subscribe to the bots state
	stateSubscriber = handle.subscribe("/engine/get_state", 1000, &CommandDispatcher::listenerBotState,  this);

	cmdVel 			= handle.advertise<geometry_msgs::Twist>("cmd_vel", 50);
	cmdBodyPose 	= handle.advertise<geometry_msgs::Twist>("/engine/cmd_pose", 50);
	cmdModePub 		= handle.advertise<pentapod_engine::engine_command_mode>("/engine/cmd_mode", 50);

	// publish initial position (required by navigation
	initalPosePub   = handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

    // wait for the action server to come up
	while(!moveBaseClient->waitForServer(ros::Duration(5.0))){
	    ROS_INFO_THROTTLE(1,"Waiting for the move_base action server to come up");
	}

}


actionlib::SimpleClientGoalState CommandDispatcher::getNavigationGoalStatus() {
	return moveBaseClient->getState();
}

Pose CommandDispatcher::getNavigationGoal() {
	return navigationGoal;
}


void CommandDispatcher::setNavigationGoal(const Pose& goalPose_world) {

	// advertise the initial position for the navigation stack anytime the navigation goal is set
	geometry_msgs::PoseWithCovarianceStamped initialPosition;
	initialPosition.pose.pose.position.x = engineState.currentFusedPose.position.x/1000.0;
	initialPosition.pose.pose.position.y = engineState.currentFusedPose.position.y/1000.0;
	initialPosition.pose.pose.position.z = 0;
	geometry_msgs::Quaternion initialPose_quat  =
			tf::createQuaternionMsgFromYaw(
					engineState.currentBodyPose.orientation.z +
					engineState.currentNoseOrientation);

	initialPosition.pose.pose.orientation = initialPose_quat;
	initialPosition.header.stamp = ros::Time::now();
	initialPosition.header.frame_id = "map";
	// initalPosePub.publish(initialPosition);
	ROS_INFO_STREAM("publishing initial pose " << engineState.currentFusedPose.position << " nose=" << degrees(engineState.currentBodyPose.orientation.z +
					engineState.currentNoseOrientation) << "");

	// navigation goal is set in terms of base_frame. Transform goalPose_world in base_frame
	navigationGoal = goalPose_world - engineState.currentFusedPose.position;
	navigationGoal.position.rotateAroundZ(- (engineState.currentBodyPose.orientation.z + engineState.currentNoseOrientation));
	navigationGoal.orientation.null();
	move_base_msgs::MoveBaseGoal goal;

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = navigationGoal.position.x/1000.0;
	goal.target_pose.pose.position.y = navigationGoal.position.y/1000.0;
	goal.target_pose.pose.position.z = 0;

	geometry_msgs::Quaternion goalPoseQuat  = tf::createQuaternionMsgFromYaw(navigationGoal.orientation.z);
	goal.target_pose.pose.orientation = goalPoseQuat;

	if (navigationGoal.isNull())
		moveBaseClient->cancelGoal();
	moveBaseClient->sendGoal(goal);

	ROS_INFO_STREAM("setting navigation goal " << navigationGoal.position);
}


string getResponse(bool ok) {
	std::ostringstream s;
	if (ok) {
		s << "\"ok\"=true";
	} else {
		s << "\"ok\"=false, \"error\":" << getLastError() << ", \"errormessage\":" << stringToJSonString(getErrorMessage(getLastError()));
	}
	string response = s.str();
	return response;
}

// central dispatcher of all url requests arriving at the webserver
// returns true, if request has been dispatched within dispatch. Otherwise the caller
// should assume that static content is to be displayed.
bool  CommandDispatcher::dispatch(string uri, string query, string body, string &response, bool &okOrNOk) {

	response = "";
	string urlPath = getPath(uri);

	vector<string> urlParamName;
	vector<string> urlParamValue;

	compileURLParameter(query,urlParamName,urlParamValue);

	// check if direct cortex command defined via URL parameter
	// example: /cortex/LED?blink
	ROS_DEBUG("url: %s query:%s", uri.c_str(), query.c_str());

	// check, if TransactionExecutor is called with orchestrated calls
	if (hasPrefix(uri, "/engine/")) {
		string engineCommand = uri.substr(string("/engine/").length());
		pentapod_engine::engine_command_mode cmdMode;

		// /engine/start?force=true or /engine/start
		if (hasPrefix(engineCommand, "turnon")) {
			cmdMode.command = pentapod_engine::engine_command_mode::POWER_MODE_ON_CMD;
			cmdModePub.publish(cmdMode);

			okOrNOk = true;
			response = getResponse(okOrNOk);
			return true;
		}
		// /engine/stop
		else if (hasPrefix(engineCommand, "turnoff")) {
			cmdMode.command = pentapod_engine::engine_command_mode::POWER_MODE_OFF_CMD;
			cmdModePub.publish(cmdMode);
			okOrNOk = true;
			response = getResponse(okOrNOk);
			return true;
		}
		// /engine/wakeup?bodypose={...}
		else if (hasPrefix(engineCommand, "wakeup")) {
			cmdMode.command = pentapod_engine::engine_command_mode::AWAKE_CMD;
			cmdModePub.publish(cmdMode);
			okOrNOk = true;
			response = getResponse(okOrNOk);
			return true;
		}
		// /engine/fallasleep
		else if (hasPrefix(engineCommand, "fallasleep")) {
			cmdMode.command = pentapod_engine::engine_command_mode::FALL_ASLEEP_CMD;
			cmdModePub.publish(cmdMode);
			okOrNOk = true;
			response = getResponse(okOrNOk);
			return true;
		}
		// /engine/alarmstart?bodypose={...}
		else if (hasPrefix(engineCommand, "terrain")) {
			string terrainModeOnStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "on", terrainModeOnStr);
			bool terrainModeOn = jsonStringToBool(terrainModeOnStr, ok);

			if (terrainModeOn)
				cmdMode.command = pentapod_engine::engine_command_mode::TERRAIN_MODE_CMD;
			else
				cmdMode.command = pentapod_engine::engine_command_mode::WALKING_MODE_CMD;

			cmdModePub.publish(cmdMode);
			okOrNOk = true;
			response = getResponse(okOrNOk);
			return true;
		}
		// /engine/move?bodypose={...}&noseorientation=10&movementspeed=10&movementrotatez=10&movementdirection=10
		else if (hasPrefix(engineCommand, "setspeed")) {
			string movementSpeedStr, movementRotateZStr, movementDirectionStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "movementspeed", movementSpeedStr);
			ok = ok && getURLParameter(urlParamName, urlParamValue, "movementrotatez", movementRotateZStr);
			ok = ok && getURLParameter(urlParamName, urlParamValue, "movementdirection", movementDirectionStr);

			mmPerMillisecond movementSpeed = stringToFloat(movementSpeedStr, ok);
			radPerSecond movementRotateZ = stringToFloat(movementRotateZStr, ok);
			angle_rad absWalkingDirection = stringToFloat(movementDirectionStr, ok);

			if (ok) {
				geometry_msgs::Twist twist;

				// compute the deviation of the walking direction from the current walking direction
				angle_rad movementDirectionDevation = absWalkingDirection - engineState.currentWalkingDirection;

				twist.linear.x = movementSpeed/1000.0*(cos(movementDirectionDevation));
				twist.linear.y = movementSpeed/1000.0*(sin(movementDirectionDevation));
				twist.angular.z = movementRotateZ;
				cmdVel.publish(twist);
				okOrNOk = true;
			} else {
				okOrNOk = false;
			}

			response = getResponse(okOrNOk);
			return true;
		}
		// /engine/move?bodypose={...}&noseorientation=10&movementspeed=10&movementrotatez=10&movementdirection=10
		else if (hasPrefix(engineCommand, "setpose")) {

			string bodyPoseStr, noseOrientationStr, movementSpeedStr, movementRotateZStr, movementDirectionStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "bodypose", bodyPoseStr);

			std::stringstream paramIn(bodyPoseStr);
			string paramstr = parseString(paramIn, ok);
			std::stringstream in(paramstr);
			Pose bodyPose;
			bodyPose.deserialize(in,ok);

			geometry_msgs::Twist twist;
			twist.linear.x = bodyPose.position.x/1000.0;
			twist.linear.y = bodyPose.position.y/1000.0;
			twist.linear.z = bodyPose.position.z/1000.0;

			twist.angular.x = bodyPose.orientation.x;
			twist.angular.y = bodyPose.orientation.y;
			twist.angular.z = bodyPose.orientation.z;
			cmdBodyPose.publish(twist);

			response = getResponse(ok);
			return true;
		}
		/*
		// engine/setfrontleg?bodypose={...}
		else if (hasPrefix(engineCommand, "setfrontleg")) {
			string frontLegPoseStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "frontleg", frontLegPoseStr);
			std::stringstream paramIn (frontLegPoseStr);
			string paramstr = parseString(paramIn, ok);
			std::stringstream in (paramstr);
			Point frontLeg;
			frontLeg.deserialize(in,ok);
			if (ok)
				engine->setTargetFrontLegPose(frontLeg);
			response = getResponse(ok);
			return true;
		}
		*/
		// engine/get
		else if (hasPrefix(engineCommand, "get")) {
			std::ostringstream out;
			engineState.serialize(out);
			response = out.str() + "," + getResponse(true);
			return true;
		}
	}

	// deliver the slam and costmaps map
	if (hasPrefix(uri, "/map/")) {
		string mapCommand = uri.substr(string("/map/").length());
		// map/get
		if (hasPrefix(mapCommand, "get")) {
			string generationNumberStr ;
			bool ok = getURLParameter(urlParamName, urlParamValue, "no", generationNumberStr);
			int generationNumber;
			if (ok) {
				generationNumber = stringToInt(generationNumberStr,ok);
				if (!ok || (generationNumber < mapGenerationNumber)) {
					// passed version is older than ours, deliver map
					response = serializedMap + "," + getResponse(true);
				}
				else {
					// client has already the current map version, dont deliver it again
					response = getResponse(true);
				}
			} else {
				// deliver map without version check
				response = serializedMap + "," + getResponse(true);
			}
			okOrNOk = true;
			return true;
		}
	}

	if (hasPrefix(uri, "/costmap/")) {
		string mapCommand = uri.substr(string("/map/").length());
		// map/get
		if (hasPrefix(mapCommand, "get")) {
			string generationNumberStr ;
			bool ok = getURLParameter(urlParamName, urlParamValue, "no", generationNumberStr);
			int generationNumber;
			if (ok) {
				generationNumber = stringToInt(generationNumberStr,ok);
				if (!ok || (generationNumber < costMapGenerationNumber)) {
					// passed version is older than ours, deliver map
					response = globalCostMapSerialized + "," + getResponse(true);
				}
				else {
					// client has already the current map version, dont deliver it again
					response = getResponse(true);
				}
			} else {
				// deliver map without version check
				response = globalCostMapSerialized + "," + getResponse(true);
			}
			okOrNOk = true;
			return true;
		}
	}

	// delivery the laser scan
	if (hasPrefix(uri, "/scan/")) {
		string command = uri.substr(string("/scan/").length());
		// map/get
		if (hasPrefix(command, "get")) {

			if (serializedLaserScan != "")
				response = serializedLaserScan+ "," + getResponse(true);
			else
				response = getResponse(false);
			okOrNOk = true;
			return true;
		}
	}

	// delivery the trajectory
	if (hasPrefix(uri, "/trajectory/")) {
		string command = uri.substr(string("/trajectory/").length());
		// map/get
		if (hasPrefix(command, "get")) {
			if (serializedTrajectory != "")
				response = serializedTrajectory+ "," + getResponse(true);
			else
				response = getResponse(false);
			okOrNOk = true;
			return true;
		}
	}

	if (hasPrefix(uri, "/navigation/")) {
		string command = uri.substr(string("/navigation/").length());
		// map/get
		if (hasPrefix(command, "goal/set")) {
			string bodyPoseStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "bodypose", bodyPoseStr);
			std::stringstream paramIn(bodyPoseStr);
			string paramstr = parseString(paramIn, ok);
			std::stringstream in(paramstr);
			Pose goalPose;
			goalPose.deserialize(in,ok);

			setNavigationGoal(goalPose);
			response = getResponse(true);
			okOrNOk = true;
			return true;
		}
		else if (hasPrefix(command,"get")) {
			std::ostringstream out;
			int navStatus = NavigationStatusType::NavPending;
			if (!navigationGoal.isNull()) {
				navStatus = (int)getNavigationGoalStatus().state_;
			}
			out << "\"goal\"=" << navigationGoal << ", \"status\"=" << navStatus;
			response = out.str() + "," + getResponse(true);
			okOrNOk = true;
		}
		return okOrNOk;
	}


	okOrNOk = false;
	return false;
}

void CommandDispatcher::setLaserScan (const sensor_msgs::LaserScan::ConstPtr& scanPtr ) {
	LaserScan laserScan;
	std::vector<int_millimeter> newScan;
	angle_rad startAngle = scanPtr->angle_min;
	angle_rad angleIncrement = scanPtr->angle_increment;
	angle_rad endAngle = scanPtr->angle_max;

	angle_rad angle = startAngle;
	newScan.clear();
	newScan.reserve(scanPtr->ranges.size());

	for (int i = 0;i<scanPtr->ranges.size();i++) {
		float distance = scanPtr->ranges[i];
		if ((distance <= scanPtr->range_max) && (distance >= scanPtr->range_min) && (distance != inf)) {
			newScan.push_back((int)(distance*1000.0));
		}
		else
			newScan.push_back(-1);
	}

	laserScan.setLaserScan(engineState.currentFusedPose, newScan, startAngle, angleIncrement, endAngle);
	std::stringstream out;
	laserScan.serialize(out);
	serializedLaserScan = out.str();


}

void convertOccupancygridToMap(const nav_msgs::OccupancyGrid::ConstPtr& og, const Map& map, int& generationNumber,  string& serializedMap) {
	Map m;
		if ((og->info.width > 0) && (og->info.height > 0)) {
			m.setGridDimension(og->info.width, og->info.height, og->info.resolution*1000.0);
			int mapArraySize = og->info.width * og->info.height;
			int width = og->info.width;
			for (int i = 0;i<mapArraySize;i++) {
				// this is the regular call, but we take the short (and ugly) route
				// m.setOccupancyByGridCoord(i/width,i % width, occupancyGrid.data[i]);
				int occupancy = og->data[i];

				// the occupancy Map has different values than Map::GridState
				switch (occupancy) {
					case -1: 	m.getVector()[i] = Map::GridState::UNKNOWN; break;
					case 0: 	m.getVector()[i] = Map::GridState::FREE; break;
					case 100: 	m.getVector()[i] = Map::GridState::OCCUPIED; break;
				}
			}
			m.setGenerationNumber(++generationNumber);
		}

		std::stringstream out;
		m.serialize(out);
		serializedMap = out.str();
}

void CommandDispatcher::listenerOccupancyGrid (const nav_msgs::OccupancyGrid::ConstPtr& og ) {
	Map m;
	convertOccupancygridToMap(og, m, mapGenerationNumber, serializedMap);
}

void CommandDispatcher::listenerGlobalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og ) {
	convertOccupancygridToMap(og, globalCostMap, costMapGenerationNumber, globalCostMapSerialized);
}

void CommandDispatcher::listenerSLAMout (const geometry_msgs::PoseStamped::ConstPtr&  og ) {

	// mapPose of hector mapping is given in [m], we need [mm]
	mapPose.position.x = og->pose.position.x*1000.0;
	mapPose.position.y = og->pose.position.y*1000.0;
	mapPose.position.z = og->pose.position.z*1000.0;

	Quaternion q(og->pose.orientation.x, og->pose.orientation.y, og->pose.orientation.z, og->pose.orientation.w);
	mapPose.orientation = EulerAngles(q);
	fusedMapOdomPose = mapPose;

	engineState.currentMapPose = mapPose;
	engineState.currentFusedPose = fusedMapOdomPose; // reset pose that is fused of map and odom
}

// subscription to path
void CommandDispatcher::listenToTrajectory(const nav_msgs::Path::ConstPtr& path) {
	Trajectory trajectory;
	trajectory.clear();
	for (unsigned int i = 0;i<path->poses.size();i++) {
		Point p(path->poses[i].pose.position.x*1000.0,path->poses[i].pose.position.y*1000.0,path->poses[i].pose.position.z*1000.0);
		Quaternion q(path->poses[i].pose.orientation.x,
				     path->poses[i].pose.orientation.y,
					 path->poses[i].pose.orientation.z,
					 path->poses[i].pose.orientation.w);

		StampedPose sp(Pose(p,q),path->poses[i].header.stamp.toNSec()/1000);
		trajectory.add(sp);
	}

	std::stringstream out;
	trajectory.serialize(out);
	serializedTrajectory= out.str();
}

// subscription to odom
void CommandDispatcher::listenerOdometry(const nav_msgs::Odometry::ConstPtr& odom) {
	// receive odometry from bot
	odomPose.position.x = odom->pose.pose.position.x*1000.0;
	odomPose.position.y = odom->pose.pose.position.y*1000.0;
	Quaternion q(odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
	odomPose.orientation = EulerAngles(q);

	realnum speedX = odom->twist.twist.linear.x*1000.0;
	realnum speedY = odom->twist.twist.linear.y*1000.0;
	realnum angularSpeedZ  = odom->twist.twist.angular.z;
}


// subscription to bots state
void CommandDispatcher::listenerBotState(const std_msgs::String::ConstPtr&  fullStateStr) {
	Pose previousOdom = engineState.currentOdomPose;

	std::istringstream in(fullStateStr->data);
	engineState.deserialize(in);

	// update fused pose of map and odom (map pose is discrete, odom pose is continously)
	Point odomDiff = (engineState.currentOdomPose.position - previousOdom.position);
	fusedMapOdomPose += odomDiff;
 	engineState.currentFusedPose = fusedMapOdomPose;
 	engineState.currentMapPose = mapPose;
}


