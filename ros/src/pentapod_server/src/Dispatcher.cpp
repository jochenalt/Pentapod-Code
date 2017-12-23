
#include <vector>

#include <move_base_msgs/MoveBaseAction.h>
#include "std_srvs/Empty.h"

#include "setup.h"
#include "core.h"
#include "Map.h"
#include "Trajectory.h"
#include "basics/util.h"

#include "Dispatcher.h"
#include "IntoDarkness.h"

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

Dispatcher::Dispatcher() {
	slamMapGenerationNumber = 0;
	lidarIsOn = false;
	lastLidarShouldBeOn = false;
	advertisedAutonomousBodyPose.null();
}


void Dispatcher::setup(ros::NodeHandle& handle) {

	// subscribe to the SLAM map coming from hector slamming
	ROS_INFO_STREAM("subscribe to /map");
	slamMapSubscriber = handle.subscribe("map", 1000, &Dispatcher::listenerSlamGrid, this);

	// subscribe to the laser scaner directly in order to display the nice red pointcloud
	ROS_INFO_STREAM("subscribe to /scan");
	laserScanSubscriber = handle.subscribe("scan", 1000, &Dispatcher::listenToLaserScan, this);

	// subscribe to the SLAM topic that deliveres the etimated position
	ROS_INFO_STREAM("subscribe to /slam_out_pose");
	slamPoseSubscriber = handle.subscribe("slam_out_pose", 1000, &Dispatcher::listenerSLAMout,  this);

	// subscribe to the bots odom (without being fused with SLAM)
	ROS_INFO_STREAM("subscribe to /odom");
	odomSubscriber = handle.subscribe("odom", 1000, &Dispatcher::listenerOdometry, this);

	// subscribe to the bots state
	ROS_INFO_STREAM("subscribe to /engine/get_state");
	stateSubscriber = handle.subscribe("/engine/get_state", 1000, &Dispatcher::listenerBotState,  this);

	// service to start or stop the lidar motor
	startLidarService = handle.serviceClient<std_srvs::Empty>("/start_motor");
	stopLidarService = handle.serviceClient<std_srvs::Empty>("/stop_motor");


	cmdVel 			= handle.advertise<geometry_msgs::Twist>("cmd_vel", 50);
	cmdBodyPose 	= handle.advertise<geometry_msgs::Twist>("/engine/cmd_pose", 50);
	cmdModePub 		= handle.advertise<pentapod_engine::engine_command_mode>("/engine/cmd_mode", 50);

	// publish initial position (required by navigation
	initalPosePub   = handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

	// initialoze the dark hole finder
	Navigator::getInstance().setup(handle);

	// initialize the dark hole finder
	IntoDarkness::getInstance().setup(handle);

	FreeWill::getInstance().setup();
}


// retrurns a standardized response
string getResponse(bool ok) {
	std::ostringstream s;
	if (ok) {
		s << "\"ok\":true";
	} else {
		s << "\"ok\":false, \"error\":" << getLastError() << ", \"errormessage\":" << stringToJSonString(getErrorMessage(getLastError()));
	}
	string response = s.str();
	return response;
}

// central dispatcher of all url requests arriving at the webserver
// returns true, if request has been dispatched successfully. Otherwise the caller
// should assume that static content is to be displayed.
bool Dispatcher::dispatch(string uri, string query, string body, string &response, bool &okOrNOk) {

	response = "";
	string urlPath = getPath(uri);

	vector<string> urlParamName;
	vector<string> urlParamValue;

	compileURLParameter(query,urlParamName,urlParamValue);

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

			advertiseBodyPoseToEngine(bodyPose);

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

	// deliver all global data like slam map, cost map, and dark holes computed out of cost map
	if (hasPrefix(uri, "/map/")) {
		string mapCommand = uri.substr(string("/map/").length());
		// map/get
		if (hasPrefix(mapCommand, "get")) {
			string generationNumberStr ;
			bool ok = getURLParameter(urlParamName, urlParamValue, "no", generationNumberStr);
			int generationNumber;
			bool deliverContent = false;
			if (ok) {
				generationNumber = stringToInt(generationNumberStr,ok);
				if (!ok || (generationNumber < slamMapGenerationNumber)) {
					// passed version is older than ours, deliver map
					deliverContent = true;
				}
				else {
					// client has already the current map version, dont deliver it again
					response = getResponse(true);
				}
			} else {
				// deliver map without version check
				deliverContent = true;
			}
			if (deliverContent)
				response = serializedSlamMap + "," + Navigator::getInstance().getGlobalCostmapSerialized() + "," + IntoDarkness::getInstance().getDarkScaryHolesSerialized() + ","  + getResponse(true);

			okOrNOk = true;
			return true;
		}
	}

	if (hasPrefix(uri, "/costmap/")) {
		string mapCommand = uri.substr(string("/costmap/").length());
		string generationNumberStr ;
			if (hasPrefix(mapCommand, "local/get")) {
			bool deliverContent = false;
			bool ok = getURLParameter(urlParamName, urlParamValue, "no", generationNumberStr);
			if (ok) {
				int generationNumber = stringToInt(generationNumberStr,ok);
				if (!ok || (generationNumber < Navigator::getInstance().getLocalCostmapGenerationNumber())) {
					// passed version is older than ours, deliver map
					deliverContent = true;
				}
				else {
					// client has already the current map version, dont deliver it again
					response = getResponse(true);
				}
			} else {
				// deliver map localCostMapSerialized version check
				deliverContent = true;
			}
			if (deliverContent)
				response = Navigator::getInstance().getLocalCostmapSerialized() + "," + getResponse(true);

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

	if (hasPrefix(uri, "/plan/")) {
		string command = uri.substr(string("/plan/").length());
		if (hasPrefix(command, "get")) {
			bool deliverContent = false;
			if (Navigator::getInstance().getLocalPlanSerialized() != "") {
				string generationNumberStr ;
				bool ok = getURLParameter(urlParamName, urlParamValue, "no", generationNumberStr);
				if (ok) {
					int generationNumber = stringToInt(generationNumberStr,ok);
					if (!ok || (generationNumber < Navigator::getInstance().getLocalPlanGenerationNumber()))
						deliverContent = true;
				} else
					deliverContent = true;
			}

			response = getResponse(true);
			if (deliverContent)
				response = Navigator::getInstance().getGlobalPlanSerialized() +"," + Navigator::getInstance().getLocalPlanSerialized() + "," + response;


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

			string latchOrientationStr;
			bool setNavigationOrientation = false;

			ok = getURLParameter(urlParamName, urlParamValue, "latchorientation", latchOrientationStr);
			if (ok) {
				setNavigationOrientation = latchOrientationStr == "true";
			}
			// cout << "ok=" << ok << "latchsr=" << latchOrientationStr << "sno=" << setNavigationOrientation << endl;
			Navigator::getInstance().setNavigationGoal(goalPose, setNavigationOrientation);
			response = getResponse(true);
			okOrNOk = true;
			return true;
		}
		else if (hasPrefix(command,"goal/get")) {
			std::ostringstream out;
			int navStatus = (int)Navigator::getInstance().getNavigationStatusType();

			Navigator::getInstance().getNavigationGoal().serialize(out);
			out << ", \"status\":" << navStatus;
			response = out.str() + "," + getResponse(true);
			okOrNOk = true;
		}
		return okOrNOk;
	}

	if (hasPrefix(uri, "/lidar/")) {
		string command = uri.substr(string("/lidar/").length());
		// map/get
		if (hasPrefix(command, "start")) {
			startLidar(true);
			response = getResponse(true);
			okOrNOk = true;
			return true;
		}
		else if (hasPrefix(command,"stop")) {
			startLidar(false);
			response = getResponse(true);
			okOrNOk = true;
		}
		else if (hasPrefix(command,"get")) {
			response = string(lidarIsOn?"true":"false") + "," + getResponse(true);
			okOrNOk = true;
		}

		return okOrNOk;
	}

	okOrNOk = false;
	return false;
}


void Dispatcher::listenToLaserScan (const sensor_msgs::LaserScan::ConstPtr& scanPtr ) {
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

	laserScan.setLaserScan(engineState.baseLinkInMapFrame, newScan, startAngle, angleIncrement, endAngle);
	std::stringstream out;
	laserScan.serialize(out);
	serializedLaserScan = out.str();
}

// we use the same data type Map for costmaps and for slam maps.
enum MapType { SLAM_MAP_TYPE, COSTMAP_TYPE };
void convertOccupancygridToMap(const nav_msgs::OccupancyGrid::ConstPtr& inputMap, MapType type,  Map& outputMap, int& generationNumber,  string& serializedMap) {
	if ((inputMap->info.width > 0) && (inputMap->info.height > 0)) {
		outputMap.null();
		outputMap.setGridDimension(inputMap->info.width, inputMap->info.height, inputMap->info.resolution*1000.0);
		int mapArraySize = inputMap->info.width * inputMap->info.height;
		int width = inputMap->info.width;
		for (int i = 0;i<mapArraySize;i++) {
			// the regular call is m.setOccupancyByGridCoord(i/width,i % width, occupancyGrid.data[i])
			// but we take the short (and ugly) route
			int occupancy = inputMap->data[i];

			// the occupancy Map has different values than Map::GridState
			if (type == SLAM_MAP_TYPE) {
				switch (occupancy) {
					case -1: 	outputMap.getVector()[i] = Map::GridState::UNKNOWN; break;
					case 0: 	outputMap.getVector()[i] = Map::GridState::FREE; break;
					case 100: 	outputMap.getVector()[i] = Map::GridState::OCCUPIED; break;
				}
			} else {
				outputMap.getVector()[i] = occupancy;
			}
		}
		outputMap.setGenerationNumber(++generationNumber);
	}

	std::stringstream out;
	outputMap.serialize(out);
	serializedMap = out.str();
}

void Dispatcher::listenerSlamGrid (const nav_msgs::OccupancyGrid::ConstPtr& og ) {
	convertOccupancygridToMap(og, SLAM_MAP_TYPE, slamMap, slamMapGenerationNumber, serializedSlamMap);
}


void Dispatcher::listenerSLAMout (const geometry_msgs::PoseStamped::ConstPtr&  og ) {
	// mapPose of hector mapping is given in [m], we need [mm]
	mapPose.position.x = og->pose.position.x*1000.0;
	mapPose.position.y = og->pose.position.y*1000.0;
	mapPose.position.z = og->pose.position.z*1000.0;
	Quaternion q(og->pose.orientation.x, og->pose.orientation.y, og->pose.orientation.z, og->pose.orientation.w);
	mapPose.orientation = EulerAngles(q);

	// compute odomFrame for map->odom transformation
	odomFrame = mapPose.applyInverseTransformation(odomPose);

	engineState.currentMapPose = mapPose;
	ROS_INFO_STREAM_THROTTLE(5, "/slam out odom_pose=" << odomPose);
	engineState.baseLinkInMapFrame = mapPose; // reset base_link to slam pose, odomFrame stored the new deviation of odometry
}


// subscription to odom
void Dispatcher::listenerOdometry(const nav_msgs::Odometry::ConstPtr& odom) {
	// receive odometry from bot
	odomPose.position.x = odom->pose.pose.position.x*1000.0;
	odomPose.position.y = odom->pose.pose.position.y*1000.0;
	odomPose.position.z = 0;
	Quaternion q(odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
	odomPose.orientation = EulerAngles(q);

	realnum speedX = odom->twist.twist.linear.x*1000.0;
	realnum speedY = odom->twist.twist.linear.y*1000.0;
	realnum angularSpeedZ  = odom->twist.twist.angular.z;
}

void Dispatcher::startLidar(bool on) {
 	std_srvs::Empty srv;
	if (on) {
	    ROS_INFO("RPLidar is turned on");
		 startLidarService.call(srv);
		 lidarIsOn = true;
	}
	else  {
	    ROS_INFO("RPLidar is turned off");
 		stopLidarService.call(srv);
		lidarIsOn = false;
	}
}



// subscription to bots state
void Dispatcher::listenerBotState(const std_msgs::String::ConstPtr&  fullStateStr) {
	std::istringstream in(fullStateStr->data);
	engineState.deserialize(in);

 	// engineState.currentBaselinkPose.position = odomFrame.position + odomPose.position.getRotatedAroundZ(odomFrame.orientation.z);
 	// engineState.currentBaselinkPose.orientation = odomFrame.orientation  + odomPose.orientation;

	// ignore passed base_link and use odom_frame and /odom topic
	engineState.baseLinkInMapFrame = odomFrame.applyTransformation(odomPose);
 	engineState.currentMapPose = mapPose;
 	engineState.currentScaryness = IntoDarkness::getInstance().getCurrentScariness();
 	//  	cout << " " << holeFinder.getCurrentScariness() << endl;

 	// turn on the lidar if we wake up
 	// turn it off when we fall asleep
 	bool lidarShouldBeOn = ((engineState.engineMode == WalkingMode) || (engineState.engineMode == TerrainMode));
 	if (lidarShouldBeOn != lastLidarShouldBeOn) {
	    ROS_INFO_STREAM("Bot " << (lidarShouldBeOn?"is awakened":"falls asleep") << ", turn lidar " << (lidarShouldBeOn?"on":"off"));
    	startLidar(lidarShouldBeOn);
	}
 	lastLidarShouldBeOn = lidarShouldBeOn;
}


void Dispatcher::broadcastTransformationMapToOdom() {
	broadcaster.sendTransform(
		  tf::StampedTransform(
			tf::Transform(tf::createQuaternionFromYaw(odomFrame.orientation.z),
					      tf::Vector3(odomFrame.position.x/1000.0,odomFrame.position.y/1000.0,0)),
			ros::Time::now(),"map", "odom"));
}

void Dispatcher::advertiseBodyPoseToEngine(const Pose& bodyPose) {
	geometry_msgs::Twist twist;
	twist.linear.x = bodyPose.position.x/1000.0;
	twist.linear.y = bodyPose.position.y/1000.0;
	twist.linear.z = bodyPose.position.z/1000.0;

	twist.angular.x = bodyPose.orientation.x;
	twist.angular.y = bodyPose.orientation.y;
	twist.angular.z = bodyPose.orientation.z;
	cmdBodyPose.publish(twist);
}

void Dispatcher::advertiseBodyPose() {
	if (Navigator::getInstance().getNavigationStatusType() == NavigationStatusType::NavActive) {
		Pose toBePose = FreeWill::getInstance().getAutonomousBodyPose();
		if (abs(toBePose.position.z - advertisedAutonomousBodyPose.position.z) > floatPrecision) {
			ROS_INFO_STREAM("advertise new autonomous body pose curr=" << engineState.currentBodyPose << " tobe=" << toBePose << " scariness=" << Dispatcher::getInstance().getEngineState().currentScaryness );
			advertiseBodyPoseToEngine(toBePose);
			advertisedAutonomousBodyPose = toBePose;
		}
	}
}
