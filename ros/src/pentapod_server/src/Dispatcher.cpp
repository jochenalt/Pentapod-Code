
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
	mapGenerationNumber = 0;
	trajectoryGenerationNumber = 0;
	lidarIsOn = false;
	lastLidarShouldBeOn = false;
	latchedGoalOrientationToPath = false;
}


void Dispatcher::setupNavigationStackTopics(ros::NodeHandle& handle) {
	localCostmapGenerationNumber = -1;
	globalCostmapGenerationNumber = -1;
	localPlanGenerationNumber = -1;
	globalPlanGenerationNumber = -1;

	// subscribe to the navigation stack topic that delivers the global costmap
    ROS_INFO_STREAM("subscribe to /move_base/global_costmap/costmap");
	globalCostmapSubscriber = handle.subscribe("/move_base/global_costmap/costmap", 1000, &Dispatcher::listenerGlobalCostmap, this);

	// subscribe to the navigation stack topic that delivers the local costmap
    ROS_INFO_STREAM("subscribe to /move_base/local_costmap/costmap");
	localCostmapSubscriber = handle.subscribe("/move_base/local_costmap/costmap", 1000, &Dispatcher::listenerLocalCostmap, this);

	// subscribe to the right topic of the local planner used
	std::string base_local_planner;
	ros::param::get("/move_base/base_local_planner", base_local_planner);
	std::string local_plan_topic_name = "/move_base" + base_local_planner.substr(base_local_planner.find("/")) + "/local_plan";
	std::string global_plan_topic_name = "/move_base" + base_local_planner.substr(base_local_planner.find("/")) + "/global_plan";

	// subscribe to path of local planner
    ROS_INFO_STREAM("subscribe to local plan from " << local_plan_topic_name);
	localPathSubscriber = handle.subscribe(local_plan_topic_name, 1000, &Dispatcher::listenerLocalPlan, this);

	// subscribe to path of global planner
	ROS_INFO_STREAM("subscribe to global plan from " << global_plan_topic_name);
	globalPathSubscriber = handle.subscribe(global_plan_topic_name, 1000, &Dispatcher::listenerGlobalPlan, this);
}

void Dispatcher::setup(ros::NodeHandle& handle) {

	//tell the action client that we want to spin a thread by default and wait until up and running
	moveBaseClient = new MoveBaseClient("move_base", true);

	// subscribe to the SLAM map coming from hector slamming
	ROS_INFO_STREAM("subscribe to /map");
	occupancyGridSubscriber = handle.subscribe("map", 1000, &Dispatcher::listenerOccupancyGrid, this);

	// subscribe to the laser scaner directly in order to display the nice red pointcloud
	ROS_INFO_STREAM("subscribe to /scan");
	laserScanSubscriber = handle.subscribe("scan", 1000, &Dispatcher::listenToLaserScan, this);

	// subscribe to the SLAM topic that deliveres the etimated position
	ROS_INFO_STREAM("subscribe to /slam_out_pose");
	estimatedSLAMPoseSubscriber = handle.subscribe("slam_out_pose", 1000, &Dispatcher::listenerSLAMout,  this);

	// subscribe to the path
	ROS_INFO_STREAM("subscribe to /trajectory");
	pathSubscriber = handle.subscribe("/trajectory", 1000, &Dispatcher::listenToTrajectory,  this);

	// subscribe to the bots odom (without being fused with SLAM)
	ROS_INFO_STREAM("subscribe to /odom");
	odomSubscriber = handle.subscribe("odom", 1000, &Dispatcher::listenerOdometry, this);

	// subscribe to the bots state
	ROS_INFO_STREAM("subscribe to /engine/get_state");
	stateSubscriber = handle.subscribe("/engine/get_state", 1000, &Dispatcher::listenerBotState,  this);

	// service to start or stop the lidar motor
	startLidarService = handle.serviceClient<std_srvs::Empty>("/start_motor");
	stopLidarService = handle.serviceClient<std_srvs::Empty>("/stop_motor");
	clearCostmapService = handle.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");


	cmdVel 			= handle.advertise<geometry_msgs::Twist>("cmd_vel", 50);
	cmdBodyPose 	= handle.advertise<geometry_msgs::Twist>("/engine/cmd_pose", 50);
	cmdModePub 		= handle.advertise<pentapod_engine::engine_command_mode>("/engine/cmd_mode", 50);

	// publish initial position (required by navigation
	initalPosePub   = handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

	// initialoze the dark hole finder
	IntoDarkness::getInstance().setup(handle);

    // wait for the action server to come up. Do this in a 10Hz loop while producing the
	// identical map->odom transformation which is required by the navigation stack.
	// After this method, the map->odom transformation will be taken up by the main loop
	ros::Time now = ros::Time::now();
	ROS_INFO_STREAM("starting up move_base");
	while (!moveBaseClient->waitForServer(ros::Duration(0.1)) && (ros::Time::now() - now < ros::Duration(10.0))) {
		if (ros::Time::now() - now > ros::Duration(4.0))
			ROS_INFO_THROTTLE(1, "waiting for move base to come up");
		broadcastTransformationMapToOdom();
	}

	if (moveBaseClient->isServerConnected()) {
		setupNavigationStackTopics(handle);
	}
	else {
	    ROS_ERROR("move_base did not come up!!!");
	}

	FreeWill::getInstance().setup(slamMap, globalCostMap, localCostMap, odomFrame, baseLinkInMapFrame, engineState);
}


actionlib::SimpleClientGoalState Dispatcher::getNavigationGoalStatus() {
	return moveBaseClient->getState();
}

Pose Dispatcher::getNavigationGoal() {
	return navigationGoal_world;
}


void Dispatcher::cancelNavigationGoal() {

	if (!navigationGoal.isNull() && (!getNavigationGoalStatus().isDone())) {
		ROS_INFO_STREAM("cancel previous goal " << navigationGoal.position);
		moveBaseClient->cancelGoal();
	}

}

void Dispatcher::setNavigationGoal(const Pose& goalPose_world,  bool setOrientationToPath) {
	cancelNavigationGoal();

	// advertise the initial position for the navigation stack everytime the navigation goal is set
	navigationGoal_world = goalPose_world;

	// navigation goal is set from the base_links perspective. Convert goalPose into base_links frame
	navigationGoal = engineState.baseLinkInMapFrame.inverse().applyTransformation(goalPose_world);
	ROS_INFO_STREAM("convert goal(world)" << goalPose_world << " from base_link=" << engineState.baseLinkInMapFrame << " = navigation gaol(base_link)" << navigationGoal);


	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = navigationGoal.position.x/1000.0;
	goal.target_pose.pose.position.y = navigationGoal.position.y/1000.0;
	goal.target_pose.pose.position.z = 0;

	geometry_msgs::Quaternion goalPoseQuat  = tf::createQuaternionMsgFromYaw(navigationGoal.orientation.z);
	goal.target_pose.pose.orientation = goalPoseQuat;

	moveBaseClient->sendGoal(goal);

	ROS_INFO_STREAM("setting navigation goal (" << goalPose_world << ") " << navigationGoal.position << string(setOrientationToPath?" (latched orientation)":""));

	// latch the flag indicating that once the global path has been computed by navigation stack, set the orientation
	latchedGoalOrientationToPath = setOrientationToPath;
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
			if (ok) {
				generationNumber = stringToInt(generationNumberStr,ok);
				if (!ok || (generationNumber < mapGenerationNumber)) {
					// passed version is older than ours, deliver map
					response = serializedMap + "," + globalCostMapSerialized + "," + darkScaryHolesSerialized + ","  + getResponse(true);
				}
				else {
					// client has already the current map version, dont deliver it again
					response = getResponse(true);
				}
			} else {
				// deliver map without version check
				response = serializedMap + "," + globalCostMapSerialized + "," + darkScaryHolesSerialized + ","  + getResponse(true);
			}
			okOrNOk = true;
			return true;
		}
	}

	if (hasPrefix(uri, "/costmap/")) {
		string mapCommand = uri.substr(string("/costmap/").length());
		string generationNumberStr ;
			if (hasPrefix(mapCommand, "local/get")) {
			bool ok = getURLParameter(urlParamName, urlParamValue, "no", generationNumberStr);
			if (ok) {
				int generationNumber = stringToInt(generationNumberStr,ok);
				if (!ok || (generationNumber < localCostmapGenerationNumber)) {
					// passed version is older than ours, deliver map
					response = localCostMapSerialized + "," + getResponse(true);
				}
				else {
					// client has already the current map version, dont deliver it again
					response = getResponse(true);
				}
			} else {
				// deliver map localCostMapSerialized version check
				response = localCostMapSerialized + "," + getResponse(true);
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
			bool deliverContent = false;
			if (serializedTrajectory != "") {
				string generationNumberStr ;
				bool ok = getURLParameter(urlParamName, urlParamValue, "no", generationNumberStr);
				if (ok) {
					int generationNumber = stringToInt(generationNumberStr,ok);
					if (!ok || (generationNumber < trajectoryGenerationNumber))
						deliverContent = true;
				} else
					deliverContent = true;
			}

			response = getResponse(true);
			if (deliverContent)
				response = serializedTrajectory + "," + response;


			okOrNOk = true;
			return true;
		}
	}

	if (hasPrefix(uri, "/plan/")) {
		string command = uri.substr(string("/plan/").length());
		if (hasPrefix(command, "get")) {
			bool deliverContent = false;
			if (localPlanSerialized != "") {
				string generationNumberStr ;
				bool ok = getURLParameter(urlParamName, urlParamValue, "no", generationNumberStr);
				if (ok) {
					int generationNumber = stringToInt(generationNumberStr,ok);
					if (!ok || (generationNumber < localPlanGenerationNumber))
						deliverContent = true;
				} else
					deliverContent = true;
			}

			response = getResponse(true);
			if (deliverContent)
				response = globalPlanSerialized +"," + localPlanSerialized + "," + response;


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
			setNavigationGoal(goalPose, setNavigationOrientation);
			response = getResponse(true);
			okOrNOk = true;
			return true;
		}
		else if (hasPrefix(command,"goal/get")) {
			std::ostringstream out;
			int navStatus = (int)getNavigationStatusType();

			getNavigationGoal().serialize(out);
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

NavigationStatusType Dispatcher::getNavigationStatusType() {
	if (!getNavigationGoal().isNull())
		return (NavigationStatusType)(int)getNavigationGoalStatus().state_;
	return NavigationStatusType::NavPending;
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

void Dispatcher::listenerOccupancyGrid (const nav_msgs::OccupancyGrid::ConstPtr& og ) {
	convertOccupancygridToMap(og, SLAM_MAP_TYPE, slamMap, mapGenerationNumber, serializedMap);
}

void Dispatcher::listenerGlobalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og ) {
	convertOccupancygridToMap(og, COSTMAP_TYPE, globalCostMap, globalCostmapGenerationNumber, globalCostMapSerialized);

	IntoDarkness::getInstance().feedGlobalMap(slamMap, globalCostMap, odomFrame, odomPose);

	vector<Point> holes;
	IntoDarkness::getInstance().getDarkScaryHoles(holes);
	std::stringstream out;
	serializeVectorOfSerializable(holes,out);
	darkScaryHolesSerialized = out.str();

}

void Dispatcher::listenerLocalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og ) {
	convertOccupancygridToMap(og, COSTMAP_TYPE, localCostMap, localCostmapGenerationNumber, localCostMapSerialized);
	IntoDarkness::getInstance().feedLocalMap(localCostMap);
}

void convertPoseStampedToTrajectory( const nav_msgs::Path::ConstPtr&  inputPlan, const Pose& odomFrame, Trajectory& outputPlan, int& generationNumber,  string& serializedPlan) {
	outputPlan.clear();
	for (unsigned int i = 0;i<inputPlan->poses.size();i++) {
		Point p(inputPlan->poses[i].pose.position.x*1000.0,inputPlan->poses[i].pose.position.y*1000.0,0);
		Quaternion q(inputPlan->poses[i].pose.orientation.x,
					 inputPlan->poses[i].pose.orientation.y,
					 inputPlan->poses[i].pose.orientation.z,
					 inputPlan->poses[i].pose.orientation.w);

		StampedPose sp(odomFrame.applyTransformation(Pose(p,q)),inputPlan->poses[i].header.stamp.toSec()*1000.0);
		outputPlan.add(sp);
	}

	outputPlan.setGenerationNumber(++generationNumber);

	std::stringstream out;
	outputPlan.serialize(out);
	serializedPlan = out.str();
}

void Dispatcher::listenerLocalPlan(const nav_msgs::Path::ConstPtr& og ) {
	convertPoseStampedToTrajectory(og, odomFrame, localPlan, localPlanGenerationNumber, localPlanSerialized);
}

void Dispatcher::listenerGlobalPlan(const nav_msgs::Path::ConstPtr& og ) {
	convertPoseStampedToTrajectory(og, Pose(), globalPlan, globalPlanGenerationNumber, globalPlanSerialized);

	// in case the need to set the goal orientation, do it now

	if (!navigationGoal.isNull() && latchedGoalOrientationToPath && (globalPlan.size() > 5)) {
		// global path does not go straight to the goal, very often the last piece is bent
		// so identify the direction the bot is coming from by taking the vector of the last piece
		const millimeter lastPieceLength = 400; // consider at least the radius of the bot as minimu distance
		int curr = globalPlan.size()-1;
		StampedPose lastPose = globalPlan[curr];
		while ((curr > 0) && (globalPlan[curr].pose.position.distance(lastPose.pose.position) < lastPieceLength))
			curr--;

		StampedPose prevPose = globalPlan[curr]; // this pose is at least 3000ms earlier than the predicted goal arrival time
		// compute the orientation
		Point orientationVec =  prevPose.pose.position-navigationGoal_world.position;
		navigationGoal_world.orientation.z = 0*engineState.currentBodyPose.orientation.z + 0*engineState.currentNoseOrientation + atan2(orientationVec.y, orientationVec.x);

		ROS_INFO_STREAM("setting new goal'orientation " << navigationGoal_world << " vec=" << orientationVec << " atan=" << atan2(orientationVec.y, orientationVec.x) << " nose=" << engineState.currentNoseOrientation );
		setNavigationGoal(navigationGoal_world, false);
	}
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

// subscription to path
void Dispatcher::listenToTrajectory(const nav_msgs::Path::ConstPtr& path) {
	Trajectory trajectory;
	trajectory.clear();
	for (unsigned int i = 0;i<path->poses.size();i++) {
		Point p(path->poses[i].pose.position.x*1000.0,path->poses[i].pose.position.y*1000.0,path->poses[i].pose.position.z*1000.0);
		Quaternion q(path->poses[i].pose.orientation.x,
				     path->poses[i].pose.orientation.y,
					 path->poses[i].pose.orientation.z,
					 path->poses[i].pose.orientation.w);

		StampedPose sp(Pose(p,q),path->poses[i].header.stamp.toSec()*1000.0);
		trajectory.add(sp);
	}

	std::stringstream out;
	trajectory.serialize(out);
	serializedTrajectory = out.str();
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


void Dispatcher::clearCostmaps() {
 	std_srvs::Empty srv;
    ROS_INFO("clear costmaps");
	clearCostmapService.call(srv);
}

// subscription to bots state
void Dispatcher::listenerBotState(const std_msgs::String::ConstPtr&  fullStateStr) {
	std::istringstream in(fullStateStr->data);
	engineState.deserialize(in);

 	// engineState.currentBaselinkPose.position = odomFrame.position + odomPose.position.getRotatedAroundZ(odomFrame.orientation.z);
 	// engineState.currentBaselinkPose.orientation = odomFrame.orientation  + odomPose.orientation;

	// ignore passed base_link and use odom_frame and /odom topic
	baseLinkInMapFrame = odomFrame.applyTransformation(odomPose);
	engineState.baseLinkInMapFrame = baseLinkInMapFrame;
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
	if (getNavigationStatusType() == NavigationStatusType::NavActive) {
		Pose toBePose = FreeWill::getInstance().getAutonomousBodyPose();
		if (abs(toBePose.position.z - engineState.currentBodyPose.position.z) > floatPrecision) {
			ROS_INFO_STREAM("advertise new body pose" << toBePose);
			advertiseBodyPoseToEngine(toBePose);
		}
	}
}
