/*
 * Navigator.cpp
 *
 *  Created on: Dec 15, 2017
 *      Author: jochenalt
 */

// common ROS message types
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

#include "Navigator.h"
#include "IntoDarkness.h"
#include "Dispatcher.h"

#include <move_base_msgs/MoveBaseAction.h>
#include "std_srvs/Empty.h"

Navigator::Navigator() {
}

Navigator::~Navigator() {
}


void convertOccupancygridToMap(const nav_msgs::OccupancyGrid::ConstPtr& inputMap,  Map& outputMap, int& generationNumber,  string& serializedMap) {
  if ((inputMap->info.width > 0) && (inputMap->info.height > 0)) {
    outputMap.null();
    outputMap.setGridDimension(inputMap->info.width, inputMap->info.height, inputMap->info.resolution*1000.0);
    int mapArraySize = inputMap->info.width * inputMap->info.height;
    int width = inputMap->info.width;
    for (int i = 0;i<mapArraySize;i++) {
      // the regular call is m.setOccupancyByGridCoord(i/width,i % width, occupancyGrid.data[i])
      // but we take the short (and ugly) route
      int occupancy = inputMap->data[i];

      outputMap.getVector()[i] = occupancy;
    }
    outputMap.setGenerationNumber(++generationNumber);
  }

  std::stringstream out;
  outputMap.serialize(out);
  serializedMap = out.str();
}


void Navigator::listenerGlobalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og ) {
	convertOccupancygridToMap(og, globalCostMap, globalCostmapGenerationNumber, globalCostMapSerialized);

	IntoDarkness::getInstance().feedGlobalMap();
}

void Navigator::listenerLocalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og ) {
  convertOccupancygridToMap(og, localCostMap, localCostmapGenerationNumber, localCostMapSerialized);
  IntoDarkness::getInstance().feedLocalMap();
}

void Navigator::setup(ros::NodeHandle& handle) {
	localCostmapGenerationNumber = -1;
	globalCostmapGenerationNumber = -1;
	localPlanGenerationNumber = -1;
	globalPlanGenerationNumber = -1;

	//tell the action client that we want to spin a thread by default and wait until up and running
	moveBaseClient = new MoveBaseClient("move_base", true);

	// wait for the action server to come up. Do this in a 10Hz loop while producing the
	// identical map->odom transformation which is required by the navigation stack.
	// After this method, the map->odom transformation will be taken up by the main loop
	ros::Time now = ros::Time::now();
	ROS_INFO_STREAM("starting up move_base");
	while (!moveBaseClient->waitForServer(ros::Duration(0.1)) && (ros::Time::now() - now < ros::Duration(10.0))) {
		if (ros::Time::now() - now > ros::Duration(4.0))
			ROS_INFO_THROTTLE(1, "waiting for move base to come up");
		Dispatcher::getInstance().broadcastTransformationMapToOdom();
	}

	if (!moveBaseClient->isServerConnected()) {
		ROS_ERROR("move_base did not come up!!!");
		return;
	}

	// subscribe to the navigation stack topic that delivers the global costmap
    ROS_INFO_STREAM("subscribe to /move_base/global_costmap/costmap");
	globalCostmapSubscriber = handle.subscribe("/move_base/global_costmap/costmap", 1000, &Navigator::listenerGlobalCostmap, this);

	// subscribe to the navigation stack topic that delivers the local costmap
	ROS_INFO_STREAM("subscribe to /move_base/local_costmap/costmap");
	localCostmapSubscriber = handle.subscribe("/move_base/local_costmap/costmap", 1000, &Navigator::listenerLocalCostmap, this);

	clearCostmapService = handle.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	// subscribe to the right topic of the local planner used
	std::string base_local_planner;
	ros::param::get("/move_base/base_local_planner", base_local_planner);
	std::string local_plan_topic_name = "/move_base" + base_local_planner.substr(base_local_planner.find("/")) + "/local_plan";
	std::string global_plan_topic_name = "/move_base" + base_local_planner.substr(base_local_planner.find("/")) + "/global_plan";

	// subscribe to path of local planner
    ROS_INFO_STREAM("subscribe to local plan from " << local_plan_topic_name);
	localPathSubscriber = handle.subscribe(local_plan_topic_name, 1000, &Navigator::listenerLocalPlan, this);

	// subscribe to path of global planner
	ROS_INFO_STREAM("subscribe to global plan from " << global_plan_topic_name);
	globalPathSubscriber = handle.subscribe(global_plan_topic_name, 1000, &Navigator::listenerGlobalPlan, this);
}


void Navigator::clearCostmaps() {
 	std_srvs::Empty srv;
    ROS_INFO("clear costmaps");
	clearCostmapService.call(srv);
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


void Navigator::listenerLocalPlan(const nav_msgs::Path::ConstPtr& og ) {
	convertPoseStampedToTrajectory(og, Dispatcher::getInstance().getOdomFrame(), localPlan, localPlanGenerationNumber, localPlanSerialized);
}

void Navigator::listenerGlobalPlan(const nav_msgs::Path::ConstPtr& og ) {
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
		navigationGoal_world.orientation.z = atan2(orientationVec.y, orientationVec.x);

		ROS_INFO_STREAM("setting new goal'orientation " << navigationGoal_world << " vec=" << orientationVec << " atan=" << atan2(orientationVec.y, orientationVec.x)  );
		setNavigationGoal(navigationGoal_world, false);
	}
}


void Navigator::cancelNavigationGoal() {
	if (!navigationGoal.isNull() && (!moveBaseClient->getState().isDone())) {
		ROS_INFO_STREAM("cancel previous goal " << navigationGoal.position);
		moveBaseClient->cancelGoal();
	}
}

void Navigator::setNavigationGoal(const Pose& goalPose_world,  bool setOrientationToPath) {
	cancelNavigationGoal();

	// advertise the initial position for the navigation stack everytime the navigation goal is set
	navigationGoal_world = goalPose_world;

	// navigation goal is set from the base_links perspective. Convert goalPose into base_links frame
	navigationGoal = Dispatcher::getInstance().getEngineState().baseLinkInMapFrame.inverse().applyTransformation(goalPose_world);
	ROS_INFO_STREAM("convert goal(world)" << goalPose_world << " from base_link=" << Dispatcher::getInstance().getEngineState().baseLinkInMapFrame << " = navigation gaol(base_link)" << navigationGoal);


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

actionlib::SimpleClientGoalState Navigator::getNavigationGoalStatus() {
	return moveBaseClient->getState();
}

Pose Navigator::getNavigationGoal() {
	return navigationGoal_world;
}


NavigationStatusType Navigator::getNavigationStatusType() {
	if (!getNavigationGoal().isNull())
		return (NavigationStatusType)(int)getNavigationGoalStatus().state_;
	return NavigationStatusType::NavPending;
}


void Navigator::advertiseBodyPose() {
	if (getNavigationStatusType() == NavigationStatusType::NavActive) {
		Pose toBePose = FreeWill::getInstance().getAutonomousBodyPose();
		if (abs(toBePose.position.z - Dispatcher::getInstance().getEngineState().currentBodyPose.position.z) > floatPrecision) {
			ROS_INFO_STREAM("advertise new body pose" << toBePose);
			Dispatcher::getInstance().advertiseBodyPoseToEngine(toBePose);
		}
	}
}
