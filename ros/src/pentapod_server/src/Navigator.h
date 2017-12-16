/*
 * Navigator.h
 *
 *  Created on: Dec 15, 2017
 *      Author: jochenalt
 */

#ifndef PENTAPOD_SERVER_SRC_NAVIGATOR_H_
#define PENTAPOD_SERVER_SRC_NAVIGATOR_H_


#include <vector>
#include <stdio.h>
#include <string>

// pentapod engine basics
#include "basics/spatial.h"
#include "setup.h"
#include "Map.h"
#include "Trajectory.h"

// ros basic types
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "nav_msgs/OccupancyGrid.h"

// move base action client
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigator {
public:
	Navigator();
	virtual ~Navigator();

	void setup(ros::NodeHandle& handle);

	static Navigator& getInstance() { static Navigator singleton; return singleton; };

	Map& getGlobalCostmap() { return globalCostMap; };
	Map& getLocalCostmap() { return localCostMap; };

	std::string getGlobalCostmapSerialized() { return globalCostMapSerialized; };
	std::string getLocalCostmapSerialized() { return localCostMapSerialized; };

	int getLocalCostmapGenerationNumber() { return localCostmapGenerationNumber; };
	int getGlobalCostmapGenerationNumber() { return globalCostmapGenerationNumber; };

	std::string getGlobalPlanSerialized() { return globalPlanSerialized; };
	std::string getLocalPlanSerialized() { return localPlanSerialized; };

	int getGlobalPlanGenerationNumber() { return globalPlanGenerationNumber; };
	int getLocalPlanGenerationNumber() { return localPlanGenerationNumber; };

	// call the move_base service to clear all costmaps.
	void clearCostmaps();

	void setNavigationGoal(const Pose& goalPose, bool setOrientationToPath = false);
	Pose getNavigationGoal();
	void cancelNavigationGoal();
	NavigationStatusType getNavigationStatusType();
	actionlib::SimpleClientGoalState getNavigationGoalStatus();
	void advertiseBodyPose();
private:
	void listenerGlobalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og );
	void listenerLocalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og );
	void listenerLocalPlan(const nav_msgs::Path::ConstPtr& og );
	void listenerGlobalPlan(const nav_msgs::Path::ConstPtr& og );


	Map globalCostMap;
	std::string globalCostMapSerialized;

	Map localCostMap;
	std::string localCostMapSerialized;
	int localCostmapGenerationNumber;
	int globalCostmapGenerationNumber;


	ros::Subscriber globalCostmapSubscriber;
	ros::Subscriber localCostmapSubscriber;
	ros::ServiceClient clearCostmapService;

	Trajectory localPlan;
	std::string localPlanSerialized;
	ros::Subscriber localPathSubscriber;
	int localPlanGenerationNumber;

	Trajectory globalPlan;
	std::string globalPlanSerialized;
	ros::Subscriber globalPathSubscriber;
	int globalPlanGenerationNumber;

	MoveBaseClient* moveBaseClient;
	Pose navigationGoal;
    Pose navigationGoal_world;
	bool latchedGoalOrientationToPath;
};

#endif /* PENTAPOD_SERVER_SRC_NAVIGATOR_H_ */
