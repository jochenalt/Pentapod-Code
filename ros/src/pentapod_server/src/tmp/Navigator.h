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
	void listenerGlobalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og );
	void listenerLocalCostmap(const nav_msgs::OccupancyGrid::ConstPtr& og );

	Map& getGlobalCostmap() { return globalCostMap; };
	Map& getLocalCostmap() { return localCostMap; };

	std::string getGlobalCostmapSerialized() { return globalCostMapSerialized; };
	std::string getLocalCostmapSerialized() { return localCostMapSerialized; };

	int getLocalCostmapGenerationNumber() { return localCostmapGenerationNumber; };
	int getGlobalCostmapGenerationNumber() { return globalCostmapGenerationNumber; };

private:
	Map globalCostMap;
	std::string globalCostMapSerialized;

	Map localCostMap;
	std::string localCostMapSerialized;
	int localCostmapGenerationNumber;
	int globalCostmapGenerationNumber;


	ros::Subscriber globalCostmapSubscriber;
	ros::Subscriber localCostmapSubscriber;
};

#endif /* PENTAPOD_SERVER_SRC_NAVIGATOR_H_ */
