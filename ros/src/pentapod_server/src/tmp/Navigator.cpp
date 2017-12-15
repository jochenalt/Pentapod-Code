/*
 * Navigator.cpp
 *
 *  Created on: Dec 15, 2017
 *      Author: jochenalt
 */

#include "Navigator.h"
#include "IntoDarkness.h"

Navigator::Navigator() {
	// TODO Auto-generated constructor stub

}

Navigator::~Navigator() {
	// TODO Auto-generated destructor stub
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
	IntoDarkness::getInstance().feedLocalMap(localCostMap);
}

void Navigator::setup(ros::NodeHandle& handle) {
	localCostmapGenerationNumber = -1;
	globalCostmapGenerationNumber = -1;

	// subscribe to the navigation stack topic that delivers the global costmap
    ROS_INFO_STREAM("subscribe to /move_base/global_costmap/costmap");
	globalCostmapSubscriber = handle.subscribe("/move_base/global_costmap/costmap", 1000, &Navigator::listenerGlobalCostmap, this);

	// subscribe to the navigation stack topic that delivers the local costmap
    ROS_INFO_STREAM("subscribe to /move_base/local_costmap/costmap");
	localCostmapSubscriber = handle.subscribe("/move_base/local_costmap/costmap", 1000, &Navigator::listenerLocalCostmap, this);

}
