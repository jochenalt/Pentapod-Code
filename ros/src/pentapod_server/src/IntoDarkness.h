/*
 * DarkHoleFinder.h
 *
 * Class to find all dark holes where the pentapod can hide.
 * For that purpose it stores a data structure to keep track of all
 * existing dark holes and to work incrementally on deltas only.
 * Requires
 *  o a slam map to idenfify known free grids.
 *  o a 2DCostmap to identify grids with free space and
 *    to check how close walls are at each grid cells
 *  o a window stating where the robot currently is to limit the
 *    search algorithm to the window where changes are happening.
 *
 * Maintains a data structure to deliver all dark holes and to
 * find the closest dark hole to a given position
 *
 *  Created on: Nov 9, 2017
 *      Author: Jochen Alt
 */

#ifndef PENTAPOD_SERVER_SRC_DARKHOLEFINDER_H_
#define PENTAPOD_SERVER_SRC_DARKHOLEFINDER_H_

#include <vector>
#include <stdio.h>
#include <string>
#include "setup.h"
#include "Engine.h"
#include "basics/spatial.h"
#include <Map.h>
#include <LaserScan.h>

class IntoDarkness {
public:
	IntoDarkness();
	virtual ~IntoDarkness();
	void setup(ros::NodeHandle handle);
	void feedGlobalMap(const Map& slamMap, const Map& globalCostmap, const Pose& odomFrame, const Pose& pose);
	void feedLocalMap(const Map& localCostMap);
	void feedLaserMap(const LaserScan& laserScan);

	void getDarkScaryHoles(std::vector<Point>& holes);
	realnum getCurrentScariness();

	static IntoDarkness& getInstance() { static IntoDarkness intoDarkness; return intoDarkness; };
private:
	void findDarkAndScaryHoles();
	void removeIfBetterHoleInNeighbourhood(millimeter_int x, millimeter_int y);
	void removeLighterHole(int hashIdx);

	void addDarkScaryHole(millimeter_int x, millimeter_int y, realnum s);
	realnum getScariness(millimeter_int x, millimeter_int y);
	realnum getScariness(int hashIdx);

	int getHashIdx(millimeter_int x, millimeter_int y);
	void getCoordByHashIdx(int hashIdx, millimeter_int& x, millimeter_int& y);

	bool isCandidate(millimeter_int x, millimeter_int y);
	realnum computeLocalScariness(millimeter_int x, millimeter_int y) const;
	realnum computeGlobalScariness(millimeter_int x, millimeter_int y, int numberOfRays, realnum maxRayDistance) const;



	double width;
	double height;
	double rayMinDistance;
	double rayMaxDistance;
	double wallClosenessMaxDistance;

	double scarynessthreshold;
	Map *slamMap = NULL;
	Map *costMap = NULL;
	Map *localCostMap = NULL;
	LaserScan *laserScan = NULL;
	Pose* odomFrame = NULL;
	Pose* pose = NULL;

	std::map<int, realnum> foundDarkHoles;
};

#endif /* PENTAPOD_SERVER_SRC_DARKHOLEFINDER_H_ */
