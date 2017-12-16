/*
 * DarkHoleFinder.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: jochenalt
 */

#include "IntoDarkness.h"
#include "Dispatcher.h"

// in ros costmaps, lethal zones are 100
const int LethalThreshold = 99;
// according to our setting of inflation area in the costmap, 80 is a costmap value that is still reachable but close to a wall
const int CandidateThreshold = 80;

IntoDarkness::IntoDarkness() {
	pose = new Pose();
}

IntoDarkness::~IntoDarkness() {
}


void IntoDarkness::setup(ros::NodeHandle handle) {
	handle.param<double>("dark_hole_finder/width", width, 6.0);
	handle.param<double>("pentapod_server/into_darkness/height", height, 6.0);
	handle.param<double>("pentapod_server/into_darkness/ray_min_distance", rayMinDistance, 0.35);
	handle.param<double>("pentapod_server/into_darkness/ray_max_distance", rayMaxDistance, 8.00);
	handle.param<double>("pentapod_server/into_darkness/close_wall_max_distance", wallClosenessMaxDistance, 1.00);
	handle.param<double>("pentapod_server/into_darkness/scaryness_threshold", scarynessthreshold,0.50);


	// convert to mm
	width *= 1000.0;
	height *= 1000.0;
	rayMinDistance *= 1000.0;
	rayMaxDistance *= 1000.0;
	wallClosenessMaxDistance *= 1000.0;
}

void IntoDarkness::feedGlobalMap() {
	// store the reference to the passed maps, do not copy for performance reasons

	slamMap = (Map*)&Dispatcher::getInstance().getSlamMap();
	pose = (Pose*)&Dispatcher::getInstance().getOdomPose();
	costMap = (Map*)&Navigator::getInstance().getGlobalCostmap();
	pose = (Pose*)&Dispatcher::getInstance().getOdomPose();
	findDarkAndScaryHoles();


	std::stringstream out;
	vector<Point> holes;
	getDarkScaryHoles(holes);
	serializeVectorOfSerializable(holes,out);
	darkScaryHolesSerialized = out.str();
}

void IntoDarkness::feedLocalMap() {
	// store the reference to the passed maps, do not copy for performance reasons
}

void IntoDarkness::feedLaserMap(const LaserScan& newLaserScan) {
	laserScan = (LaserScan*)&newLaserScan;
}

// find candidate by xor-ing each grid cell against the pattern below and take all cells with a result of at least 5 as candidates
//    111
//   1   1
//   1 0 1
//   1   1
//    111

bool IntoDarkness::isCandidate(millimeter_int x, millimeter_int y) {
	int pivotValue = Navigator::getInstance().getGlobalCostmap().getValueByWorld(x,y);
	realnum holeValue = 0;
	int wallCounter = 0;
	const int radius = 2;
	if ((pivotValue < CandidateThreshold ) && (pivotValue >= 0) && (slamMap->getOccupancyByWorld(x,y) ==  Map::FREE)) {
		for (int xc = -radius;xc <= radius; xc++) {
			for (int yc = -radius;yc <= radius; yc++) {
				if (((abs(xc) == radius) || (abs(yc) == radius)) && (abs(yc) != abs(xc))) {
					int value = Navigator::getInstance().getGlobalCostmap().getValueByWorld(x+xc*slamMap->getGridSize(),y+yc*slamMap->getGridSize());
					if (value > pivotValue) {
						if (value > LethalThreshold)
							wallCounter += 2;
						if (value == LethalThreshold)
							wallCounter += 1;
					}
				}
			}
		}
	}
	if (wallCounter >= 5)
		return true;
	return false;
}

// scariness is a metric between 0..1 identifying how tight the walls are around a location. 1 is the most scariness, kinda a coffin, 0 is free space
realnum IntoDarkness::computeGlobalScariness(millimeter_int x, millimeter_int y, int numberOfRays, realnum maxRayDistance) const {
	if (slamMap == NULL)
		return -1;
	// send out 16 rays and measure the distance to the next wall
	int gridSize = slamMap->getGridSize();
	realnum scaryness = 0.0;
	for (realnum alpha = 0; alpha < 2.0*M_PI;alpha += M_PI*2.0/numberOfRays) {
		realnum s = sin(alpha);
		realnum c = cos(alpha);
		realnum gridDistance = sqrt(s*s + c*c)*gridSize;
		for (realnum distance = rayMinDistance+gridDistance;distance < maxRayDistance;distance += gridDistance) {
			if (slamMap->getOccupancyByWorld(x + c*distance, y + s*distance) == Map::OCCUPIED) {
				scaryness += 1000.0/(1000.0 + distance-rayMinDistance);
				break;
			}
		}
	}
	return (scaryness/numberOfRays);
}


realnum IntoDarkness::getCurrentScariness() {
	return computeGlobalScariness(pose->position.x,pose->position.y, 32, wallClosenessMaxDistance );
}



// diagonalize the 2d space into one hash code
int IntoDarkness::getHashIdx(millimeter_int x, millimeter_int y) {

	int gridSize = slamMap->getGridSize();
	int w = slamMap->getGridsWidth();
	int h = slamMap->getGridsHeight();

	if ((abs(x) >= w*gridSize/2) || (abs(y) >= h*gridSize/2)) {
		return -1;
	}

	int xi = ((int)(x) + gridSize/2)/gridSize + w/2;
	int yi = ((int)(y) + gridSize/2)/gridSize + h/2;
	int hashIdx = w*yi + xi;
	return hashIdx;
}

// reverse diagonalization, get 2d coordinates out of a hash code
void IntoDarkness::getCoordByHashIdx(int hashIdx, millimeter_int& x, millimeter_int& y) {
	int gridSize = slamMap->getGridSize();
	int w = slamMap->getGridsWidth();
	int h = slamMap->getGridsHeight();

	int xi = hashIdx % w;
	int yi = hashIdx / w;

	x = (xi-w/2)*gridSize - gridSize/2;
	y = (yi-h/2)*gridSize - gridSize/2;
}

realnum IntoDarkness::getScariness(millimeter_int x, millimeter_int y) {
	int hashIdx = getHashIdx(x,y);
	return getScariness(hashIdx);
}

realnum IntoDarkness::getScariness(int hashIdx) {
	std::map<int, realnum>::iterator holeIterator = foundDarkHoles.find(hashIdx);
	if (holeIterator == foundDarkHoles.end())
		return 0;
	return holeIterator->second;
}

void IntoDarkness::addDarkScaryHole(millimeter_int x, millimeter_int y, realnum s) {
	int hashIdx = getHashIdx(x,y);
	// dark hole is found, keep in temporary list before reducing this list
	foundDarkHoles[hashIdx] = s;
}

void IntoDarkness::removeIfBetterHoleInNeighbourhood(millimeter_int x, millimeter_int y) {
	int gridSize = slamMap->getGridSize();
	int w = slamMap->getGridsWidth();
	int h = slamMap->getGridsHeight();
	// return if outside the grid
	if ((abs(x) >= w*gridSize/2) || (abs(y) >= h*gridSize/2))
		return;

	int hashIdx = getHashIdx(x,y);
	realnum pivotValue = getScariness(hashIdx);
	if (pivotValue > 0) {
		for (int xc = -1;xc <= 1; xc++) {
			for (int yc = -1;yc <= 1; yc++) {
				if ((xc != 0) || (yc != 0)) {
					int hashIdx = getHashIdx(x + xc*gridSize, y+yc*gridSize);

					if ((hashIdx >= 0) && (getScariness(hashIdx) < pivotValue)) {
						// we found a better hole, but continue to check the direct environment
						foundDarkHoles.erase(hashIdx);

						// continue to check the direct environment
						removeIfBetterHoleInNeighbourhood(x + xc*gridSize, y+yc*gridSize);
					}
				}
			}
		}
	}
}

// feed the hole finder with a new costmap and maintain the inner structure of all found holes
void IntoDarkness::findDarkAndScaryHoles() {
	int gridSize = slamMap->getGridSize();
	int w = slamMap->getGridsWidth();
	int h = slamMap->getGridsHeight();

	// set position to a multiple of gridsize
	Point origin = pose->position;
	origin.x = ((int)(origin.x/gridSize + 0.5)) * gridSize;
	origin.y = ((int)(origin.y/gridSize + 0.5)) * gridSize;

	// clear all dark holes in the area that we are about to examine
	for (millimeter x = origin.x - width/2; x < origin.x + width/2; x += gridSize) {
		for (millimeter y = origin.y - height/2; y < origin.y + height/2; y += gridSize) {
			int hashIdx = getHashIdx(x,y);
			foundDarkHoles.erase(hashIdx);
		}
	}


	std::vector<int> holes; // holes that have been found in this run

	// examine the given area, find candidates and real dark scary holes
	for (millimeter x = origin.x - width/2; x < origin.x + width/2; x += gridSize) {
		for (millimeter y = origin.y - height/2; y < origin.y + height/2; y += gridSize) {

			int xGrid, yGrid;
			int hashIdx = getHashIdx(x,y);
			getCoordByHashIdx(hashIdx, xGrid, yGrid);

			// check if pivot grid cell is a dark hole candidate
			if (isCandidate(x,y)) {

				realnum s = computeGlobalScariness(xGrid,yGrid, 32, rayMaxDistance);

				if (s > scarynessthreshold) {
					// dark hole is found, store in map and keep in temporary list as well
					foundDarkHoles[hashIdx] = s;
					holes.push_back(hashIdx);
				} else
					foundDarkHoles.erase(hashIdx);
			} else
				foundDarkHoles.erase(hashIdx);
		}
	}

	// now remove all dark holes that have better dark holes in their direct neighborhood
	for (int i = 0;i<holes.size();i++) {
		int hashIdx = holes[i];
		if (foundDarkHoles[hashIdx] > 0) {
			millimeter_int x,y;
			getCoordByHashIdx(hashIdx, x,y);
			removeIfBetterHoleInNeighbourhood(x,y);
		}
	}
}

void IntoDarkness::getDarkScaryHoles(std::vector<Point>& holes) {
	holes.clear();

	std::map<int, realnum>::iterator holeIterator = foundDarkHoles.begin();
	while (holeIterator != foundDarkHoles.end()) {
		millimeter_int x,y;
		getCoordByHashIdx(holeIterator->first, x,y);
		holes.push_back(Point(x,y,0));
		holeIterator++;
	}
}

