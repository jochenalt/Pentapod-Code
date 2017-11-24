/*
 * DarkHoleFinder.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: jochenalt
 */

#include "DarkHoleFinder.h"

const int LethalThreshold = 99;
DarkHoleFinder::DarkHoleFinder() {
}

DarkHoleFinder::~DarkHoleFinder() {
}


void DarkHoleFinder::setup(ros::NodeHandle handle) {
	handle.param<double>("dark_hole_finder/width", width, 6.0);
	handle.param<double>("dark_hole_finder/height", height, 6.0);
	// convert to mm
	width *= 1000;
	height *= 1000;

}

void DarkHoleFinder::feed(const Map& newSlamMap, const Map& newCostMap, const Pose& newPose) {
	slamMap = newSlamMap;
	costMap = newCostMap;
	pose = newPose;

	findHole();
}

bool DarkHoleFinder::isCandidate(millimeter_int x, millimeter_int y) {
	int pivotValue = costMap.getValueByWorld(x,y);
	realnum holeValue = 0;
	int wallCounter = 0;
	if ((pivotValue < LethalThreshold ) && (pivotValue >= 0) && (slamMap.getOccupancyByWorld(x,y) ==  Map::FREE)) {
		for (int xc = -1;xc <= 1; xc++) {
			for (int yc = -1;yc <= 1; yc++) {
				if ((xc != 0) || (yc != 0)) {
					int value = costMap.getValueByWorld(x+xc*slamMap.getGridSize(),y+yc*slamMap.getGridSize());
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
	if (wallCounter > 4)
		return true;
	return false;
}

realnum DarkHoleFinder::computeScariness(millimeter_int x, millimeter_int y) {
	return 0.8;
}


int DarkHoleFinder::getHashIdx(millimeter_int x, millimeter_int y) {

	int gridSize = slamMap.getGridSize();
	int w = slamMap.getGridsWidth();
	int h = slamMap.getGridsHeight();

	if ((abs(x) >= w*gridSize/2) || (abs(y) >= h*gridSize/2)) {
		return -1;
	}


	int xi = ((int)(x) + gridSize/2)/gridSize + w/2;
	int yi = ((int)(y) + gridSize/2)/gridSize + h/2;
	int hashIdx = w*yi + xi;
	return hashIdx;
}

void DarkHoleFinder::getCoordByHashIdx(int hashIdx, millimeter_int& x, millimeter_int& y) {
	int gridSize = slamMap.getGridSize();
	int w = slamMap.getGridsWidth();
	int h = slamMap.getGridsHeight();

	int xi = hashIdx % w;
	int yi = hashIdx / w;

	x = (xi-w/2)*gridSize - gridSize/2;
	y = (yi-h/2)*gridSize - gridSize/2;
}

realnum DarkHoleFinder::getScariness(millimeter_int x, millimeter_int y) {
	int hashIdx = getHashIdx(x,y);
	return getScariness(hashIdx);
}

realnum DarkHoleFinder::getScariness(int hashIdx) {
	std::map<int, realnum>::iterator holeIterator = foundDarkHoles.find(hashIdx);
	if (holeIterator == foundDarkHoles.end())
		return 0;
	return holeIterator->second;
}

void DarkHoleFinder::addDarkScaryHole(millimeter_int x, millimeter_int y, realnum s) {
	int hashIdx = getHashIdx(x,y);
	// dark hole is found, keep in temporary list before reducing this list
	foundDarkHoles[hashIdx] = s;
}

void DarkHoleFinder::removeIfBetterHoleInNeighbourhood(millimeter_int x, millimeter_int y) {
	int gridSize = slamMap.getGridSize();
	int w = slamMap.getGridsWidth();
	int h = slamMap.getGridsHeight();
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
void DarkHoleFinder::findHole() {
	foundDarkHoles.clear();
	int gridSize = slamMap.getGridSize();
	int w = slamMap.getGridsWidth();
	int h = slamMap.getGridsHeight();

	// set position to a multiple of gridsize
	Point origin = pose.position;
	origin.x = ((int)(origin.x/gridSize + 0.5)) * gridSize;
	origin.y = ((int)(origin.y/gridSize + 0.5)) * gridSize;

	std::vector<int> holes; // holes that have been found in this run
	for (millimeter x = origin.x - width/2; x < origin.x + width/2; x += gridSize) {
		for (millimeter y = origin.y - height/2; y < origin.y + height/2; y += gridSize) {

			int xGrid, yGrid;
			int hashIdx = getHashIdx(x,y);
			getCoordByHashIdx(hashIdx, xGrid, yGrid);

			// check if pivot grid cell is a dark hole candidate
			if (isCandidate(x,y)) {

				// compute scaryness
				realnum s = computeScariness(xGrid,yGrid);

				if (s > 0.7) {
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

void DarkHoleFinder::getDarkScaryHoles(std::vector<Point>& holes) {
	holes.clear();

	std::map<int, realnum>::iterator holeIterator = foundDarkHoles.begin();
	while (holeIterator != foundDarkHoles.end()) {
		millimeter_int x,y;
		getCoordByHashIdx(holeIterator->first, x,y);
		holes.push_back(Point(x,y,0));
		holeIterator++;
	}
}

