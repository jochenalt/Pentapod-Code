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

bool DarkHoleFinder::isCandidate(millimeter x, millimeter y) {
	int pivotValue = costMap.getValueByWorld(x,y);
	realnum holeValue = 0;
	int wallCounter = 0;
	if ((pivotValue < LethalThreshold ) && (pivotValue >= 0)) {
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
	if (wallCounter > 5)
		return true;
	return false;
}

realnum DarkHoleFinder::computeScariness(millimeter x, millimeter y) {
	return 0.8;
}


int DarkHoleFinder::getHashIdx(millimeter x, millimeter y) {
	int gridSize = slamMap.getGridSize();
	int w = slamMap.getGridsWidth();
	int h = slamMap.getGridsHeight();
	if ((abs(x) >= w*gridSize/2) || (abs(y) >= h*gridSize/2))
		cout << "(x,y)" << x << "," << y << endl;


	int xi = x/gridSize + w/2;
	int yi = y/gridSize + h/2;
	int hashIdx = w*yi + xi;
	int xi_check = hashIdx % w;
	int yi_check = hashIdx / w;
	millimeter x_check = (xi_check-w/2)*gridSize;
	millimeter y_check = (yi_check-h/2)*gridSize;
	if (abs(x-x_check) > 50)
		cout << "(x,y)=" << x << "," << y << " " << hashIdx << " ->" << x_check << "," << y_check << endl;
	if (abs(y-y_check) > 50)
		cout << "(x,y)=" << x << "," << y << " " << hashIdx << " ->" << x_check << "," << y_check << endl;

	return hashIdx;
}

void DarkHoleFinder::getCoordByHashIdx(int hashIdx, millimeter& x, millimeter& y) {
	int gridSize = slamMap.getGridSize();
	int w = slamMap.getGridsWidth();
	int h = slamMap.getGridsHeight();

	int xi = hashIdx % w;
	int yi = hashIdx / w;

	x = (xi-w/2)*gridSize;
	y = (yi-h/2)*gridSize;

}

realnum DarkHoleFinder::getScariness(millimeter x, millimeter y) {
	int hashIdx = getHashIdx(x,y);
	return getScariness(hashIdx);
}

realnum DarkHoleFinder::getScariness(int hashIdx) {
	std::map<int, realnum>::iterator holeIterator = foundDarkHoles.find(hashIdx);
	if (holeIterator == foundDarkHoles.end())
		return 0;
	return holeIterator->second;
}

void DarkHoleFinder::addDarkScaryHole(millimeter x, millimeter y, realnum s) {
	int hashIdx = getHashIdx(x,y);
	// dark hole is found, keep in temporary list before reducing this list
	foundDarkHoles[hashIdx] = s;
}

void DarkHoleFinder::removeIfBetterHoleInNeighbourhood(millimeter x, millimeter y) {
	int hashIdx = getHashIdx(x,y);
	realnum pivotValue = foundDarkHoles[hashIdx];
	if (pivotValue > 0) {
		for (int xc = -1;xc <= 1; xc++) {
			for (int yc = -1;yc <= 1; yc++) {
				if ((xc != 0) || (yc != 0)) {
					int hashIdx = getHashIdx(x + xc*slamMap.getGridSize(), y+yc*slamMap.getGridSize());
					if (getScariness(hashIdx) < pivotValue) {
						// we found a better hole, but continue to check the direct environment
						foundDarkHoles.erase(hashIdx);

						// continue to check the direct environment
						removeIfBetterHoleInNeighbourhood(x + xc*slamMap.getGridSize(), y+yc*slamMap.getGridSize());
					}
				}
			}
		}
	}
}

// feed the hole finder with a new costmap and maintain the inner structure of all found holes
void DarkHoleFinder::findHole() {
	foundDarkHoles.clear();

	std::vector<int> holes; // holes that have been found in this run
	for (millimeter x = pose.position.x - width/2; x < pose.position.x + width/2; x += slamMap.getGridSize()) {
		for (millimeter y = pose.position.y - height/2; y < pose.position.y + height/2; y += slamMap.getGridSize()) {
			int hashIdx = getHashIdx(x,y);

			// check if pivot grid cell is a dark hole candidate
			if (isCandidate(x,y)) {

				// compute scaryness
				realnum s = computeScariness(x,y);

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
			millimeter x,y;
			getCoordByHashIdx(hashIdx, x,y);
			removeIfBetterHoleInNeighbourhood(x,y);
		}
	}
}

void DarkHoleFinder::getDarkScaryHoles(std::vector<Point>& holes) {
	holes.clear();

	std::map<int, realnum>::iterator holeIterator = foundDarkHoles.begin();
	while (holeIterator != foundDarkHoles.end()) {
		millimeter x,y;
		getCoordByHashIdx(holeIterator->first, x,y);
		holes.push_back(Point(x,y,0));
		holeIterator++;
	}
}

