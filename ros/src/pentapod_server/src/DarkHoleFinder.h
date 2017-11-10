/*
 * DarkHoleFinder.h
 *
 * Class to find all dark holes where the pentapod can hide.
 * For that purpose it stores a data structure to keep track of all
 * existing dark holes and to work incrementally on deltas only.
 * Requires
 *  o a slam map to idenfify known free grids.
 *  o a 2DCostmap to identify grids with free space and
 *    to check if walls are close to this grid (ideally
 *    in the shape of a corner or a U)
 *  o a window stating where the robot currently is to limit the
 *    search algorithm to that window where changes can happen.
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
#include "spatial.h"
#include <Map.h>

class DarkHoleFinder {
public:
	DarkHoleFinder();
	virtual ~DarkHoleFinder();

	void setMaxLaserRange (millimeter maxRange);

	void feed(const Map& slamMap, const Map& globalCostmap, const Pose& pose);
private:
	millimeter maxLaserRange;
};

#endif /* PENTAPOD_SERVER_SRC_DARKHOLEFINDER_H_ */