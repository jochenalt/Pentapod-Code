/*
 * DarkHoleFinder.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: jochenalt
 */

#include "DarkHoleFinder.h"

DarkHoleFinder::DarkHoleFinder() {
}

DarkHoleFinder::~DarkHoleFinder() {
}

void DarkHoleFinder::setMaxLaserRange (millimeter maxRange) {
	maxLaserRange = maxRange;
}

// feed the hole finder with a new costmap and maintain the inner structure of all found holes
void DarkHoleFinder::feed(const Map& slamMap, const Map& globalCostmap, const Pose& pose) {

}
