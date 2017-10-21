/*
 * Map.h
 *
 *  Created on: 27.07.2017
 *      Author: JochenAlt
 */

#ifndef LASER_SCAN_H
#define LASER_SCAN_H

#include "basics/serializer.h"
#include "basics/types.h"
#include "basics/point.h"
#include "spatial.h"

#include <vector>
#include "setup.h"


class LaserScan  : public Serializable {
public:
	LaserScan();
	LaserScan(const LaserScan& map);
	void operator=(const LaserScan& m);

	virtual ~LaserScan();
	void null();
	bool isNull() const;

	virtual std::ostream& serialize(std::ostream &out) const;
	virtual std::istream& deserialize(std::istream &in, bool& ok);

	// set a new laser scan
	void setLaserScan(const Pose& newScanpose, const std::vector<int_millimeter> &newScan, angle_rad newStartAngle, angle_rad newAngleIncrement, angle_rad newEndAngle);
	int getNumberOfLaserScan() { return (int)laserScan.size(); };

	// return the scan of one point (i = 0..360/angleIncrement) in coordinate system of the laser scanner
	Point getLaserScan(int i);
	Pose& getLaserScanPose() { return poseWhenLaserScan; };

private:
	std::vector<int_millimeter> laserScan;
	Pose poseWhenLaserScan;
	angle_rad startAngle;
	angle_rad angleIncrement;
	angle_rad endAngle;
};

#endif /* MAP_H_ */
