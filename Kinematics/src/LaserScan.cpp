/*
 * Map.cpp
 *
 *  Created on: 27.07.2017
 *      Author: JochenAlt
 */

#include <LaserScan.h>
#include "setup.h"
#include "basics/zip.h"

LaserScan::LaserScan() {
	null();
}

LaserScan::~LaserScan() {
}


void LaserScan::null() {
	startAngle = 0;
	endAngle = 0;
	angleIncrement = 0;
	laserScan.clear();
	poseWhenLaserScan.null();
}

bool LaserScan::isNull() const {
	return (laserScan.size() == 0);
}


LaserScan::LaserScan(const LaserScan& m) {
	(*this) = m;
}


std::ostream& LaserScan::serialize(std::ostream &out) const {
	out << "{ ";
	out << "\"sa\":";
	serializePrim(out, startAngle);
	out << ",\"ea\":";
	serializePrim(out, endAngle);
	out << ",\"ai\":";
	serializePrim(out, angleIncrement);
	out << ",\"scanpose\":";
	poseWhenLaserScan.serialize(out);
	out << ",\"scan\":";
	serializeVectorOfPrimitives(laserScan, out);

	out << "}";
	return out;
}


std::istream& LaserScan::deserialize(std::istream &in, bool &ok) {
	if (in) {
		null();
    	parseCharacter(in, '{', ok);
    	parseString(in, ok); // "sa"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, startAngle, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "ea"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, endAngle, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "ai"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, angleIncrement, ok);
    	parseCharacter(in, ',', ok);
       	parseString(in, ok); // "scanpose"
       	parseCharacter(in, ':', ok);
       	poseWhenLaserScan.deserialize(in, ok);
       	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "scan"
    	parseCharacter(in, ':', ok);
    	deserializeVectorOfPrimitives(in, laserScan, ok);
    	parseCharacter(in, '}', ok);
	}
	return in;
}


void LaserScan::operator=(const LaserScan& m) {
	startAngle = m.startAngle;
	endAngle = m.endAngle;
	angleIncrement = m.angleIncrement;
	laserScan = m.laserScan;
	poseWhenLaserScan = m.poseWhenLaserScan;
}

void LaserScan::setLaserScan(const Pose& newScanpose, const std::vector<int_millimeter> &newScan, angle_rad newStartAngle, angle_rad newAngleIncrement, angle_rad newEndAngle) {
	laserScan = newScan;
	startAngle = newStartAngle;
	endAngle = newEndAngle;
	angleIncrement = newAngleIncrement;
	poseWhenLaserScan = newScanpose;
}


Point LaserScan::getLaserScan(int i) {
	angle_rad angle = startAngle + i*angleIncrement;
	int_millimeter distance = laserScan[i];
	if (distance == -1)
		return Point(); // null value
	millimeter real_distance = distance;
	return Point(cos(angle)*real_distance, sin(angle)*real_distance, 0);
};

