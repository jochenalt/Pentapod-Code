/*
 * Leg.h
 *
 *  Created on: 01.05.2017
 *      Author: JochenAlt
 */

#ifndef LEG_H_
#define LEG_H_

#include "core.h"
#include "HerkulexServoDrive.h"

typedef  float LimbAnglesType[NumberOfLimbs];

class Leg {
public:
	Leg();

	// to be called before doing anything.
	// Initializes the serial line and id of that leg, the Herkulex Manager
	// and initializes all servos
	void setup(int id, uint8_t serialId);

	// read a distance from sensor. Available via getDistance
	void fetchDistance();

	// set the target position of that leg in terms of angles and duration
	// for the movement to that target position. Actual movement happens by
	// calling servos[i].loop()
	void setAngles(LimbAnglesType angles, uint16_t duration_ms);

	// print leg status to serial console
	void logStatus();

	// get a status of the most sick leg
	ServoStatusType getStatus(int limbID);

	// last measured distance (cached)
	int getDistance();

	// true if distance has been measured
	bool distanceAvailable();

	// id of that leg, starting from 0
	int legId;

	// last measured distance from toe to ground
	int distance;

	// current status of distance measurement
	int distanceStatus;

	// all servos of that leg
	HerkulexServoDrive	servos[NumberOfLimbs];

	// HerkuleX Manager that controls the serial communication to all servos in that leg
	HerkulexClass herkulexMgr;

	// underlying serial line
	HardwareSerial* herkulexSerial;
};

#endif /* LEG_H_ */
