/*
 * BotDrawer.h
 *
 * Draw the bot given by stl files in openGL
 *
 *  Created on: 18.08.2016
 *      Author: JochenAlt
 */

#ifndef UI_BOTDRAWER_H_
#define UI_BOTDRAWER_H_

#include "STLObject.h"
#include "LegKinematics.h"
#include "core.h"

class BotDrawer {
public:
	BotDrawer() {};
	static BotDrawer& getInstance() {
		static BotDrawer instance;
		return instance;
	}

	// display the bot with the given joint angles in the current openGL window
	void displayBot(angle_rad orientation,const Pose& bodyPose, const PentaPoseType& hipPose, const LegAnglesType& legAngles);

	// display the bot with the given joint angles in the current openGL window
	void displayLeg( const LimbAngles& angles, const GLfloat* color, const GLfloat* accentColor);

	// setup by looking for the STL files
	void setup();
private:
	// read the stl files per actuator in that path
	void readSTLFiles(string path);


	STLObject body;
	STLObject lidar;

	STLObject hip;
	STLObject hipJoint;
	STLObject thigh;

	STLObject kneeJoint;

	STLObject lowerLeg;
};

#endif /* UI_BOTDRAWER_H_ */
