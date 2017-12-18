/*
 * BotDrawer.cpp
 *
 * Author: JochenAlt
 */


#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>
#include <GL/Glui.h>

#include <core.h>
#include "basics/StringHelper.h"

#include <BotDrawer.h>

#include "uiconfig.h"
#include "setup.h"

void BotDrawer::displayBot( angle_rad noseOrientation, const Pose& bodyPose, const PentaPoseType& hipPose, const PentaLegAngleType& legAngles) {
	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();

	// glLoadIdentity();             // Reset the model-view matrix to world coordinate system
	glRotatef(-90, 1.0,0.0,0.0);
	glRotatef(-90, 0.0,0.0,1.0);

	// move to body's centre
	glRotatef(degrees(noseOrientation), 0.0,0.0,1.0);
	glTranslatef(bodyPose.position.x, bodyPose.position.y,bodyPose.position.z);

	// rotate in zyx convention, as used in Kinematics::RotationMatrix
	glRotatef(degrees(bodyPose.orientation.z), 0.0,0.0,1.0);
	glRotatef(degrees(bodyPose.orientation.y), 0.0,1.0,0.0);
	glRotatef(degrees(bodyPose.orientation.x), 1.0,0.0,0.0);

	glPushMatrix();
	glTranslatef(0,0,CAD::BodyHipHeight + (CAD::HipCentreDistance + CAD::HipLength)*sin(radians(CAD::HipNickAngle)));
	body.display(glBotBodyColor,glBotBodyColor);
	lidar.display(glLidarColor,glLidarColor);

	glPopMatrix();

	for (int i = 0;i<NumberOfLegs;i++) {
		glPushMatrix();

		const GLfloat* color;
		if ((((NumberOfLegs % 2) == 0) && ((i == (NumberOfLegs/2) -1) || (i == (NumberOfLegs/2) ))) ||
			(((NumberOfLegs % 2) == 1) &&  (i == (NumberOfLegs/2))))
			color = glBotFrontLegColor;
		else
			color = glBotLegColor;

		Pose hipFromBellyPerspective = hipPose[i];
		LimbAngles angles = legAngles[i];


		// move to the hip
		glRotatef(degrees(hipFromBellyPerspective.orientation.x), 1.0,0.0,0.0);
		glRotatef(degrees(hipFromBellyPerspective.orientation.y), 0.0,1.0,0.0);
		glRotatef(degrees(hipFromBellyPerspective.orientation.z), 0.0,0.0,1.0);
		glRotatef(90, 0.0,1.0,0.0);
		glTranslatef(hipFromBellyPerspective.position.x, hipFromBellyPerspective.position.y,hipFromBellyPerspective.position.z);

		// compensate orientation of hip stl file
		glRotatef(90, 0.0,0.0,1.0);
		glRotatef(-90,1.0,0.0, 0.0);
		glRotatef(CAD::HipNickAngle, 1.0,0.0,0.0);

		hip.display(color,color);
		glRotatef(degrees(angles[LimbConfiguration::HIP]),0.0,0.0, 1.0);

		hipJoint.display(color,color);

		glTranslatef(0.0,-CAD::HipJointLength, 0.0);
		glRotatef(90,0.0,1.0, 0.0);
		glRotatef(-(degrees(angles[LimbConfiguration::THIGH])),0.0,0.0, 1.0);
		thigh.display(color,color);

		glTranslatef(0.0,-CAD::ThighKneeGapLength -CAD::ThighLength, 0.0);
		glRotatef(-degrees(angles[LimbConfiguration::KNEE])+90.0,0.0,1.0, 0.0);
		kneeJoint.display(color,color);

		glTranslatef(0.0,- CAD::KneeJointLength,0.0);
		glRotatef(-degrees(angles[LimbConfiguration::LOWERLEG])-90.0,1.0,0.0, 0.0);
		lowerLeg.display(color,color);
		glPopMatrix();
	}

	glPopMatrix();
	glPopAttrib();
}


void BotDrawer::displayLeg(const LimbAngles& angles, const GLfloat* color, const GLfloat* accentColor) {
	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();             // Reset the model-view matrix

	glTranslatef(0.0,200.0,0.0);
	glRotatef(-90,1.0,0.0, 0.0);
	hip.display(color,color);

	glRotatef(degrees(angles[LimbConfiguration::HIP]),0.0,0.0, 1.0);
	hipJoint.display(color,color);

	glTranslatef(0.0,-CAD::HipJointLength, 0.0);
	glRotatef(90,0.0,1.0, 0.0);
	glRotatef(-(degrees(angles[LimbConfiguration::THIGH])),0.0,0.0, 1.0);
	thigh.display(color,color);

	glTranslatef(0.0,-CAD::ThighKneeGapLength - CAD::ThighLength, 0.0);
	glRotatef(-degrees(angles[LimbConfiguration::KNEE])+90.0,0.0,1.0, 0.0);
	kneeJoint.display(color,color);

	glTranslatef(0.0,-CAD::KneeJointLength,0.0);
	glRotatef(-degrees(angles[LimbConfiguration::LOWERLEG])-90.0,1.0,0.0, 0.0);
	lowerLeg.display(color,color);

	glPopMatrix();
	glPopAttrib();
}



void BotDrawer::readSTLFiles(string path) {
	body.loadFile(path + "/body.stl");
	hip.loadFile(path + "/hip.stl");
	hipJoint.loadFile(path + "/hipjoint.stl");
	thigh.loadFile(path + "/thigh.stl");
	kneeJoint.loadFile(path + "/knee.stl");
	lowerLeg.loadFile(path + "/foot.stl");
	lidar.loadFile(path + "/Lidar.stl");

}

void BotDrawer::setup() {
	static bool setupDone = false;
	if (!setupDone) {
		// search for stl files
		if (fileExists("./stl/hip.stl")) {
			readSTLFiles("./stl");
		} else {
			if (fileExists("./hip.stl"))
				readSTLFiles("./");
			if (fileExists("../../cad/simplified/hip.stl"))
				readSTLFiles("../../cad/simplified");
			else
				if (fileExists("../../../cad/simplified/hip.stl"))
					readSTLFiles("../../../cad/simplified");
		}
		setupDone = true;
	}
}
