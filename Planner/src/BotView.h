/*
 * BotViewController.h
 *
 *  Created on: 07.08.2016
 *      Author: JochenAlt
 */

#ifndef UI_BOTVIEW_H_
#define UI_BOTVIEW_H_

#include <string>
#include "BotDrawer.h"
#include "core.h"
#include "BaseView.h"

using namespace std;

class BotView : public BaseView {
public:
	BotView();
	int create(int mainWindow, string pTitle);

	void display();
	void reshape(int x,int y, int w, int h);

	void setSingleLegPose(const LegPose& singleLegPose);

	void setPose(angle_rad orientation, const Pose& bodyPose, const PentaPointType& footTouchPoint, const PentaPoseType &hipPose, const PentaLegAngleType& legAngles);

	void hide();
	void show();

	void setSingleLegActive(bool ok);
	void MotionCallback(int x, int y);
	void MouseCallback(int button, int button_state, int x, int y);
private:
	void drawCoordSystem(bool withRaster );
	void drawGaitRefPoints(realnum heightOverGround, const bool footOnGround[NumberOfLegs]);
	void drawGroundPoints(realnum heightOverGround, const PentaPointType &groundPoints );
	void drawGroundDistancePoints(realnum heightOverGround, const PentaPointType &groundDistancePoints  );

	void drawStandingArea(const bool footOnGround[NumberOfLegs]);


	string title;
	LimbAngles singleLegAngles;
	LegPose singleLegPose;
	Pose mouseBodyPose;
	LegPose mouseSingleLeg;
	GLint viewport[4];                  // Where The Viewport Values Will Be Stored

	PentaPointType gaitRefPoints;
	PentaLegAngleType legAngles;
	PentaPoseType hipFromBellyPerspective;
	Pose bodyPose;
	angle_rad noseOrientation;
	bool singleLegActive;

	int lastMouseX = 0;
	int lastMouseY = 0;
	int lastMouseScroll = 0;
	enum mousePaneType { VIEW_PANE, BOT_BODY_PANE, BOT_BODYORIENTATION_PANE, BOT_FRONTLEG_HORIZ_PANE, BOT_FRONTLEG_VERTICAL_PANE, NO_PANE };
	mousePaneType mousePane = NO_PANE;
};

#endif /* UI_BOTVIEW_H_ */
