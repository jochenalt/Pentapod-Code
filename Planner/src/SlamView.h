/*
 * SlamView.h
 *
 *  Created on: 26.07.2017
 *      Author: JochenAlt
 */

#ifndef SLAMVIEW_H_
#define SLAMVIEW_H_

#include <string>
#include <BaseView.h>
#include <Map.h>
#include <spatial.h>

using namespace std;

class SlamView : public BaseView {
public:
	SlamView();
	int create(int mainWindow, string pTitle);
	void display();
	void reshape(int x,int y, int w, int h);
	void drawMap();

	void MotionCallback(int x, int y);
	void MouseCallback(int button, int button_state, int x, int y);
	virtual ~SlamView();
	bool botIsVisible();

private:
	void drawFreeSlamGrid( const Point &p1,const Point &p2, const Point &p3, const Point &p4 );
	void drawOccupiedSlamGrid(bool onlyTop,  const Point &p1,const Point &p2, const Point &p3, const Point &p4 );
	void drawLaserScanPoint(const Point &p);

	void drawSmallBot(const Pose& pose);
	string title;

	int lastMouseX = 0;
	int lastMouseY = 0;
	int lastMouseScroll = 0;
	bool mouseViewPane = false;
	bool mousePlaneXY = false;

	PentaPoseType defaultHipPoseWorld;
	LegAnglesType defaultLegAngles;
	Pose defaultBodyPose;

	Pose lastFusedPosition;
	milliseconds manualLookAtAdjustTime;
};

#endif /* SLAMVIEW_H_ */
