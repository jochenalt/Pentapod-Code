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
#include <EngineProxy.h>

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
	Point get3DByMouseClick (int x,int y);
	void setNavigationGoal(const Pose& p);
	void defineNavigationGoal();

	void drawFreeSlamGrid( const Point &p1,const Point &p2, const Point &p3, const Point &p4 );
	void drawOccupiedSlamGrid(bool onlyTop,  const Point &p1,const Point &p2, const Point &p3, const Point &p4 );
	void drawCostmapGrid( int value, const Point &g1,const Point &g2, const Point &g3, const Point &g4 );
	void drawLaserScan();
	void drawNavigationGoal();
	void drawCoordRaster();
	void drawMapBackground();
	void drawTrajectory(EngineProxy::TrajectoryType type);
	void drawCoordSystem();
	void drawSlamMap();
	void drawCostMap();

	void drawSmallBot(const Pose& pose);
	string title;

	int lastMouseX = 0;
	int lastMouseY = 0;
	int lastMouseScroll = 0;

	bool mouseViewPlane = false; 				// true when we move the view point via mouse
	bool mousePlaneXY = false;   				// true when we translate the xy plane
	bool mouseNavigationGoalDirection = false; 	// true, when the direction of the navigation goal is chosen

	PentaPoseType defaultHipPoseWorld;
	LegAnglesType defaultLegAngles;
	Pose defaultBodyPose;

	// generic marker that leaves a flag in the map. Use for navigation goal.
	Pose navigationGoal;

	Pose lastFusedPosition;
	milliseconds manualLookAtAdjustTime;
};

#endif /* SLAMVIEW_H_ */
