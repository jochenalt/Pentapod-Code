/*
 * SlamView.cpp
 *
 *  Created on: 26.07.2017
 *      Author: JochenAlt
 */


#include "SlamView.h"

#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>
#include <GL/Glui.h>

#include "basics/spatial.h"
#include "basics/util.h"

#include "uiconfig.h"
#include "setup.h"
#include "WindowController.h"
#include "EngineProxy.h"
#include "EngineState.h"
#include "UIUtils.h"

SlamView::SlamView() {
}

SlamView::~SlamView() {
}


void SlamViewMotionCallback(int x,int y) {
	WindowController::getInstance().slamView.MotionCallback(x,y);
}

void SlamView3DMouseCallback(int button, int button_state, int x, int y ) {
	WindowController::getInstance().slamView.MouseCallback( button,  button_state,  x,  y );
}

void displaySlamView() {
	WindowController::getInstance().slamView.display();
}

int SlamView::create(int mainWindow, string pTitle) {

	manualLookAtAdjustTime = 0;
	latchGoalOrientation = false;
	defaultHipPoseWorld = EngineProxy::getInstance().getHipPoseWorld();
	defaultLegAngles = EngineProxy::getInstance().getLegAngles();
	defaultBodyPose = EngineProxy::getInstance().getImuAwareBodyPose();

	title = pTitle;
	windowHandle = glutCreateSubWindow(mainWindow, 1,1,1,1);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   							// Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    							// Set the type of depth-test
	glShadeModel(GL_SMOOTH);   							// Enable smooth shading
	glEnable(GL_BLEND); 								// enable transparent views
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA  );// way of transparency
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); 	// Nice perspective corrections

	setLights();
	setEyePosition(ViewMapEyeDistance, radians(0), radians(-45));

	glutSetWindow(windowHandle);
	glutDisplayFunc(displaySlamView);			// register display function
	glutInitDisplayMode(GLUT_DOUBLE); 			// double bufferinh
	glutMotionFunc( SlamViewMotionCallback); 	// register mouse function
	glutMouseFunc( SlamView3DMouseCallback); 	// register mouse function

	return windowHandle;
}


void SlamView::display() {
	int savedWindowHandle = glutGetWindow();
	glutSetWindow(windowHandle);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();             // Reset the model-view matrix

	displayEyePosition();

	glMatrixMode(GL_MODELVIEW);

	drawMap();

	glFlush();  		// Render now
	glutSwapBuffers(); 	// use double buffering
	glutSetWindow(savedWindowHandle);
}

void SlamView::reshape(int x,int y, int w, int h) {
	glutSetWindow(windowHandle);
	glutShowWindow();
	glutPositionWindow(x, y);
	glutReshapeWindow(w, h);
	glViewport(0, 0, w, h);

	display();
}

void SlamView::drawLaserScan() {
	// draw red laser scan dots
	LaserScan& laserScan = EngineProxy::getInstance().getLaserScan();
	int numberOfScans = laserScan.getNumberOfLaserScan();

	glPushMatrix();
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glLaserScanColor4v);
	glColor3fv(glLaserScanColor4v);
	const Pose& laserScanPose = laserScan.getLaserScanPose();
	for (int i = 0;i<numberOfScans; i++) {
		Point laserScanPoint = laserScan.getLaserScan(i);
		if (!laserScanPoint.isNull()) {
			laserScanPoint.z = 150;
			glLoadIdentity();
			glTranslatef(laserScanPose.position.y, laserScanPose.position.z, laserScanPose.position.x);
			glRotatef(degrees(laserScanPose.orientation.z), 0.0,1.0,.0);
			glTranslatef(laserScanPoint.y, laserScanPoint.z, laserScanPoint.x);

			glutSolidSphere(15, 3, 3);
		}
	}
	glPopMatrix();
}

void drawOneNavigationGoal(const Pose &goal) {
	glPushMatrix();
	glLoadIdentity();

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glMapMarkerColor4v);
	glColor3fv(glMapMarkerColor4v);

	if (!goal.isNull())
	{
		glTranslatef(goal.position.y, goal.position.z, goal.position.x);
		glRotatef(-90, 1,0,0);
		GLUquadricObj *quadratic = gluNewQuadric();
		gluCylinder(quadratic, 20, 10, 1000, 12, 1);
		glRotatef(90, 1,0,0);

		// draw the sphere in the color of the status
		NavigationStatusType navStatus = EngineProxy::getInstance().getCurrentNavigationStatus();
		int sphereRGBColor;
		switch (navStatus) {
			case NavPending:   sphereRGBColor = YellowGrey;break;
			case NavActive:    sphereRGBColor = TrafficGreen;break;
			case NavPreempted: sphereRGBColor = GreenBrown;break;
			case NavSucceeded: sphereRGBColor = TurquoiseGreen;break;
			case NavAborted:   sphereRGBColor = RedBrown;break;
			case NavRejected:  sphereRGBColor = RedLilac;break;
			case NavLost:      sphereRGBColor = RedOrange;break;
			case NavRecalled:  sphereRGBColor = BlackRed;break;

		}
		GLfloat sphereColor[4] = GL_COLOR_4v( sphereRGBColor, glAlphaSolid);

		glTranslatef(0, 1000, 0);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, sphereColor);
		glColor3fv(sphereColor);
		glutSolidSphere(50, 12, 12);

		// draw the small flag on top
		glRotatef(degrees(goal.orientation.z), 0, 1,0);
		glBegin(GL_TRIANGLE_STRIP);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glMapSphereMarkerColor4v);
			glColor4fv(glMapSphereMarkerColor4v);
			glNormal3f(0.0,1.0,0.0);
			glVertex3f(0,-50,0);
			glVertex3f(0,0,300);
			glVertex3f(0,50,0);
		glEnd();
	}

	glPopMatrix();
}


void SlamView::drawNavigationGoal() {
	drawOneNavigationGoal(tempNavigationGoal);
	drawOneNavigationGoal(EngineProxy::getInstance().getCurrentNavigationGoal());

}

void SlamView::drawDarkScaryHoles() {
	glPushMatrix();

	vector<Point> darkScaryHoles = EngineProxy::getInstance().getDarkScaryHoles();
	for (unsigned int i = 0;i<darkScaryHoles.size(); i++) {
		Point hole = darkScaryHoles[i];
		glLoadIdentity();

		glTranslatef(hole.y, hole.z, hole.x);
		glRotatef(-90, 1,0,0);
		GLUquadricObj *quadratic = gluNewQuadric();
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glScaryHoleMarkerColor4v);
		glColor3fv(glScaryHoleMarkerColor4v);

		gluCylinder(quadratic, 5, 30, 200, 12, 1);
		glRotatef(90, 1,0,0);

		glTranslatef(0, 200, 0);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glScaryHoleSphereColor4v);
		glColor3fv(glScaryHoleSphereColor4v);
		glutSolidSphere(50, 12, 12);
	}

	glPopMatrix();

}

void SlamView::drawFreeSlamGrid( const Point &g1,const Point &g2, const Point &g3, const Point &g4 ) {
	glBegin(GL_TRIANGLE_STRIP);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glSlamFreeTopColor4v);
		glColor4fv(glSlamFreeTopColor4v);
		glNormal3f(0.0,1.0,0.0);
		glVertex3f(g1.y, g1.z, g1.x);
		glVertex3f(g2.y, g2.z, g2.x);
		glVertex3f(g4.y, g4.z, g4.x);
		glVertex3f(g3.y, g3.z, g3.x);
	glEnd();
}

void SlamView::drawCostmapGrid( CostmapType type, int value, const Point &g1,const Point &g2, const Point &g3, const Point &g4 ) {
	float lethalNess= value/100.0;

	// spread the lethalNess a bit; lethal or dangerous value are full red,
	if (value >= 99)
		lethalNess = 1.0;
	else
		lethalNess *= 0.7;

	// show lethal points in red, and harmless grid cells in green
	GLfloat color[] = { 0,0,0, glAlphaTransparent};

	bool display = true;
	if (type == GLOBAL_COSTMAP) {
		if (value <= 99) {
			color[0] = lethalNess/2.0f;
			color[1] = (1.0f-lethalNess)/2.0f;
			color[2] = 0.0;
			color[3] = 0.4;
		} else
			display = false;
	} else {
		color[0] = lethalNess/2.0f;
		color[1] = (1.0f-lethalNess)/2.0f;
		color[2] = 0.0;
		color[3] = 0.3;
	}
	if (display) {
		glBegin(GL_TRIANGLE_STRIP);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
			glColor4fv(color);
			glNormal3f(0.0,1.0,0.0);
			int offset = 10;
			if (type == GLOBAL_COSTMAP)
				offset = 5;
			glVertex3f(g1.y, g1.z + offset, g1.x);
			glVertex3f(g2.y, g2.z + offset, g2.x);
			glVertex3f(g4.y, g4.z + offset, g4.x);
			glVertex3f(g3.y, g3.z + offset, g3.x);
		glEnd();
	}
}

void SlamView::drawOccupiedSlamGrid(bool onlyTop,  const Point &g1,const Point &g2, const Point &g3, const Point &g4 ) {
	glBegin(GL_TRIANGLE_STRIP);
		// top
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glSlamOccupiedTopColor4v);
		glColor4fv(glSlamOccupiedTopColor4v);
		glNormal3f(0.0,1.0,0.0);
		glVertex3f(g1.y, g1.z, g1.x);
		glVertex3f(g2.y, g2.z, g2.x);
		glVertex3f(g4.y, g4.z, g4.x);
		glVertex3f(g3.y, g3.z, g3.x);
	glEnd();
	if (!onlyTop) {
		glBegin(GL_TRIANGLE_STRIP);
			// around
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glSlamOccupiedAroundTopColor4v);
			glColor4fv(glSlamOccupiedAroundTopColor4v);
			glNormal3f(0.0,0.0,1.0);
			glVertex3f(g1.y, 0, g1.x);glVertex3f(g1.y, g1.z, g1.x);
			glVertex3f(g2.y, 0, g2.x);glVertex3f(g2.y, g2.z, g2.x);
			glNormal3f(1.0,0.0,0.0);
			glVertex3f(g2.y, 0, g2.x);glVertex3f(g3.y, g3.z, g3.x);
			glNormal3f(0.0,0.0,-1.0);
			glVertex3f(g3.y, 0, g3.x);glVertex3f(g4.y, g4.z, g4.x);
			glNormal3f(1.0,0.0,0.0);
			glVertex3f(g4.y, 0, g4.x);glVertex3f(g1.y, g1.z, g1.x);
			glVertex3f(g1.y, 0, g1.x);
		glEnd();
	}
}

void SlamView::drawSmallBot(const Pose& pose) {

	glPushMatrix();
	glLoadIdentity();

	glTranslatef(pose.position.y, pose.position.z, pose.position.x);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glFootTouchPointColor);
	glColor3fv(glFootTouchPointColor);

	BotDrawer::getInstance().displayBot(
			EngineProxy::getInstance().getNoseOrientation(),
			EngineProxy::getInstance().getImuAwareBodyPose(),
			EngineProxy::getInstance().getHipPoseWorld(),
			EngineProxy::getInstance().getLegAngles());
	glPopMatrix();

}

bool SlamView::botIsVisible() {
	bool botIsclose = (getEyeDistance() < 5000);
	float botDistanceToLookAtPoint = getLookAtPosition().distance(EngineProxy::getInstance().getMapPose().position);
	return (botIsclose && (botDistanceToLookAtPoint < getEyeDistance()));
}

void SlamView::drawMapBackground() {
	// draw one area as base. This is necessary to allow clicking in an 3D
	// object in order to give gluUnProject an objekt where the z-axis ends
	Map& map = EngineProxy::getInstance().getMap();
	int fullMapSizeX = map.getMapSizeX();
	int fullMapSizeY = map.getMapSizeY();
	glBegin(GL_QUADS);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glMapBackgroundColor4v);
		glColor3fv(glMapBackgroundColor4v);
        glNormal3f(0.0,1.0,0.0);
		glVertex3f(-fullMapSizeY/2, -5, -fullMapSizeX/2); glVertex3f(fullMapSizeY/2, 0, -fullMapSizeX/2);
		glVertex3f(fullMapSizeY/2, -5, fullMapSizeX/2);
		glVertex3f(-fullMapSizeY/2, -5, fullMapSizeX/2);
		glVertex3f(-fullMapSizeY/2, -5, -fullMapSizeX/2);
	glEnd();
}

void SlamView::drawCoordRaster() {
	// draw the basic raster in 1m x 1m
	millimeter rasterUnitLength = 1000; // one grid represents one meter, coord system unit is a millimeter
	Map& map = EngineProxy::getInstance().getMap();
	int fullMapSizeX = map.getMapSizeX();
	int fullMapSizeY = map.getMapSizeY();

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glSlamGridColor4v);
	glColor3fv(glSlamGridColor4v);
	for (int rasterX = -rasterUnitLength*(int)((fullMapSizeX/2)/rasterUnitLength); rasterX<fullMapSizeX/2;rasterX += rasterUnitLength ) {
		glBegin(GL_LINES);
			glVertex3f( -fullMapSizeY/2, 0, rasterX);glVertex3f( fullMapSizeY/2, 0, rasterX);
		glEnd();
	}
	for (int rasterY = -rasterUnitLength*(int)((fullMapSizeY/2)/rasterUnitLength);rasterY<fullMapSizeY/2;rasterY += rasterUnitLength ) {
		glBegin(GL_LINES);
			glVertex3f(rasterY,0, -fullMapSizeX/2);glVertex3f(rasterY, 0, fullMapSizeY/2);
		glEnd();
	}
}

void SlamView::drawTrajectory(EngineProxy::TrajectoryType type) {

	Trajectory& trajectory = EngineProxy::getInstance().getTrajectory(type);

	// draw trajectory
	unsigned int pathLength = trajectory.size();
	glPushMatrix();

	if (pathLength > 0) {
		switch (type) {
			case EngineProxy::TRAJECTORY: {
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glTrajectoryColor4v);
				glColor3fv(glTrajectoryColor4v);
				break;
			}
			case EngineProxy::LOCAL_PLAN: {
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glLocalPlanColor4v);
				glColor3fv(glLocalPlanColor4v);
				break;
			}
			case EngineProxy::GLOBAL_PLAN: {
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glGlobalPlanColor4v);
				glColor3fv(glGlobalPlanColor4v);
				break;
			}
		}
	}
	StampedPose prevPose;

	// save  current line width
	GLfloat savedLineRange[2];
	glGetFloatv(GL_LINE_WIDTH, savedLineRange);
	glLineWidth(4);

	for (unsigned int i = 0;i<pathLength; i++) {
		StampedPose sp = trajectory[i];
		if (!sp.isNull()) {
			sp.pose.position.z = 10;
			if (i>0) {
				Point relativePiece = sp.pose.position-prevPose.pose.position;
				glPushMatrix();
				glLoadIdentity();

				glTranslatef(prevPose.pose.position.y,  15.0, prevPose.pose.position.x);
				glRotatef(-degrees(atan2(relativePiece.x, relativePiece.y))+90,0,1,0);
				GLUquadricObj *quadratic = gluNewQuadric();
				gluCylinder(quadratic, 10, 5, relativePiece.length(), 6, 1);
				glPopMatrix();
				/*
				glBegin(GL_LINES);
					glVertex3f(sp.pose.position.y, sp.pose.position.z, sp.pose.position.x);
					glVertex3f(prevPose.pose.position.y, prevPose.pose.position.z, prevPose.pose.position.x);
				glEnd();
				*/
			}
		}
		prevPose = sp;
	}

	glLineWidth(savedLineRange[0]);
	glPopMatrix();
}

void SlamView::drawCoordSystem() {
	// draw coord system in red
	glBegin(GL_LINES);
		glColor3fv(glSlamMapCoordSystemColor4v);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glSlamMapCoordSystemColor4v);

		glVertex3f(0,0,0);glVertex3f(0, 500, 0);
		glVertex3f(0,0,0);glVertex3f(0, 0, 2000);
		glVertex3f(0,0,0);glVertex3f(1000, 0, 0);
	glEnd();
}

void SlamView::drawSlamMap() {
	Map& map = EngineProxy::getInstance().getMap();
	int fullMapSizeX = map.getMapSizeX();
	int fullMapSizeY = map.getMapSizeY();
	int gridLength = map.getGridSize();

	// limit the drawn structure to an area that is three times the viewing distance
	int mapSizeX =2*gridLength*(int)(constrain((int)(getEyeDistance()*3.0), 0, fullMapSizeX)/gridLength/2);
	int mapSizeY =2*gridLength*(int)(constrain((int)(getEyeDistance()*3.0), 0, fullMapSizeY)/gridLength/2);

	// draw the SLAM map.
	Point lookAtPosition(getLookAtPosition());

	// take care that the origin is dividable by gridLength in order to
	// have the grids assigned correctly
	lookAtPosition.x = gridLength*2*((int)(lookAtPosition.x/gridLength/2));
	lookAtPosition.y = gridLength*2*((int)(lookAtPosition.y/gridLength/2));

	int minX = constrain(- mapSizeX/2 + (int)lookAtPosition.x, -fullMapSizeX/2, fullMapSizeX/2);
	int maxX = constrain(+ mapSizeX/2 + (int)lookAtPosition.x, -fullMapSizeX/2, fullMapSizeX/2);
	int minY = constrain(- mapSizeY/2 + (int)lookAtPosition.y, -fullMapSizeY/2, fullMapSizeY/2);
	int maxY = constrain(+ mapSizeY/2 + (int)lookAtPosition.y, -fullMapSizeY/2, fullMapSizeY/2);

	for (int localGridX = minX;localGridX < maxX ;localGridX += gridLength) {
		int xFrom = localGridX-0*gridLength;
		int xTo = xFrom + gridLength;
		for (int localGridY = minY;localGridY < maxY;localGridY += gridLength) {
			Map::GridState occupancy = map.getOccupancyByWorld(localGridX,localGridY);

			if (occupancy != Map::GridState::UNKNOWN) {
				int yFrom = localGridY-0*gridLength;
				int yTo = yFrom  + gridLength;
				int offset = 3;

				if (occupancy == Map::GridState::FREE) {
					int z = 1;
					Point g1(xFrom+offset, 	(yFrom+offset), 	z);
					Point g2(xTo-offset, 	(yFrom+offset),		z);
					Point g3(xTo-offset, 	(yTo-offset),		z);
					Point g4(xFrom+offset, 	(yTo-offset),		z);
					drawFreeSlamGrid(g1, g2, g3, g4);
				}
				if (occupancy == Map::GridState::OCCUPIED) {
					int z = 50;
					Point g1(xFrom+offset, 	(yFrom+offset), 	z);
					Point g2(xTo-offset, 	(yFrom+offset),		z);
					Point g3(xTo-offset, 	(yTo-offset),		z);
					Point g4(xFrom+offset, 	(yTo-offset),		z);
					drawOccupiedSlamGrid(false,g1, g2, g3, g4);
				}
			}
		}
	}
}


void SlamView::drawCostMap(CostmapType type, const Pose& odom) {

	Map map = EngineProxy::getInstance().getGlobalCostmap();
	Map slamMap = EngineProxy::getInstance().getMap();

	Point origin = odom.position;

	Map* costmap = NULL;
	if (type == GLOBAL_COSTMAP) {
		costmap = &EngineProxy::getInstance().getGlobalCostmap();
		origin.null();
	}
	else
		costmap = &EngineProxy::getInstance().getLocalCostmap();

	int fullMapSizeX = costmap->getMapSizeX();
	int fullMapSizeY = costmap->getMapSizeY();
	int gridLength = costmap->getGridSize();

	if ((fullMapSizeX == 0) || (fullMapSizeY == 0) || (gridLength == 0))
		return;

	// limit the drawn structure to an area that is three times the viewing distance
	int mapSizeX =2*gridLength*(int)(constrain((int)(getEyeDistance()*3.0), 0, map.getMapSizeX())/gridLength/2);
	int mapSizeY =2*gridLength*(int)(constrain((int)(getEyeDistance()*3.0), 0, map.getMapSizeY())/gridLength/2);

	// draw the SLAM costmap.
	Point lookAtPosition(getLookAtPosition());

	// take care that the origin is dividable by gridLength in order to
	// have the grids assigned correctly
	lookAtPosition.x = gridLength*2*((int)(lookAtPosition.x/gridLength/2));
	lookAtPosition.y = gridLength*2*((int)(lookAtPosition.y/gridLength/2));

	int minX = constrain(- mapSizeX/2 + (int)lookAtPosition.x, -fullMapSizeX/2, fullMapSizeX/2);
	int maxX = constrain(+ mapSizeX/2 + (int)lookAtPosition.x,-fullMapSizeX/2, fullMapSizeX/2);
	int minY = constrain(- mapSizeY/2 + (int)lookAtPosition.y,-fullMapSizeY/2, fullMapSizeY/2);
	int maxY = constrain(+ mapSizeY/2 + (int)lookAtPosition.y,-fullMapSizeY/2, fullMapSizeY/2);

	Point null;
	for (int localGridX = minX;localGridX < maxX ;localGridX += gridLength) {
		int xFrom = localGridX + origin.x-0*gridLength;
		int xTo = xFrom + gridLength;
		for (int localGridY = minY;localGridY < maxY;localGridY += gridLength) {

			if (Point(localGridX, localGridY).length() <= fullMapSizeX/2.0 + gridLength/2.0) {
				int costValue = costmap->getValueByWorld(localGridX,localGridY);
				if ((costValue > 0) &&
				    (slamMap.getValueByWorld(localGridX,localGridY) != Map::UNKNOWN)) {
					int yFrom = localGridY + origin.y -0*gridLength;
					int yTo = yFrom + gridLength;

					int offset = 3;
					int z = 10;
					Point g1(xFrom+offset, 	(yFrom+offset), 	z);
					Point g2(xTo-offset, 	(yFrom+offset),		z);
					Point g3(xTo-offset, 	(yTo-offset),		z);
					Point g4(xFrom+offset, 	(yTo-offset),		z);
					drawCostmapGrid(type, costValue, g1, g2, g3, g4);
				}
			}
		}
	}
}


void SlamView::drawMap() {

	const Pose& fusedPose = EngineProxy::getInstance().getBaseLinkInMapFrame();

	// if the manually set viewpoint is 5s old, and the bot has
	// moved 50mm, switch view back to automated mode such that the bot can be seen
	if ((millis()-manualLookAtAdjustTime > 5000) &&
		(lastFusedPosition.position.distance(fusedPose.position) > 50)) {
		setLookAtPosition(fusedPose.position);
	};

	glPushAttrib(GL_CURRENT_BIT);
	glPushAttrib(GL_LIGHTING_BIT);
	glPushMatrix();
	glLoadIdentity();

	drawCoordRaster();
	drawSmallBot(fusedPose);
	drawNavigationGoal();
	drawSlamMap();
	drawCostMap(GLOBAL_COSTMAP, Pose());
	drawCostMap( LOCAL_COSTMAP, fusedPose);
	drawLaserScan();
	drawTrajectory(EngineProxy::GLOBAL_PLAN);
	drawTrajectory(EngineProxy::LOCAL_PLAN);
	drawDarkScaryHoles();

	drawCoordSystem();

	// this is required for having an object to click for 3D mouse projection
	drawMapBackground();

	glPopAttrib();
	glPopAttrib();
	glPopMatrix();
}


void SlamView::defineNavigationGoal(bool latchOrientation) {
	sentNavigationGoal = tempNavigationGoal;
	EngineProxy::getInstance().setNavigationGoal(sentNavigationGoal, latchOrientation);
}

void SlamView::setNavigationGoal(const Pose& p) {
	tempNavigationGoal = p;
}

void SlamView::MotionCallback(int x, int y) {
	float diffX = (float) (x-lastMouseX);
	float diffY = (float) (y-lastMouseY);
	if (mouseViewPlane) {
		changeEyePosition(0, -diffX, -diffY);
		postRedisplay();
	} else {
		if (mousePlaneXY) {
			Point lookAt = getLookAtPosition();
			Point lookAtDiff = Point(-diffY,-diffX,0 )*getCurrentEyeDistance()/1200; // WindowSize
			lookAtDiff = lookAtDiff.getRotatedAroundZ(getXYPaneAngle());
			lookAt += lookAtDiff;
			Map& map = EngineProxy::getInstance().getMap();
			lookAt.x = constrain(lookAt.x, (realnum)-map.getMapSizeX()/2, (realnum)map.getMapSizeX()/2);
			lookAt.y = constrain(lookAt.y, (realnum)-map.getMapSizeY()/2, (realnum)map.getMapSizeY()/2);

			manualLookAtAdjustTime = millis();
			lastFusedPosition = EngineProxy::getInstance().getBaseLinkInMapFrame();
			setLookAtPosition(lookAt);
			postRedisplay();
		} else {
			if (mouseNavigationGoalDirection){
				// check current projection of mouse position
				Point targetDirection = get3DByMouseClick(x,y);
				// compute direction from goal set to current position
				Point directionPoint = targetDirection - tempNavigationGoal.position;
				tempNavigationGoal.orientation.z = atan2(directionPoint.y, directionPoint.x);
			} else {
				if (lastMouseScroll != 0) {
					WindowController::getInstance().slamView.changeEyePosition(-getCurrentEyeDistance()*2*lastMouseScroll/100, 0,0);
					postRedisplay();
					lastMouseScroll = 0;
				}
			}
		}
	}

	if (mouseViewPlane || mousePlaneXY || mouseNavigationGoalDirection) {
		lastMouseX = x;
		lastMouseY = y;
	}
}


Point SlamView::get3DByMouseClick(int screenX,int screenY) {
	// compute the 3D coordinates of the clicked object
	// requires an object to click on, so take care that a opengl pane is drawn.

	// retrieve view port (X, Y, Width, Height)
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// get modelview matrix
	GLdouble modelview[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

	// get projection matrix
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	// read the z-coordinate at the mouse position
	GLfloat winX, winY, winZ;
	winX = screenX;
	winY = viewport[3] - screenY;
	glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
	GLdouble x,y,z;
	bool success =  gluUnProject( winX, winY, winZ, modelview, projection, viewport, &x, &y, &z);
	if (success == GL_TRUE) {

		// if the object hit is higher than y = 200 (which actually is z = 200)
		// we identified a dark scary hole. In that case we latch the orientation
		// i.e. the orientation of the goal is defined by the last piece of the path
		latchGoalOrientation = (y > 200);
		if (latchGoalOrientation) {
			// find the dark scary hole
			std::vector<Point> holes = EngineProxy::getInstance().getDarkScaryHoles();
			for (unsigned i = 0;i<holes.size();i++) {
				if (holes[i].distance(Point(z, x,0)) <= 50)
					return Point(holes[i].x,holes[i].y,0); // holes[i].x, holes[i].y,0);
			}
		}

		return Point(z, x,0 );

	}
	else
		return Point(0,0,0);
}

void SlamView::MouseCallback(int button, int button_state, int x, int y )
{
	bool withShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
	bool withCtrl= glutGetModifiers() & GLUT_ACTIVE_CTRL;

	// When releasing left-ctrl, navigation goal is really set
	if ( mouseNavigationGoalDirection && button == GLUT_LEFT_BUTTON && button_state == GLUT_UP && !withShift && withCtrl) {
		defineNavigationGoal(latchGoalOrientation);
	}

	mouseViewPlane = false;
	mousePlaneXY = false;
	mouseNavigationGoalDirection = false;

	// left button turns the view around the same lookat point
	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN && !withShift && !withCtrl) {
	    mouseViewPlane = true;
	}

	// right button translates the view moving the look-at point
	if (button == GLUT_RIGHT_BUTTON && (button_state == GLUT_DOWN && !withCtrl && !withShift))
		mousePlaneXY = true;

	// Left+Ctrl-click sets the navigation point and allows to move the direction
	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN && !withShift && withCtrl) {
		Point p = get3DByMouseClick(x,y);
		Pose navigationGoal(p, Rotation());
		setNavigationGoal(navigationGoal);
		mouseNavigationGoalDirection = true;
	}


	// Wheel reports as button 3(scroll up) and button 4(scroll down)
	if ((button == 3) || (button == 4)) // It's a wheel event
	{
		// Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
		if (button_state != GLUT_UP) { // Disregard redundant GLUT_UP events
			if (button == 3)
				lastMouseScroll++;
			else
				lastMouseScroll--;
			MotionCallback(x,y); // scroll wheel does not trigger a glut call of MotionCallback, so we do it manually
		}
	}

	if (mouseViewPlane || mousePlaneXY) {
	    lastMouseX = x;
	    lastMouseY = y;
	}
}
