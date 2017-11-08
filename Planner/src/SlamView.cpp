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

#include "spatial.h"
#include "Util.h"

#include "uiconfig.h"
#include "setup.h"
#include "WindowController.h"
#include "EngineProxy.h"
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

	defaultHipPoseWorld = EngineProxy::getInstance().getHipPoseWorld();
	defaultLegAngles = EngineProxy::getInstance().getLegAngles();
	defaultBodyPose = EngineProxy::getInstance().getBodyPose();

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

void SlamView::drawNavigationGoal() {
	glPushMatrix();
	glLoadIdentity();

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glMapMarkerColor4v);
	glColor3fv(glMapMarkerColor4v);

	if (!navigationGoal.isNull())
	{
		glTranslatef(navigationGoal.y, navigationGoal.z, navigationGoal.x);
		glRotatef(-90, 1,0,0);
		GLUquadricObj *quadratic = gluNewQuadric();
		gluCylinder(quadratic, 20, 10, 1000, 12, 1);
		glRotatef(90, 1,0,0);

		glTranslatef(0, 1000, 0);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glMapSphereMarkerColor4v);
		glColor3fv(glMapSphereMarkerColor4v);
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

void SlamView::drawCostmapGrid( int value, const Point &g1,const Point &g2, const Point &g3, const Point &g4 ) {
	float red = value/100.0;
	GLfloat color[] = { red, 0.5, 0.5, glAlphaTransparent};
	glBegin(GL_TRIANGLE_STRIP);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
		glColor4fv(color);
		glNormal3f(0.0,1.0,0.0);
		glVertex3f(g1.y, g1.z, g1.x);
		glVertex3f(g2.y, g2.z, g2.x);
		glVertex3f(g4.y, g4.z, g4.x);
		glVertex3f(g3.y, g3.z, g3.x);
	glEnd();
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
	glLoadIdentity();             // Reset the model-view matrix

	glTranslatef(pose.position.y, pose.position.z, pose.position.x);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glFootTouchPointColor);
	glColor3fv(glFootTouchPointColor);

	drawFilledCircle(0,0,10, maxFootTouchPointRadius);

	BotDrawer::getInstance().displayBot(
			EngineProxy::getInstance().getNoseOrientation(),
			EngineProxy::getInstance().getBodyPose(),
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
		glVertex3f(-fullMapSizeY/2, 0, -fullMapSizeX/2); glVertex3f(fullMapSizeY/2, 0, -fullMapSizeX/2);
		glVertex3f(fullMapSizeY/2, 0, fullMapSizeX/2);
		glVertex3f(-fullMapSizeY/2, 0, fullMapSizeX/2);
		glVertex3f(-fullMapSizeY/2, 0, -fullMapSizeX/2);
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

void SlamView::drawTrajectory() {
	Trajectory& trajectory = EngineProxy::getInstance().getTrajectory();

	// draw trajectory
	unsigned int pathLength = trajectory.size();
	glPushMatrix();
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glTrajectoryColor4v);
	glColor3fv(glTrajectoryColor4v);
	StampedPose prevPose;

	// save  current line width
	GLfloat savedLineRange[2];
	glGetFloatv(GL_LINE_WIDTH, savedLineRange);
	glLineWidth(3);

	for (unsigned int i = 0;i<pathLength; i++) {
		StampedPose sp = trajectory[i];
		if (!sp.isNull()) {
			sp.pose.position.z = 20;
			// glLoadIdentity();
			// glTranslatef(sp.pose.position.y, sp.pose.position.z, sp.pose.position.x);
			// cout << "path=" << sp.pose.position << endl;
			if (i>0) {
				glBegin(GL_LINES);
					glVertex3f(sp.pose.position.y, sp.pose.position.z, sp.pose.position.x);
					glVertex3f(prevPose.pose.position.y, prevPose.pose.position.z, prevPose.pose.position.x);
				glEnd();
			}
		}
		prevPose = sp;
	}

	const Pose& fusedPose = EngineProxy::getInstance().getFusedPose();

	// last piece to current position
	glBegin(GL_LINES);
		glVertex3f(prevPose.pose.position.y, prevPose.pose.position.z, prevPose.pose.position.x);
		glVertex3f(fusedPose.position.y, fusedPose.position.z, fusedPose.position.x);
	glEnd();

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
	millimeter gridLength = map.getGridSize();
	// limit the drawn structures to an area that is three times the viewing distance
	int mapSizeX = constrain((int)getEyeDistance()*3, 0, fullMapSizeX);
	int mapSizeY = constrain((int)getEyeDistance()*3, 0, fullMapSizeY);

	// draw the SLAM map.
	Point lookAtPosition(getLookAtPosition());

	// take care that the origin is dividable by gridLength in order to
	// have the grids assigned correctly and one line is going right through the origin
	lookAtPosition.x = ((int)(lookAtPosition.x/gridLength))*gridLength;
	lookAtPosition.y = ((int)(lookAtPosition.y/gridLength))*gridLength;

	int minX = constrain(- mapSizeX/2 + (int)lookAtPosition.x, -fullMapSizeX/2, fullMapSizeX/2);
	int maxX = constrain(+ mapSizeX/2 + (int)lookAtPosition.x,-fullMapSizeX/2, fullMapSizeX/2);
	int minY = constrain(- mapSizeY/2 + (int)lookAtPosition.y,-fullMapSizeY/2, fullMapSizeY/2);
	int maxY = constrain(+ mapSizeY/2 + (int)lookAtPosition.y,-fullMapSizeY/2, fullMapSizeY/2);

	for (int localGridX = minX;localGridX < maxX ;localGridX += gridLength) {
		int xFrom = localGridX;
		int xTo = localGridX + gridLength;
		for (int localGridY = minY;localGridY < maxY;localGridY += gridLength) {
			Map::GridState  occupancy = map.getOccupancyByWorld(localGridX,localGridY);

			if (occupancy != Map::GridState::UNKNOWN) {
				int yTo = localGridY + gridLength/2;
				int yFrom = localGridY - gridLength/2;

				if (occupancy == Map::GridState::FREE) {
					int offset = 3;
					int z = 2;
					Point g1(xFrom+offset, 	(yFrom+offset), 	z);
					Point g2(xTo-offset, 	(yFrom+offset),		z);
					Point g3(xTo-offset, 	(yTo-offset),		z);
					Point g4(xFrom+offset, 	(yTo-offset),		z);
					drawFreeSlamGrid(g1, g2, g3, g4);
				}
				if (occupancy == Map::GridState::OCCUPIED) {
					int offset = 3;
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

void SlamView::drawCostMap() {
	Map& map = EngineProxy::getInstance().getCostmap();
	int fullMapSizeX = map.getMapSizeX();
	int fullMapSizeY = map.getMapSizeY();
	millimeter gridLength = map.getGridSize();
	// limit the drawn structures to an area that is three times the viewing distance
	int mapSizeX = constrain((int)getEyeDistance()*3, 0, fullMapSizeX);
	int mapSizeY = constrain((int)getEyeDistance()*3, 0, fullMapSizeY);

	// draw the SLAM map.
	Point lookAtPosition(getLookAtPosition());

	// take care that the origin is dividable by gridLength in order to
	// have the grids assigned correctly and one line is going right through the origin
	lookAtPosition.x = ((int)(lookAtPosition.x/gridLength))*gridLength;
	lookAtPosition.y = ((int)(lookAtPosition.y/gridLength))*gridLength;

	int minX = constrain(- mapSizeX/2 + (int)lookAtPosition.x, -fullMapSizeX/2, fullMapSizeX/2);
	int maxX = constrain(+ mapSizeX/2 + (int)lookAtPosition.x,-fullMapSizeX/2, fullMapSizeX/2);
	int minY = constrain(- mapSizeY/2 + (int)lookAtPosition.y,-fullMapSizeY/2, fullMapSizeY/2);
	int maxY = constrain(+ mapSizeY/2 + (int)lookAtPosition.y,-fullMapSizeY/2, fullMapSizeY/2);

	for (int localGridX = minX;localGridX < maxX ;localGridX += gridLength) {
		int xFrom = localGridX;
		int xTo = localGridX + gridLength;
		for (int localGridY = minY;localGridY < maxY;localGridY += gridLength) {

			int costValue = map.getValueByWorld(localGridX,localGridY);
			if (costValue > 0) {
				int yTo = localGridY + gridLength/2;
				int yFrom = localGridY - gridLength/2;

				int offset = 3;
				int z = 20;
				Point g1(xFrom+offset, 	(yFrom+offset), 	z);
				Point g2(xTo-offset, 	(yFrom+offset),		z);
				Point g3(xTo-offset, 	(yTo-offset),		z);
				Point g4(xFrom+offset, 	(yTo-offset),		z);
				drawCostmapGrid(costValue, g1, g2, g3, g4);
			}
		}
	}
}

void SlamView::drawMap() {

	const Pose& fusedPose = EngineProxy::getInstance().getFusedPose();

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
	// drawMapBackground();
	drawSmallBot(fusedPose);
	drawNavigationGoal();
	drawSlamMap();
	drawCostMap();

	drawLaserScan();
	drawTrajectory();
	drawCoordSystem();

	glPopAttrib();
	glPopAttrib();
	glPopMatrix();
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
			lookAtDiff.rotateAroundZ(getXYPaneAngle());
			lookAt += lookAtDiff;
			Map& map = EngineProxy::getInstance().getMap();
			lookAt.x = constrain(lookAt.x, (realnum)-map.getMapSizeX()/2, (realnum)map.getMapSizeX()/2);
			lookAt.y = constrain(lookAt.y, (realnum)-map.getMapSizeY()/2, (realnum)map.getMapSizeY()/2);

			manualLookAtAdjustTime = millis();
			lastFusedPosition = EngineProxy::getInstance().getFusedPose();
			setLookAtPosition(lookAt);
			postRedisplay();
		} else {
			if (lastMouseScroll != 0) {
				WindowController::getInstance().slamView.changeEyePosition(-getCurrentEyeDistance()*2*lastMouseScroll/100, 0,0);
				postRedisplay();
				lastMouseScroll = 0;
			}
		}
	}

	if (mouseViewPlane || mousePlaneXY) {
		lastMouseX = x;
		lastMouseY = y;
	}
}

void SlamView::setNavigationGoal(const Point& p) {
	navigationGoal = p;
	Pose goal;
	goal.position = p;
	EngineProxy::getInstance().setNavigationGoal(goal);
}


Point SlamView::get3DByMouseClick(int screenX,int screenY) {
	// use opengl to compute the 3D coordinates of the clicked object

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
	if (success == GL_TRUE)
		return Point(z, x,0 );
	else
		return Point(0,0,0);
}

void SlamView::MouseCallback(int button, int button_state, int x, int y )
{
	mouseViewPlane = false;
	mousePlaneXY = false;

	bool withShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
	bool withCtrl= glutGetModifiers() & GLUT_ACTIVE_CTRL;

	// left button turns the view around the same lookat point
	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN && !withShift && !withCtrl) {
	    mouseViewPlane = true;
	}


	// right button translates the view moving the lookat point
	if (button == GLUT_RIGHT_BUTTON && (button_state == GLUT_DOWN && !withCtrl && !withShift))
		mousePlaneXY = true;

	// Left+Ctrl sets the navigation point
	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN && !withShift && withCtrl) {
		Point p = get3DByMouseClick(x,y);
		setNavigationGoal(p);
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
