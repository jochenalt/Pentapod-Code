/*
 * BotViewController.cpp
 *
 * Author: JochenAlt
 */


#include <windows.h>
#include <thread>

#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>
#include <GL/Glui.h>

#include "basics/spatial.h"
#include "basics/util.h"

#include <BotView.h>
#include <WindowController.h>

#include "BotDrawer.h"
#include "uiconfig.h"
#include "Engine.h"
#include "EngineProxy.h"

#include "uiconfig.h"
#include "setup.h"

using namespace std;

extern void mapFunction (Point &p);
extern realnum getLiftHeight();

BotView::BotView() {
	windowHandle = 0; // set by derived view class

	BotDrawer::getInstance().setup();
}


void BotView::drawCoordSystem(bool withRaster) {

	// draw coordinate system
	const float axisLength = 300.0f;
	const float arrowLength = 20.0f;
	const float unitLength = 30.0f;
	const float rasterLineLength = axisLength*2;
	if (withRaster) {
		// compute movement since last call
		realnum noseOrientation, speed, rotateZ, walkingDirection;
		EngineProxy::getInstance().getCurrentMovement(noseOrientation, speed, rotateZ, walkingDirection);

		// the raster field is moved via translation. In order to not run out of the screen
		// every unitlength the displayed raster jumps by unitLength

		// current offset of the rasterfield, moves from 0 to unitlength, and is set back by a jump
		Point currPos = EngineProxy::getInstance().getOdomPose().position;
		static Point posOffset(EngineProxy::getInstance().getOdomPose().position) ;  // offset between displayer raster and world raster

		if (currPos.x - posOffset.x > unitLength) {
			posOffset.x += unitLength;
		}
		if (currPos.x - posOffset.x < -unitLength) {
			posOffset.x -= unitLength;
		}
		if (currPos.y - posOffset.y > unitLength) {
			posOffset.y += unitLength;
		}
		if (currPos.y - posOffset.y < -unitLength) {
			posOffset.y -= unitLength;
		}

		Point localRasterMovement = currPos - posOffset;
		glPushMatrix();
		glLoadIdentity();

		glTranslatef(0, 0, -localRasterMovement.x);
		glTranslatef(-localRasterMovement.y, 0, 0);

		glPushAttrib(GL_LIGHTING_BIT);

		for (float rasterX = -rasterLineLength;rasterX<=rasterLineLength;rasterX = rasterX + unitLength ) {
			for (float rasterY = -rasterLineLength;rasterY<=rasterLineLength;rasterY = rasterY + unitLength ) {
				Point raster(rasterX,rasterY,0);
                Point mapP1(rasterX+posOffset.x,				rasterY+posOffset.y);
                Point mapP2(rasterX+posOffset.x+unitLength,		rasterY+posOffset.y);
                Point mapP3(rasterX+posOffset.x,				rasterY+posOffset.y+unitLength);
                Point mapP4(rasterX+posOffset.x + unitLength,	rasterY+posOffset.y+unitLength);

                mapFunction(mapP1);
                mapFunction(mapP2);
                mapFunction(mapP3);
                mapFunction(mapP4);

                Point p1(rasterX, 				rasterY, 			mapP1.z);
                Point p2(rasterX+unitLength, 	rasterY,			mapP2.z);
                Point p3(rasterX, 				rasterY+unitLength,	mapP3.z);
                Point p4(rasterX+unitLength, 	rasterY+unitLength,	mapP4.z);

				bool highlightX = true;
				bool highlightY = true;

				glBegin(GL_LINES);
					if (highlightX) {
						glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glRasterColor4v);
						glColor3fv(glRasterColor4v);
					} else {
						glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glMapAreaColor4v);
						glColor3fv(glMapAreaColor4v);
					}
					if (rasterX > -rasterLineLength) {
						glVertex3f(p1.y, p1.z, p1.x);
						glVertex3f(p3.y, p3.z, p3.x);
					}

					if (highlightY) {
						glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glRasterColor4v);
						glColor3fv(glRasterColor4v);
					} else {
						glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glMapAreaColor4v);
						glColor3fv(glMapAreaColor4v);
					}
					if (rasterY  > -rasterLineLength) {
						glVertex3f(p1.y, p1.z, p1.x);
						glVertex3f(p2.y, p2.z, p2.x);
					}

				glEnd();
				glBegin(GL_TRIANGLES);
					glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glMapAreaColor4v);
					glColor4fv(glMapAreaColor4v);
		            glNormal3f(0.0,1.0,0.0);
					glVertex3f(p1.y, p1.z-3, p1.x);	glVertex3f(p2.y, p2.z-3, p2.x);
					glVertex3f(p3.y, p3.z-3, p3.x);	glVertex3f(p2.y, p2.z-3, p2.x);
					glVertex3f(p4.y, p4.z-3, p4.x); glVertex3f(p3.y, p3.z-3, p3.x);
				glEnd();
			}
		}

		glPopAttrib();
		glPopMatrix();
	}

	glBegin(GL_LINES);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glCoordSystemAxisColor4v);
		glColor4fv(glCoordSystemAxisColor4v);


		float z = 2.0f;
		// robot's x-axis
		glVertex3f(0.0f, z, -arrowLength);glVertex3f(0.0f, z, axisLength);
		glVertex3f(0.0f, z, axisLength);glVertex3f(0.0f,+arrowLength/2, axisLength-arrowLength);
		glVertex3f(0.0f, z, axisLength);glVertex3f(0.0f,-arrowLength/2, axisLength-arrowLength);
		for (float i = 0;i<axisLength;i = i + unitLength ) {
			glVertex3f(0.0f, -arrowLength/2, i);glVertex3f(0.0f,+arrowLength/2, i);
		}

		// robot's y-axis
		glVertex3f(-arrowLength, z, 0.0f);glVertex3f(axisLength, z, 0.0f);
		glVertex3f(axisLength, z, 0.0f);glVertex3f(axisLength-arrowLength, -arrowLength/2, 0.0f);
		glVertex3f(axisLength, z, 0.0f);glVertex3f(axisLength-arrowLength, arrowLength/2, 0.0f);
		for (float i = 0;i<axisLength;i = i + unitLength ) {
			glVertex3f(i, -arrowLength/2, 0.0f);glVertex3f(i,+arrowLength/2, 0.0f);
		}

		// robot's z-axis
		glVertex3f(0.0f, -arrowLength, 0.0f);glVertex3f(0.0f, axisLength,0.0f);
		glVertex3f(0.0f, axisLength,0.0f);glVertex3f(+arrowLength/2,axisLength-arrowLength, 0.0f);
		glVertex3f(0.0f, axisLength,0.0f);glVertex3f(-arrowLength/2, axisLength-arrowLength,0.0f);
		for (float i = 0;i<axisLength;i = i + unitLength ) {
			glVertex3f(-arrowLength/2, i,0.0f);glVertex3f(+arrowLength/2, i,0.0f);
		}
	glEnd();

	glRasterPos3f(axisLength+arrowLength, 0.0f, 0.0f);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) "y");
	glRasterPos3f(0.0f, 0.0f, axisLength+arrowLength);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) "x");
	glRasterPos3f(0.0f, axisLength+arrowLength,0.0f);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) "z");
}


void BotView::setPose(angle_rad newOrientation, const Pose& newBodyPose, const PentaPointType& newGaitRefPoint, const PentaPoseType& newHipPose, const PentaLegAngleType& newAngles) {
	bodyPose = newBodyPose;
	noseOrientation = newOrientation;
	for (int i = 0;i<NumberOfLegs;i++) {
		hipFromBellyPerspective[i] = newHipPose[i];
		legAngles[i] = newAngles[i];
		gaitRefPoints[i] = newGaitRefPoint[i];
	}
}

void BotView::drawGroundPoints(realnum heightOverGround, const PentaPointType &groundPoints  ) {
	glPushMatrix();

	// draw small balls at the touch point
	for (int i = 0;i<NumberOfLegs;i++) {
		// move GL coordinate system into drawn coordinate system
		glLoadIdentity();
		glRotatef(-90, 1.0,0.0,0.0);
		glRotatef(-90, 0.0,0.0,1.0);

		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glGroundPointColor); // front

		glTranslatef(groundPoints[i].x,groundPoints[i].y,groundPoints[i].z+heightOverGround);
		glutSolidSphere(6, 18, 18);
	}

	glPopMatrix();
}

void BotView::drawGroundDistancePoints(realnum heightOverGround, const PentaPointType &groundDistancePoints  ) {
	glPushMatrix();

	// draw small balls at the touch point
	for (int i = 0;i<NumberOfLegs;i++) {
		// move GL coordinate system into drawn coordinate system
		glLoadIdentity();
		glRotatef(-90, 1.0,0.0,0.0);
		glRotatef(-90, 0.0,0.0,1.0);

		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glGroundDistancePointColor); // front

		glTranslatef(groundDistancePoints[i].x,groundDistancePoints[i].y,groundDistancePoints[i].z+heightOverGround);
		glutSolidSphere(10, 18, 18);
	}

	glPopMatrix();
}


void BotView::drawGaitRefPoints(realnum heightOverGround, const bool footOnGround[NumberOfLegs] ) {
	glPushMatrix();

	// draw small balls at the touch point
	for (int i = 0;i<NumberOfLegs;i++) {
		// move GL coordinate system into drawn coordinate system
		glLoadIdentity();
		glRotatef(-90, 1.0,0.0,0.0);
		glRotatef(-90, 0.0,0.0,1.0);

		if (footOnGround[i])
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glFootTouchPointColor); // front
		else
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glFootTouchPointAirColor); // back

		glTranslatef(gaitRefPoints[i].x,gaitRefPoints[i].y,gaitRefPoints[i].z+heightOverGround);
		glutSolidSphere(10, 18, 18);
	}

	glPopMatrix();
}

void BotView::drawStandingArea(const bool footOnGround[NumberOfLegs]) {
	glPushMatrix();
	glPushAttrib(GL_LIGHTING_BIT);

	// draw the area that is support by legs touching the ground
	glLoadIdentity();
	glRotatef(-90, 1.0,0.0,0.0);
	glRotatef(-90, 0.0,0.0,1.0);

	glColor4fv(glFootTouchPointAreaColor);
	glBegin(GL_POLYGON);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glFootTouchPointAreaColor);
		glColor4fv(glFootTouchPointAreaColor);

		for (int i = 0;i<NumberOfLegs;i++) {
			if (footOnGround[i]) {
				glNormal3f(0.0,1.0,0.0);
				glVertex3f(gaitRefPoints[i].x, gaitRefPoints[i].y, 4.0);
			}
		}
	glEnd();
	glPopAttrib();
	glPopMatrix();
}


void displayBotView() {
	WindowController::getInstance().mainBotView.display();
}

void BotViewMotionCallback(int x, int y) {
	WindowController::getInstance().mainBotView.MotionCallback(x,y);
}
void BotViewMouseCallback(int button, int button_state, int x, int y ) {
	WindowController::getInstance().mainBotView.MouseCallback(button, button_state,  x,  y);
}


int BotView::create(int mainWindow, string pTitle) {
	// initially start with zero size, will be resized in reshape
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
	setEyePosition(ViewEyeDistance, radians(-60), radians(-10));

	singleLegActive = false;

	// Main Bot view has comprehensive mouse motion
	glutSetWindow(windowHandle);
	glutMotionFunc( BotViewMotionCallback);
	glutMouseFunc( BotViewMouseCallback);
	glutDisplayFunc(displayBotView);

	return windowHandle;
}


void BotView::setSingleLegActive(bool ok) {
	singleLegActive = ok;
}

void BotView::display() {
	int savedWindowHandle = glutGetWindow();
	glutSetWindow(windowHandle);

	// fetch current Body data
	setPose(EngineProxy::getInstance().getNoseOrientation(),
			EngineProxy::getInstance().getImuAwareBodyPose(),
			EngineProxy::getInstance().getGaitRefPoints(),
			EngineProxy::getInstance().getHipPoseWorld(),
			EngineProxy::getInstance().getLegAngles());

	glutSetWindow(windowHandle);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();             // Reset the model-view matrix
	displayEyePosition();

	glMatrixMode(GL_MODELVIEW);
	if (singleLegActive)
		BotDrawer::getInstance().displayLeg(singleLegAngles, glFootTouchPointFrontColor, glFootTouchPointBackColor);
	else {
		// draw full hexapod
		BotDrawer::getInstance().displayBot(noseOrientation, bodyPose, hipFromBellyPerspective, legAngles);

		// ground points
		drawGroundPoints(0,EngineProxy::getInstance().getGroundPoints());

		// foot touch points
		drawGaitRefPoints(0,EngineProxy::getInstance().getFootOnGround());
	}

	// coord system
	drawCoordSystem(true);

	// draw polygon where the bot standing on
	drawStandingArea(EngineProxy::getInstance().getFootOnGround());

	glFlush();  		// Render now
	glutSwapBuffers(); 	// use double buffering

	glutSetWindow(savedWindowHandle);
}

void BotView::reshape(int x,int y, int w, int h) {
	glutSetWindow(windowHandle);
	glutShowWindow();
	glutPositionWindow(x, y);
	glutReshapeWindow(w, h);
	glViewport(0, 0, w, h);

	display();
}

void BotView::setSingleLegPose(const LegPose& pSingleLegpose) {
	singleLegAngles = pSingleLegpose.angles;
	singleLegPose = pSingleLegpose;
}

void BotView::hide() {
	glutSetWindow(windowHandle);
	glutHideWindow();
}

void BotView::show() {
	glutSetWindow(windowHandle);
	glutShowWindow();
}

void BotView::MotionCallback(int x, int y) {
	// if display has not yet been called after the last motion, dont execute
	// this one, but wait for

	float diffX = (float) (x-lastMouseX);
	float diffY = (float) (y-lastMouseY);
	switch (mousePane) {
	case BOT_BODY_PANE: {
		Pose pose = EngineProxy::getInstance().getImuAwareBodyPose();
		pose.position.x += diffX;
		pose.position.z -= diffY;
		EngineProxy::getInstance().setTargetBodyPose(pose);
		cout << "pose=" << pose << endl;
		postRedisplay();
		break;
	}
	case BOT_BODYORIENTATION_PANE: {
		Pose pose = EngineProxy::getInstance().getImuAwareBodyPose();
		pose.orientation.z += diffX/60;
		pose.orientation.y -= diffY/60;
		EngineProxy::getInstance().setTargetBodyPose(pose);
		cout << "pose=" << pose << endl;

		postRedisplay();
		break;
	}

	case BOT_FRONTLEG_HORIZ_PANE: {
		LegPose pose = EngineProxy::getInstance().getFrontLegPoseWorld();
		pose.position.y += diffX;
		pose.position.z -= diffY;
		EngineProxy::getInstance().setTargetFrontLegPoseWorld(pose.position);
		cout << "front=" << pose << endl;

		postRedisplay();
		break;
	}
	case BOT_FRONTLEG_VERTICAL_PANE: {
		LegPose pose = EngineProxy::getInstance().getFrontLegPoseWorld();
		pose.position.y += diffX;
		pose.position.z -= diffY;
		EngineProxy::getInstance().setTargetFrontLegPoseWorld(pose.position);
		cout << "front=" << pose << endl;

		postRedisplay();
		break;
	}

	case VIEW_PANE:
		WindowController::getInstance().mainBotView.changeEyePosition(0, -diffX, -diffY);
		postRedisplay();
		break;
	default:
		if (lastMouseScroll != 0) {
			WindowController::getInstance().mainBotView.changeEyePosition(-getCurrentEyeDistance()*2*lastMouseScroll/100, 0,0);
			postRedisplay();
			lastMouseScroll = 0;
		}
	}

	if (mousePane != NO_PANE) {
		lastMouseX = x;
		lastMouseY = y;
	}
}


void BotView::MouseCallback(int button, int button_state, int x, int y )
{
	mousePane = NO_PANE;

	bool withShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
	bool withCtrl= glutGetModifiers() & GLUT_ACTIVE_CTRL;
	bool withAlt = glutGetModifiers() & GLUT_ACTIVE_ALT;

	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN && !withShift && !withCtrl && !withAlt) {
		mousePane = VIEW_PANE;
	} else {
		if ( button == GLUT_RIGHT_BUTTON && button_state == GLUT_DOWN && !withShift && !withCtrl &&  !withAlt) {
			mousePane = BOT_BODY_PANE;
		}
		if ( button == GLUT_RIGHT_BUTTON && button_state == GLUT_DOWN && !withShift && !withCtrl &&  withAlt) {
			mousePane = BOT_BODYORIENTATION_PANE;
		}
		if ( button == GLUT_RIGHT_BUTTON && button_state == GLUT_DOWN && withShift && withCtrl &&  !withAlt) {
			mousePane = BOT_FRONTLEG_HORIZ_PANE;
		}
		if ( button == GLUT_RIGHT_BUTTON && button_state == GLUT_DOWN && withShift && !withCtrl &&  !withAlt) {
			mousePane = BOT_FRONTLEG_VERTICAL_PANE;
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
				BotViewMotionCallback(x,y); // scroll wheel does not trigger a glut call of MotionCallback, so we do it manually
			}
		}
	}

	if (mousePane != NO_PANE) {
	    lastMouseX = x;
	    lastMouseY = y;
	}
}

