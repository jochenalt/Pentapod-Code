
#include <stdio.h>

#include "basics/logger.h"

#include "WindowController.h"
#include "Util.h"

#include "LegKinematics.h"
#include "BotView.h"
#include "uiconfig.h"
#include "core.h"
#include "setup.h"
#include "EngineProxy.h"

using namespace std;

// Initial Window size
int WindowWidth = 1200;
int WindowHeight = 750;

// GLUI Window handlers
int wMain, wMainBotView, wSLAMView;

// kinematics widgets
GLUI_Spinner* poseSpinner[3] = {NULL,NULL,NULL};
int poseSpinnerLiveVar[3] = {0,0,0 };

// target body position, orientation and speed
Pose inputBodyPose;
realnum inputWalkingDirection = 0;
realnum inputSpeed = 0;
realnum inputRotate = 0;
realnum inputNoseOrientation = 0;

// body pose widget
GLUI_Spinner* bodyPosePositionSpinner[3] = { NULL, NULL, NULL};
int bodyPosePositionSpinnerLiveVar[3] = {0,0,0};
const int bodyPosePositionWidgetNo = 0;
GLUI_Spinner* bodyPoseOrientationSpinner[3] = { NULL,NULL, NULL};
int bodyPoseOrientationSpinnerLiveVar[3] = {0,0,0};
const int bodyPoseOrientationWidgetNo = 3;
const int bodyPoseResetWidgetNo = 7;

// choice of single leg or full pentapod
GLUI_Checkbox* singleLegActiveCheckBox = NULL;
int singleLegActiveLiveVar;

// gait controls
GLUI_Spinner* gaitWakingDirectionSpinner = NULL;
GLUI_Spinner* gaitSpeedSpinner = NULL;
GLUI_Spinner* gaitRotateSpinner = NULL;
GLUI_Spinner* gaitNoseOrientationSpinner = NULL;

int gaitDirectionLiveVar = 0;
int gaitSpeedLiveVar = 0;
int gaitRotateLiveVar = 0;
int gaitNoseOrientationLiveVar = 0;

const int WalkingDirectionID = 0;
const int MovementSpeedID = 1;
const int MovementRotationID = 2;
const int MovementOrientationID = 3;
const int MovementResetID = 4;
GLUI_RadioGroup* gaitRadioGroup = NULL;
int gaitLiveVar = 0;

GLUI_Checkbox* mapControl = NULL;
int mapLiveVar;

GLUI_Checkbox* powerCheckbox = NULL;
int powerLiveVar;
const int PowerCheckBoxID = 0;
GLUI_Checkbox* wakeUpCheckbox = NULL;
int wakeUpLiveVar;
const int WakeUpCheckBoxID = 1;

GLUI_Checkbox* terrainModeCheckbox = NULL;
int terrainLiveVar;
const int TerrainModeCheckBoxID = 2;


// each mouse motion call requires a display() call before doing the next mouse motion call. postDisplayInitiated
// is a semaphore that coordinates this across several threads
// (without that, we have so many motion calls that rendering is bumpy)
// postDisplayInitiated is true, if a display()-invokation is pending but has not yet been executed (i.e. allow a following display call)
volatile static bool postDisplayInitiated = true;

// to show a single leg, the UI gets an own local engine. communication to the remote engine does not happen here.
Engine singleLegEngine;

extern void copyMovementToView();
extern void mapFunction (Point &p);


void WindowController::postRedisplay() {
	int saveWindow = glutGetWindow();
	glutSetWindow(wMain);
	postDisplayInitiated = true;
	glutPostRedisplay();
	glutSetWindow(saveWindow );
}

void copySingleLegPoseToView() {
	const LegPose& tcp = EngineProxy::getInstance().getFrontLegPoseWorld();
	for (int i = 0;i<3;i++) {
		realnum value;
		value = tcp.position[i];
		value = roundValue(value);
		if (value != poseSpinnerLiveVar[i]) {
			poseSpinner[i]->set_int_val(value); // set only when necessary, otherwise the cursor blinks
		}
	}
}

LegPose getSingleLegPoseFromView() {
	LegPose pose;
	for (int i = 0;i<3;i++) {
		pose.position[i] = poseSpinnerLiveVar[i];
	}

	return pose;
}


/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void displayMainView() {
	if (!postDisplayInitiated)
		return;

	postDisplayInitiated = false;
	// glutSetWindow(wMain);
	glClear(GL_COLOR_BUFFER_BIT);

	// copy bot data to view
	copySingleLegPoseToView();

	// single leg view, compute and show it
	if (singleLegActiveLiveVar) {
		LegPose singleLegPose = getSingleLegPoseFromView();
		singleLegEngine.computeSingleLegAdHoc(singleLegPose);
		WindowController::getInstance().mainBotView.setSingleLegPose(singleLegPose);
	}
}

void nocallback(int value) {
}

void reshape(int w, int h) {
	int savedWindow = glutGetWindow();
	WindowWidth = w;
	WindowHeight = h;
	glViewport(0, 0, w, h);

	int MainSubWindowWidth = (w - 3 * WindowGap)/2;
	int MainSubWindowHeight = (h - InteractiveWindowHeight - 3 * WindowGap);

	WindowController::getInstance().mainBotView.reshape(WindowGap, WindowGap,MainSubWindowWidth, MainSubWindowHeight);
	WindowController::getInstance().slamView.reshape(WindowGap + MainSubWindowWidth + WindowGap, WindowGap,MainSubWindowWidth, MainSubWindowHeight);

	glutSetWindow(savedWindow);
}

void GluiReshapeCallback( int x, int y )
{
	reshape(x,y);
	int tx, ty, tw, th;
	int saveWindow = glutGetWindow();
	glutSetWindow(wMain);
	GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
	glViewport( tx, ty, tw, th );
	glutSetWindow(saveWindow);
	WindowController::getInstance().postRedisplay();
}


void resetBodyPosition() {
	inputBodyPose.null();
	inputBodyPose.position.z = 140;
}

void copyBodyPositionToView() {
	Pose currentBodyPose = EngineProxy::getInstance().getBodyPose();
	for (CoordDimType coordIdx = X; coordIdx <= Z; coordIdx = CoordDimType(CoordDimType((int)coordIdx) + 1)) {
		if (bodyPosePositionSpinnerLiveVar[coordIdx] != (int)currentBodyPose.position[coordIdx])
			bodyPosePositionSpinner[coordIdx]->set_int_val(currentBodyPose.position[coordIdx]);
	}
}

void copyBodyPositionFromView() {
	inputBodyPose.position = Point( bodyPosePositionSpinnerLiveVar[X],
									bodyPosePositionSpinnerLiveVar[Y],
									bodyPosePositionSpinnerLiveVar[Z]);
	inputBodyPose.orientation.x = radians(bodyPoseOrientationSpinnerLiveVar[X]);
	inputBodyPose.orientation.y = radians(bodyPoseOrientationSpinnerLiveVar[Y]);
	inputBodyPose.orientation.z = radians(bodyPoseOrientationSpinnerLiveVar[Z]);
}


void imposeGroundDistance() {

	if (mapLiveVar != 0) {

		// compute the distance which would have been measured by the sensors
		realnum distances[NumberOfLegs];
		for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
			Point footPointWorld = EngineProxy::getInstance().getToePointsWorld(legNo);
			Point mapAbsWorld = footPointWorld + EngineProxy::getInstance().getOdomPose().position;
			mapFunction(mapAbsWorld);
			realnum perpendicularDistance = footPointWorld.z - mapAbsWorld.z;

			// transform perpendicular height to height from foots perspective
			distances[legNo] = perpendicularDistance / cos(EngineProxy::getInstance().getFootAngle(legNo));

			// simulate warping of 30mm, when on the ground, the kinematic needs to lift the leg
			// for at least 30mm until the leg really goes up.
			if ((EngineProxy::getInstance().getLegsGaitPhase(legNo) == LegOnGround) ||
				(EngineProxy::getInstance().getLegsGaitPhase(legNo) == LegMovesUp))
				{
				if (distances[legNo] < 20)
					distances[legNo] = 0;
				else
					distances[legNo] -= 20;
			} else {
				if (distances[legNo] > 20)
					distances[legNo] -= 20;
				else
					distances[legNo] = 0;
			}

		}
		EngineProxy::getInstance().imposeDistanceSensors(distances);
	}
}


// Idle callback is called by GLUI when nothing is to do.
void idleCallback( void )
{
	const milliseconds gaitLoopTime = CORTEX_SAMPLE_RATE;
	const milliseconds emergencyRefreshRate = 1000; 		// refresh everything once a second at least due to refresh issues

	milliseconds now = millis();

	bool newBotData = EngineProxy::getInstance().isBotDataAvailable();
	bool newMapData = EngineProxy::getInstance().isMapDataAvailable();
	bool newMapPoseData = EngineProxy::getInstance().isEstimatedPoseAvailable();

	static milliseconds lastDisplayRefreshCall = millis();
	// update all screens once a second in case of refresh issues (happens)
	if (newBotData || (now - lastDisplayRefreshCall > emergencyRefreshRate)) {
		copyMovementToView();
		copyBodyPositionToView();
		copySingleLegPoseToView();
		WindowController::getInstance().mainBotView.postRedisplay();
	}
	if (newMapData || newMapPoseData || (now - lastDisplayRefreshCall > emergencyRefreshRate))
		WindowController::getInstance().slamView.postRedisplay();

	if ((now - lastDisplayRefreshCall > emergencyRefreshRate) || newMapData || newBotData)
		lastDisplayRefreshCall = now;


	static milliseconds lastGaitCall = millis();
	if (now - lastGaitCall > gaitLoopTime) {
		imposeGroundDistance();
		lastGaitCall = now;
	}
}

void singleLegActiveCallback(int buttonNo) {
	WindowController::getInstance().mainBotView.setSingleLegActive(singleLegActiveLiveVar);
}

void singleLegResetCallback (int buttonNo) {

	Point nullPoint = Point(CAD::HipLength + CAD::HipJointLength + CAD::ThighLength + CAD::ThighKneeGapLength + CAD::KneeJointLength + CAD::FootLength + CAD::DampenerLength,0,0);
	poseSpinner[X]->set_int_val(nullPoint[X]);
	poseSpinner[Y]->set_int_val(nullPoint[Y]);
	poseSpinner[Z]->set_int_val(nullPoint[Z]);

	// tell controller to re-compute the pose and display new pose
	EngineProxy::getInstance().setTargetFrontLegPoseWorld(getSingleLegPoseFromView().position);
}


void singleLegPoseCallback( int tcpCoordId ) {
	EngineProxy::getInstance().setTargetFrontLegPoseWorld(getSingleLegPoseFromView().position);
}


// callback for pose and orientation change, and for reset
void bodyPoseCallback( int coordId)
{
	if (coordId == bodyPoseResetWidgetNo) {
		resetBodyPosition();
		copyBodyPositionToView();
	}

	// copy widget values into global pose data
	copyBodyPositionFromView();

	// send to bot
	EngineProxy::getInstance().setTargetBodyPose(inputBodyPose);
}

// called when a control (noseDirection, WalkingDirection, Speed, Rotation) is called
void gaitSpeedCallback (int controlId) {

	switch (controlId) {
	case WalkingDirectionID: {
		inputWalkingDirection =  radians(gaitDirectionLiveVar);
		EngineProxy::getInstance().setTargetMovement(inputSpeed, inputRotate,  inputWalkingDirection);
		break;
	}
	case MovementSpeedID:
		inputSpeed = gaitSpeedLiveVar;
		EngineProxy::getInstance().setTargetMovement(inputSpeed, inputRotate,  inputWalkingDirection);
		break;
	case MovementRotationID:
		inputRotate= radians(gaitRotateLiveVar);
		EngineProxy::getInstance().setTargetMovement(inputSpeed, inputRotate,  inputWalkingDirection);
		break;
	case MovementOrientationID:
		inputNoseOrientation= radians(gaitNoseOrientationLiveVar);
		break;
	case MovementResetID:
		inputSpeed = 0;
		inputRotate = 0;
		gaitSpeedSpinner->set_int_val(0);
		gaitRotateSpinner->set_int_val(0);
		gaitNoseOrientationSpinner->set_int_val(0);
		EngineProxy::getInstance().setTargetMovement(inputSpeed, inputRotate,  inputWalkingDirection);
		break;
	default:
		break;
	}

}

void mapCallback(int controlId) {
}

// fetch copy bot state, and copy it to the spinners in the view
void copyMovementToView() {

	realnum noseOrientation, speed, rotateZ, walkingDirection;
	EngineProxy::getInstance().getCurrentMovement(noseOrientation, speed, rotateZ, walkingDirection);
	cout << "view dir " << degrees(walkingDirection) << endl;

	if ((int)degrees(walkingDirection) != gaitDirectionLiveVar) {
		gaitWakingDirectionSpinner->set_int_val(degrees(walkingDirection));
	}

	if ((int)speed != gaitSpeedLiveVar) {
		gaitSpeedSpinner->set_int_val(speed);
	}

	if ((int)(degrees(rotateZ) !=  gaitRotateLiveVar)) {
		gaitRotateSpinner->set_int_val((int)degrees(rotateZ));
	}

	if ((int)(degrees(noseOrientation)) != gaitNoseOrientationLiveVar) {
		gaitNoseOrientationSpinner->set_int_val(degrees(noseOrientation));
	}
}


// called by control that defines the GaitModeType. Pushes the new gate mode to bot
void gaitModeControlCallback(int widgetNo) {
	EngineProxy::getInstance().setGaitMode(GaitModeType(gaitLiveVar));
}

// called by control that switched the general mode of the bot (on, off, terrain, walking)
void powerControlCallback(int controlNo) {
	switch (controlNo) {
		case PowerCheckBoxID:
			if (powerLiveVar == 1) {
				EngineProxy::getInstance().turnOn();
				copyBodyPositionToView();
			} else {
				EngineProxy::getInstance().turnOff();
			}
			break;
		case WakeUpCheckBoxID:
			if (powerLiveVar == 0) {
				powerCheckbox->set_int_val(1);
				powerControlCallback(PowerCheckBoxID);
			}

			if ((wakeUpLiveVar == 1) && (powerLiveVar == 1)) {
				inputBodyPose.position = Point(0,0,standardBodyHeigh);
				inputBodyPose.orientation = Rotation(0,0,0);
				EngineProxy::getInstance().wakeUp();
				EngineProxy::getInstance().setTargetBodyPose(inputBodyPose);

				wakeUpCheckbox->set_name("fall asleep");
			} else {
				EngineProxy::getInstance().fallAsleep();
				wakeUpCheckbox->set_name("wake up");
			}
			break;
		case TerrainModeCheckBoxID:
			if (powerLiveVar == 0) {
				powerCheckbox->set_int_val(1);
				powerControlCallback(PowerCheckBoxID);
			}

			if ( powerLiveVar == 1) {
				inputBodyPose.position = Point(0,0,170);
				inputBodyPose.orientation = Rotation(0,0,0);
				copyBodyPositionToView();
				EngineProxy::getInstance().terrainMode(true);
				EngineProxy::getInstance().setTargetBodyPose(inputBodyPose);

				terrainModeCheckbox->set_int_val(0);
				wakeUpCheckbox->set_int_val(1);
			}
			break;
		default:
			break;
	}
}


GLUI* WindowController::createInteractiveWindow(int mainWindow) {

	string emptyLine = "                                               ";

	GLUI *windowHandle= GLUI_Master.create_glui_subwindow( wMain,  GLUI_SUBWINDOW_BOTTOM);
	windowHandle->set_main_gfx_window( wMain );

	GLUI_Panel* interactivePanel = new GLUI_Panel(windowHandle,"interactive panel", GLUI_PANEL_NONE);
	GLUI_Panel* kinematicsPanel = new GLUI_Panel(interactivePanel,"kinematics panel", GLUI_PANEL_NONE);

	// int i = actuatorConfigType[0].minAngle;
	GLUI_StaticText* text = new GLUI_StaticText(kinematicsPanel,"Single Leg Inverse Kinematics");
	text->set_alignment(GLUI_ALIGN_CENTER);

	GLUI_Panel* TCPPanel= new GLUI_Panel(kinematicsPanel,"IK Panel", GLUI_PANEL_RAISED);
	poseSpinner[X]= new GLUI_Spinner(TCPPanel,"x", GLUI_SPINNER_INT,&poseSpinnerLiveVar[X],X, singleLegPoseCallback);
	poseSpinner[Y]= new GLUI_Spinner(TCPPanel,"y", GLUI_SPINNER_INT,&poseSpinnerLiveVar[Y],Y, singleLegPoseCallback);
	poseSpinner[Z]= new GLUI_Spinner(TCPPanel,"z", GLUI_SPINNER_INT,&poseSpinnerLiveVar[Z],Z, singleLegPoseCallback);
	poseSpinner[X]->set_int_limits(-300,400);
	poseSpinner[Y]->set_int_limits(-300,300);
	poseSpinner[Z]->set_int_limits(-300,300);


	GLUI_Panel* SingleLegButtonPanel= new GLUI_Panel(TCPPanel,"single Leg Button Panel", GLUI_PANEL_NONE);

	/* GLUI_Button* resetSingleLegButton = */ new GLUI_Button(SingleLegButtonPanel, "Null Foot", 0, singleLegResetCallback);
	windowHandle->add_column_to_panel(SingleLegButtonPanel, false);
	singleLegActiveCheckBox = new GLUI_Checkbox(SingleLegButtonPanel, "Single Leg Active", &singleLegActiveLiveVar, 0,singleLegActiveCallback);
	singleLegActiveCheckBox->set_int_val(0);

	windowHandle->add_column_to_panel(interactivePanel, false);


	GLUI_Panel* bodyPositionPanel = new GLUI_Panel(interactivePanel,"Body Position Panel", GLUI_PANEL_NONE);
	text = new GLUI_StaticText(bodyPositionPanel,"Body Position and Orientation");
	text->set_alignment(GLUI_ALIGN_CENTER);

	GLUI_Panel* bodyPositionInputPanel = new GLUI_Panel(bodyPositionPanel,"Body Position Input Panel", GLUI_PANEL_RAISED);

	bodyPosePositionSpinner[X]= new GLUI_Spinner(bodyPositionInputPanel,"x", GLUI_SPINNER_INT,&bodyPosePositionSpinnerLiveVar[X],0, bodyPoseCallback);
	bodyPosePositionSpinner[Y]= new GLUI_Spinner(bodyPositionInputPanel,"y", GLUI_SPINNER_INT,&bodyPosePositionSpinnerLiveVar[Y],1, bodyPoseCallback);
	bodyPosePositionSpinner[Z]= new GLUI_Spinner(bodyPositionInputPanel,"z", GLUI_SPINNER_INT,&bodyPosePositionSpinnerLiveVar[Z],2, bodyPoseCallback);
	bodyPosePositionSpinner[X]->set_int_limits(-100,100);
	bodyPosePositionSpinner[Y]->set_int_limits(-100,100);
	bodyPosePositionSpinner[Z]->set_int_limits(minBodyHeight,maxBodyHeight);

	windowHandle->add_column_to_panel(bodyPositionInputPanel, false);
	bodyPoseOrientationSpinner[X]= new GLUI_Spinner(bodyPositionInputPanel,"roll", GLUI_SPINNER_INT,&bodyPoseOrientationSpinnerLiveVar[X],4, bodyPoseCallback);
	bodyPoseOrientationSpinner[Y]= new GLUI_Spinner(bodyPositionInputPanel,"nick", GLUI_SPINNER_INT,&bodyPoseOrientationSpinnerLiveVar[Y],5, bodyPoseCallback);
	bodyPoseOrientationSpinner[Z]= new GLUI_Spinner(bodyPositionInputPanel,"yaw", GLUI_SPINNER_INT,&bodyPoseOrientationSpinnerLiveVar[Z],6, bodyPoseCallback);
	bodyPoseOrientationSpinner[X]->set_int_limits(-45,45);
	bodyPoseOrientationSpinner[Y]->set_int_limits(-45,45);
	bodyPoseOrientationSpinner[Z]->set_int_limits(-45,45);

	GLUI_Button* resetBodyPoseButton = new GLUI_Button(bodyPositionPanel, "Null Body Pose", bodyPoseResetWidgetNo, bodyPoseCallback);
	resetBodyPoseButton->set_w(320);

	windowHandle->add_column_to_panel(interactivePanel, false);

	GLUI_Panel* gaitPanel = new GLUI_Panel(interactivePanel,"Gait Panel", GLUI_PANEL_NONE);
	text = new GLUI_StaticText(gaitPanel,"Gait Panel");
	text->set_alignment(GLUI_ALIGN_CENTER);
	GLUI_Panel* gaitInputPanel = new GLUI_Panel(gaitPanel,"Gait Panel", GLUI_PANEL_RAISED);

	gaitRadioGroup =  new GLUI_RadioGroup(gaitInputPanel, &gaitLiveVar, 0, gaitModeControlCallback);
	new GLUI_RadioButton(gaitRadioGroup, "one foot up");
	new GLUI_RadioButton(gaitRadioGroup, "two feet up");
	new GLUI_RadioButton(gaitRadioGroup, "sexy walk");
	new GLUI_RadioButton(gaitRadioGroup, "auto");
	new GLUI_RadioButton(gaitRadioGroup, "four leg walk");

	gaitRadioGroup->set_int_val(EngineProxy::getInstance().getGaitMode());
	windowHandle->add_column_to_panel(gaitInputPanel, false);

	windowHandle->add_column_to_panel(interactivePanel, false);

	text = new GLUI_StaticText(interactivePanel,"Movement Panel");
	text->set_alignment(GLUI_ALIGN_CENTER);

	GLUI_Panel* movementPanel = new GLUI_Panel(interactivePanel,"Movement Panel", GLUI_PANEL_RAISED);

	gaitWakingDirectionSpinner= new GLUI_Spinner(movementPanel,"dir", GLUI_SPINNER_INT,&gaitDirectionLiveVar,WalkingDirectionID, gaitSpeedCallback);
	gaitWakingDirectionSpinner->set_int_limits(-180,180);

	gaitSpeedSpinner= new GLUI_Spinner(movementPanel,"v", GLUI_SPINNER_INT,&gaitSpeedLiveVar,MovementSpeedID, gaitSpeedCallback);
	gaitSpeedSpinner->set_int_limits(-200,200);

	gaitRotateSpinner= new GLUI_Spinner(movementPanel,"yaw", GLUI_SPINNER_INT,&gaitRotateLiveVar,MovementRotationID, gaitSpeedCallback);
	gaitRotateSpinner->set_int_limits(-90,90);

	gaitNoseOrientationSpinner= new GLUI_Spinner(movementPanel,"ori", GLUI_SPINNER_INT,&gaitNoseOrientationLiveVar,MovementOrientationID, gaitSpeedCallback);
	gaitNoseOrientationSpinner->set_int_limits(-180,180);

	mapControl = new GLUI_Checkbox(movementPanel,"map", &mapLiveVar, 4, mapCallback);
	mapControl->set_int_val(0);

	/* GLUI_Button* resetGaitButton = */ new GLUI_Button(movementPanel, "Null Gait", MovementResetID, gaitSpeedCallback);

	windowHandle->add_column_to_panel(interactivePanel, false);

	GLUI_Panel* powerPanel = new GLUI_Panel(interactivePanel,"Gait Panel", GLUI_PANEL_NONE);

	text = new GLUI_StaticText(powerPanel,"Power Panel");
	text->set_alignment(GLUI_ALIGN_CENTER);
	GLUI_Panel* powerInputPanel = new GLUI_Panel(powerPanel,"Power Panel", GLUI_PANEL_RAISED);

	powerCheckbox = new GLUI_Checkbox(powerInputPanel, "Power", &powerLiveVar, PowerCheckBoxID,powerControlCallback);
	powerCheckbox->set_int_val(0);

	wakeUpCheckbox = new GLUI_Checkbox(powerInputPanel, "Wakeup", &wakeUpLiveVar, WakeUpCheckBoxID,powerControlCallback);
	wakeUpCheckbox->set_int_val(0);

	terrainModeCheckbox = new GLUI_Checkbox(powerInputPanel, "Terrain", &terrainLiveVar, TerrainModeCheckBoxID,powerControlCallback);
	terrainModeCheckbox->set_int_val(0);

	return windowHandle;
}


bool WindowController::setup(int argc, char** argv) {
	glutInit(&argc, argv);

	// start the initialization in a thread so that this function returns
	// (the thread runs the endless GLUT main loop)
	// So, the main thread can do something different while the UI is running
	eventLoopThread = new std::thread(&WindowController::UIeventLoop, this);

	// wait until UI is ready
	unsigned long startTime  = millis();
	do { delay_ms(10); }
	while ((millis() - startTime < 20000) && (!uiReady));

	return uiReady;
}

void mapFunction (Point &p) {
	realnum s1 = 0;
	realnum c1 = 0;

	if (mapLiveVar != 0) {
		float xx = float(p.x)/800.0*2.0*M_PI;
		float yy = float(p.y)/800.0*2.0*M_PI;

		s1 = (2.0*sin(xx*0.2) + 1.5*sin(1.3*xx) + sin(2.7*xx))/(2.0+1.5+1.0);
		c1 = (1.4*sin(0.7*yy) + 1.7*sin(1.2*yy) + sin(2.1*yy))/(1.4+1.7+1.0);
	}
	p.z = 100.0*s1*c1;
}



void WindowController::UIeventLoop() {
	LOG(DEBUG) << "BotWindowCtrl::UIeventLoop";

	glutInitWindowSize(WindowWidth, WindowHeight);
    wMain = glutCreateWindow("Manfred"); // Create a window with the given title
	glutInitWindowPosition(20, 20); // Position the window's initial top-left corner
	glutDisplayFunc(displayMainView);
	glutReshapeFunc(reshape);

	GLUI_Master.set_glutReshapeFunc( GluiReshapeCallback );
	GLUI_Master.set_glutIdleFunc( idleCallback);

	wMainBotView= mainBotView.create(wMain,"");

	// Main Bot view has comprehensive mouse motion
	glutSetWindow(wMainBotView);

	// double buffering
	glutInitDisplayMode(GLUT_DOUBLE);

	// single leg view gets a local engine
	singleLegEngine.setupSimulation();

	// initialize all widgets
	createInteractiveWindow(wMain);

	// fetch current pose from Engine
	inputBodyPose = EngineProxy::getInstance().getBodyPose();
	EngineProxy::getInstance().getCurrentMovement(inputNoseOrientation, inputSpeed, inputRotate, inputWalkingDirection);

	// copy initial pose into view
	copySingleLegPoseToView();
	copyBodyPositionToView();

	// switch off single leg mode
	WindowController::getInstance().mainBotView.setSingleLegActive(false);

	// create the SLAM window
	wSLAMView = slamView.create(wMain,"");

	uiReady = true; 							// tell calling thread to stop waiting for ui initialization
	LOG(DEBUG) << "starting GLUT main loop";
	glutMainLoop();
}

