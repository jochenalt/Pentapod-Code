/*
 * uiconfig.h
 *
 * Configuration of UI with colors, dimensions and stuff
 *
 * Author: JochenAlt
 */

#ifndef UI_UICONFIG_H_
#define UI_UICONFIG_H_

// constants used in the UI
const float ViewEyeDistance 		= 1000.0f;	// distance of the eye to the bot
const float ViewBotHeight 			= 200.0f;	// height of the bot to be viewed

const float ViewMapEyeDistance 		= 5000.0f;	// distance of the eye to the map

const int WindowGap=10;							// gap between frame and subwindow
const int InteractiveWindowHeight=182;			// height of the interactive window

// include RAL colors
#include "ral.h"

// transarency values
const float glAlphaDefault = 1.0;
const float glAlphaSolid= 0.0;
const float glAlphaTransparent = 0.5;
static const GLfloat glCoordSystemAreaColor4v[]		= { 0.2f, 0.2f, 0.2f, glAlphaTransparent};
static const GLfloat glCoordSystemAxisColor4v[] 	= { 0.1, 0.1, 0.1, glAlphaDefault };
static const GLfloat glRasterColor3Xv[] 			= { 225./255., 215./255., 180./255., glAlphaDefault };
static const GLfloat glRasterColor3Yv[] 			= { .15f, .45f, 0.3f, glAlphaDefault };

static const GLfloat glSubWindowColor3DView[] 		= { 219.0/256.0, 238.0/256.0, 244.0/256.0, glAlphaSolid};

static const GLfloat glGroundPointColor[] 			= { 30.0/255.0, 13.0/255.0, 219.0/255.0,glAlphaDefault };
static const GLfloat glGroundDistancePointColor[] 	= { 30.0/255.0, 213.0/255.0, 49.0/255.0,glAlphaDefault };

static const GLfloat glBotBodyColor[] 				= { 62.0/255.0, 58.0/255.0, 45.0/255.0, glAlphaDefault };
static const GLfloat glBotFrontLegColor[] 			= { 128.0/255.0, 123.0/255.0, 109.0/255.0,glAlphaDefault };
static const GLfloat glBotLegColor[] 				= { 148.0/255.0, 143.0/255.0, 129.0/255.0, glAlphaDefault  };

static const GLfloat glFootTouchPointColor[] 		= { 230.0/255.0, 13.0/255.0, 19.0/255.0,glAlphaDefault };
static const GLfloat glFootTouchPointAirColor[] 	= GL_COLOR_4v(SquirrelGrey, glAlphaDefault );
static const GLfloat glFootTouchPointBackColor[] 	= GL_COLOR_4v(ResedaGreen, glAlphaDefault ); // Reseda grün
static const GLfloat glFootTouchPointFrontColor[] 	= GL_COLOR_4v(OliveYellow, glAlphaDefault);
static const GLfloat glFootTouchPointAreaColor[]	= GL_COLOR_4v(Vermilion, glAlphaDefault);

static const GLfloat glMapAreaColor4v[]		    		= GL_COLOR_4v( GreyWhite, glAlphaTransparent);
static const GLfloat glSlamGridColor4v[]		    	= GL_COLOR_4v( ConcreteGrey, glAlphaTransparent);
static const GLfloat glSlamFreeTopColor4v[]		    	= GL_COLOR_4v( LightIvory, glAlphaTransparent);
static const GLfloat glSlamOccupiedAroundTopColor4v[] 	= GL_COLOR_4v( CapriBlue, glAlphaDefault );
static const GLfloat glSlamOccupiedTopColor4v[] 		= GL_COLOR_4v( BlackBlue, glAlphaDefault );
static const GLfloat glSlamUnknownColor4v[]				= GL_COLOR_4v( OysterWhite, glAlphaDefault );
static const GLfloat glLaserScanColor4v[]		    	= GL_COLOR_4v( RedOrange, glAlphaDefault);
static const GLfloat glTrajectoryColor4v[]		    	= GL_COLOR_4v( GrassGreen, glAlphaDefault);

#endif /* UI_UICONFIG_H_ */
