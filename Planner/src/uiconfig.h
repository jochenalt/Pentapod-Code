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

// RAL colors
#include "ral.h"

// macro to define an openGL color array of a RAL color with a specific alpha(transparency) value
#define GL_COLOR_4v(rgb,alpha) { ((float)(rgb>>16))/255.0, ((float)((rgb>>8) & 0xFF))/255.0, ((float)(rgb & 0xFF))/255.0, alpha }


// transarency values
const float glAlphaSolid = 1.0;
const float glAlphaTransparent = 0.5;

static const GLfloat glCoordSystemAreaColor4v[]			= GL_COLOR_4v(TeleGrey4, glAlphaSolid );
static const GLfloat glCoordSystemAxisColor4v[] 		= GL_COLOR_4v(PearlMouseGrey, glAlphaSolid );
static const GLfloat glRasterColor4v[] 					= GL_COLOR_4v(GraniteGrey, glAlphaSolid );

static const GLfloat glGroundPointColor[] 				=  GL_COLOR_4v(PebbleGrey, glAlphaSolid );
static const GLfloat glGroundDistancePointColor[] 		=  GL_COLOR_4v(MintGreen, glAlphaSolid );

static const GLfloat glBotBodyColor[] 					= GL_COLOR_4v(WineRed, glAlphaSolid );
static const GLfloat glLidarColor[] 					= GL_COLOR_4v(BlackBrown, glAlphaSolid );

static const GLfloat glBotFrontLegColor[] 				= GL_COLOR_4v(PearlBeige, glAlphaSolid );
static const GLfloat glBotLegColor[] 					= GL_COLOR_4v(PearlMouseGrey, glAlphaSolid );

static const GLfloat glFootTouchPointColor[] 			= GL_COLOR_4v(Ivory, 0.2 );
static const GLfloat glFootTouchPointAirColor[] 		= GL_COLOR_4v(SquirrelGrey, glAlphaSolid );
static const GLfloat glFootTouchPointBackColor[] 		= GL_COLOR_4v(ResedaGreen, glAlphaSolid );
static const GLfloat glFootTouchPointFrontColor[] 		= GL_COLOR_4v(OliveYellow, glAlphaSolid);
static const GLfloat glFootTouchPointAreaColor[]		= GL_COLOR_4v(BlackRed, glAlphaTransparent);

static const GLfloat glMapBackgroundColor4v[]		    = GL_COLOR_4v( AnthraciteGrey, 0.01);
static const GLfloat glMapAreaColor4v[]		    		= GL_COLOR_4v( AnthraciteGrey, glAlphaTransparent);
static const GLfloat glSlamGridColor4v[]		    	= GL_COLOR_4v( ConcreteGrey, glAlphaTransparent);
static const GLfloat glSlamFreeTopColor4v[]		    	= GL_COLOR_4v( LightIvory, glAlphaTransparent);
static const GLfloat glMapMarkerColor4v[]		    	= GL_COLOR_4v( OxideRed, glAlphaSolid);
static const GLfloat glMapSphereMarkerColor4v[]		    = GL_COLOR_4v( SalmonPink, glAlphaSolid);

static const GLfloat glSlamOccupiedAroundTopColor4v[] 	= GL_COLOR_4v( CapriBlue, glAlphaSolid );
static const GLfloat glSlamOccupiedTopColor4v[] 		= GL_COLOR_4v( BlackBlue, glAlphaSolid );
static const GLfloat glSlamUnknownColor4v[]				= GL_COLOR_4v( OysterWhite, glAlphaTransparent );
static const GLfloat glLaserScanColor4v[]		    	= GL_COLOR_4v( RedOrange, glAlphaSolid);
static const GLfloat glSlamMapCoordSystemColor4v[]		= GL_COLOR_4v( RedViolet, glAlphaTransparent);

static const GLfloat glTrajectoryColor4v[]		    	= GL_COLOR_4v( GrassGreen, glAlphaSolid);
static const GLfloat glGlobalPlanColor4v[]		    	= GL_COLOR_4v( SkyBlue, glAlphaSolid);
static const GLfloat glLocalPlanColor4v[]		    	= GL_COLOR_4v( Telemagenta, glAlphaSolid);

#endif /* UI_UICONFIG_H_ */
