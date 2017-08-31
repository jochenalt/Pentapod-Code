#include "core.h"
#include <sstream>
#include <string>
ErrorCodeType glbError = ErrorCodeType::ABSOLUTELY_NO_ERROR;

void resetError() {
	glbError = ErrorCodeType::ABSOLUTELY_NO_ERROR;
}

ErrorCodeType getLastError() {
	return glbError;
}

void setError(ErrorCodeType err) {
	// set error only if error has been reset upfront
	if (glbError == ErrorCodeType::ABSOLUTELY_NO_ERROR)
		glbError = err;
}

bool isError() {
	return glbError != ErrorCodeType::ABSOLUTELY_NO_ERROR;
}


std::string getErrorMessage(ErrorCodeType err) {
	std::ostringstream msg;
	switch (err) {
	case ABSOLUTELY_NO_ERROR: msg << "no error";break;

	// hostCommunication
	case CHECKSUM_EXPECTED: 			msg << "checksum expected";break;
	case CHECKSUM_WRONG: 				msg << "checksum wrong";break;
	case PARAM_WRONG: 					msg << "parameter wrong";break;
	case PARAM_NUMBER_WRONG: 			msg << "number of parameter wrong";break;
	case UNRECOGNIZED_CMD: 				msg << "unknown command";break;
    case CORTEX_WRONG_MAGIC:            msg << "wrong magic number";break;

	// IMU
	case IMU_NOT_CALIBRATED: 			msg << "IMU not fully calibrated";break;

	// servos
	case HERKULEX_COMMUNICATION_FAILED: msg << "HerkuleX communication failed";break;
	case HERKULEX_STATUS_FAILED: 		msg << "HerkuleX status could not be retrieved";break;
	case SERVO_NOT_POWERED: 			msg << "HerkuleX servo not powered";break;
	case SERVO_NOT_SETUP: 				msg << "HerkuleX servo not setup";break;
	case SERVO_NOT_ENABLED: 			msg << "HerkuleX servo not enabled";break;

	// Cortex Controller
	case CORTEX_CONNECTION_FAILED: 		msg << "cortex connection failed";break;
	case CORTEX_COM_FAILED: 			msg << "cortex not connected, setup of communication failed";break;
	case CORTEX_NO_RESPONSE: 			msg << "no response from cortex";break;
	case CORTEX_POWER_ON_WITHOUT_SETUP: msg << "cannot power on without being setup";break;
	case CORTEX_SETUP_MISSING: 			msg << "call setup upfront";break;
	case CORTEX_RESPONSE_NOT_PARSED:	msg << "cortex reponse could not be parsed";break;

	// Webserver
	case WEBSERVER_TIMEOUT: 			msg << "no response from webserver (timeout)";break;

	case UNKNOWN_ERROR: 				msg << "mysterious error";break;

	default:
		msg << "unknown error message";
	}
	msg << " (" << (int)err << ")";

	return msg.str();
}

std::string getLastErrorMessage() {
	if (isError())
		return getErrorMessage(getLastError());
	return "";
}

// don't official function, since this is used by all three tiers, where <math.h> is not always present.
#define M_PI		3.14159265358979323846
#define radians(deg) (deg*(M_PI/180.0))

// configuration data of joints
// (offset, min[rad], max[rad], gearbox ratio
// gearbox reduces the knee servos angle only, for a knee turn of x° the servo needs to turn by x*26.0/25.0
LegConfiguration actuatorConfigType =  {
	// actuator						null-offset		min angle		max angle		 reducer ratio
	{ LimbConfiguration::HIP,		0.0,			radians(-80.0f)	,radians(80.0f), 1.0		},
	{ LimbConfiguration::THIGH,  	0.0,			radians(-95.0f)	,radians(95.0f), 1.0		},
	{ LimbConfiguration::KNEE,   	0.0,			radians(-65.0f)	,radians(65.0f), 26.0/25.0 	},
	{ LimbConfiguration::LOWERLEG,	radians(90.0),	radians(-65.0f)	,radians(147.0f), 1.0		}
};



// function pointers implementing a cortex command.
// On the webserver, these commands are implemented with empty functions
extern void cmdLED();
extern void cmdSETUP();
extern void cmdECHO();
extern void cmdCONSOLE();
extern void cmdMOVE();
extern void cmdMOVELEG();
extern void cmdDISABLE();
extern void cmdENABLE();
extern void cmdGET();
extern void cmdCONFIG();
extern void cmdMEM();
extern void cmdCHECKSUM();
extern void cmdLOG();
extern void cmdHELP();
extern void cmdINFO();
extern void cmdCONFIG();
extern void cmdBIN();

CortexCommandDefinitionType commDef[CortexCommandDefinitionType::NumberOfCommands] {
	//cmd ID						Name, 		timeout,	function pointer
	{ CortexCommandDefinitionType::ECHO_CMD,	    "ECHO", 	500, 		cmdECHO },
	{ CortexCommandDefinitionType::HELP_CMD,	    "HELP", 	500, 		cmdHELP },
	{ CortexCommandDefinitionType::CONSOLE_CMD,		"CONSOLE", 	500, 		cmdCONSOLE},
	{ CortexCommandDefinitionType::ENABLE_CMD,		"ENABLE", 	2000, 		cmdENABLE},
	{ CortexCommandDefinitionType::DISABLE_CMD,		"DISABLE", 	200, 		cmdDISABLE },
	{ CortexCommandDefinitionType::SETUP_CMD,		"SETUP", 	2000, 		cmdSETUP},
	{ CortexCommandDefinitionType::CHECKSUM_CMD,	"CHECKSUM", 200, 		cmdCHECKSUM},
	{ CortexCommandDefinitionType::MEM_CMD,	        "MEM", 		200, 		cmdMEM},
	{ CortexCommandDefinitionType::CONFIG_CMD,      "CONFIG", 	100, 		cmdCONFIG},
	{ CortexCommandDefinitionType::GET_CMD,	        "GET", 		300, 		cmdGET},
	{ CortexCommandDefinitionType::MOVELEG_CMD,	    "MOVELEG", 	200, 		cmdMOVELEG},
	{ CortexCommandDefinitionType::MOVE_CMD,	    "MOVE", 	300, 		cmdMOVE},
	{ CortexCommandDefinitionType::LOG_CMD,	        "Log", 		200, 		cmdLOG },
	{ CortexCommandDefinitionType::INFO_CMD,	    "INFO", 	500, 		cmdINFO },
	{ CortexCommandDefinitionType::BIN_CMD,	    	"BIN", 		500, 		cmdBIN}
};

// returns command definition of the passed command
CortexCommandDefinitionType* CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType cmd) {
	for (int i = 0;i<NumberOfCommands;i++) {
		if (commDef[i].cmd == cmd)
			return &commDef[i];
	}
	return 0;
}


