/*
 * core.cpp
 * Basic definitions like constants and types
 *
 * Author: JochenAlt
 */

#ifndef _PENTAPOD_CORE_H_
#define _PENTAPOD_CORE_H_

#include <stdlib.h>
#include <string>

using namespace std;


// pentapod, right?
const int NumberOfLegs = 5;

// 4 servos per leg
const int NumberOfLimbs= 4;

// total number of servos
const int NumberOfServos = NumberOfLimbs * NumberOfLegs;

// physically, a servo angle may go 5� above joint limit (used in cortex only)
const float angleLimitOffset = 5;

// use that instead of double or float in order to be flexible depending on FPU
typedef double realnum;

// CAD dimensions of the pentapod, measured within the Inventor CAD model
namespace CAD {
	// data measured from Body.iam
	constexpr static realnum HipNickAngle  = 19.29; 			// angle each hip goes down against the xy pane
	constexpr static realnum HipCentreDistance  = 52.601+0.1;	// distance between z-axis and the circle where all hips are arranged, but orthoginal to hip mounting pane
	constexpr static realnum HipLength= 35.528;                 // distance between hip mounting point and hip joint axis
	constexpr static realnum HipMountingPointOverBody = 53.145; // distance between the body's as and the hip mointing point

	// measured from HipJoint.ipt
	constexpr static realnum HipJointLength = 45.249;

	// measured from Thigh.iam
	constexpr static realnum ThighLength = 53.500;
	constexpr static realnum ThighKneeGapLength = 0.5;
	constexpr static realnum KneeJointLength = 81.018;

	// measured from Foot.iam
	constexpr static realnum FootLength = 135.928;
	constexpr static realnum DampenerLength = 5.0;				// length of the silicone dampener
	constexpr static realnum BodyHipHeight = 14.478;
	constexpr static realnum FootDampenerDiameter = 24.0;		// diameter of the rubber at the leg's end

	// measured from Body.iam
	constexpr static realnum LaserSensorHeight = 75.8;			// height of the lasers measurement plane above the xy-plane
};

// static data of a limb
struct LimbConfiguration {
	enum LimbIdentifier {HIP=0, THIGH=1, KNEE=2, LOWERLEG=3 };
	LimbIdentifier 	id;
	float 		nullOffset;		// move the null angle to a certain offset
	float 		minAngle;		// minimum limit in degree
	float 		maxAngle;		// maximum limit in degree
	float 		gearRatio;		// for knee only, there is a 90� gear
};

// static definition of a leg with regards to limits and gearratio
typedef LimbConfiguration LegConfiguration[NumberOfLimbs];	// all actuators
extern LegConfiguration actuatorConfigType;

// all errors coming from the cortex
enum ErrorCodeType { ABSOLUTELY_NO_ERROR = 0,
	// cortex communication errors
	CHECKSUM_EXPECTED = 1 , CHECKSUM_WRONG = 2,	PARAM_WRONG = 3, PARAM_NUMBER_WRONG = 4, UNRECOGNIZED_CMD = 5,
	CORTEX_POWER_ON_WITHOUT_SETUP= 6,	CORTEX_SETUP_MISSING = 7, CORTEX_PACKAGE_SIZE_ERROR = 8,CORTEX_WRONG_MAGIC = 9,

	// IMU errors
	IMU_NOT_CALIBRATED = 10,

	// herkulex errors
	HERKULEX_COMMUNICATION_FAILED = 40, HERKULEX_STATUS_FAILED = 41,SERVO_NOT_ENABLED= 42, SERVO_NOT_POWERED =43, SERVO_NOT_SETUP = 44,

	// Cortex errors
	CORTEX_CONNECTION_FAILED = 50, CORTEX_COM_FAILED = 51, CORTEX_LOG_COM_FAILED=52, CORTEX_NO_RESPONSE =53,CORTEX_RESPONSE_NOT_PARSED = 54,CORTEX_RESPONSE_SIZE_WRONG = 55,

	// Webserver errors
	WEBSERVER_TIMEOUT = 60,

	// IMU errors
	IMU_NOT_DETECTED = 70,

	// last exit Brooklyn
	UNKNOWN_ERROR= 99
};



// set global error to NO_ERROR
void resetError();

// return last error set by setError
ErrorCodeType getLastError();

// set the passed error
void setError(ErrorCodeType err);

// returns prosa message of error code
string getErrorMessage(ErrorCodeType err);

// errors are stored in a globale variables. This gives the most recent one unless resetError is called
string getLastErrorMessage();

// true, if error has been set. When called again after an error, false is returned.
bool isError();

#define WIFI_WEB_SERVER_HOST "192.168.178.58" 	// static IP of Odroid X2 hosting the Cortex
#define LINUX_WEB_SERVER_PORT 8000
#define WIN_WEB_SERVER_HOST "192.168.178.57" 	// Jochens Notebook
#define WIN_WEB_SERVER_PORT 8080

// communication between engine ros node and Cortex via serial interface
#ifdef _WIN32
	#define CORTEX_CLI_SERIAL_PORT "COM1"
	#define CORTEX_LOGGER_SERIAL_PORT "COM1"
#else
	#define CORTEX_CLI_SERIAL_PORT "ttyS1"
	#define CORTEX_I2C_PORT "i2c-1"
	#define CORTEX_I2C_ADDRESS 0x8

#endif

#define CORTEX_CLI_SERIAL_BAUDRATE 230400

// status information of servos
// HERKULEX STATUS ERROR - See Herkulex Manual p39
#define H_STATUS_OK					 0x00
#define H_ERROR_INPUT_VOLTAGE 		 0x01
#define H_ERROR_POS_LIMIT			 0x02
#define H_ERROR_TEMPERATURE_LIMIT	 0x04
#define H_ERROR_INVALID_PKT			 0x08
#define H_ERROR_OVERLOAD			 0x10
#define H_ERROR_DRIVER_FAULT  		 0x20
#define H_ERROR_EEPREG_DISTORT		 0x40

// Possible states of a Herkulex servo
enum ServoStatusType { 	SERVO_STAT_OK,				// no error
						SERVO_STAT_VOLTAGE,			// voltage not within 7.4-10V
						SERVO_STAT_LIMIT,			// angle out of bounds
						SERVO_STAT_TEMP,			// temperature too high
						SERVO_STAT_INV_PKT,			// dont know
						SERVO_STAT_OVERLOAD,		// servo overloaded, need to switch it off and on
						SERVO_STAT_DRIVER_FAULT,	// driver error, that's really bad
						SERVO_STAT_EEPREG_DISTORT,	// even worse
						SERVO_STAT_NO_COMM,			// communication error between cortex and servo
					};

// return long name of servo status
std::string getServoStatusTypeName(ServoStatusType stat);

// every [ms] the motors get a new position. 11.2ms is the unit
// Herkulex servos are working with, sample rate should be a multiple of that
// With 22ms, we run at 45Hz
#define HERKULEX_MIN_SAMPLE 11.2
#define CORTEX_SAMPLE_RATE  22

// there is a low prio loop running in the cortex (1Hz) that checks the
// voltage, the servo status and other stuff
#define LOW_PRIO_LOOP_RATE_MS ((int)CORTEX_SAMPLE_RATE*20)


// The cortex has two types of interfaces: The regular ascii terminal and the binary interface.
// The asci terminal is used to manually key in commands via putty, regular communication is
// done with binary packages (for bandwidth reasons). These binary packages follow a regular "bin" command
struct CortexCommandDefinitionType {
	static const int NumberOfCommands = 13;

	// all possible commands the cortex provides
	enum CortexCommandType { 		ECHO_CMD = 0,			// echos the parameter
									HELP_CMD = 1, 			// prints out a help screen
									ENABLE_CMD = 2,			// puts torque on all servos
									DISABLE_CMD = 3,		// turns off torque from servos
									MEM_CMD = 4,			// prints contents of eeprom
									GET_CMD = 5,			// gets angles of all servos
									CONFIG_CMD = 6,			// configures limits and null values of a servo
									MOVE_CMD = 7,			// moves all servos to a certain angle
									MOVELEG_CMD = 8,		// moves one leg to certain angles
									LOG_CMD = 9,			// turns on/off logging
									INFO_CMD = 10,			// prints ic2 information and stuff
									SETUP_CMD = 11,			// calls setup procedure
									BIN_CMD = 12			// passes a binary commmand
	};

	CortexCommandType cmd;

	// name of the command
	const char*  name;

	// allowed timeout
	int timeout_ms;

	// Pointer to the default handler function (used in cortex only)
    void (*cmdFunction)();

    // returns the cortex command by a given enum value
	static CortexCommandDefinitionType* get(CortexCommandType cmd);
};

// structure keeping all
extern CortexCommandDefinitionType commDef[];

#endif
