/*
 * MicroControllerInterface.h
 *
 *  Created on: 02.03.2015
 *      Author: JochenAlt
 */

#ifndef MICROCONTROLLERINTERFACE_H_
#define MICROCONTROLLERINTERFACE_H_

#include <thread>
#include "string.h"

#include "core.h"

#include "rs232/SerialPort.h"
#include "basics/Util.h"

#include "I2CMaster.h"

#include "setup.h"
#include "basics/spatial.h"
#include "CortexComPackage.h"

using namespace std;

class CortexClient {
public:
	CortexClient();

	void setup();
	// initialize a safe communication. uC's setup is not called, bot remains silent
	bool setupCortexCommunication(string i2cPort, int i2cadr, string serialPort, int baudRate);

	// setup the Bot, do not yet switch it on
	bool setupBot();

	// enable all actuators
	bool enableBot();

	// enable all actuators
	bool disableBot();

	// fetch all angles
	bool fetchAngles(LegAnglesType& legAngles);

	// set the movement that is carried out asynchronously within loop
	void setMovement(const LegAnglesType& legAngles, realnum duration_ms);

	// move synchronously, i.e. this method returns when duration_ms is over
	bool moveSync(const LegAnglesType& legAngles, realnum duration_ms);

	// retrieve values of distance sensors
	void getDistanceSensor(realnum distance[NumberOfLegs]);

	// retrieve IMU orientation
	Rotation getIMUOrientation();

	// returns true, if values returned by getIMUOrientations are accurate and have been measured recently (i.e. within the last cycle)
	bool isIMUValueValid(int sinceMeasurement = CORTEX_SAMPLE_RATE*2);

	// retrieve voltage in cortex
	realnum getCortexVoltage();

	// return recently received angles
	const LegAnglesType& getLegAngles() { return lastLegAngles; };

	// switch on/off the bot. Requires setupBot upfront
	void actLikePower(bool onOff);

	// get status information about the bot
	bool info(bool &enabled);

	// send movement commands to the cortex periodically
	void loop();

	// returns true, if a successful communication to cortex has happened in the last cycle
	// if false, setupCommunication has to be called
	bool isCortexCommunicationOk();

	// true, if enable() has been called
	bool isEnabled() { return enabled; };

	bool betterShutMeDown() { return betterShutdown; };
private:
	bool retry(bool replyOk);

	bool callMicroController(string& cmd, string& response, int timeout_ms);
	bool binaryCallMicroController(uint8_t request[], int size, uint8_t response[], int responseSize, int delayTime_ms, int timeout_ms);
	bool readResponse(const Cortex::ResponsePackageData& response);

	bool receive(string& str, int timeout_ms);
	bool checkReponseCode(string &s, string& plainResponse, bool &OkOrNOk);
	void sendString(string str);

	bool cmdCHECKSUM(bool onOff);
	bool cmdECHO(string s);
	bool cmdSETUP();
	bool cmdDISABLE();
	bool cmdENABLE();
	bool cmdMOVE(const LegAnglesType& legAngles, int duration_ms);
	bool cmdGETall();
	bool cmdSET(int legNo, realnum minAngle, realnum maxAngle, realnum nullAngle);
	bool cmdBinaryGetAll();
	bool cmdBinaryMOVE(const LegAnglesType& legAngles, int duration_ms);
	bool cmdBinaryCommand(Cortex::Command cmd);

	bool cmdLOGsetup(bool onOff);
	bool cmdLOGservos(bool onOff);
	bool cmdLOGtest(bool onOff);
	bool cmdINFO(bool &enabled);

	void computeChecksum(string s,uint8_t& hash);

	SerialPort serialCmd; 			// serial port to transfer commands

	I2CMaster i2cPort;

	// true after setup when we switched to checksum mode that appends a checksum to any command
	bool withChecksum;

	// true if connection has been established succesfully
	bool cortexConnected = false;

	// retry counter (max 3)
	int cortexCommRetryCounter = 0;

	// true if enable() has been called
	bool enabled = false;

	// movement passed by setMovement
	LegAnglesType toBeAngles;
	realnum movementDuration;

	// most recent angles that has been returned from cortex
	LegAnglesType lastLegAngles;

	// most recent toe distance from cortex
	int measuredDistance[NumberOfLegs];

	// most recent IMU orientation
	Rotation measuredOrientation;

	// IMU state: number 0-3, 0 means the value is unusable, 3 is fully calibrated
	int imuStatusSys;
	int imuStatusAcc;
	int imuStatusGyro;

	// voltage of cortex (for servos, not the uC). Should be around 10V
	realnum measuredBatteryVoltage;


	// wall clock time of a cortex loop. Should be smaller than actual loop rate = CORTEX_LOOP_TIME
	int cortexWallClockLooptime;

	// last time of an IMU value, used in isIMUValueValid to check if the value can still be used
	milliseconds timeOfLastIMUValue;

	// true, if a error occurs that requires a controlled shutdown of the bot
	bool betterShutdown;
};

#endif /* MICROCONTROLLERINTERFACE_H_ */
