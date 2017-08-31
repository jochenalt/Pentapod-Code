/* 
* HostCommunication.cpp
*
* Created: 26.06.2016 22:16:52
* Author: JochenAlt
*/


#include "I2CPortScanner.h"
#include "HostCommunication.h"
#include "Controller.h"
#include "BotMemory.h"
#include "utilities.h"
#include "core.h"
#include "PowerVoltage.h"
#include "CortexComPackage.h"

HostCommunication hostComm;
extern Controller controller;
extern BotMemory botMemory;

extern void setCortexBoardLED(bool onOff);
extern void setLEDPattern();

// rotary encoders are connected via I2C
extern i2c_t3* Wires[5];
extern i2c_t3* IMUWire;

void printHex(uint8_t i) {
	if (i<16)
		cmdSerial->print(F("0"));
	cmdSerial->print(i,HEX);
}


void replyOk() {
	cmdSerial->println(F(">ok"));
	cmdSerial->print(F(">"));
	cmdSerial->flush();
}

void replyError(int errorCode) {
	int patchedErrorCode = errorCode;
	if (errorCode == PARAM_NUMBER_WRONG) {
		if (hostComm.sCmd.getErrorCode() != 0) {
			patchedErrorCode = hostComm.sCmd.getErrorCode();
		}
	}
	cmdSerial->print(F(">nok("));
	cmdSerial->print(patchedErrorCode);
	cmdSerial->println(")");
	cmdSerial->print(F(">"));
	cmdSerial->flush();
}


void cmdLOG() {
	char* logClass= NULL;
	char* onOff= NULL;

	bool paramsOK = hostComm.sCmd.getParamString(logClass);
	paramsOK = hostComm.sCmd.getParamString(onOff) && paramsOK;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = false;
		bool onOffSet = false;
		bool onOffFlag = false;
		logger->print(F("log "));
		logger->print(logClass);
		logger->print(F(" "));
		logger->println(onOff);

		if (strncasecmp(onOff, "on", 2) == 0) {
			onOffFlag = true;
			onOffSet = true;
		}
		if (strncasecmp(onOff, "off", 3) == 0) {
			onOffFlag = false;
			onOffSet = true;
		}
		
		if (onOffSet && (strncasecmp(logClass, "setup", 5) == 0)) {
			memory.persMem.logSetup = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}
		
		if (onOffSet && (strncasecmp(logClass, "servo", 5) == 0)) {
			memory.persMem.logServo = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}

		if (onOffSet && (strncasecmp(logClass, "encoder", 5) == 0)) {
			memory.persMem.logEncoder = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}
		if (onOffSet && (strncasecmp(logClass, "test", 5) == 0)) {
			valueOK = true;
			replyOk();
			return;
		}
		if (onOffSet && (strncasecmp(logClass, "loop", 5) == 0)) {
			memory.persMem.logLoop = onOffFlag;

			valueOK = true;
			replyOk();
			return;
		}


		if (valueOK) 
			replyOk();
		else
			replyError(PARAM_WRONG);
	} else {
		replyError(PARAM_NUMBER_WRONG);
	}
}


void cmdINFO() {
	bool paramsOK = true;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		cmdSerial->print(F("{status("));
		if (controller.isSetup())
			cmdSerial->print(F(" setup=1"));
		else
			cmdSerial->print(F(" setup=0"));

		if (controller.isEnabled())
			cmdSerial->print(F(" enable=1"));
		else
			cmdSerial->print(F(" enable=0"));

		cmdSerial->print(F(")"));


		cmdSerial->print(F(" I2C("));
		bool first = true;
		int count = 0;
		for (int i = 0x20;i<0x40;i++) {
			byte error;
			bool yes = scanI2CAddress(IMUWire, i, error);
			if (yes) {
				count++;
				if (!first)
					cmdSerial->print(F(" "));
				cmdSerial->print(F("0x"));
				printHex(i);
				first = false;
			}
		}
		cmdSerial->print(") Legs(");

		for (int i = 0;i<NumberOfLegs;i++) {
			if (i>0)
				logger->print(" ");
			controller.getLeg(i).logStatus();
		}
		cmdSerial->print(")}");

		replyOk();
	} else {
		replyError(PARAM_NUMBER_WRONG);
	}
}

// the only command that does not require a checksum
void cmdCHECKSUM() {
	char* onoff = 0;
	bool paramsOK = hostComm.sCmd.getParamString(onoff);

	if (paramsOK) {
		bool valueOK = false;
		if (strncasecmp(onoff, "on", 2) == 0) {
			hostComm.sCmd.useChecksum(true);
			valueOK = true;
		}
		if (strncasecmp(onoff, "off", 3) == 0) {
			hostComm.sCmd.useChecksum(false);
			valueOK = true;
		}
		if (valueOK) {
			replyOk();
		}
		else
			replyError(PARAM_WRONG);
	} else {
			replyError(PARAM_NUMBER_WRONG);
	}
}

void headlessSetup();

void cmdSETUP() {
	bool paramsOK = false;
	paramsOK = hostComm.sCmd.endOfParams(false);
	if (paramsOK) {
		resetError();
		headlessSetup();
		if (!isError())
			replyOk();		
		else
			replyError(getLastError());
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}


void cmdENABLE() {
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		controller.enable();			// enable all actuators
		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdDISABLE(){
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		controller.disable();			// disable first, in order to avoid ticks
		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdCONSOLE(){
	char* param = 0;
	bool paramsOK = hostComm.sCmd.getParamString(param);
	paramsOK = hostComm.sCmd.endOfParams(false) && paramsOK;

	if (paramsOK) {
		if (strncasecmp(param, "off", 2) == 0) {
			hostComm.sCmd.useChecksum(true);
			memory.persMem.logSetup = false;
			memory.persMem.logServo = false;

			replyOk();
			return;
		}
		if (strncasecmp(param, "on", 3) == 0) {
			hostComm.sCmd.useChecksum(false);
			memory.persMem.logSetup = true;
			memory.persMem.logServo = true;

			replyOk();
			return;

		}
		setError(PARAM_WRONG);
		replyError(getLastError());
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}


void cmdMEM() {
	char* cmdParam = 0;
	bool paramsOK = hostComm.sCmd.getParamString(cmdParam);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = false;

		if (strncasecmp(cmdParam, "reset", 5) == 0) {
			BotMemory::setDefaults();
			memory.println();
			memory.save();
			valueOK = true;
		}

		if (strncasecmp(cmdParam, "list", 4) == 0) {
			controller.logConfiguration();

			orientationSensor.logSensorCalibration();

			valueOK = true;
		}

		if (strncasecmp(cmdParam, "nullimu", 7) == 0) {
			orientationSensor.nullify();
			memory.println();
			memory.save();
		}

		if (strncasecmp(cmdParam, "saveimucalib", 12) == 0) {
			if (orientationSensor.isFullyCalibrated()) {
				orientationSensor.saveCalibration();
				orientationSensor.logSensorCalibration();

				valueOK = true;
			}
			else
				replyError(IMU_NOT_CALIBRATED);
		}

		if (valueOK)
			replyOk();
		else
			replyError(PARAM_WRONG);
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdCONFIG() {
	int16_t legNo = 0;
	int16_t limbNo = 0;
	float minValue, maxValue, nullValue = 0;
	bool minValueSet, maxValueSet, nullValueSet = false;

	bool paramsOK = hostComm.sCmd.getParamInt(legNo);
	paramsOK = paramsOK && hostComm.sCmd.getParamInt(limbNo);

	paramsOK = hostComm.sCmd.getNamedParamFloat("min",minValue,minValueSet) && paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("max",maxValue,maxValueSet) && paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("null",nullValue,nullValueSet)&& paramsOK;

	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = ((legNo>=0) && (legNo<=NumberOfLegs));
		valueOK = valueOK && ((limbNo>=0) && (limbNo<=NumberOfLimbs));

		if (valueOK) {
			if (nullValueSet)
				memory.persMem.legs[legNo].limbs[limbNo].nullAngle = nullValue;
			if (minValueSet)
				memory.persMem.legs[legNo].limbs[limbNo].minAngle = minValue;
			if (maxValueSet)
				memory.persMem.legs[legNo].limbs[limbNo].maxAngle = maxValue;

			memory.println();
			memory.delayedSave();
			replyOk();
		}
		else
			replyError(PARAM_WRONG);
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}


void cmdECHO() {
	bool paramsOK = true;
	char* param = 0;
	paramsOK = hostComm.sCmd.getParamString(param) && paramsOK;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;


	if (paramsOK) {
		cmdSerial->print(param);

		replyOk();
	}
	else {
		replyError(PARAM_NUMBER_WRONG);
	}
}


void cmdBIN() {
	Cortex::RequestPackageData request;
	Cortex::ResponsePackageData response;

	// timeout is twice the time required by transfer
	uint32_t timeout = 1+ 2*1000*Cortex::RequestPackageData::Size/(CORTEX_CLI_SERIAL_BAUDRATE/10);

	int bytesRead = hostComm.sCmd.readBinPackage(request.data, Cortex::RequestPackageData::Size, timeout);
	bool ok = (bytesRead == Cortex::RequestPackageData::Size);

	if (ok) {
		Cortex::Command cmd;
		float angles[NumberOfLimbs*NumberOfLegs];
		int duration_ms;
		ok = Cortex::ComPackage::readRequest(request, cmd, angles, duration_ms);

		switch (cmd) {
		case Cortex::SETUP:
			headlessSetup();
			break;
		case Cortex::ENABLE:
			controller.enable();
			break;
		case Cortex::DISABLE:
			controller.disable();
			break;
		case Cortex::MOVE: {
			controller.adaptSynchronisation();
			LimbAnglesType legAngles;
			for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
				Leg& leg = controller.getLeg(legNo);
				for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
					legAngles[limbNo] = angles[limbNo + legNo*NumberOfLimbs];
				}
				leg.setAngles(legAngles, duration_ms);
			}
			break;
		}
		case Cortex::GET:
			// do nothing, just return current state
			break;
		}

		Cortex::Status status;
		if (controller.isEnabled())
			status = Cortex::ENABLED;
		else
			status = Cortex::DISABLED;

		uint8_t servoStatus[NumberOfLegs*NumberOfLimbs];
		uint8_t distance[NumberOfLegs];
		for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
			Leg& leg = controller.getLeg(legNo);
			distance[legNo] = leg.getDistance();
			for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
				float angle = leg.servos[limbNo].getCurrentAngle();
				angles[limbNo + legNo*NumberOfLimbs] = angle;
				servoStatus[limbNo + legNo*NumberOfLimbs] = leg.getStatus(limbNo);
			}
		}

		// collect IMU data
		float imuX,imuY,z;
		uint8_t newSystem, newGyro, newAcc, newMag;
		orientationSensor.getData(imuX, imuY, z, newSystem, newGyro, newAcc, newMag);
		int imuStatus = newSystem*1000 + newGyro*100 + newAcc*10 + newMag;

		// create response
		ok = Cortex::ComPackage::createResponse(
				status, angles, distance, servoStatus,
				imuX, imuY, imuStatus,
				voltage.getVoltage(),
				controller.looptime(),
				response);

		if (ok) {
			// send back
			cmdSerial->write(response.data, Cortex::ResponsePackageData::Size);
		}
		else {
			// we could not parse the package, clear input buffer and try next time
			while (Serial.available() > 0)
				Serial.read();
		}
	}
}

void cmdGET() {
	bool isAll = false;
	char* actuatorStr= 0;
	bool paramsOK = hostComm.sCmd.getParamString(actuatorStr);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = false;

		if (actuatorStr) {
			isAll = (strncasecmp(actuatorStr, "all", 3) == 0);
			if (isAll) {
				for (int legNo = 0;legNo<NUMBER_OF_LEGS;legNo++) {
					if (legNo > 0)
						cmdSerial->print("  ");

					cmdSerial->print("(");

					Leg& leg = controller.getLeg(legNo);
					for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
						float angle = leg.servos[limbNo].getCurrentAngle();
						cmdSerial->print(angle,2);
						cmdSerial->print(" ");
					}
					cmdSerial->print(leg.getDistance());
					cmdSerial->print(")");
				}

				// return IMU's orientation
				cmdSerial->print(" (");
				float x,y,z;
				uint8_t newSystem, newGyro, newAcc, newMag;
				orientationSensor.getData(x, y, z, newSystem, newGyro, newAcc, newMag);
				cmdSerial->print(x, 1);
				cmdSerial->print(' ');
				cmdSerial->print(y, 1);
				cmdSerial->print(' ');
				cmdSerial->print(z, 1);
				cmdSerial->print(' ');
				cmdSerial->print(newSystem);
				cmdSerial->print(newGyro);
				cmdSerial->print(newAcc);
				cmdSerial->print(newMag);

				// return power voltage
				cmdSerial->print(' ');
				cmdSerial->print(voltage.getVoltage(),1);

				// return loop time
				cmdSerial->print(' ');
				cmdSerial->print(controller.looptime());

				cmdSerial->print(")");


				valueOK = true;
			}
			else
				if ((actuatorStr != NULL) && (actuatorStr[0] >= '0') && (actuatorStr[0] <= '5')) {
					int legNo = actuatorStr[0] - '0';
					Leg& leg = controller.getLeg(legNo);

					for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
						if (limbNo > 0)
							cmdSerial->print(" ");

						float angle = leg.servos[limbNo].readCurrentAngle();
						cmdSerial->print(angle ,2);
						cmdSerial->print(" ");
						cmdSerial->print(memory.persMem.legs[legNo].limbs[limbNo].minAngle,2);
						cmdSerial->print(" ");
						cmdSerial->print(memory.persMem.legs[legNo].limbs[limbNo].maxAngle,2);
						cmdSerial->print(" ");
						cmdSerial->print(memory.persMem.legs[legNo].limbs[limbNo].nullAngle,2);

					}

					cmdSerial->print(" ");
					cmdSerial->print(leg.getDistance());

					valueOK = true;
				} else
					setError(PARAM_WRONG);
		}

		if (valueOK) {
			valueOK = false;
			if (controller.isSetup()) {
				replyOk();
			} else {
				setError(CORTEX_SETUP_MISSING);
				replyError(getLastError());
			}
		} else 
			replyError(PARAM_WRONG);			
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdMOVE() {
	LimbAnglesType legAngles[NUMBER_OF_LEGS];
	bool paramsOK = true;

	int16_t duration = 0;
	for (int legNo = 0;legNo<NUMBER_OF_LEGS;legNo++) {
		for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
			float angle;
			paramsOK = paramsOK && hostComm.sCmd.getParamFloat(angle);
			if (abs(angle) > 360.00)
				paramsOK = false;
			if (paramsOK)
				legAngles[legNo][limbNo] = angle;
		}
	}
	paramsOK = hostComm.sCmd.getParamInt(duration) && (duration <= 9999) && (duration>=10) && paramsOK;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		if (memory.persMem.logLoop) {
			logger->print(F("moveLeg "));
		}
		else if (!controller.isSetup())
			replyError(SERVO_NOT_SETUP);
		else if (!controller.isEnabled())
			replyError(SERVO_NOT_ENABLED);
		else {
			controller.adaptSynchronisation();

			// set the angles of the legs
			for (int legNo = 0;legNo<NUMBER_OF_LEGS;legNo++) {
				Leg& leg = controller.getLeg(legNo);
				leg.setAngles(legAngles[legNo], duration);
				for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
					cmdSerial->print((int)leg.getStatus(limbNo));
				}
			}

			for (int legNo = 0;legNo<NUMBER_OF_LEGS;legNo++) {
				cmdSerial->print(' ');
				Leg& leg = controller.getLeg(legNo);
				cmdSerial->print(leg.getDistance());
			}

			// return IMU's orientation
			float x,y,z;
			uint8_t newSystem, newGyro, newAcc, newMag;
			orientationSensor.getData(x, y, z, newSystem, newGyro, newAcc, newMag);
			cmdSerial->print(' ');
			cmdSerial->print(x, 1);
			cmdSerial->print(' ');
			cmdSerial->print(y, 1);
			cmdSerial->print(' ');
			cmdSerial->print(z, 1);
			cmdSerial->print(' ');
			cmdSerial->print(newSystem);
			cmdSerial->print(newGyro);
			cmdSerial->print(newAcc);
			cmdSerial->print(newMag);

			// return power voltage
			cmdSerial->print(' ');
			cmdSerial->print(voltage.getVoltage(),1);

			// return loop time
			cmdSerial->print(' ');
			cmdSerial->print(controller.looptime());
			replyOk();
		}
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdMOVELEG() {
	float angles[NumberOfLimbs] = {0,0,0,0};
	int16_t legNo;
	bool paramsOK = hostComm.sCmd.getParamInt(legNo);
	if ((legNo < 0) || (legNo >= NumberOfLegs))
		paramsOK = false;

	int16_t duration = 0;
	for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++)
		paramsOK = hostComm.sCmd.getParamFloat(angles[limbNo]) && (abs(angles[limbNo]) <= 360.0) && paramsOK;

	paramsOK = hostComm.sCmd.getParamInt(duration) && (duration <= 9999) && (duration>=20) && paramsOK;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;

	if (paramsOK) {
		if (!controller.isSetup())
			replyError(SERVO_NOT_SETUP);
		else if (!controller.isEnabled())
			replyError(SERVO_NOT_ENABLED);
		else {
			if (memory.persMem.logLoop) {
				logger->print(F("moveLeg "));
			}
			Leg& leg = controller.getLeg(legNo);
			leg.setAngles(angles, duration);


			// send the reply
			if (memory.persMem.logLoop) {
				for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
					if (limbNo>0)
						logger->print(",");
					logger->print(angles[limbNo]);
				}
			}
			// return distance
			cmdSerial->print(leg.getDistance());

			if (memory.persMem.logLoop) {
				logger->print(",");
				logger->print(duration);
				logger->println();
			}
			replyOk();
		}
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

// This gets set as the default handler, and gets called when no other command matches.
void cmdUnrecognized(const char *command) {
	cmdSerial->print(command);
	replyError(UNRECOGNIZED_CMD);
}

void cmdHELP() {
	bool paramsOK = hostComm.sCmd.endOfParams(false);
	if (paramsOK) {
		cmdSerial->println(F("usage:"));
		cmdSerial->println(F("\tLED <on|off|blink>"));
		cmdSerial->println(F("\tSETUP"));
		cmdSerial->println(F("\tCONSOLE <on|off>"));
		cmdSerial->println(F("\tENABLE"));
		cmdSerial->println(F("\tDISABLE"));
		cmdSerial->println(F("\tCHECKSUM <on|off>"));
		cmdSerial->println(F("\tMEM (<reset>|<list>|saveimucalib)"));
		cmdSerial->println(F("\tCONFIG <LegNo> <LimbNo> [min=<min>] [max=<max>] [null=<nullvalue>] "));
		cmdSerial->println(F("\tGET <LegNo> : {ang=<angle> min=<min> max=<max> null=<null>}"));
		cmdSerial->println(F("\tGET all : (i=<id> n=<name> ang=<angle> min=<min> max=<max> null=<null>)"));
		cmdSerial->println(F("\tMOVELEG <LegNo> <angle1> <angle2> <angle3> <angle4> <durationMS>"));
		cmdSerial->println(F("\tMOVE 	(<angle1> <angle2> <angle3>)x5 <durationMS>"));

		cmdSerial->println(F("\tLOG <setup|servo|stepper|encoder|loop> <on|off>"));
		cmdSerial->println(F("\tINFO"));

		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

// default constructor
HostCommunication::HostCommunication()
{
	setupTime = 0;
} //HostCommunication



void HostCommunication::setup() {
	// Setup callbacks for SerialCommand commands
	for (int i = 0;i<CortexCommandDefinitionType::NumberOfCommands;i++) {
		sCmd.addCommand(commDef[i].name, commDef[i].cmdFunction);
	}

	sCmd.setDefaultHandler(cmdUnrecognized);   // Handler for command that isn't matched  (says "What?")

	sCmd.useChecksum(false);

	setupTime = millis();
}

void HostCommunication::loop() {
	// start receiving commands 500ms after setup
	// since we need to wait for IMU to settle, its first values are kind of messed up
	if (millis() > setupTime + 1000)
		sCmd.readSerial();     // there's not much done, just processing of serial commands.
}

