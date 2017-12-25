/*
 * CortexController.cpp
 *
 *
 * Author: JochenAlt
 */


#include <iostream>
#include <thread>
#include <chrono>
#include "unistd.h"
#include <sstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <iomanip>

#include "core.h"

#include "basics/stringhelper.h"
#include "basics/logger.h"
#include "rs232/SerialPort.h"
#include "basics/util.h"
#include "basics/types.h"

#include "setup.h"
#include "CortexClient.h"

#include "CortexComPackage.h"

using namespace std;

const string reponseOKStr =">ok\r\n>";	 // reponse code from uC: >ok or >nok(errornumber)
const string reponseNOKStr =">nok(";

string replaceWhiteSpace(string s) {
	replace (s.begin(), s.end(), '\r' , 'R');
	replace (s.begin(), s.end(), '\n' , 'N');
	return s;
}

// the following functions are dummys, real functions are used in the uC. Purpose is to have
// one communication interface header between uC and host containing all commands. uC uses a
// library that works with function pointers to parse the commands, here we use regular method calls,
// since in most cases we send in fire-and-forget style
void cmdHELP() {};
void cmdCONSOLE(){};
void cmdECHO(){};
void cmdDISABLE(){};
void cmdSETUP(){};
void cmdENABLE(){};
void cmdGET(){};
void cmdCONFIG(){};
void cmdMOVE(){};
void cmdMOVELEG(){};
void cmdMEM(){};
void cmdCHECKSUM(){};
void cmdKNOB(){};
void cmdLOG(){};
void cmdHELP();
void cmdINFO(){};
void cmdPRINT(){};
void cmdPRINTLN(){};
void cmdBIN(){};


CortexClient::CortexClient() {
}

void CortexClient::setup() {
	withChecksum = false;
	cortexCommRetryCounter = 0;
	enabled = false;
	cortexWallClockLooptime = 0;
	movementDuration = 0;
	measuredBatteryVoltage = 0;
	for (int i = 0;i<NumberOfLegs;i++)
		measuredDistance[i] = 0;
	measuredOrientation.isNull();
	imuStatusSys = 0;
	imuStatusAcc = 0;
	imuStatusGyro = 0;
	timeOfLastIMUValue = 0;
	betterShutdown = false;
}


bool CortexClient::isIMUValueValid(int sinceMeasurement) {
	return ((millis()-timeOfLastIMUValue) < (unsigned)sinceMeasurement) && (imuStatusSys >= 1) && (imuStatusAcc >= 1) && (imuStatusGyro >= 1);
}

Rotation CortexClient::getIMUOrientation() {
	return measuredOrientation;
};



bool CortexClient::cmdCHECKSUM(bool onOff) {
	string cmd = "";
	CortexCommandDefinitionType* comm = CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType::CHECKSUM_CMD);

	bool ok = false;
	do {
		cmd.append(comm->name);
		if (onOff)
			cmd.append(" on");
		else
			cmd.append(" off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->timeout_ms);
		if (ok)
			withChecksum = onOff;
	} while (retry(ok));

	return ok;
}

bool CortexClient::cmdSETUP() {
	betterShutdown = false;
	return cmdBinaryCommand(Cortex::Command::SETUP);
}


bool CortexClient::cmdDISABLE() {
	return cmdBinaryCommand(Cortex::Command::DISABLE);
}

bool CortexClient::cmdENABLE() {
	return cmdBinaryCommand(Cortex::Command::ENABLE);
}


bool CortexClient::cmdMOVE(const PentaLegAngleType& legAngles, int duration_ms) {

	return cmdBinaryMOVE(legAngles, duration_ms);
}

bool CortexClient::retry(bool replyOk) {
	if ((!replyOk) && (cortexCommRetryCounter>=3))
		ROS_ERROR_STREAM(cortexCommRetryCounter << ".th failed retry. quitting");

	return ((!replyOk) && (cortexCommRetryCounter>0) && (cortexCommRetryCounter<5));
}


bool CortexClient::cmdSET(int ActuatorNo, realnum minAngle, realnum maxAngle, realnum nullAngle) {

	bool ok = false;
	string cmd;
	do {
		cmd = "";
		CortexCommandDefinitionType* comm = CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType::CONFIG_CMD);

		cmd.append(comm->name);
		cmd.append(" ");
		cmd.append(floatToString(degrees(minAngle),2));
		cmd.append(" ");
		cmd.append(floatToString(degrees(maxAngle),2));
		cmd.append(" ");
		cmd.append(floatToString(degrees(nullAngle),2));
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->timeout_ms);
	} while (retry(ok));

	if (!ok)
		ROS_ERROR_STREAM("cmdSET(" << cmd << ") failed with " << getLastError() << " " << getLastErrorMessage());

	return ok;
}

bool CortexClient::cmdGETall() {
	bool ok = cmdBinaryGetAll();
	return ok;

	ok = false;
	string responseStr;
	string cmd;
	do {
		cmd = "";
		CortexCommandDefinitionType* comm = CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType::GET_CMD);

		cmd.append(comm->name);
		cmd.append(" all");
		ok = callMicroController(cmd, responseStr, comm->timeout_ms);
	} while (retry(ok));


	std::istringstream is(responseStr);
	string token;
	std::stringstream ta;
	ta.precision(2);

	std::string out = "";
	out += ta.str() + "\n";

	// format: 	(anglehip anglethigh angleknee anglefoot) ... () (orientationx orientationy orientationxz orientationstatus)
	int imuStatus = 0;
	double angles [5][4];
	int cortexLooptime;
	realnum imuDegreeX, imuDegreeY, imuDegreeZ;

	int noOfItems = sscanf(responseStr.c_str(),"(%lf %lf %lf %lf %i) (%lf %lf %lf %lf %i) (%lf %lf %lf %lf %i) (%lf %lf %lf %lf %i) (%lf %lf %lf %lf %i) (%lf %lf %lf %i %lf %i)",
				&angles[0][0],&angles[0][1],&angles[0][2],&angles[0][3],&measuredDistance[0],
				&angles[1][0],&angles[1][1],&angles[1][2],&angles[1][3],&measuredDistance[1],
				&angles[2][0],&angles[2][1],&angles[2][2],&angles[2][3],&measuredDistance[2],
				&angles[3][0],&angles[3][1],&angles[3][2],&angles[3][3],&measuredDistance[3],
				&angles[4][0],&angles[4][1],&angles[4][2],&angles[4][3],&measuredDistance[4],
				&imuDegreeX, &imuDegreeY, &imuDegreeZ, &imuStatus, &measuredBatteryVoltage, &cortexLooptime);
	if (noOfItems != 5*5 + 6)
		setError(CORTEX_RESPONSE_NOT_PARSED);
	else {
		for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
			for (int j = 0;j<NumberOfLimbs;j++)
				lastLegAngles[legNo][j] = radians(angles[legNo][j]);
		}

		imuStatusSys  = imuStatus/100;
		imuStatusGyro = (imuStatus/10) % 10;
		imuStatusAcc  = imuStatus  % 10;
		measuredOrientation = Rotation(radians(imuDegreeX),radians(imuDegreeY),0);
		timeOfLastIMUValue = millis();
		// check distances for errors, take only valid values
		for (int i = 0;i<NumberOfLegs;i++) {
			if ((measuredDistance[i] < 0) && (measuredDistance[i] >= 200))
				measuredDistance[i] = -1;
		}


	}

	if (!ok)
		ROS_ERROR_STREAM("cmdGETAll(" << cmd << ") failed with " << getLastError() << " " << getLastErrorMessage());

	return ok;
}

bool CortexClient::readResponse(const Cortex::ResponsePackageData& response) {

	// read all variables from cortex temporarily before saving them
	Cortex::Status status;
	float angles[NumberOfLegs*NumberOfLimbs];
	uint8_t distance[NumberOfLegs];
	uint8_t servoStatus[NumberOfLimbs*NumberOfLegs];
	float imuDegreeX,imuDegreeY;
	int imuStatus;
	float voltage;
	int looptime;

	bool ok = Cortex::ComPackage::readResponse(response, status, angles, distance, servoStatus, imuDegreeX, imuDegreeY, imuStatus, voltage,looptime);
	if (ok) {
		measuredBatteryVoltage = voltage;
		cortexWallClockLooptime = looptime;
		for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
			for (int j = 0;j<NumberOfLimbs;j++)
				lastLegAngles[legNo][j] = radians(angles[j + legNo*NumberOfLimbs]);
		}

		// read IMU status and data
		imuStatusSys  = imuStatus/100;
		imuStatusGyro = (imuStatus/10) % 10;
		imuStatusAcc  = imuStatus  % 10;
		timeOfLastIMUValue = millis(); // used to check if last IMU value is recent enough

		if ((abs(imuDegreeX) > 30) || (abs(imuDegreeY) > 30)) {
			ROS_ERROR_STREAM("IMU is out of bounds=(" << imuDegreeX << "," << imuDegreeY << ")");
			imuDegreeX = 0;
			imuDegreeY = 0;
		}
		measuredOrientation = Rotation(radians(imuDegreeX),radians(imuDegreeY),0);

		// check distances for errors, take only valid values
		for (int i = 0;i<NumberOfLegs;i++) {
			measuredDistance[i] = distance[i];
			if ((measuredDistance[i] < 0) && (measuredDistance[i] >= 200))
				measuredDistance[i] = -1;
		};

		// check if one of a servo has a status error
		for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
			for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
				ServoStatusType stat = (ServoStatusType)servoStatus[NumberOfLimbs*legNo + limbNo];
				if (stat != SERVO_STAT_OK) {
					ok = false;
					betterShutdown = true;
					ROS_ERROR_STREAM("leg " << legNo << " servo " << limbNo << " failed with error " << getServoStatusTypeName(stat) << "(" << stat << ")");
				}
			}
		}

		// store general status (enabled, disabled)
		enabled = (status == Cortex::Status::ENABLED);
		ROS_DEBUG_STREAM("cortex-response"
				<< std::fixed << std::setprecision(1)
				<< " imu=(" << degrees(measuredOrientation.x) << "," << degrees(measuredOrientation.y) << "|"
				<< imuStatusSys << imuStatusGyro << imuStatusAcc << ")"
				<< " U=" << std::fixed << std::setprecision(1) << measuredBatteryVoltage << "V"
				<< " t(loop)=" << looptime << "ms"
				<< " distance=(" << measuredDistance[0] << "," << measuredDistance[1] << "," << measuredDistance[2] << "," << measuredDistance[3] << "," << measuredDistance[4] << ")"
				<< std::fixed << std::setprecision(1)
				<< " legs=(" << degrees(lastLegAngles[0][0]) << ","<< degrees(lastLegAngles[0][1]) << "," << degrees(lastLegAngles[0][2]) << ","<< degrees(lastLegAngles[0][3]) << ")"
				<< "(" << degrees(lastLegAngles[1][0]) << ","<< degrees(lastLegAngles[1][1]) << "," << degrees(lastLegAngles[1][2]) << ","<< degrees(lastLegAngles[1][3]) << ")"
				<< "(" << degrees(lastLegAngles[2][0]) << ","<< degrees(lastLegAngles[2][1]) << "," << degrees(lastLegAngles[2][2]) << ","<< degrees(lastLegAngles[2][3]) << ")"
				<< "(" << degrees(lastLegAngles[3][0]) << ","<< degrees(lastLegAngles[3][1]) << "," << degrees(lastLegAngles[3][2]) << ","<< degrees(lastLegAngles[3][3]) << ")"
				<< "(" << degrees(lastLegAngles[4][0]) << ","<< degrees(lastLegAngles[4][1]) << "," << degrees(lastLegAngles[4][2]) << ","<< degrees(lastLegAngles[4][3]) << ")");
	}
	return ok;
}


bool CortexClient::cmdBinaryGetAll() {

	bool ok = false;
	Cortex::RequestPackageData request;
	Cortex::ResponsePackageData response;

	ok = Cortex::ComPackage::createGetRequest(request);

	cortexCommRetryCounter = 0;
	do {
		ok = binaryCallMicroController(request.data, Cortex::RequestPackageData::Size, response.data, Cortex::ResponsePackageData::Size, 70,100);
	} while (retry(ok));

	if (ok)
		ok = readResponse(response);

	if (!ok)
		ROS_ERROR_STREAM("cmdBinaryGetAll failed with " << getLastError() << " " << getLastErrorMessage());
	return ok;
}

bool CortexClient::cmdBinaryMOVE(
            const PentaLegAngleType& legAngles, int duration_ms) {

    bool ok = false;
    Cortex::RequestPackageData request;
    Cortex::ResponsePackageData response;

    float flatDegAngles[NumberOfLegs*NumberOfLimbs];
    for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
        for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
        	flatDegAngles[limbNo + legNo*NumberOfLimbs] = degrees(legAngles[legNo][limbNo]);
        }
    }

    long startTime = millis();
    ok = Cortex::ComPackage::createMoveRequest(flatDegAngles, duration_ms, request);

    cortexCommRetryCounter = 0;
    do {
        ok = binaryCallMicroController(request.data, Cortex::RequestPackageData::Size, response.data, Cortex::ResponsePackageData::Size, 11, 2*CORTEX_SAMPLE_RATE);
    } while (retry(ok));

	if (ok)
		ok = readResponse(response);
    long endTime = millis();
	ROS_DEBUG_STREAM("MOVEBIN"
			<< std::fixed << std::setprecision(1)
			<< " leg0=(" << degrees(legAngles[0][0]) << ","<< degrees(legAngles[0][1]) << "," << degrees(legAngles[0][2]) << ","<< degrees(legAngles[0][3]) << ")"
			<< " leg1=(" << degrees(legAngles[1][0]) << ","<< degrees(legAngles[1][1]) << "," << degrees(legAngles[1][2]) << ","<< degrees(legAngles[1][3]) << ")"
			<< " leg2=(" << degrees(legAngles[2][0]) << ","<< degrees(legAngles[2][1]) << "," << degrees(legAngles[2][2]) << ","<< degrees(legAngles[2][3]) << ")"
			<< " leg3=(" << degrees(legAngles[3][0]) << ","<< degrees(legAngles[3][1]) << "," << degrees(legAngles[3][2]) << ","<< degrees(legAngles[3][3]) << ")"
			<< " leg4=(" << degrees(legAngles[4][0]) << ","<< degrees(legAngles[4][1]) << "," << degrees(legAngles[4][2]) << ","<< degrees(legAngles[4][3]) << ")"
			<< " duration=" << duration_ms << "ms" << " t=" << millis() << " / " << endTime-startTime);


    if (!ok)
        ROS_ERROR_STREAM("cmdBinaryGetAll failed with " << getLastError() << " " << getLastErrorMessage());

    return ok;
}

bool CortexClient::cmdBinaryCommand(Cortex::Command cmd) {
	ROS_DEBUG_STREAM("cmdBinaryCommand(cmd=" << Cortex::commandName(cmd) << ")");

	Cortex::RequestPackageData request;
	Cortex::ResponsePackageData response;

	bool ok = Cortex::ComPackage::createCommandRequest(cmd, request);
	cortexCommRetryCounter = 0;
	do {
		ok = binaryCallMicroController(request.data, Cortex::RequestPackageData::Size, response.data, Cortex::ResponsePackageData::Size, 0, 7000);
	} while (retry(ok));

	if (ok) {
		ok = readResponse(response); // returns false, if checksum error
	}

	if (!ok)
		ROS_ERROR_STREAM("cmdBinaryCommand failed with " << getLastError() << " " << getLastErrorMessage());

	return ok;
}

bool CortexClient::cmdLOGsetup(bool onOff) {

	bool ok = false;
	string cmd;
	do {
		cmd = "";
		CortexCommandDefinitionType* comm = CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType::LOG_CMD);

		cmd.append(comm->name);
		if (onOff)
			cmd.append(" setup on");
		else
			cmd.append(" setup off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->timeout_ms);
	} while (retry(ok));

	if (!ok)
		ROS_ERROR_STREAM("cmdLOGsetup(" << cmd << ") failed with " << getLastError() << " " << getLastErrorMessage());

	return ok;
}

bool CortexClient::cmdLOGtest(bool onOff) {
	bool ok = false;
	string cmd;
	do {
		cmd = "";
		CortexCommandDefinitionType* comm = CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType::LOG_CMD);

		cmd.append(comm->name);
		if (onOff)
			cmd.append(" test on");
		else
			cmd.append(" test off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->timeout_ms);
	} while (retry(ok));

	if (!ok)
		ROS_DEBUG_STREAM("cmdLOGtest(" << cmd << ") failed with " << getLastError() << " " << getLastErrorMessage());

	return ok;
}

bool CortexClient::cmdLOGservos(bool onOff) {

	bool ok = false;
	string cmd;
	do {
		cmd = "";
		CortexCommandDefinitionType* comm = CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType::LOG_CMD);

		cmd.append(comm->name);
		cmd.append(onOff?" servo on":" servo off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->timeout_ms);
	} while (retry(ok));

	if (!ok)
		ROS_ERROR_STREAM("cmdLOGservos(" << cmd << ") failed with " << getLastError() << " " << getLastErrorMessage());

	return ok;
}


bool CortexClient::cmdINFO( bool &pEnabled) {
	bool ok = false;
	string responseStr;
	string cmd;
	do {
		cmd = "";
		CortexCommandDefinitionType* comm = CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType::INFO_CMD);

		cmd.append(comm->name);

		ok = callMicroController(cmd, responseStr, comm->timeout_ms);
	} while (retry(ok));


	enabled = (responseStr.find("enabled")!=std::string::npos);

	pEnabled = enabled;
	if (!ok)
		ROS_ERROR_STREAM("cmdINFO(" << cmd << ") failed with " << getLastError() << " " << getLastErrorMessage());

	return ok;
}

bool CortexClient::cmdECHO(string s) {
	bool ok = false;

	string cmd;

	do {
		cmd = "";
		CortexCommandDefinitionType* comm = CortexCommandDefinitionType::get(CortexCommandDefinitionType::CortexCommandType::ECHO_CMD);
		cmd.append(comm->name);
		cmd.append(" ");
		cmd.append(s);
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->timeout_ms);
	} while (retry(ok));

	if (!ok)
		ROS_ERROR_STREAM("cmdECHO(" << cmd << ") failed with " << getLastError() << " " << getLastErrorMessage());

	return ok;
}


bool CortexClient::setupCortexCommunication(string i2cport, int i2cadr, string serialPort, int baudRate) {
	ROS_DEBUG_STREAM("setupCommunication(" << i2cport << ", 0x" << i2cadr << " uart=" << serialPort << "," << baudRate <<" baud )");

	// start the i2c interface
	bool ok = i2cPort.connect(i2cport, i2cadr);
	if (ok) {
		ROS_INFO_STREAM("CortexClient::i2c connect done");

		// start the i2c interface
		i2cPort.connect(i2cport, i2cadr);

		cortexConnected = true;
	}
	else {
		ROS_ERROR_STREAM("CortexClient::no i2c port found");
		setError(CORTEX_COM_FAILED);
		cortexConnected = false;
	}

	// now start command interface
	serialCmd.disconnect();

	if (cortexConnected) {
		ok = cmdSETUP();

		if (!ok) {
			ROS_ERROR_STREAM("cortex setup failed with (" << getLastError()<< ")");
		}
	}
	return ok;
}

bool CortexClient::isCortexCommunicationOk() {
	return (cortexConnected && (cortexCommRetryCounter < 5));
}

// reponse code of uC is >ok or >nok(error). Parse this, return true if ok or nok has been parsed,
// extract the remaining payload (plainReponse) and return a flag weather ok or nok has been parsed
bool CortexClient::checkReponseCode(string &s, string &plainReponse, bool &OkOrNOk) {
	resetError();

	OkOrNOk = false; // are we able to read ok or nok ?
	int startOkSearchIdx = s.length()-reponseOKStr.length();
	if ((startOkSearchIdx >= 0) && (s.compare(startOkSearchIdx, reponseOKStr.length(), reponseOKStr) == 0)) {
		plainReponse = s.substr(0,s.length()-reponseOKStr.length());
		OkOrNOk = true;
		return true;
	}

	int start = s.length()-reponseNOKStr.length()-6;
	if (start < 0)
		start = 0;
	int errorcodeIdx = s.find(reponseNOKStr,start);
	if (errorcodeIdx >= 0) {
		OkOrNOk = false;
		int errorcodeEndIdx = s.find(")",errorcodeIdx+1);
		if (errorcodeEndIdx <= errorcodeIdx)
			errorcodeEndIdx = errorcodeIdx;
		string errorStr = s.substr(errorcodeIdx+reponseNOKStr.length(),errorcodeEndIdx-errorcodeIdx-reponseNOKStr.length());
		int code = atoi(errorStr.c_str());
		setError((ErrorCodeType)code);
		plainReponse = s.substr(0,errorcodeIdx);
		return true;
	}
	return false;
}

void CortexClient::computeChecksum(string s,uint8_t& hash) {
	int c;

	int i = 0;
	while ((c = s[i++])) {
		if (c != ' ')
			hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}
}

bool  CortexClient::setupBot() {
	return cmdSETUP();
}

bool CortexClient::enableBot() {
	ROS_INFO_STREAM("enable bot");
	return cmdENABLE();
}

bool CortexClient::disableBot() {
	ROS_INFO_STREAM("disable bot");
	return cmdDISABLE();
}


bool CortexClient::info(bool &pEnabled) {
	ROS_INFO_STREAM("get bot");
	return cmdINFO(pEnabled);
}


bool CortexClient::fetchAngles(PentaLegAngleType& legAngles) {
	bool ok = cmdGETall();
	legAngles = getLegAngles();
	return ok;
}


void CortexClient::setMovement(const PentaLegAngleType& legAngles, realnum duration_ms) {
	movementDuration = max((realnum)HERKULEX_MIN_SAMPLE*2.0, duration_ms); // minimum time is 22ms, since the wall clock time of the cortex loop is approx. 16ms
	toBeAngles = legAngles;
}


bool CortexClient::moveSync(const PentaLegAngleType& legAngles, realnum duration_ms) {

	bool ok = cmdMOVE(legAngles, duration_ms);
	delay_ms(duration_ms-HERKULEX_MIN_SAMPLE);

	return ok;
}

void CortexClient::loop() {
	if (enabled) {
		string cmd = "";
		bool ok = cmdMOVE(toBeAngles, movementDuration);

		if (!ok) {
			ROS_ERROR_STREAM("sending failed (" << getLastError() << ")");
		}
	}
}

// retrieve values of distance sensors
void CortexClient::getDistanceSensor(realnum distance[NumberOfLegs]) {
	for (int i = 0;i<NumberOfLegs;i++) {
		realnum distanceIncludingError = measuredDistance[i];
		if (distanceIncludingError < 0)
			distanceIncludingError = 0;
		if ((distanceIncludingError >= 0) && (distanceIncludingError < 200))
			distance[i] = distanceIncludingError;
		else
			distance[i] = qnan;
	}
}

// retrieve values of distance sensors
realnum CortexClient::getCortexVoltage() {
	return measuredBatteryVoltage;
}


void CortexClient::sendString(string str) {
	uint8_t checksum = 0;
	computeChecksum(str, checksum);

	if (withChecksum) {
		// add checksum to string
		str +=" chk=";
		str += std::to_string(checksum); // ITOS(checksum);
	}

	serialCmd.sendString(str);
}

bool CortexClient::binaryCallMicroController(uint8_t request[], int requestSize, uint8_t response[], int responseSize, int expectedResponse, int timeout_ms) {
	resetError();

	uint32_t sendStart = millis();
    stringstream requestStream;
	for (int i = 0;i<requestSize;i++) {
		requestStream << (int)request[i] << ' ';
	}

	i2cPort.sendArray(request,requestSize);
	uint32_t sendEnd = millis();

	uint32_t receiveStart = millis();
	if (expectedResponse > 5)
		delay_ms(expectedResponse-5);
	int bytesRead = i2cPort.receiveArray(response, responseSize, timeout_ms - (sendEnd - sendStart));
	uint32_t receiveEnd= millis();

	int fullDuration = receiveEnd - sendStart;

	bool ok = true;
	if (bytesRead  == responseSize) {
	    stringstream responseStream;
		for (int i = 0;i<bytesRead;i++) {
			responseStream << (int)response[i] << ' ';
		}
	} else {
		ok = false;
		setError(CORTEX_RESPONSE_SIZE_WRONG);
		ROS_ERROR_STREAM("binaryCallMicroController:response size wrong:" << bytesRead << " instead of " << responseSize);
	}
	if ((expectedResponse > 0) && (fullDuration > expectedResponse))
		ROS_WARN_STREAM("cortex-call took too long:" << sendEnd - sendStart << "ms/" << receiveEnd - receiveStart << "ms/" << fullDuration << "ms. Expected: " << expectedResponse << "ms" );

	return ok;
}


bool CortexClient::callMicroController(string& cmd, string& response, int timeout_ms) {
	resetError();

	uint32_t now = millis();
	uint32_t duration_ms;
	// check command to identify timeout
	for (int i = 0;i<CortexCommandDefinitionType::NumberOfCommands;i++) {
		string cmdStr = string(commDef[i].name);
		if (hasPrefix(cmd,cmdStr)) {
			timeout_ms = commDef[i].timeout_ms;
			break;
		}
	}

	sendString(cmd);
	delay_ms(5);

	bool ok = receive(response, timeout_ms-5);

	replace (response.begin(), response.end(), '\r' , ' ');
	duration_ms = millis()-now;

	ROS_DEBUG_STREAM("send -> \"" << cmd << " timeout=" << timeout_ms << "-> \"" << response << "\"" << " t=" << duration_ms << " ok=" << string(ok?"true":"false") << " (" << getLastError() << ")");
	return ok;
}



bool CortexClient::receive(string& str, int timeout_ms) {

	string response="";
	string reponsePayload="";
	unsigned long startTime = millis();
	int bytesRead;
	bool replyIsOk = false; // does not necessarily mean that the answer is "ok", we only received a reply which we can read
	bool okOrNOK = false;

	// read from serial until "ok" or "nok" has been read or timeout occurs
	int retryCount= 3;// check at least three times to receive an response with a certain delay in between
	string rawResponse ="";
	bool isTimeout = false;
	do {
		bytesRead = serialCmd.receive(rawResponse);
		if (bytesRead > 0) {
			response += rawResponse;
			replyIsOk = checkReponseCode(response, reponsePayload,okOrNOK);
		} else {
			delay_us(100); // enough to transfer 64 byte
		}
		retryCount--;
		isTimeout = (millis() - startTime > (unsigned long)timeout_ms);
	}
	while (((retryCount > 0) || !isTimeout) && (!replyIsOk));


	if (replyIsOk) {
		cortexCommRetryCounter = 0; // communication was ok, reset any previous failure
		str = reponsePayload;
		if (okOrNOK) {
			ROS_DEBUG_STREAM("response \""
				<< replaceWhiteSpace(reponsePayload)
				<< "\" & " << (okOrNOK?"OK(":"NOK(") << getLastError() <<  ")");
		} else {
			ROS_DEBUG_STREAM("response \""
				<< replaceWhiteSpace(reponsePayload)
				<< "\" & " << (okOrNOK?"OK(":"NOK(") << getLastError() <<  ")");
		}
	} else {
		if (isTimeout) {
			ROS_DEBUG_STREAM("no response");
			setError(CORTEX_NO_RESPONSE);
			okOrNOK = false;
		} else {
			str = "";
			ROS_DEBUG_STREAM("response-error \"" << replaceWhiteSpace(response) << "|" << replaceWhiteSpace(reponsePayload) << "\" not parsed");
		}
		// communication received no parsable string, reset any remains in serial buffer
		serialCmd.clear();
		delay_ms(10);
		serialCmd.clear();
		cortexCommRetryCounter++;
	}

	return okOrNOK;
}
