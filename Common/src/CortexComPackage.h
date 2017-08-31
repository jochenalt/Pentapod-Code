/*
 * CortexWireInterface.h
 *
 * Definition of packages used to for binary communication between engine (Odroid) and cortex (Teensy) via I2C.
 * In general one request type and one response type is used. This call happens often in the communciation loop between ODroid and Teensy
 *
 *  Created on: 12.08.2017
 *      Author: JochenAlt
 */

#ifndef CORTEXWIREINTERFACE_H_
#define CORTEXWIREINTERFACE_H_

#include "stdint.h"
#include "core.h"

namespace Cortex {

// possible commands for cortex
enum Command { SETUP, ENABLE, DISABLE, MOVE, GET };

// return the name of a Command, i.e. SETUP returns "SETUP"
string commandName (Command cmd );

// status of cortex
enum Status { ENABLED, DISABLED };

// magic number indicates start of a valid request or response
const int MagicNumber = 165;

// magic number indicating that the teensy is not yet ready to respond. This happens in long calls only (like setup)
const int NotYetReadyMagicNumber = 166;

const int DataMagicNumberSize = 1;

// angles are stored as degrees/100, so 2 byte are sufficient for 160°
const int DataAnglesSize = 2*NumberOfLimbs*NumberOfLegs;
const int DataDurationSize = 2;
const int DataCommandSize = 1;
const int DataStatusSize = 1;
const int DataDistanceSize = NumberOfLegs;
const int DataServoStatusSize = 2*NumberOfLegs;

const int DataVoltageDataSize = 2;

// two short floats for x and y, once short for imu status
const int DataImuSize = 2 + 2 + 2;
const int DataLoopTimeSize = 1;
const int DataChecksumSize = 1;


// plain data that is communicated for requests
struct RequestPackageData {
	static const int Size  = DataMagicNumberSize + DataCommandSize + DataAnglesSize + DataDurationSize + DataChecksumSize;
	uint8_t data[Size];
};

// plain data that is communicated for responses
struct ResponsePackageData {
	static const int Size = DataMagicNumberSize + DataStatusSize + DataAnglesSize + DataDistanceSize + DataServoStatusSize + DataImuSize + DataVoltageDataSize + DataLoopTimeSize +  DataChecksumSize;
	uint8_t data[Size];
};

class ComPackage {
public:
	// create a command request package, i.e. a package containing SETUP, ENABLE, or DISABLE.
	static bool createCommandRequest(Command cmd, RequestPackageData &data);

	// create a request package of a MOVE command, i.e. defines the to-be angles of all legs and the duration the movement should take.
	static bool createMoveRequest(float angles[NumberOfLimbs*NumberOfLegs], int duration_ms, RequestPackageData &data);

	// create a request of a GET command, i.e. a request for all status information (especially current leg angles)
	static bool createGetRequest(RequestPackageData &data);

	// read a request package
	static bool readRequest(const RequestPackageData& request, Command& cmd, float angles[NumberOfLimbs*NumberOfLegs], int &duration_ms);

	// create a response package out of complete status information like leg angles, servo status etc.
	static bool createResponse(Status status,
						float angles[NumberOfLimbs*NumberOfLegs], 			// current angles of all legs in row major order
						uint8_t distance[NumberOfLegs],						// current measurement of distance sensors in the toe
						uint8_t servoStatus[NumberOfLimbs*NumberOfLegs], 	// current status of all services in row major order
						float imuX, float imuY, int imuStatus,				// current IMU status, 4-digit number, abcd (sys state, gyro state, accel state, magneto state)
						float voltage, 										// current voltage in cortex used to drive motors (ca. 10V)
						int cortexLoopTime_ms,								// current wall time of one loop in cortex
						ResponsePackageData& response 						// resulting package
						);

	// read the response package in engine and fetch all status information of cortex. Values are like above
	static bool readResponse(const ResponsePackageData& response,
						Status &status,
						float angles[NumberOfLimbs*NumberOfLegs],
						uint8_t distance[NumberOfLegs],
						uint8_t servoStatus[NumberOfLimbs*NumberOfLegs],
						float &imuX, float &imuY, int &imuStatus,
						float &voltage,
						int &cortexLoopTime_ms
					);

private:
	// small helper functions that convert a data type into single bytes and vice versa (to avoid the indian problem)
	static void shortToHiLo (int x, uint8_t & hi, uint8_t& lo);
	static void hiLoToShort(uint8_t hi, uint8_t lo, int& x);

	// a small float is a float that is between -160 and + 160 (like angles). It is stored as fixed point with two digits after the comma
	static void smallFloatToHiLo(float x, uint8_t& hi, uint8_t& lo);
	static void hiLoToSmallFloat(uint8_t hi, uint8_t lo, float& x);

	// small helper functions to add a data type to a plain byte array
	static void addSmallFloat(float x, uint8_t data[], int& idx);
	static void addShort(short x, uint8_t data[], int& idx);
	static void addByte(int8_t x, uint8_t data[], int& idx);
    static void addUnsignedByte(uint8_t x, uint8_t data[], int& idx);
    static int8_t readByte(const uint8_t data[], int&idx);
    static uint8_t readUnsignedByte(const uint8_t data[], int&idx);
    static 	short  readShort(const uint8_t data[], int&idx);
	static float readSmallFloat(const uint8_t data[], int&idx);

	// compute the checksum of an array. Last byte of a package is a checksum
	static uint8_t checksum(const uint8_t data[], int size);

	// full version of a request creation that is used internally
	static bool createRequest(Command cmd, float angles[NumberOfLimbs*NumberOfLegs], int duration_ms, RequestPackageData& request);
	static bool createRequest(Command cmd, RequestPackageData &request);
};

} // namespace cortex

#endif /* CORTEXWIREINTERFACE_H_ */
