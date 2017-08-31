/*
 * CortexWireInterface.cpp
 *
 *  Created on: 12.08.2017
 *      Author: JochenAlt
 */

#include "CortexComPackage.h"


string Cortex::commandName (Command cmd ) {
	switch (cmd) {
		case SETUP: return "SETUP";
		case ENABLE: return "ENABLE";
		case DISABLE: return "DISABLE";
		case MOVE: return "MOVE";
		case GET: return "GET";
	}
	return "";
}


void Cortex::ComPackage::shortToHiLo (int x, uint8_t & hi, uint8_t& lo) {
	int p = (int)(x) + (1<<15);
	lo = p & 0xFF;
	hi = (p >> 8) & 0xFF;
}

void Cortex::ComPackage::hiLoToShort(uint8_t hi, uint8_t lo, int& x) {
	x = lo + (hi << 8);
	x -= (1<<15);
}

void Cortex::ComPackage::smallFloatToHiLo(float x, uint8_t& hi, uint8_t& lo) {
	shortToHiLo(x*100.0, hi,lo);
}

void Cortex::ComPackage::hiLoToSmallFloat(uint8_t hi, uint8_t lo, float& x) {
	int i;
	hiLoToShort(hi,lo,i);
	x = ((float)i)/100.0;
}

void Cortex::ComPackage::addSmallFloat(float x, uint8_t data[], int& idx) {
	uint8_t hi,lo;
	smallFloatToHiLo(x, hi,lo);

	data[idx++] = hi;
	data[idx++] = lo;
}
void Cortex::ComPackage::addShort(short x, uint8_t data[], int& idx) {
	uint8_t hi;
	uint8_t lo;
	shortToHiLo(x, hi,lo);

	data[idx++] = hi;
	data[idx++] = lo;
}
void Cortex::ComPackage::addByte(int8_t x, uint8_t data[], int& idx) {
	data[idx++] = x;
}
void Cortex::ComPackage::addUnsignedByte(uint8_t x, uint8_t data[], int& idx) {
	data[idx++] = x;
}

int8_t Cortex::ComPackage::readByte(const uint8_t data[], int&idx) {
	return data[idx++];
}
uint8_t Cortex::ComPackage::readUnsignedByte(const uint8_t data[], int&idx) {
	return data[idx++];
}

short  Cortex::ComPackage::readShort(const uint8_t data[], int&idx) {
	uint8_t hi = data[idx++];
	uint8_t lo = data[idx++];
	int x;
	hiLoToShort(hi,lo,x);
	return x;
}

float Cortex::ComPackage::readSmallFloat(const uint8_t data[], int&idx) {
	uint8_t hi = data[idx++];
	uint8_t lo = data[idx++];
	float x;
	hiLoToSmallFloat(hi,lo,x);
	return x;
}

uint8_t Cortex::ComPackage::checksum(const uint8_t data[], int size) {
	uint8_t sum = 0xFE;
	for (int i = 0;i<size-1;i++)
		sum ^= data[i];
	return sum;
}

bool Cortex::ComPackage::createCommandRequest(Command cmd, RequestPackageData &data) {
	return createRequest(cmd, data);
}

bool Cortex::ComPackage::createMoveRequest(float angles[NumberOfLimbs*NumberOfLegs], int duration_ms, RequestPackageData &data) {
	return createRequest(MOVE, angles, duration_ms, data);
}

bool Cortex::ComPackage::createGetRequest(RequestPackageData &data) {
	return createRequest(GET, data);
}

bool Cortex::ComPackage::createRequest(Command cmd, float angles[NumberOfLimbs*NumberOfLegs], int duration, RequestPackageData& request) {
	int idx = 0;

	// write magic number
	addUnsignedByte(MagicNumber, request.data, idx);

	// write command
	addByte(cmd, request.data, idx);

	// write angles
	for (int i = 0;i<NumberOfLimbs*NumberOfLegs;i++)
		addSmallFloat(angles[i], request.data, idx);

	// write duration
	addShort(duration, request.data, idx);

	// write checksum
	uint8_t sum = checksum(request.data, RequestPackageData::Size);
	addUnsignedByte(sum, request.data, idx);
	if (idx != RequestPackageData::Size) {
		setError(ErrorCodeType::CORTEX_PACKAGE_SIZE_ERROR);
		return false;
	}
	return true;
}

bool Cortex::ComPackage::readRequest(const RequestPackageData& request, Command& cmd, float angles[NumberOfLimbs*NumberOfLegs], int &duration_ms) {
	int idx = 0;

	// read magic number
	uint8_t magicNumber= readUnsignedByte(request.data, idx);
	if (magicNumber != MagicNumber) {
		setError(ErrorCodeType::CORTEX_WRONG_MAGIC);
		return false;
	}

	// read command
	cmd = (Command)readByte(request.data, idx);

	// read angles
	for (int i = 0;i<NumberOfLimbs*NumberOfLegs;i++) {
		angles[i]= readSmallFloat(request.data, idx);
	}

	// read duration
	duration_ms = readShort(request.data, idx);

	// read  checksum of data
	uint8_t sum = readUnsignedByte(request.data, idx);
	uint8_t computedSum = checksum(request.data, RequestPackageData::Size);
	if (computedSum != sum) {
		setError(CHECKSUM_WRONG);
		return false;
	}

	if (idx != RequestPackageData::Size) {
		setError(ErrorCodeType::CORTEX_PACKAGE_SIZE_ERROR);
		return false;
	}
	return true;
}

bool Cortex::ComPackage::createResponse(Status status, float angles[NumberOfLimbs*NumberOfLegs],
					uint8_t distance[NumberOfLegs],
					uint8_t servoStatus[NumberOfLimbs*NumberOfLegs],
					float imuX, float imuY, int imuStatus,
					float voltage, int cortexLoopTime_ms,
					ResponsePackageData& response ) {
	int idx = 0;

	// write magic number
	addUnsignedByte(MagicNumber, response.data, idx);

	// write status
	addByte(status, response.data, idx);

	// write angles
	for (int i = 0;i<NumberOfLimbs*NumberOfLegs;i++)
		addSmallFloat(angles[i], response.data, idx);

    // write distances
    for (int i = 0;i<NumberOfLegs;i++)
        addUnsignedByte(distance[i], response.data, idx);

    // write servo status
    for (int i = 0;i<NumberOfLegs;i++) {
        int status = servoStatus[i*NumberOfLimbs] +
                      (servoStatus[i*NumberOfLimbs+1] << 4);
        addByte(status, response.data, idx);
        status =      servoStatus[i*NumberOfLimbs+2] +
                      (servoStatus[i*NumberOfLimbs+3] << 4);
        addByte(status, response.data, idx);
    }

	// write IMU
	addSmallFloat(imuX, response.data, idx);
	addSmallFloat(imuY, response.data, idx);
	addShort(imuStatus, response.data, idx);

	// write voltage of cortex
	addSmallFloat(voltage, response.data, idx);

	// write duration of cortex loop
	addUnsignedByte(cortexLoopTime_ms, response.data, idx);

	// write checksum
	uint8_t sum = checksum(response.data, ResponsePackageData::Size);
	addUnsignedByte(sum, response.data, idx);

	if (idx != ResponsePackageData::Size) {
		setError(ErrorCodeType::CORTEX_PACKAGE_SIZE_ERROR);
		return false;
	}
	return true;
}

bool Cortex::ComPackage::readResponse(const ResponsePackageData& response, Status &status, float angles[NumberOfLimbs*NumberOfLegs],
				  uint8_t distance[NumberOfLegs],
				  uint8_t servoStatus[NumberOfLimbs*NumberOfLegs],
				  float &imuX, float &imuY, int &imuStatus,
				  float &voltage, int &cortexLoopTime_ms
				) {

	// read status
	int idx = 0;

	// read magic number
	uint8_t magicNumber = readUnsignedByte( response.data, idx);
	if (magicNumber != MagicNumber) {
		setError(ErrorCodeType::CORTEX_WRONG_MAGIC);
		return false;
	}

	status = (Status)readByte(response.data, idx);

	// read angles
	for (int i = 0;i<NumberOfLimbs*NumberOfLegs;i++) {
		angles[i]= readSmallFloat(response.data, idx);
	}

	// read distances
	for (int i = 0;i<NumberOfLegs;i++) {
        distance[i]= readUnsignedByte(response.data, idx);
	}

	// read servo status
    for (int i = 0;i<NumberOfLegs;i++) {
        int status = readByte(response.data, idx);
        servoStatus[i*NumberOfLimbs] = status & 15;
        servoStatus[i*NumberOfLimbs+1] = status >> 4;
        status = readByte(response.data, idx);
        servoStatus[i*NumberOfLimbs+2] = status & 15;
        servoStatus[i*NumberOfLimbs+3] = status >> 4;
    }


	// read IMU
	imuX = readSmallFloat(response.data, idx);
	imuY = readSmallFloat(response.data, idx);
	imuStatus = readShort(response.data, idx);

	// read voltage
	voltage = readSmallFloat(response.data, idx);

	// read duration of one loop
	cortexLoopTime_ms = readUnsignedByte(response.data, idx);

	// check checksum of data
	uint8_t sum = readUnsignedByte(response.data, idx);
	uint8_t computedSum = checksum(response.data, ResponsePackageData::Size);
	if (computedSum != sum) {
		setError(CHECKSUM_WRONG);
		return false;
	}

	if (idx != ResponsePackageData::Size) {
		setError(ErrorCodeType::CORTEX_PACKAGE_SIZE_ERROR);
		return false;
	}
	return true;
}

bool Cortex::ComPackage::createRequest(Command cmd, RequestPackageData &request) {
	float emptyAngles[NumberOfLimbs*NumberOfLegs] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	return createRequest(cmd, emptyAngles,0,  request);
}
