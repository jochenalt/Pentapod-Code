/*
 * I2CMaster.cpp
 *
 * Class to communicate with cortex via I2C
 *
 * Author: JochenAlt
 */

#include "string.h"
#include <time.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "core.h"
#include "I2CMaster.h"
#include "basics/logger.h"
#include "CortexComPackage.h"

using namespace std;


static uint32_t clock_ms() {
	static uint32_t clockPerMs = (CLOCKS_PER_SEC)/1000;
	uint32_t c = clock();
	return c/clockPerMs;
}

I2CMaster::I2CMaster() {
}

I2CMaster::~I2CMaster() {
}

bool I2CMaster::connect(string i2cName, int i2cAdr) {
	i2c.setup(i2cName, i2cAdr);
	bool ok = i2c.open( I2CInterface::ReadWrite );
	return ok;
}

void I2CMaster::disconnect(void) {
}

int I2CMaster::sendArray(uint8_t *buffer, int len) {
	// split the block into 32 byte blocks
	int numberOfBlocks = (len+31)/32;
	int positionInBuffer = 0;
	int remainingLen = len;
	// send all blocks, register marks the number of blocks
	for (int blockNo = 0;blockNo<numberOfBlocks;blockNo++) {
		int blockSize = remainingLen;
		if (blockSize > 32)
			blockSize = 32;

		// adress is used to indicate the total number and sequence number of of 32-byte blocks
		int adr = (numberOfBlocks << 4) + blockNo;
		i2c.writeBlock(adr,&(buffer[positionInBuffer]), blockSize);
		remainingLen -= blockSize;
		positionInBuffer  += blockSize;
	}
	// closing byte (any value). Otherwise the message is not received by the cortex
	// but with the next invokation. Dont know why.
	i2c.writeByte(255,0);

	return len;
}


int I2CMaster::getArray (uint8_t *buffer, int len) {
	i2c.writeByte(255,0); // request status
	int bytesRead = i2c.readLine(buffer, len);
	return bytesRead;
}

int I2CMaster::sendString(string str) {
	int written = sendArray((uint8_t*)str.c_str(), str.length());
	return written;
}


int I2CMaster::receiveArray(uint8_t* buffer, int RemainingBufferSize, int timeout_ms) {
    int totalBytesRead = 0;
    int bytesRead= 0;
    uint32_t start = clock_ms();
    do {
    	bytesRead= getArray(&buffer[totalBytesRead], RemainingBufferSize);
    	if ((bytesRead > 0) && (buffer[totalBytesRead] ==  Cortex::NotYetReadyMagicNumber)) {
        	bytesRead = 0;
        	// ROS_DEBUG_STREAM ("receive_array" << bytesRead <<i2CPort::receive({" << str.str() << "}, len=" << totalBytesRead << ")");
        	std::this_thread::sleep_for(std::chrono::milliseconds(1));
    	}
    	if (bytesRead > 0) {
            totalBytesRead += bytesRead;
            RemainingBufferSize -= bytesRead;
        }
    } while ((clock_ms() < start + timeout_ms) && (RemainingBufferSize > 0));

    return totalBytesRead;
}


int I2CMaster::receive(string& str) {
	const int bufferSize = 512;
	char buffer[bufferSize];

	int len = receiveArray((uint8_t*)buffer, bufferSize, 1000);
	string s (buffer);
	str = s;
	return len;
}

void I2CMaster::clear() {
	i2c.close();
}
