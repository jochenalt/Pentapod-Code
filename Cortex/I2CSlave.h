/*
 * I2CSlave.h
 *
 * Recipient for commands via I2C. Able to sum up several I2C blocks into a bigger data chunk
 * Interprets the commands (like Setup, get legs angles, or move) coming in, executes them and sends back a response
 *  Created on: 14.08.2017
 *      Author: JochenAlt
 */

#ifndef I2CSLAVE_H_
#define I2CSLAVE_H_

#include "i2c_t3-v9.1.h"
#include "CortexComPackage.h"

class I2CSlave {
public:
	I2CSlave() {};
	virtual ~I2CSlave() {};

	// only one I2C line is managed
	static I2CSlave& getInstance();

	void setup(i2c_t3* wire);
	bool loop();

private:
	// if a request is available, carry it out
	void executeRequest();

	// called when all i2c blocks are concatenated to a complete array
	void onBlockReceive(size_t howManyBytes);

	// called as interrupt when a i2c request comes in
	void onRequest();

	// our i2c line to cortex
	i2c_t3* wire = NULL;

	// buffer to concatenate multiple requests to one big data array
	// (i2c has a maximum size of 32 bytes per request)
	static const unsigned int maxBufferSize = 512;
	uint8_t receiveBuffer[maxBufferSize];
	unsigned int receiveBufferLen;

	// semaphor set in onRequest indicating that a request as been received and is ready to be executed in the main loop
	// (within a interrupt, no big tasks should be executed)
	bool requestPending;
	Cortex::RequestPackageData request;

	// semanphor set in onReceive, indicating that a response has been created that can be sent to the master
	bool responsePending;
	Cortex::ResponsePackageData response;

};

extern I2CSlave i2cSlave;

#endif /* I2CSLAVE_H_ */
