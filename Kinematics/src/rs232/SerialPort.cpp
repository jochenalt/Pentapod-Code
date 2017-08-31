/*
 * SerialPort.cpp
 *
 * Author: JochenAlt
 */

#include "basics/logger.h"
#include "rs232/rs232.h"

#include "rs232/SerialPort.h"
#include "string.h"
#include <time.h>

using namespace std;


static uint32_t clock_ms() {
	static uint32_t clockPerMs = (CLOCKS_PER_SEC)/1000;
	uint32_t c = clock();
	return c/clockPerMs;
}
SerialPort::SerialPort() {
	_port = -1;
	comEnumerate();
}

SerialPort::~SerialPort() {
	comTerminate();
}

bool SerialPort::connect( string device, int baudRate) {
	_port = comFindPort(device.c_str());
	if (_port < 0 ) {
		ROS_ERROR_STREAM("port " << device << " not available.");
		for (int i = 0;i< comGetNoPorts(); i++) {
			ROS_DEBUG_STREAM("port " << i << ":" << comGetInternalName(i) << ", " << comGetPortName(i));
		}
		return false;
	}
	bool ok  = comOpen(_port, baudRate);
	if (!ok) {
		ROS_ERROR_STREAM("port " << device << " found, but connection failed");
		return false;
	}

	// clear buffer
	clear();

	return ok;
}

void SerialPort::disconnect(void) {
	if (_port >=0 ) {
		ROS_ERROR_STREAM("disconnect from serial port");
		comClose(_port);
		_port = -1;
	}
}

int SerialPort::sendArray(char *buffer, int len) {
	int bytesWritten = comWrite(_port, buffer,len);
	return bytesWritten ;
}

int SerialPort::sendArray(uint8_t *buffer, int len) {
	int bytesWritten = comWrite(_port, (char*)buffer,len);
	return bytesWritten ;
}

int SerialPort::getArray (char *buffer, int len) {
	int bytesRead= comRead(_port, buffer,len);
	return bytesRead;
}

int SerialPort::sendString(string str) {
	str += newlineStr;
	int written = sendArray((char*)str.c_str(), str.length());
	return written;
}


int SerialPort::receiveArray(uint8_t* buffer, int RemainingBufferSize, int timeout_ms) {
    int totalBytesRead = 0;
    int bytesRead= 0;
    char* charBuffer = (char*)buffer;
    uint32_t start = clock_ms();
    cout << "receiveArray len=" << RemainingBufferSize;
    do {

        bytesRead= getArray(&charBuffer[totalBytesRead], RemainingBufferSize);
        cout << "receiveArray read=" << bytesRead;
        if (bytesRead > 0) {
            totalBytesRead += bytesRead;
            RemainingBufferSize -= bytesRead;
        }
    } while ((clock_ms() < start + timeout_ms) && (RemainingBufferSize > 0));
    cout << "receiveArray read=" << totalBytesRead;

    return totalBytesRead;
}


int SerialPort::receive(string& str) {
	str = "";
	int totalBytesRead = 0;
	int bytesRead= 0;
	const int BufferSize = 512;
	char buffer[BufferSize];

	do {
		bytesRead= getArray(buffer, BufferSize);
		if (bytesRead > 0) {
			totalBytesRead += bytesRead;
			str += string(buffer, bytesRead);
		}
	} while ((bytesRead > 0) && (bytesRead == BufferSize));

	return totalBytesRead;
}

void SerialPort::clear() {
	if (_port >= 0) {
		string str;
		while (receive(str) > 0);
	}
}
