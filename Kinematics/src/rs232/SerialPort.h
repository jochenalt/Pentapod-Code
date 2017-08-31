/*
 * SerialPort.h
 *
 * Encapsulation class for communicating with the Cortex via serial connection. Uses a portable library underneath.
 *
 * Author: JochenAlt
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <string>

using namespace std;

const string newlineStr = "\r";

class SerialPort {
private:

public:
	SerialPort();
	~SerialPort();

	bool connect (string device, int baudRate);
	void disconnect(void);
	int sendString(string str);
	int receive(string& str);
	int sendArray(uint8_t *buffer, int len);
	int receiveArray(uint8_t* buffer, int size, int timeout_ms);

	void clear();
private:
	int sendArray(char *buffer, int len);
	int getArray (char *buffer, int len);

	int _port;
};

#endif /* SERIALPORT_H_ */
