/*
 * I2CMaster.h
 *
 * Encapsulation class for communicating with the Cortex via serial connection. Uses a portable library underneath.
 *
 * Author: JochenAlt
 */

#ifndef I2C_MASTER_H_
#define I2C_MASTER_H_

#include <string>
#include "rs232/i2c.h"
using namespace std;

class I2CMaster {
private:

public:
	I2CMaster();
	~I2CMaster();

	bool connect (string i2cname, int adr);
	void disconnect(void);
	int sendString(string str);
	int receive(string& str);
	int getArray (uint8_t *buffer, int len);
	int sendArray(uint8_t *buffer, int len);
	int receiveArray(uint8_t* buffer, int size, int timeout_ms);

	void clear();
private:
	I2CInterface i2c;
};

#endif /* SERIALPORT_H_ */
