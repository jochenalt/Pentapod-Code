/* i2c-win.cpp
*/

#ifdef _WIN32

#include <iostream>
#include "string.h"
#include "i2c.h"

I2CInterface::I2CInterface() {
}

void I2CInterface::setup(std::string i2cPath /* e.g. /dev/i2c-0 */, unsigned int i2cDeviceAddress) {
}

bool I2CInterface::open(OpenMode openMode) {
	return false;
}

bool I2CInterface::writeBlock(uint8_t registerAddr, uint8_t *writeBuffer, size_t bufferSize) {
	return false;
}

bool I2CInterface::writeByte(uint8_t registerAddr, uint8_t value) {
	return false;
}

int I2CInterface::readLine(uint8_t *readBuffer, size_t bufferSize) {
	return false;
}

bool I2CInterface::close() {
	return false;
}

#endif
