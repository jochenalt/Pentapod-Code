/*
 * PowerVoltage.h
 *
 *  Created on: 05.06.2017
 *      Author: JochenAlt
 */

#ifndef POWERVOLTAGE_H_
#define POWERVOLTAGE_H_

#include <Arduino.h>
class PowerVoltage {
public:
	PowerVoltage();

	void setup();
	void loop(uint32_t now);
	void print();
	float getHighVoltage();
	float getLowVoltage();

private:
	float measuredLowVoltage;
	float measuredHighVoltage;

};

extern PowerVoltage voltage;

#endif /* POWERVOLTAGE_H_ */
