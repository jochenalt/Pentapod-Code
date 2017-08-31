/*
 * PowerVoltage.cpp
 *
 *  Created on: 05.06.2017
 *      Author: SuperJochenAlt
 */

#include <Arduino.h>
#include <PowerVoltage.h>
#include <pins.h>
#include "TimePassedBy.h"
#include <core.h>

PowerVoltage voltage;


PowerVoltage::PowerVoltage() {
	measuredVoltage = 0;
}

void PowerVoltage::setup() {
	pinMode(POWER_VOLTAGE_PIN, INPUT);
	analogReference(INTERNAL1V2);
	// first measurement during setup
	measuredVoltage = (float)analogRead(POWER_VOLTAGE_PIN) / 1024.0 * 2.0;
}

void PowerVoltage::loop(uint32_t now) {
	// do a voltage measurement with 1Hz
	static TimePassedBy timer;
	if (timer.isDue_ms(LOW_PRIO_LOOP_RATE_MS, now)) {
		// voltage divider of 10K/86K, maximum reference voltage is 1.2V
		const float calibration = 1.0;
		measuredVoltage = (float)analogRead(POWER_VOLTAGE_PIN) / 1024.0 * 1.2 / ( 10.0/(10.0+86.0)) * calibration;
	}
}

float PowerVoltage::getVoltage() {
	return measuredVoltage;
}

void PowerVoltage::print() {
	logger->print("voltage ");
	logger->print(measuredVoltage,2);
	logger->println("V");
}

