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
	measuredLowVoltage = 0;
	measuredHighVoltage = 0;

}

void PowerVoltage::setup() {
	pinMode(POWER_HIGH_VOLTAGE_PIN, INPUT);
	pinMode(POWER_LOW_VOLTAGE_PIN, INPUT);

	analogReference(INTERNAL1V2);
	// first measurement during setup
	measuredHighVoltage = (float)analogRead(POWER_HIGH_VOLTAGE_PIN) / 1024.0 * 2.0;
	measuredLowVoltage = (float)analogRead(POWER_LOW_VOLTAGE_PIN) / 1024.0 * 2.0;

}

void PowerVoltage::loop(uint32_t now) {
	// do a voltage measurement every second
	static TimePassedBy timer;
	if (timer.isDue_ms(LOW_PRIO_LOOP_RATE_MS, now)) {
		const float TeensyReferenceVoltage = 1.2;

		// voltage divider of 10K/150K
		const float voltageDivider = 10.0/(10.0+150.0);

		// small correction factors due to inaccurate resistors has been measured manually
		measuredLowVoltage = (float)analogRead(POWER_LOW_VOLTAGE_PIN) / 1024.0 * TeensyReferenceVoltage / voltageDivider * 1.02;
		measuredHighVoltage = (float)analogRead(POWER_HIGH_VOLTAGE_PIN) / 1024.0 * TeensyReferenceVoltage / voltageDivider * 1.04;
	}
}

// supposed to return 9.5V for DRS-0101
float PowerVoltage::getLowVoltage() {
	return measuredLowVoltage;
}

// supposed to return 14.7V for DRS-0401
float PowerVoltage::getHighVoltage() {
	return measuredHighVoltage;
}

void PowerVoltage::print() {
	logger->print("voltage ");
	logger->print(measuredHighVoltage,2);
	logger->println("V,");
	logger->print(measuredLowVoltage,2);
	logger->println("V");

}

