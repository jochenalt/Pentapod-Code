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
	measured10Voltage = 0;
	measured14Voltage = 0;

}

void PowerVoltage::setup() {
	pinMode(POWER_VOLTAGE_14V_PIN, INPUT);
	pinMode(POWER_VOLTAGE_10V_PIN, INPUT);

	analogReference(INTERNAL1V2);
	// first measurement during setup
	measured14Voltage = (float)analogRead(POWER_VOLTAGE_14V_PIN) / 1024.0 * 2.0;
	measured10Voltage = (float)analogRead(POWER_VOLTAGE_10V_PIN) / 1024.0 * 2.0;

}

void PowerVoltage::loop(uint32_t now) {
	// do a voltage measurement with 1Hz
	static TimePassedBy timer;
	if (timer.isDue_ms(LOW_PRIO_LOOP_RATE_MS, now)) {
		// voltage divider of 10K/150K, maximum reference voltage is 1.2V

		// small correction due to inprecise resistors has been measured manually
		measured10Voltage = (float)analogRead(POWER_VOLTAGE_10V_PIN) / 1024.0 * 1.2 / ( 10.0/(10.0+150.0)) * 1.02;
		measured14Voltage = (float)analogRead(POWER_VOLTAGE_14V_PIN) / 1024.0 * 1.2 / ( 10.0/(10.0+150.0)) * 1.03;

	}
}

float PowerVoltage::get10Voltage() {
	return measured10Voltage;
}

float PowerVoltage::get14Voltage() {
	return measured14Voltage;
}

void PowerVoltage::print() {
	logger->print("voltage ");
	logger->print(measured14Voltage,2);
	logger->println("V,");
	logger->print(measured10Voltage,2);
	logger->println("V");

}

