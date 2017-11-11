/*
 * pins.h
 *
 * Author: JochenAlt
 */


#ifndef PINS_H_
#define PINS_H_

#include "HardwareSerial.h"
#include <Arduino.h>
#include "i2c_t3-v9.1.h"


// global variables used for interfacing and for logging
extern HardwareSerial* cmdSerial;
extern HardwareSerial* logger;

// IMU PINS via I2C0 plus reset PIN
#define PIN_SDA0 16
#define PIN_SCL0 17
#define IMU_RESET_PIN 15

// one serial line is used per leg
#define PIN_RX6 47
#define PIN_TX6 48

#define PIN_RX5 34
#define PIN_TX5 33

#define PIN_RX4 31
#define PIN_TX4 32

#define PIN_RX3 7
#define PIN_TX3 8

#define PIN_RX2 9
#define PIN_TX2 10

// command line interface is Serial1
#define PIN_RX1 0
#define PIN_TX1 1

#define LED_PIN 13			// blinking LED on Teensy

// ADC of power line
#define POWER_HIGH_VOLTAGE_PIN 22
#define POWER_LOW_VOLTAGE_PIN 23

// pin for relay that turns on power to the servos
#define RELAY_PIN 39

#endif
