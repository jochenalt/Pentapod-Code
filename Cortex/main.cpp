#include <Arduino.h>
#include <Config.h>
#include "i2c_t3-v9.1.h"
#include "watchdog.h"
#include "PatternBlinker.h"
#include <I2CPortScanner.h>
#include <I2CSlave.h>
#include <pins.h>
#include "hostCommunication.h"
#include "Controller.h"
#include "BotMemory.h"
#include "core.h"
#include "OrientationSensor.h"
#include "PowerVoltage.h"

// global variables declared in pins.h
HardwareSerial* cmdSerial = &Serial5; 		// UART used for commands
HardwareSerial* logger = &Serial5;			// UART used to log

i2c_t3* Wires[3] = { &Wire, &Wire1, &Wire2};		// we have two I2C buses due to conflicting sensor addresses
i2c_t3* IMUWire = NULL;
i2c_t3* cortexWire = NULL;


// blinking patterns for LED on teensy board.
static uint8_t IdlePattern[2] = { 0b10000000, 0b00000000, };				// boring
static uint8_t DefaultPattern[3] = { 0b11001000, 0b00001100, 0b10000000 };	// nice!
static uint8_t LEDOnPattern[1] = { 0b11111111 };
static uint8_t LEDOffPattern[1] = { 0b00000000 };
PatternBlinker ledBlinker(LED_PIN, 100 /* ms */); // one bit in the patterns above is active for 100ms

void setCortexBoardLED(bool onOff) {
	// LEDs blinks a nice pattern during normal operations
	if (onOff)
		ledBlinker.set(LEDOnPattern,sizeof(LEDOnPattern));
	else
		ledBlinker.set(LEDOffPattern,sizeof(LEDOffPattern));
}


void setLEDPattern() {
	if (controller.isEnabled())
		ledBlinker.set(DefaultPattern,sizeof(DefaultPattern));
	else
		ledBlinker.set(IdlePattern,sizeof(IdlePattern));
}


// emergency method, that resets the I2C bus in case something went wrong (i.e. arbitration lost)
void resetI2CWhenNecessary(int ic2no) {
	if (Wires[ic2no]->status() != I2C_WAITING) {
		logger->println();

		switch(Wires[ic2no]->status())
		    {
		    case I2C_WAITING:  logger->print("I2C waiting, no errors "); break;
		    case I2C_ADDR_NAK: logger->print("Slave addr not acknowledged "); break;
		    case I2C_DATA_NAK: logger->print("Slave data not acknowledged "); break;
		    case I2C_ARB_LOST: logger->print("Bus Error: Arbitration Lost "); break;
		    case I2C_TIMEOUT:  logger->print("Bus Error: Time out "); break;
		    default:           logger->print("I2C busy "); break;
		}
		logger->print("I2C");
		logger->print(ic2no);
		logger->print(F("("));
		logger->print(Wires[ic2no]->status());
		logger->print(F(")"));

		Wires[ic2no]->resetBus();
		Wires[ic2no]->begin();
		Wires[ic2no]->setDefaultTimeout(1000);
		Wires[ic2no]->setRate(I2C_BUS_RATE);

		logger->print(F(" stat="));
		logger->println(Wires[ic2no]->status());
	}
}

uint8_t getRXPin(uint8_t serialId) {
	static uint8_t RxPins[6] = { 0,9,7,31, 34,47 };
	return RxPins[serialId-1];
}

uint8_t getTXPin(uint8_t serialId) {
	static uint8_t TxPins[6] = { 1,10,8,32, 35,48};
	return TxPins[serialId-1];
}

void logPinAssignment() {
	if (memory.persMem.logSetup) {
		logger->println("--- pin assignment");
		logger->print("PCB LED             = ");
		logger->println(LED_PIN);

		logger->print("Relay PIN           = ");
		logger->println(RELAY_PIN);

		logger->print("VDD 14V_AD PIN      = ");
		logger->println(POWER_HIGH_VOLTAGE_PIN);
		logger->print("VDD 10V AD PIN      = ");
		logger->println(POWER_LOW_VOLTAGE_PIN);

		logger->print("Cmd      RX,TX      = (");
		logger->print(getRXPin(1));
		logger->print(",");
		logger->print(getTXPin(1));
		logger->println(")");
		logger->print("Logger   RX,TX      = (");
		logger->print(getRXPin(1));
		logger->print(",");
		logger->print(getTXPin(1));
		logger->println(")");

		logger->print("IMU      SCL,SDA    = (");
		logger->print(PIN_SCL0);
		logger->print(",");
		logger->print(PIN_SDA0);
		logger->println(")");


		for (int i = 0;i<NUMBER_OF_LEGS;i++) {
			logger->print("leg ");
			logger->print(i);
			logger->print("    RX");
			logger->print(memory.persMem.legs[i].serialId);
			logger->print(" TX");
			logger->print(memory.persMem.legs[i].serialId);
			logger->print("    = (");
			logger->print(getRXPin(memory.persMem.legs[i].serialId));
			logger->print(",");
			logger->print(getTXPin(memory.persMem.legs[i].serialId));
			logger->println(")");
		}
	}
}

// when the servos are in a weired state and need to be reset with the relay, in some cases, the current on the
// serial lines is sufficient to keep them in a weired state. So set everything to low when servos are switched off.
// servos are using Serial6, Serial1, Serial4, Serial3, Serial
void setSetSerialServoLinesLow() {
	pinMode(PIN_RX6, OUTPUT);
	pinMode(PIN_TX6, OUTPUT);
	pinMode(PIN_RX6, LOW);
	pinMode(PIN_TX6, LOW);

	pinMode(PIN_RX1, OUTPUT);
	pinMode(PIN_TX1, OUTPUT);
	pinMode(PIN_RX1, LOW);
	pinMode(PIN_TX1, LOW);

	pinMode(PIN_RX4, OUTPUT);
	pinMode(PIN_TX4, OUTPUT);
	pinMode(PIN_RX4, LOW);
	pinMode(PIN_TX4, LOW);

	pinMode(PIN_RX3, OUTPUT);
	pinMode(PIN_TX3, OUTPUT);
	pinMode(PIN_RX3, LOW);
	pinMode(PIN_TX3, LOW);

	pinMode(PIN_RX2, OUTPUT);
	pinMode(PIN_TX2, OUTPUT);
	pinMode(PIN_RX2, LOW);
	pinMode(PIN_TX2, LOW);
}


void headlessSetup() {
	logger->println("SETUP: starting headless");
	// switch servos off and on and wait in between. This pulls 2A, so do not to anything else at this time
	pinMode(RELAY_PIN, OUTPUT);
	digitalWrite(RELAY_PIN, LOW);
	setSetSerialServoLinesLow();

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);
	delay(50);
	digitalWrite(LED_PIN, LOW);


	// RST Pin on IMU only used when ecplicitely reset
	// orientationSensor.reset();
	// pinMode(IMU_RESET_PIN, INPUT);

	// initialize I2C for IMU
	logger->println("IMU: initializing");

	// measure start time since we need to give IMU one second for settling, meanwhile we can do something else
	uint32_t setupStart = millis();
	orientationSensor.reset(); 	// reset IMU via reset PIN
	pinMode(IMU_RESET_PIN, INPUT);
	IMUWire = &Wire;
	IMUWire->begin(I2C_MASTER, 0, I2C_PINS_16_17, I2C_PULLUP_INT, I2C_RATE_800);
	IMUWire->setDefaultTimeout(4000); // 4ms default timeout
	IMUWire->resetBus();
	orientationSensor.setup(IMUWire); // this takes up to 1000ms, depending on IMU state!

	// wait 1500ms to give servos time to switch really off
	unsigned int waitWithServosTurnedOff = 1500;
	logger->print("SETUP: waiting ");
	logger->print(waitWithServosTurnedOff - (millis() - setupStart));
	logger->println("ms more with servos turned off");

	// wait until IMU and servco has been settling 1500 ms
	while (millis() - setupStart < waitWithServosTurnedOff)
		delay(10);

	// switch on servo power via relay
	logger->println("SETUP: switching servos on");
	digitalWrite(RELAY_PIN, HIGH);
	uint32_t servoOnTime= millis();

	// load config data from EEPROM
	memory.setup();

	// setup command line
	hostComm.setup();

	memory.persMem.logSetup = true;

	// start blinking
	ledBlinker.set(DefaultPattern, sizeof(DefaultPattern));

	// setup power line checker
	voltage.setup();

	// tell me the pins
	logPinAssignment();

	// wait at least 500ms to give servos time to switch really on
	// before starting setup controller setup
	logger->print("SETUP: waiting ");
	logger->print(500 - (millis() - servoOnTime));
	logger->println(" ms more with servos turned on");
	while (millis() - servoOnTime < 500)
		delay(50);

	// now IMU had enough time to settle, read calibration and first (rubbish) data
	orientationSensor.updateCalibration();
	orientationSensor.fetchData();

	// controller for all serial lines with the servos behind
	controller.setup();

	// for the IMU
	delay(100);
}

void setup() {
	// switch on servos
	pinMode(RELAY_PIN, OUTPUT);
	digitalWrite(RELAY_PIN, HIGH);

	// establish serial output (logging)
	cmdSerial->begin(CORTEX_CLI_SERIAL_BAUDRATE);
	cmdSerial->println("hello");
	// establish I2C interface (commands from ODroid)
	cortexWire = &Wire1;
	cortexWire->begin(I2C_SLAVE, 0x08,0, I2C_PINS_37_38, I2C_PULLUP_EXT, I2C_RATE_400);
	cortexWire->setDefaultTimeout(4000); // 4ms default timeout
	i2cSlave.setup(cortexWire);

	// headless setup
	headlessSetup();

	// ready for input
	cmdSerial->print(F(">"));
}


void loop() {
	watchdogReset();

	// high priority jobs
	uint32_t now = millis();
	i2cSlave.loop();			// loop is for receiving commands
	controller.loop(now);		// run the actuators
	hostComm.loop();			// wait for commands via serial interface
	orientationSensor.loop(now);// fetch orientation from IMU

	// low priority jobs
	ledBlinker.loop(now);    	// LED on Teensy board and LED on power switch
}
