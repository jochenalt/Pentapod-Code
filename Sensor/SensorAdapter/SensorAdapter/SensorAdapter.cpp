/*
 * SensorAdapter.cpp
 * 
 * Adapter board that makes a Adafruit VL6180X sensor behave like a
 * Herkulex Servo. Each status call returns the distance the sensor detects.
 * Allows to integrate the distance sensor into the daisy chain bus of Herkulex Servos.
 *
 * Created: 21.05.2017 17:54:39
 *  Author: JochenAlt
 */ 


#include <Arduino.h>
#include <avr/wdt.h>

#include "HardwareSerial.h"
#include "Adafruit_VL6180X.h"
#include "PatternBlinker.h"

#define BAUD_RATE (115200)

#define PACKETSIZE 7UL
#define TIME_OUT_MS ((uint8_t)(1 + ((2UL*1000000UL/(BAUD_RATE/10UL)*PACKETSIZE)/1000UL)))
#define HERKULEX_SERVO_ID 200
#define SENSOR_CMD_REQUEST_DISTANCE  200
#define SENSOR_CMD_SEND_DISTANCE  201

#define LED_PIN PIN_C0

// the serial line is used as a daisy chain bus.
// So, we wait for a packet which is determined to us
// (indicated by a Herkulex servo ID of 200)
// then take over the serial bus by using the TX line
// After transmission the TX line set to tr-state to 
// let the other servos to send their replies.
#define TX_PIN PIN_D1
#define RX_PIN PIN_D0

const uint8_t outputCmdSize= 9;		// size of output packet
uint8_t outputArray[outputCmdSize];
uint8_t inputBuffer[64]; 

Adafruit_VL6180X vl6180x;
int distance = 0;
int status = 0;


void clearBuffer(uint8_t buffer[], int size) {
	for (int i = 0;i<size;i++) {
		buffer[i] = 0;
	}
}

// try to read a block of size from serial. Returns true if successful within time out
int readData(int size, uint8_t* buffer)
{
	bool beginToSave=false; 
	uint16_t timeElapsed_us=0;

	while((Serial.available() < size) && (timeElapsed_us<= ((1+TIME_OUT_MS*1000)))) {
		timeElapsed_us += 100;
		delayMicroseconds(100);
	}
	int bufferPos = 0;

	while ((Serial.available() > 0) && (bufferPos < size)) {
		int inchar = Serial.read();

		if ( (inchar == 0xFF) & ((byte)Serial.peek() == 0xFF) ){
			beginToSave=true;
			bufferPos=0;
		}
		if (beginToSave && bufferPos<size) {
			buffer[bufferPos] = inchar;
			bufferPos++;
		}
	}

	return (bufferPos);
}


/**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 */
int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN));			// Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif
  pinMode(SDA, INPUT_PULLUP);	// Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(100);					// enforce timeout of sensor

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
	// Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) {		// still low after 2 sec error
      return 2;			// I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) {		// still low
    return 3;			// I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}


void blink(int no) {
	for (int i = 0;i<no;i++) {
		digitalWrite(LED_PIN, LOW);
		delay(100);
		digitalWrite(LED_PIN, HIGH);
		delay(100);
	}
}

// clears the I2C bus and establish the communication to the sensor
// if something goes wrong, blink
void establishSensorCommunication() {
	int clearBusResult = I2C_ClearBus();
	if (clearBusResult == 0) {
		bool ok = vl6180x.begin();	// initialize distance sensor
		if (!ok) {
			blink(2);
			delay(1000); // wait for reset to restart
		}
	} else {
		blink(1);
		delay(1000); // wait for reset to restart
	}
}

// clear buffer (throw away content) and wait 200us
void clearSerialBuffer() {
	
	Serial.flush(); // wait actively until send buffer is empty
	while (Serial.available()){
		Serial.read();
		uint32_t start = micros();
		
		// if a character is available wait some time if another character is coming
		// (give it necessary time to send two characters)
		while ((micros() - start) >= (2L*1000000L/(BAUD_RATE/10))) {
			delayMicroseconds(10);
		}
	}
}

int computeChecksum1(int size, int id, int cmd, byte* data, int lenghtString)
{
	int sum = 0;
	sum = sum ^ size;
	sum = sum ^ id;
	sum = sum ^ cmd;
	for (int i = 0; i < lenghtString; i++)
	{
		sum = sum ^ data[i];
	}
	return sum&0xFE;
}


// checksum2
int computeChecksum2(int checksum1)
{
	return (~checksum1)&0xFE;
}

void setup() {
	wdt_enable(WDTO_1S);
	pinMode(TX_PIN,INPUT);			// switch off transmission pin to not confuse the other servos
	pinMode(RX_PIN,INPUT);			// switch off transmission pin to not confuse the other servos

	pinMode(LED_PIN,OUTPUT);
	digitalWrite(LED_PIN, HIGH);	// LED is off

	Serial.begin(BAUD_RATE);
	Serial.turnTXOff();

	// initialize Wire interface and start sensor
	delay(50);
	establishSensorCommunication();
	delay(50);
	
	distance = vl6180x.readRange();
	status = vl6180x.readRangeStatus();
}

bool checkForCommand() {
	const uint8_t requestDistancePacketSize= 7;
	bool commandRead = false;
	if (Serial.available()) {
		int bytesRead= readData(requestDistancePacketSize, &inputBuffer[0]);

		// if there's still more characters than requested, clear the buffer
		if (Serial.available())
			clearSerialBuffer();

		// check if inputArray represents a command for me
		// packetSend[0] = 0xFF; // Header
		// packetSend[1] = 0xFF; // Header
		// packetSend[2] = 7+dataLength; // Packet Size
		// packetSend[3] = ID; // ID of the servo
		// packetSend[4] = CMD; // Instruction
		// packetSend[5] = checksum1(packetSend, dataLength); // Checksum1
		// packetSend[6] = checksum2(packetSend[5]); // Checksum2

		uint8_t dataLength = inputBuffer[2];
		uint8_t servoID = inputBuffer[3];
		uint8_t cmd = inputBuffer[4];
		uint8_t chk1 = inputBuffer[5];
		uint8_t chk2 = inputBuffer[6];
		int checkChecksum1=computeChecksum1(dataLength, servoID, cmd, NULL,0);
		int checkChecksum2=computeChecksum2(checkChecksum1);

		// 	check if packet is valid and for me
		if ((bytesRead == requestDistancePacketSize) && 
		    (inputBuffer[0] == 0xFF) &&  (inputBuffer[1] == 0xFF) && 
			(dataLength == requestDistancePacketSize)  && 
			(servoID == HERKULEX_SERVO_ID) && 
			(cmd == SENSOR_CMD_REQUEST_DISTANCE) &&
			(chk1 == checkChecksum1) && (chk2 == checkChecksum2)) {

			// switch LED on, we receive a request ( it is switched off, when a new sensor valid is read)
			// So, if LED blinks fast and with low average brighness the better, since 
			// then time between reading a sensor value and sending it is very small.
			digitalWrite(LED_PIN, LOW);
			commandRead = true;

			// packet is ok, take over bus for a response
			Serial.turnTXOn();
			pinMode(TX_PIN,OUTPUT);

			// Dont read the sensor, but take the value of the last sensor read
			uint8_t reponsePacketSize = 9;
			outputArray[0] = 0xFF;			// Packet Header
			outputArray[1] = 0xFF;			// Packet Header
			outputArray[2] = reponsePacketSize;
			outputArray[3] = HERKULEX_SERVO_ID;
			outputArray[4] = SENSOR_CMD_SEND_DISTANCE;
			outputArray[7] = distance;
			outputArray[8] = status;
			outputArray[5] = computeChecksum1(reponsePacketSize, HERKULEX_SERVO_ID, SENSOR_CMD_SEND_DISTANCE, &outputArray[7], 2);
			outputArray[6] = computeChecksum2(outputArray[5]);

			// take over serial daisy chain bus and write response
			Serial.write(outputArray, reponsePacketSize);
			Serial.flush();			// wait until transmission is done

			// transmission is done, leave the bus to the other participants
			Serial.turnTXOff();
			pinMode(TX_PIN,INPUT);	// let others control the TX line
		}
	}
	return commandRead;
}

void loop() {
	uint32_t now = millis();

	wdt_reset();
	
	// check for a Herkulex command packet that we can use
	bool cmdReceived = checkForCommand(); // check for this packet and send sensor values

	static uint32_t lastCommandTime = now;	
	static uint32_t averagesSampleRate = 56; // supposed Herkulex servo rate
	uint32_t nextSensorCall = 0;
	
	// read new sensor right before the next periodic call.
	// (reading the sensor all the time disturbes the serial line, dont know why yet)
	if (cmdReceived) {
		averagesSampleRate = (7*averagesSampleRate + (now - lastCommandTime)) /8;
		nextSensorCall = now + averagesSampleRate;
		lastCommandTime = now;
	}

	// ask sensor 4ms before next assumed call	
	// or 
	// after one second with no command ask the sensor anyway
	if (( nextSensorCall > now - 4 ) || (now - lastCommandTime > 1000 )) {

		distance = vl6180x.readRange();
		status = vl6180x.readRangeStatus();
		nextSensorCall = 0;
		
		if (status == VL6180X_ERROR_NONE) {
			 // switch LED off, we got a valid reply from sensor
			 // if we are not able to get a sensor value, LED remains on
			 digitalWrite(LED_PIN, HIGH);
		}	

		if ((status != VL6180X_ERROR_NONE) &&
			(status != VL6180X_ERROR_RAWUFLOW) &&
			(status != VL6180X_ERROR_RAWOFLOW) &&
			(status != VL6180X_ERROR_ECEFAIL) &&
			(status != VL6180X_ERROR_NOCONVERGE) &&
			(status != VL6180X_ERROR_RANGEIGNORE) &&
			(status != VL6180X_ERROR_SNR) &&
			(status != VL6180X_ERROR_RAWUFLOW) &&
			(status != VL6180X_ERROR_RAWOFLOW)) {
								
			// status is no known status, re-initialize Wire connection
			digitalWrite(LED_PIN, LOW); // switch LED on in case something goes wrong it blinks 
			establishSensorCommunication();
		}
	}
}