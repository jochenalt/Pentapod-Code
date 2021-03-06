/*
 Hekulex.h - Library for Dongbu Herkulex DRS-0101/DRS-0201 
 Copyright (c) 2012 - http://robottini.altervista.org
 Created by Alessandro on 09/12/2012.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 *****************************************************************************
    PLEASE START READING: Herkulex Servo Manual (http://www.hovis.co.kr/guide/herkulexeng.pdf)
 *****************************************************************************
 
 IMPORTANT:

  The library works on Arduino UNO/2009 - Arduino Mega.
  Please with Arduino UNO/2009 works with SoftwareSerial library modified with baud rate 57.600.
  Use this begin type:
		begin(57600, int rx, int tx);
 
  For Arduino Mega, please use baud rate 115.200

 *****************************************************************************
 Contact: alegiaco@gmail.com
 Web:     http://robottini.altervista.org
 Autor:   Alessandro Giacomel
 *****************************************************************************  
*/

#ifndef Herkulex_h
#define Herkulex_h


#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "utilities.h"

#define DATA_SIZE	 30		// buffer for input data
#define DATA_MOVE  	 50		// max 10 servos <---- change this for more servos!
#define TIME_OUT_MS  5   	// timeout serial communication

// SERVO HERKULEX COMMAND - See Manual p40
#define HEEPWRITE    0x01 	//Rom write
#define HEEPREAD     0x02 	//Rom read
#define HRAMWRITE	 0x03 	//Ram write
#define HRAMREAD	 0x04 	//Ram read
#define HIJOG		 0x05 	//Write n servo with different timing
#define HSJOG		 0x06 	//Write n servo with same time
#define HSTAT	 	 0x07 	//Read error
#define HROLLBACK	 0x08 	//Back to factory value
#define HREBOOT	 	 0x09 	//Reboot

// HERKULEX LED - See Manual p29
#define LED_GREEN 	 0x01
#define LED_BLUE     0x02
#define LED_CYAN     0x03
#define LED_RED    	 0x04
#define LED_GREEN2 	 0x05
#define LED_PINK     0x06
#define LED_WHITE    0x07

// HERKULEX STATUS ERROR - See Manual p39
#define H_STATUS_OK					 0x00
#define H_ERROR_INPUT_VOLTAGE 		 0x01
#define H_ERROR_POS_LIMIT			 0x02
#define H_ERROR_TEMPERATURE_LIMIT	 0x04
#define H_ERROR_INVALID_PKT			 0x08
#define H_ERROR_OVERLOAD			 0x10
#define H_ERROR_DRIVER_FAULT  		 0x20
#define H_ERROR_EEPREG_DISTORT		 0x40

const byte BROADCAST_ID = 0xFE;

enum ServoType {HERKULEX_DRS_0101=0, HERKULEX_DRS_0201=1, HERKULEX_DRS_0401=4, INVALID_MODEL = -1};

class HerkulexClass {
public:
  void  beginSerial(HardwareSerial* serial,long baud);
  void  end();
  
  void  initialize();
  byte  stat(int servoID);
  void  ACK(int valueACK);
  ServoType  model();
  void  set_ID(int ID_Old, int ID_New);
  void  clearError(int servoID);

  int getDistance(int servoID, int &status);
  void getDistanceRequest(int servoID);
  int getDistanceResponse(int servoID, int &status);

  void  torqueON(int servoID);
  void  torqueOFF(int servoID);
  void  changeBaudRate(int servoID, int baudNum);
  void  setAccelerationMax(int servoID, int accelerationMax);
  void  setAccelerationRatio(int servoID, int accelerationRatio);
  void  setPositionKi(int servoID, int Ki = 0);
  void  setPositionKp(int servoID, int Kp = 46);
  void  setPositionKd(int servoID, int Kp = 0);
  void  setDeadZone(int servoID, int value  = 0 );
  void  setSaturatorSlope(int servoID, int value = 0);
  void  setSaturatorOffset(int servoID, int value = 0);
  void  setPWMOffset(int servoID, int value = 0);
  void  setPositionFeedForward1stGain(int servoID, int value = 0);
  void  setPositionFeedForward2stGain(int servoID, int value = 0);

  void  moveAll(int servoID, int Goal, int iLed);
  void  moveSpeedAll(int servoID, int Goal, int iLed);
  void  moveAllAngle(int servoID, float angle, int iLed);
  void  actionAll(int pTime);
  
  void  moveSpeedOne(int servoID, int Goal, int pTime, int iLed);
  void  moveOne(int servoID, int Goal, int pTime, int iLed);
  void  moveOneAngle(ServoType type, int servoID, float angle, int pTime, int iLed);
  
  int   getPosition(ServoType type, int servoID);
  float getAngle(ServoType type, int servoID, bool &error);
  int   getPWM(int servoID);
  float getVoltage(ServoType type, int servoID);
  float getTemperature(int servoID);
  		
  void  reboot(int servoID);
  void  setLed(int servoID, int valueLed);
 
  void  writeRegistryRAM(int servoID, int address, int writeByte, int length);
  void  writeRegistryEEP(int servoID, int address, int writeByte);

  void waitTransmissionTime(int len);

// private area  
private:
  void sendPacket(uint8_t ID, int CMD, const uint8_t data[], uint8_t dataLength);
  void sendData(byte* buffer, int lenght, bool waitUntilSent= true);
  bool readData(int size);
  void addData(int GoalLSB, int GoalMSB, int set, int servoID);
  int  checksum1(byte* data, int lenghtString);
  int  checksum2(int XOR);
  void clearBuffer();
  
  int pSize;
  int pID;
  int cmd;
  int lenghtString;
  int ck1;
  int ck2;

  int conta;

  int XOR;
  int playTime;
    
  byte data[DATA_SIZE]; 
  byte dataEx[DATA_MOVE+8];
  byte moveData[DATA_MOVE];

  HardwareSerial* serial;
  int baudRate;
};

#endif    // Herkulex_h
