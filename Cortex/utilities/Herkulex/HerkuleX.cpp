/*
  Hekulex.cpp - Library for Dongbu Herkulex DRS-0101/DRS-0201
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
#include "Herkulex.h"

extern HardwareSerial* logger;;

// wait until characters have been sent
void HerkulexClass::waitTransmissionTime(int len) {
	delayMicroseconds((1000000/(baudRate/10))*len);
}


void HerkulexClass::beginSerial(HardwareSerial* pSerial, long baud)
{
	baudRate = baud;
	serial = pSerial;
	serial->begin(baud);
}

// Herkulex end
void HerkulexClass::end()
{
	serial->end();
}

// initialize servos
void HerkulexClass::initialize()
{
	
        conta=0;
		lenghtString=0;
		delay(100);
        clearError(BROADCAST_ID);	// clear error for all servos

        delay(20);

        ACK(1);						// set ACK

        delay(20);

        torqueON(BROADCAST_ID);		// torqueON for all servos

        delay(20);
}



// stat
byte HerkulexClass::stat(int servoID)
{
	pSize    = 0x07;			//3.Packet size
	pID      = servoID;			//4.Servo ID - 0XFE=All servos
	cmd      = HSTAT;			//5.CMD
	
	ck1=(pSize^pID^cmd)&0xFE;
    ck2=(~(pSize^pID^cmd))&0xFE ;
  
	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	     
	sendData(dataEx, pSize);
	waitTransmissionTime(9);
	readData(9); 				// read 9 bytes from serial

	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    lenghtString=2;

	
    ck1 = (dataEx[2]^dataEx[3]^dataEx[4]^dataEx[7]^dataEx[8]) & 0xFE;
	ck2=checksum2(ck1);			
	
	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;

	return dataEx[7];			// return status
}

// torque on - 
void HerkulexClass::torqueON(int servoID)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x60;               // 10. 0x60=Torque ON
	lenghtString=3;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque ON

	sendData(dataEx, pSize);
}

// torque off - the torque is FREE, not Break
void HerkulexClass::torqueOFF(int servoID)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x00;               // 10. 0x00=Torque Free
	lenghtString=3;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque Free

    sendData(dataEx, pSize);

}

// torque off - the torque is FREE, not Break
void HerkulexClass::setAccelerationRatio(int servoID, int accelerationRatio)
{
	writeRegistryRAM(servoID,8,accelerationRatio,  1);

}

// torque off - the torque is FREE, not Break
void HerkulexClass::setAccelerationMax(int servoID, int accelerationMax)
{
	writeRegistryRAM(servoID,9,accelerationMax, 1);
}

void  HerkulexClass::setPositionKp(int servoID, int Ki)
{
	writeRegistryRAM(servoID, 24, Ki, 2);
}


void  HerkulexClass::setPositionKd(int servoID, int Ki)
{
	writeRegistryRAM(servoID, 26, Ki, 2);
}


void  HerkulexClass::setPositionKi(int servoID, int Ki)
{
	writeRegistryRAM(servoID, 28, Ki, 2);
}

void  HerkulexClass::setPositionFeedForward1stGain(int servoID, int value /* default 0 */)
{
	writeRegistryRAM(servoID, 30, value, 2);
}


void  HerkulexClass::setPositionFeedForward2stGain(int servoID, int value /* default 0 */)
{
	writeRegistryRAM(servoID, 32, value, 2);
}


void  HerkulexClass::setDeadZone(int servoID, int value /* default 0 */)
{
	writeRegistryRAM(servoID, 10, value, 1);
}

void  HerkulexClass::setSaturatorOffset(int servoID, int value /* default 0 */)
{
	writeRegistryRAM(servoID, 11, value, 1);
}

void  HerkulexClass::setSaturatorSlope(int servoID, int value /* default 0 */)
{
	writeRegistryRAM(servoID, 12, value, 2);
}

void  HerkulexClass::setPWMOffset(int servoID, int value /* default 0 */)
{
	if (value < 0)
		value += 128;
	writeRegistryRAM(servoID, 14, value, 1);
}


// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void HerkulexClass::ACK(int valueACK)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=valueACK;           // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
	lenghtString=3;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

 	sendData(dataEx, pSize);

}

// model - 1=0101, 2=0201, 4=401
ServoType HerkulexClass::model()
{
	pSize = 0x09;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = HEEPREAD;           // 5. CMD
	data[0]=0x00;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	lenghtString=2;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address
	dataEx[8] = data[1]; 		// Length

    sendData(dataEx, pSize);
    waitTransmissionTime(9);
    readData(9);
	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];           // 8. 1st byte
	lenghtString=1;              // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	if (ck1 != dataEx[5]) return INVALID_MODEL; //checksum verify
	if (ck2 != dataEx[6]) return INVALID_MODEL;
		
	return (ServoType)dataEx[7];			// return status

}

// setID - Need to restart the servo
void HerkulexClass::set_ID(int ID_Old, int ID_New)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = ID_Old;		        // 4. Servo ID OLD - original servo ID
	cmd   = HEEPWRITE;          // 5. CMD
	data[0]=0x06;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=ID_New;             // 10. ServoID NEW
	lenghtString=3;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

	sendData(dataEx, pSize);

}

// Default Baud Rate is 115,200bps
// 0x02 : 666,666bps
// 0x03 : 500,000bps
// 0x04 : 400,000bps
// 0x07 : 250,000bps
// 0x09 : 200,000bps
// 0x10 : 115,200bps
// 0x22 : 57,600bps
void HerkulexClass::changeBaudRate(int servoID, int baud) {
	int baudRateID = 0;
	switch (baud) {
		case 666666: baudRateID = 0x02;break;
		case 500000: baudRateID = 0x02;break;
		case 400000: baudRateID = 0x04;break;
		case 250000: baudRateID = 0x07;break;
		case 200000: baudRateID = 0x09;break;
		case 115200: baudRateID = 0x10;break;
		case  57600: baudRateID = 0x22;break;
	}
	if (baudRateID != 0)
		writeRegistryEEP(servoID, 0x04, baudRateID);
}


// clearError
void HerkulexClass::clearError(int servoID)
{
	pSize = 0x0B;               // 3.Packet size 7-58
	pID   = servoID;     		// 4. Servo ID - 253=all servos
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x30;               // 8. Address
	data[1]=0x02;               // 9. Lenght
	data[2]=0x00;               // 10. Write error=0
	data[3]=0x00;               // 10. Write detail error=0
	
	lenghtString=4;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value1
	dataEx[10]= data[3]; 		// Value2

	sendData(dataEx, pSize);
}

// move all servo at the same time to a position: servo list building
void HerkulexClass::moveAll(int servoID, int Goal, int iLed)
{
	  if (Goal > 1023 || Goal < 0)
		return;						 //0 <--> 1023 range
	  
	  int iMode=0;                   //mode=position
	  int iStop=0;                   //stop=0
	  

	  // Position definition
	  int posLSB=Goal & 0X00FF;					// MSB Pos
	  int posMSB=(Goal & 0XFF00) >> 8;			// LSB Pos

	  //led 
	  int iBlue=0;
	  int iGreen=0;
	  int iRed=0;
	  switch (iLed) {
	  case 1:
		iGreen=1;
		break;
	  case 2:
		iBlue=1;
		break;
	  case 3:
		iRed=1;
		break;
	  }
	  
	  int SetValue=iStop+iMode*2+iGreen*4+iBlue*8+iRed*16;	//assign led value

	  addData(posLSB, posMSB, SetValue, servoID);	//add servo data to list, pos mode
}

// move all servo at the same time to a position: servo list building
void HerkulexClass::moveAllAngle(int servoID, float angle, int iLed)
{
		if (angle > 160.0|| angle < -160.0) return; // out of the range	
		int position = (int)(angle/0.325) + 512;
		moveAll(servoID, position, iLed);
}



// move all servo at the same time with different speeds: servo list building
void HerkulexClass::moveSpeedAll(int servoID, int Goal, int iLed)
{
	  if (Goal > 1023 || Goal < -1023)
		return;								 //-1023 <--> 1023 range

	  int iMode=1;                  		// mode=continous rotation
	  int iStop=0;                  		// Stop=0

	  // Speed definition
	  int GoalSpeedSign;
	  if (Goal < 0) {
		GoalSpeedSign = (-1)* Goal ;
		GoalSpeedSign |= 0x4000;  //bit n�14 
	  } 
	  else {
		GoalSpeedSign = Goal;
	  }

	  int speedGoalLSB=GoalSpeedSign & 0X00FF; 	      		 // MSB speedGoal 
	  int speedGoalMSB=(GoalSpeedSign & 0xFF00) >> 8;        // LSB speedGoal 

	  //led 
	  int iBlue=0;
	  int iGreen=0;
	  int iRed=0;
	  switch (iLed) {
	  case 1:
		iGreen=1;
		break;
	  case 2:
		iBlue=1;
		break;
	  case 3:
		iRed=1;
		break;
	  }

	  int SetValue=iStop+iMode*2+iGreen*4+iBlue*8+iRed*16;	//assign led value

	  addData(speedGoalLSB, speedGoalMSB, SetValue, servoID);		//add servo data to list, speed mode
}



// move all servo with the same execution time
void HerkulexClass::actionAll(int pTime)
{
	if ((pTime <0) || (pTime > 2856)) return;

    pSize = 0x08 + conta;     	    // 3.Packet size 7-58
	cmd   = HSJOG;		 			// 5. CMD SJOG Write n servo with same execution time
	playTime=int((float)pTime/11.2);// 8. Execution time
 
    pID=0xFE^playTime;
    ck1=checksum1(moveData,conta);	//6. Checksum1
	ck2=checksum2(ck1);				//7. Checksum2

    pID=0xFE;
	dataEx[0] = 0xFF;				// Packet Header
	dataEx[1] = 0xFF;				// Packet Header	
	dataEx[2] = pSize;	 			// Packet Size
	dataEx[3] = pID;				// Servo ID
	dataEx[4] = cmd;				// Command Ram Write
	dataEx[5] = ck1;				// Checksum 1
	dataEx[6] = ck2;				// Checksum 2
	dataEx[7] = playTime;			// Execution time	
	
	for (int i=0; i < conta; i++)
		dataEx[i+8]=moveData[i];	// Variable servo data

	sendData(dataEx, pSize);

	conta=0; 						//reset counter   

}

// get Position
 int HerkulexClass::getPosition(ServoType type, int servoID) {
	int Position  = 0;

    pSize = 0x09;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = HRAMREAD;           // 5. CMD
	data[0]=0x3A;               // 8. Address
	data[1]=0x02;               // 9. Lenght
	
	lenghtString=2;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];      	// Address  
	dataEx[8] = data[1]; 		// Length
	
	sendData(dataEx, pSize);
    waitTransmissionTime(13);
	readData(13);

        	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    data[2]=dataEx[9];
    data[3]=dataEx[10];
    data[4]=dataEx[11];
    data[5]=dataEx[12];
    lenghtString=6;

    ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

    if (ck1 != dataEx[5]) return -1;
	if (ck2 != dataEx[6]) return -1;

	if (type == HERKULEX_DRS_0401)
		Position = ((dataEx[10]&0x07)<<8) | dataEx[9];
	else
		Position = ((dataEx[10]&0x03)<<8) | dataEx[9];

	return Position;
}

float HerkulexClass::getAngle(ServoType type, int servoID, bool& error) {
	int pos = getPosition(type, servoID);
	error = (pos==-1);

	// different servo types have a different resolution on
	if (type == HERKULEX_DRS_0401) {
		return (pos-1024) * 0.163; // resolution of 2^11 bits (2048)
	}
	else
		return (pos-512) * 0.325; // resolution of 2^10 bits (1024)
}

// reboot single servo - pay attention 253 - all servos doesn't work!
void HerkulexClass::reboot(int servoID) {
        
    pSize = 0x07;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = HREBOOT;            // 5. CMD
    ck1=(pSize^pID^cmd)&0xFE;
    ck2=(~(pSize^pID^cmd))&0xFE ; ;	

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	
	sendData(dataEx, pSize);

}

// LED  - see table of colors 
void HerkulexClass::setLed(int servoID, int valueLed)
{
	pSize   = 0x0A;               // 3.Packet size 7-58
	pID     = servoID;            // 4. Servo ID
	cmd     = HRAMWRITE;          // 5. CMD
	data[0] = 0x35;               // 8. Address 53
    data[1] = 0x01;               // 9. Lenght
	data[2] = valueLed;           // 10.LedValue
	lenghtString=3;               // lenghtData
  	  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];        // Address
	dataEx[8] = data[1];       	// Length
	dataEx[9] = data[2];        // Value

	sendData(dataEx, pSize);
}

// get the speed for one servo - values between -1023 <--> 1023
int HerkulexClass::getPWM(int servoID) {
  int speedy  = 0;

  pSize = 0x09;               // 3.Packet size 7-58
  pID   = servoID;     	   	  // 4. Servo ID 
  cmd   = HRAMREAD;           // 5. CMD
  data[0]=0x40;               // 8. Address
  data[1]=0x02;               // 9. Lenght

  lenghtString=2;             // lenghtData

  ck1=checksum1(data,lenghtString);		//6. Checksum1
  ck2=checksum2(ck1);					//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 	    // Address  
  dataEx[8] = data[1]; 		// Length

  sendData(dataEx, pSize);
  waitTransmissionTime(13);
  readData(13);


  pSize = dataEx[2];           // 3.Packet size 7-58
  pID   = dataEx[3];           // 4. Servo ID
  cmd   = dataEx[4];           // 5. CMD
  data[0]=dataEx[7];
  data[1]=dataEx[8];
  data[2]=dataEx[9];
  data[3]=dataEx[10];
  data[4]=dataEx[11];
  data[5]=dataEx[12];
  lenghtString=6;

  ck1=checksum1(data,lenghtString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  if (ck1 != dataEx[5]) return -1;
  if (ck2 != dataEx[6]) return -1;

  speedy = ((dataEx[10]&0xFF)<<8) | dataEx[9];
  return speedy;

}

// get the speed for one servo - values between -1023 <--> 1023
float HerkulexClass::getVoltage(ServoType type, int servoID) {
  pSize = 0x09;               // 3.Packet size 7-58
  pID   = servoID;     	   	  // 4. Servo ID
  cmd   = HRAMREAD;           // 5. CMD
  data[0]=54;                 // 8. Address
  data[1]=0x02;               // 9. Lenght

  lenghtString=2;             // lenghtData

  ck1=checksum1(data,lenghtString);		//6. Checksum1
  ck2=checksum2(ck1);					//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 	    // Address
  dataEx[8] = data[1]; 		// Length

  sendData(dataEx, pSize);
  waitTransmissionTime(13);
  readData(13);


  pSize = dataEx[2];           // 3.Packet size 7-58
  pID   = dataEx[3];           // 4. Servo ID
  cmd   = dataEx[4];           // 5. CMD
  data[0]=dataEx[7];
  data[1]=dataEx[8];
  data[2]=dataEx[9];
  data[3]=dataEx[10];
  data[4]=dataEx[11];
  data[5]=dataEx[12];
  lenghtString=6;

  ck1=checksum1(data,lenghtString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  if (ck1 != dataEx[5]) return -1;
  if (ck2 != dataEx[6]) return -1;

  float voltage = (float)(dataEx[9]);

  // depending on the servo type there are different factors
  // to get real voltage out of return value
  if (type == HERKULEX_DRS_0401)
	  voltage *= 0.1;
  else
	  voltage *= 0.074;

  return voltage;
}


// get the speed for one servo - values between -1023 <--> 1023
float HerkulexClass::getTemperature(int servoID) {
  pSize = 0x09;               // 3.Packet size 7-58
  pID   = servoID;     	   	  // 4. Servo ID
  cmd   = HRAMREAD;           // 5. CMD
  data[0]=55;                 // 8. Address
  data[1]=0x02;               // 9. Lenght

  lenghtString=2;             // lenghtData

  ck1=checksum1(data,lenghtString);		//6. Checksum1
  ck2=checksum2(ck1);					//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 	    // Address
  dataEx[8] = data[1]; 		// Length

  sendData(dataEx, pSize);
  waitTransmissionTime(13);
  readData(13);


  pSize = dataEx[2];           // 3.Packet size 7-58
  pID   = dataEx[3];           // 4. Servo ID
  cmd   = dataEx[4];           // 5. CMD
  data[0]=dataEx[7];
  data[1]=dataEx[8];
  data[2]=dataEx[9];
  data[3]=dataEx[10];
  data[4]=dataEx[11];
  data[5]=dataEx[12];
  lenghtString=6;

  ck1=checksum1(data,lenghtString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  if (ck1 != dataEx[5]) return -1;
  if (ck2 != dataEx[6]) return -1;

  float temperature = (((dataEx[10]&0xFF)<<8) | dataEx[9]);
  return temperature;

}
// move one servo with continous rotation
void HerkulexClass::moveSpeedOne(int servoID, int Goal, int pTime, int iLed)
{
  if (Goal > 1023 || Goal < -1023) return;              // speed (goal) non correct
  if ((pTime <0) || (pTime > 2856)) return;

  int GoalSpeedSign;
  if (Goal < 0) {
    GoalSpeedSign = (-1)* Goal ;
    GoalSpeedSign |= 0x4000;  //bit n�14 
  } 
  else {
    GoalSpeedSign = Goal;
  }
  int speedGoalLSB=GoalSpeedSign & 0X00FF; 		       // MSB speedGoal 
  int speedGoalMSB=(GoalSpeedSign & 0xFF00) >> 8;      // LSB speedGoal 

  //led 
  int iBlue=0;
  int iGreen=0;
  int iRed=0;
  switch (iLed) {
  case 1:
    iGreen=1;
    break;
  case 2:
    iBlue=1;
    break;
  case 3:
    iRed=1;
    break;
  }
  int SetValue=2+iGreen*4+iBlue*8+iRed*16;		//assign led value 

  playTime=int((float)pTime/11.2);				// 8. Execution time

  pSize = 0x0C;              					// 3.Packet size 7-58
  cmd   = HSJOG;      					        // 5. CMD

  data[0]=speedGoalLSB;            			    // 8. speedLSB
  data[1]=speedGoalMSB;              			// 9. speedMSB
  data[2]=SetValue;                          	// 10. Mode=0;
  data[3]=servoID;                    			// 11. ServoID

  pID=servoID^playTime;

  lenghtString=4;             					// lenghtData

  ck1=checksum1(data,lenghtString);				//6. Checksum1
  ck2=checksum2(ck1);							//7. Checksum2

  pID=servoID;

  dataEx[0] = 0xFF;				// Packet Header
  dataEx[1] = 0xFF;				// Packet Header	
  dataEx[2] = pSize;	 		// Packet Size
  dataEx[3] = pID;				// Servo ID
  dataEx[4] = cmd;				// Command Ram Write
  dataEx[5] = ck1;				// Checksum 1
  dataEx[6] = ck2;				// Checksum 2
  dataEx[7] = playTime;  		// Execution time	
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];
  
  sendData(dataEx, pSize);

}

// move one servo at goal position 0 - 1024
void HerkulexClass::moveOne(int servoID, int Goal, int pTime, int iLed)
{
  if (Goal > 2047 || Goal < 0) return;              // speed (goal) non correct
  if ((pTime <0) || (pTime > 2856)) return;

  // Position definition
  int posLSB=Goal & 0X00FF;								// MSB Pos
  int posMSB=(Goal & 0XFF00) >> 8;						// LSB Pos

  //led 
  int iBlue=0;
  int iGreen=0;
  int iRed=0;
  switch (iLed) {
  case 1:
    iGreen=1;
    break;
  case 2:
    iBlue=1;
    break;
  case 3:
    iRed=1;
    break;
  }
  int SetValue=iGreen*4+iBlue*8+iRed*16;	//assign led value 

  playTime=int((float)pTime/11.2);			// 8. Execution time

  pSize = 0x0C;          			    	// 3.Packet size 7-58
  cmd   = HSJOG;              				// 5. CMD

  data[0]=posLSB;               			// 8. speedLSB
  data[1]=posMSB;               			// 9. speedMSB
  data[2]=SetValue;                         // 10. Mode=0;
  data[3]=servoID;                    		// 11. ServoID

  pID=servoID^playTime;

  lenghtString=4;             				// lenghtData

  ck1=checksum1(data,lenghtString);			//6. Checksum1
  ck2=checksum2(ck1);						//7. Checksum2

  pID=servoID;

  dataEx[0] = 0xFF;				// Packet Header
  dataEx[1] = 0xFF;				// Packet Header	
  dataEx[2] = pSize;	 		// Packet Size
  dataEx[3] = pID;				// Servo ID
  dataEx[4] = cmd;				// Command Ram Write
  dataEx[5] = ck1;				// Checksum 1
  dataEx[6] = ck2;				// Checksum 2
  dataEx[7] = playTime;  		// Execution time	
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];

  sendData(dataEx, pSize, false);
}

// move one servo to an angle between -160 and 160
void HerkulexClass::moveOneAngle(ServoType type, int servoID, float angle, int pTime, int iLed) {
	if (angle > 160.0|| angle < -160.0) return;	
	int position;
	if (type == HERKULEX_DRS_0401) {
		position = (int)(angle/0.163) + 1024;
	} else
		position = (int)(angle/0.325) + 512;
	moveOne(servoID, position, pTime, iLed);
}

// write registry in the RAM: one byte 
void HerkulexClass::writeRegistryRAM(int servoID, int address, int writeByte, int length)
{
  pSize = 0x09+length;         	// 3.Packet size 7-58
  pID   = servoID;     			// 4. Servo ID - 253=all servos
  cmd   = HRAMWRITE;          	// 5. CMD
  data[0]=address;              // 8. Address
  data[1]=length;               	// 9. Lenght
  if (length == 1) {
	  data[2]=writeByte;            // 10. Write error=0
  }
  if (length == 2) {
	  data[2]=writeByte  & 0X00FF;            // 10. Write error=0
	  data[3]=(writeByte & 0xFF00) >> 8;      // 10. Write error=0

  }
  lenghtString=2+length;            // lenghtData

  ck1=checksum1(data,lenghtString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = pSize;	 	// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  sendData(dataEx, pSize);

}

// write registry in the EEP memory (ROM): one byte 
void HerkulexClass::writeRegistryEEP(int servoID, int address, int writeByte)
{
  pSize = 0x0A;                  // 3.Packet size 7-58
  pID   = servoID;     	         // 4. Servo ID - 253=all servos
  cmd   = HEEPWRITE;             // 5. CMD
  data[0]=address;               // 8. Address
  data[1]=0x01;                  // 9. Lenght
  data[2]=writeByte;             // 10. Write error=0
 
  lenghtString=3;           	 // lenghtData

  ck1=checksum1(data,lenghtString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  sendData(dataEx, pSize);

}



// Private Methods //////////////////////////////////////////////////////////////

// checksum1
int HerkulexClass::checksum1(byte* data, int lenghtString)
{
  XOR = 0;	
  XOR ^= pSize;
  XOR ^= pID;
  XOR ^= cmd;
  for (int i = 0; i < lenghtString; i++) 
    XOR ^= data[i];

  return XOR&0xFE;
}


// checksum2
int HerkulexClass::checksum2(int XOR)
{
  return (~XOR)&0xFE;
}

// add data to variable list servo for syncro execution
void HerkulexClass::addData(int GoalLSB, int GoalMSB, int set, int servoID)
{
  moveData[conta++]=GoalLSB;  
  moveData[conta++]=GoalMSB;
  moveData[conta++]=set;
  moveData[conta++]=servoID;
}


// Sending the buffer long lenght to Serial port
void HerkulexClass::sendData(byte* buffer, int lenght, bool waitUntilSent)
{
		clearBuffer(); 		//clear the serialport buffer - try to do it!

		serial->write(buffer, lenght);

		// wait until characters have been sent
		if (waitUntilSent)
			waitTransmissionTime(lenght);
}

// * Receiving the lenght of bytes from Serial port
bool HerkulexClass::readData(int size)
{
	int i = 0;
    int beginsave=0;
    int Time_Counter_us=0;

    // wait until number of bytes is there
	while((serial->available() < size) && (Time_Counter_us < TIME_OUT_MS*1000)){
       	Time_Counter_us += 50;
       	delayMicroseconds(50);
	}      	
	while (serial->available() > 0){
		byte inchar = (byte)serial->read();
		// logger->print(inchar);
		// logger->print(' ');
		//printHexByte(inchar);
       	if ( (inchar == 0xFF) & ((byte)serial->peek() == 0xFF) ){
       		beginsave=1;
       		i=0;
       	}
       	if (beginsave==1 && i<size) {
       		dataEx[i] = inchar;
       		i++;
       	}
	}
	return (i == size);
}

//clear buffer in the serial port - better - try to do this
void HerkulexClass::clearBuffer()
{
	serial->flush();

	while (serial->available()){
		serial->read();
		uint32_t start = micros();
		while ((micros() - start) >= 200) {
			yield();
		}
	}
}

int HerkulexClass::getDistance(int servoID, int &status) {
	uint8_t dataLength = 0;
	uint8_t ID = 200;
	uint8_t SENSOR_CMD_REQUEST_DISTANCE = 200;

	uint8_t packetSize = 7 + dataLength; // Lengh of the request packet
	uint8_t packetSend[packetSize]; // Request packet to send

	// Request packet structure
	packetSend[0] = 0xFF; // Header
	packetSend[1] = 0xFF; // Header
	packetSend[2] = packetSize; // Packet Size
	packetSend[3] = ID; // ID of the servo
	packetSend[4] = SENSOR_CMD_REQUEST_DISTANCE; // Instruction
	packetSend[5] = (packetSize^ID^SENSOR_CMD_REQUEST_DISTANCE) & 0xFE;
	packetSend[6] = (~packetSend[5]) & 0xFE;

	// add data to packet
	for(int i = 0 ; i < dataLength ; i++){
		packetSend[7 + i] = data[i];
	}

	// send the request packet and wait that the packet is actually sent
	sendData(packetSend,packetSize);

	// wait until reply of 9 characters has been sent.
	waitTransmissionTime(9);

	bool ok = readData(9);				// read 9 bytes from serial
	if (ok) {
										// 1.0xFF
										// 2.0xFF
		int size = dataEx[2];           // 3.Packet size 7-58
		int ID   = dataEx[3];	        // 4. Servo ID
		int cmd  = dataEx[4];           // 5. CMD
		int distance = dataEx[7];	 	// 6. measured sensor distance
		status = dataEx[8];				// 7. istance sensor status

		int ck1 = (size^ID^cmd ^ dataEx[7]^dataEx[8]) & 0xFE; 	// compute checksum1
		int ck2 = (~ck1) & 0xFE;								// compute checksum2

		if (ck1 != dataEx[5]) return -1; //checksum verify
		if (ck2 != dataEx[6]) return -2;
		return distance;
	}
	else {
		return -3;	// could not read response, dont know why
	}
}

void HerkulexClass::getDistanceRequest(int servoID) {
	uint8_t dataLength = 0;
	uint8_t ID = 200;
	uint8_t SENSOR_CMD_REQUEST_DISTANCE = 200;

	uint8_t packetSize = 7 + dataLength; // Lengh of the request packet
	uint8_t packetSend[packetSize]; // Request packet to send

	// Request packet structure
	packetSend[0] = 0xFF; // Header
	packetSend[1] = 0xFF; // Header
	packetSend[2] = packetSize; // Packet Size
	packetSend[3] = ID; // ID of the servo
	packetSend[4] = SENSOR_CMD_REQUEST_DISTANCE; // Instruction
	packetSend[5] = (packetSize^ID^SENSOR_CMD_REQUEST_DISTANCE) & 0xFE;
	packetSend[6] = (~packetSend[5]) & 0xFE;

	// add data to packet
	for(int i = 0 ; i < dataLength ; i++){
		packetSend[7 + i] = data[i];
	}

	// send the request packet and wait that the packet is actually sent
	sendData(packetSend,packetSize, false);
}

int HerkulexClass::getDistanceResponse(int servoID, int& status) {
	bool ok = readData(9);				// read 9 bytes from serial
	if (ok) {
										// 1.0xFF
										// 2.0xFF
		int size = dataEx[2];           // 3.Packet size 7-58
		int ID   = dataEx[3];	        // 4. Servo ID
		int cmd  = dataEx[4];           // 5. CMD
		int distance = dataEx[7];	 	// 6. measured sensor distance
		status = dataEx[8];				// 7. istance sensor status

		int ck1 = (size^ID^cmd ^ dataEx[7]^dataEx[8]) & 0xFE; 	// compute checksum1
		int ck2 = (~ck1) & 0xFE;								// compute checksum2

			if (ck1 != dataEx[5]) return -1; //checksum verify
		if (ck2 != dataEx[6]) return -2;
		return distance;
	}
	else {
		return -3;	// could not read response, dont know why
	}
}


void HerkulexClass::sendPacket(uint8_t ID, int CMD, const uint8_t data[], uint8_t dataLength){
	uint8_t packetSize = 7 + dataLength; // Lengh of the request packet
	uint8_t packetSend[packetSize]; // Request packet to send
	
	// Request packet structure
	packetSend[0] = 0xFF; // Header
	packetSend[1] = 0xFF; // Header
	packetSend[2] = packetSize; // Packet Size
	packetSend[3] = ID; // ID of the servo
	packetSend[4] = CMD; // Instruction
	packetSend[5] = checksum1(packetSend, dataLength); // Checksum1
	packetSend[6] = checksum2(packetSend[5]); // Checksum2
	
	// Data
	for(int i = 0 ; i < dataLength ; i++){
		packetSend[7 + i] = data[i];
	}
	
	// send the request packet and wait that the packet is actually sent
	sendData(packetSend,packetSize);
}

