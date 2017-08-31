/* 
* BotMemory.h
*
* Persistent Memory of Walter. Stores actuator configuration and logging state in EEPROM.
*
* Author: JochenAlt
*/


#ifndef __BOTMEMORY_H__
#define __BOTMEMORY_H__
#include "Arduino.h"
#include "MemoryBase.h"
#include "Config.h"
#include "OrientationSensor.h"

class BotMemory;
extern BotMemory memory;

class BotMemory : public MemoryBase {
	public:
		// initialize  default values of memory for the very first start
		BotMemory();
		void println();
		static void setDefaults();
		static bool logSetup() { return memory.persMem.logSetup;};
		static bool logServo() { return memory.persMem.logServo;};

	struct  {
		bool logSetup;
		bool logServo;
		bool logLoop;
		bool logEncoder;

		LegConfigType legs[NUMBER_OF_LEGS];
		OrientationSensorData imuCalib;
	} persMem;
};


#endif //__BOTMEMORY_H__
