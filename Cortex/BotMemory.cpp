#include "MemoryBase.h"
#include "BotMemory.h"
#include <avr/eeprom.h>
#include "pins.h"

BotMemory memory;

BotMemory::BotMemory()
: MemoryBase((void*)&(persMem),sizeof(BotMemory::persMem)) {
	// initialization for the very first start, when EEPROM is not yet initialized
	BotMemory::setDefaults();
}

void BotMemory::setDefaults() {
	memory.persMem.logSetup = false;
	memory.persMem.logServo = false;
	memory.persMem.logLoop = false;

	LegConfigType::setDefaults();
	memory.persMem.imuCalib.setDefault();
}


void BotMemory::println() {
		logger->println(F("EEPROM"));

		logger->print(F("   LOG=("));
		if (memory.persMem.logSetup)
			logger->print(F("setup "));
		if (memory.persMem.logServo)
			logger->print(F("servo "));
		if (memory.persMem.logLoop)
			logger->print(F("loop "));

		logger->println(")");

		for (int i = 0;i<NUMBER_OF_LEGS;i++) {
			persMem.legs[i].print();
			logger->println();
		}

		persMem.imuCalib.print();
}
