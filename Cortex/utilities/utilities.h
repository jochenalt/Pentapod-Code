
/*
 * utilities.h
 *
 * Created: 10.05.2016 12:01:54
 *  Author: JochenAlt
 */ 

#ifndef UTILITIES_H_
#define UTILITIES_H_

#define MAX_INT_16 ((2<<15)-1)
#define sgn(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )


extern void logFatal(const __FlashStringHelper *ifsh);
extern void logError(const __FlashStringHelper *ifsh);
extern void logPin(uint8_t PINnumber);

#include "TimePassedBy.h"

class LowPassFilter {
public:
	LowPassFilter () {};
	virtual ~LowPassFilter() {};

	LowPassFilter (float frequency) {
		init(frequency);
	}

	void init(float frequency) {
		responseTime = 1.0/frequency;
		currentValue = 0;
		sampler.dT();
	}

	void operator=(float  value) {
		float complementaryFilter = responseTime/(responseTime + sampler.dT());
		currentValue = complementaryFilter*currentValue + (1.0-complementaryFilter)*value;
	}

	void set(float  value) {
		float  complementaryFilter = responseTime/(responseTime + sampler.dT());
		currentValue = complementaryFilter*currentValue + (1.0-complementaryFilter)*value;
	}

	float get() { return currentValue; };

	operator float() const { return currentValue; };

	float  responseTime = 0;
	float  currentValue = 0;
	TimePassedBy sampler;
};

#endif
