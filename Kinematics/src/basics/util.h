/*
 * Util.h
 *
 * Various helper functions
 *
 * Author: JochenAlt
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <vector>
#include <stdio.h>
#include "math.h"
#include <stdint.h>
#include <iostream>
#include <string.h>
#include <mutex>

#include <basics/types.h>

using namespace std;

#define LFCR '\n'

class ExclusiveMutex {
public:
	void lock();
	void unlock();
	bool isLocked();
	void waitAndLock();

	ExclusiveMutex();
	virtual ~ExclusiveMutex();

	bool isInBlock;
};

class CriticalBlock  {
public:
	CriticalBlock (ExclusiveMutex &mutex);
	virtual ~CriticalBlock();
	void waitAndLock();
	ExclusiveMutex* mutex;
};

template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

template<class T>
T sgn(const T& a) {
    if(a < 0.0) {
        return -1.0;
    }
    else if(a > 0.0) {
        return +1.0;
    }
    else
        return 0;
}

#define MAX(a,b) (((a)>(b))?(a):(b))

template<class T>
T sqr(const T& x) {
    return x*x;
}

template<class T>
T pow3(const T& x) {
    return x*x*x;
}



// random helper functions
int randomInt(int min,int max);
realnum randomFloat (realnum a, realnum b);
bool randomBool();
int randomPosNeg();

// time helpers
milliseconds millis();
void delay_ms(long);
void delay_us(long);
string currentTimeToString();


bool almostEqual(realnum a, realnum b, realnum precision);		// true, if both values differ by  given percentage only
float roundValue(float x);											// round passed value to next floatPrecision number

realnum arctanApprox(realnum x);
realnum radians(realnum degrees);
realnum degrees(realnum radians);
realnum triangleAlpha(realnum a, realnum b, realnum c);			// cosine law
realnum triangleGamma(realnum a, realnum b, realnum c);			// cosine law
bool polynomRoot2ndOrder(realnum a, realnum b, realnum c, realnum& root0, realnum& root1);

realnum lowpass (realnum oldvalue, realnum newvalue, realnum tau, realnum dT);

// return a number 0..1 depending on the input 0..1.


// Usecase: break at the end of the movement such that speed goes down to zero.
// use polynom 3rd grade with (f(0) = 0, f(1) = 0, f'(0) = grade, f'(1) = 0)
// input is number between 0..1, output is number between 0..1 but moving to 1 quicker in the beginning
realnum moderate(realnum input, realnum grade);

// function with f(0) = 0, f(1) = 0, f'(0) = 0, in between the function goes up and accelerates and goes down and breakes
realnum speedUpAndDown(realnum input);

// approximate circumference of an ellipse given with two axis
realnum ellipseCircumference(realnum a, realnum b);



class TimeSamplerStatic {
public:
	TimeSamplerStatic() {
		lastCall = millis();
		lastDueCall = 0; // first call of isDue leads to true
		firstCalldT = true;
	}
	virtual ~TimeSamplerStatic() {};
	void reset() {
		firstCalldT = true;
	};
	seconds dT() {
		milliseconds now = millis();
		realnum duration = (now - lastCall)/1000.0;
		lastCall = now;
		if (firstCalldT) {
			firstCalldT = false;
			return 0;
		};
		return duration;
	};

	bool isDue(milliseconds sampleTime) {
		milliseconds now = millis();
		if ((now - lastDueCall) > sampleTime) {
			lastDueCall = now;
			return true;
		}

		return false;
	};

	void dontBeDueFor(milliseconds noDueTime) {
		if (noDueTime < 0)
			noDueTime = 0;
		lastDueCall = millis() + noDueTime;
	}
private:
	milliseconds lastCall;
	milliseconds lastDueCall;
	bool firstCalldT = true;
};

class Rate
{
	public:
	Rate(double frequency):
		start_(millis()), expected_cycle_time_(1000 / frequency), actual_cycle_time_(0) {};
	bool sleep() {
		milliseconds expected_end = start_ + expected_cycle_time_;
		milliseconds actual_end = millis();
		milliseconds sleep_time = 0;
		// detect backward jumps in time
		if (actual_end < start_)
		{
			expected_end = actual_end + expected_cycle_time_;
		}
		//calculate the time we'll sleep for
		if (expected_end > actual_end) {
			sleep_time = expected_end - actual_end;
		}

		//set the actual amount of time the loop took in case the user wants to know
		actual_cycle_time_ = actual_end - start_;
		//make sure to reset our start time
		start_ = expected_end;

		//if we've taken too much time we won't sleep
		if(sleep_time == 0)
		{
			// if we've jumped forward in time, or the loop has taken more than a full extra
			// cycle, reset our cycle
			if (actual_end > expected_end + expected_cycle_time_)
			{
				start_ = actual_end;
			}
			return true;
		} else {
			delay_ms(sleep_time);
			return false;
		}
	}
	void reset() {  start_ = millis(); }
	milliseconds cycleTime() { return actual_cycle_time_; };
	milliseconds expectedCycleTime() { return expected_cycle_time_; }
	private:
		milliseconds start_;
		milliseconds expected_cycle_time_, actual_cycle_time_;
};


// class representing a float which is passed through a 1st order low pass
// Example:
// 	  LowPassFilter x;
//    x = 10.0;
// 	  cout << "do not expect 10.0" << x;
class LowPassFilter {
public:
	LowPassFilter () {};
	virtual ~LowPassFilter() {};

	LowPassFilter (realnum frequency) {
		init(frequency);
	}

	void init(realnum frequency) {
		responseTime = 1.0/frequency;
		currentValue = 0;
		sampler.dT();
	}

	void operator=(realnum  value) {
		realnum complementaryFilter = responseTime/(responseTime + sampler.dT());
		currentValue = complementaryFilter*currentValue + (1.0-complementaryFilter)*value;
	}

	void set(realnum  value) {
		realnum  complementaryFilter = responseTime/(responseTime + sampler.dT());
		currentValue = complementaryFilter*currentValue + (1.0-complementaryFilter)*value;
	}

	operator realnum() const { return currentValue; };

	realnum  responseTime = 0;
	realnum  currentValue = 0;
	TimeSamplerStatic sampler;
};

#endif
