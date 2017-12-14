
#include <stdlib.h>
#include <chrono>
#include <unistd.h>
#include <iomanip>

#include "setup.h"
#include "basics/util.h"
#include <cstdarg>
#include <iostream>
#include <sstream>

#include "basics/stringhelper.h"

void ExclusiveMutex::lock() {
	isInBlock = true;
};


void ExclusiveMutex::unlock() {
	isInBlock = false;
};

bool ExclusiveMutex::isLocked() {
	return !isInBlock;
};
void ExclusiveMutex::waitAndLock() {
	while (isInBlock) delay_ms(1);
	lock();
};

ExclusiveMutex::ExclusiveMutex() {
	isInBlock = false;
}
ExclusiveMutex::~ExclusiveMutex() {
}

CriticalBlock::CriticalBlock (ExclusiveMutex & newMutex) {
	mutex = &newMutex;
	mutex->waitAndLock();
};

CriticalBlock::~CriticalBlock() {
	mutex->unlock();
};

void CriticalBlock::waitAndLock() {
	mutex->waitAndLock();
}


float roundValue(float x) {
	float roundedValue = sgn(x)*((int)(abs(x)*10.0f+.5f))/10.0f;
	return roundedValue;
}

realnum arctanApprox(realnum x) {
	return 3.0*x/(3.0 + x*x);
}

bool hasPrefix(string str, string prefix) {
	int idx = upcase(str).find(upcase(prefix));
	return (idx>=0);
}


long mapLong(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int randomInt(int min,int max) {
	int r = rand() % (max-min) + min;
	return r;
}

realnum randomFloat (realnum a, realnum b) {
	return randomInt(a*1000, b*1000)/1000.;
}

bool randomBool() {
	return randomInt(0,100)>50;
}

int randomPosNeg() {
	return (randomInt(0,100)>50)?+1:-1;
}

milliseconds millis() {
    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    auto now   = std::chrono::high_resolution_clock::now();
    auto mseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - epoch).count();
    return mseconds;
}

realnum lowpass (realnum oldvalue, realnum newvalue, realnum tau, realnum dT) {
	realnum alpha = tau/(tau+dT);
	realnum result = newvalue*(1.0-alpha) + oldvalue * alpha;
	return result;
}

// use polynom 3rd grade with (f(0) = 0, f(1) = 01 f'(0) = grade, f'(1) = 0)
// input is number between 0..1, output is number between 0..1 but moving to 1 quicker in the beginning
// grade has to be <=3
realnum moderate(realnum input, realnum grade) {
	return (grade-2.0)*input*input*input + (-2.0*grade+3.0)*input*input + grade*input;
}

// function with f(0) = 0, f(1) = 0, f'(0) = 0, in between the function goes down and accelerates
realnum speedUpAndDown(realnum input) {
	return 3*input*input*input - 2*input*input;
}

realnum ellipseCircumference(realnum a, realnum b) {
	realnum lambda = (a-b)/(a+b);
	return (a+b)*M_PI*(1.0 + 3.0*lambda*lambda/(10.0 + sqrt(4.0-3.0*lambda*lambda)));
}



void delay_ms(long ms) {
	// usleeps works below 1000000 only.

	while (ms > 0) {
		if (ms >= 1000) {
			sleep(1);
			ms -= 1000;
		}
		else {
			usleep(ms*1000);
			ms = 0;
		}
	}
}

void delay_us(long us) {
	usleep(us);
}


realnum radians(realnum degrees) {
	const realnum fac = (M_PI / 180.0);
	return degrees * fac;
}

realnum  degrees(realnum radians) {
	const realnum fac = (180.0 / M_PI);
	return radians * fac;
}

// cosine sentence
realnum triangleAlpha(realnum a, realnum b, realnum c) {
	realnum x = acos((a*a-b*b-c*c)/(-2.0*b*c));
    return x;
}

// cosine sentence
realnum triangleGamma(realnum a, realnum b, realnum c) {
	return triangleAlpha(c,b,a);
}

// abc formula, root of 0 = a*x*x + b*x + c;
bool polynomRoot2ndOrder(realnum a, realnum b, realnum c, realnum& root0, realnum& root1)
{
	realnum disc = b*b-4.0*a*c;
	if (disc>=0) {
		root0 = (-b + sqrt(disc)) / (2.0*a);
		root1 = (-b - sqrt(disc)) / (2.0*a);
		return true;
	}
	return false;
}


bool almostEqual(realnum a, realnum b, realnum precision) {
	if (a==b)
		return true;
	if (a == 0)
		return (abs(b)<precision);
	if (b == 0)
		return (abs(a)<precision);

	if (b<a)
		return (abs((b/a)-1.0) < precision);
	else
		return (abs((a/b)-1.0) < precision);

}



