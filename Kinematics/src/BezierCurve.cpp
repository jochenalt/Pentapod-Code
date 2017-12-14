#include <iostream>
#include <string>

#include "basics/point.h"
#include "basics/logger.h"
#include "basics/util.h"
#include "BezierCurve.h"

BezierCurve::BezierCurve() {
};

BezierCurve::BezierCurve(const BezierCurve& par) {
	a = par.a;
	supportA = par.supportA;
	b = par.b;
	supportB = par.supportB;
};
void BezierCurve::operator=(const BezierCurve& par) {
	a = par.a;
	supportA = par.supportA;
	b = par.b;
	supportB = par.supportB;
};

void BezierCurve::reset() {
	a.null();
	b.null();
	supportA.null();
	supportB.null();
}

Point& BezierCurve::getStart() {
	return a;
};

Point& BezierCurve::getEnd() {
	return b;
};


realnum  BezierCurve::computeBezier(realnum  a, realnum  supportA,  realnum  b, realnum  supportB, realnum  t) {
	if ((t>1.0 + floatPrecision) || (t<-floatPrecision)) {
		ROS_ERROR_STREAM("BezierCurve:t!=[0..1]:" << t);

	}
	t = constrain(t,0.0,1.0);

	// formula of cubic bezier curve (wikipedia)
	return (1-t)*(1-t)*(1-t)*a + 3*t*(1-t)*(1-t)*supportA + 3*t*t*(1-t)*supportB + t*t*t*b;
}


// interpolate a bezier curve between a and b by use of passeds support points
Point BezierCurve::computeBezier(const Point& a, const Point& supportA,  const Point& b, const Point& supportB, realnum  t) {
	Point result;
	for (int i = 0;i<3;i++) {
		result[i] = computeBezier(a[i], supportA[i], b[i], supportB[i],t);
	}

	return result;
}

// define a bezier curve by start and end point and two support points
void BezierCurve::set(const Point& pA, const Point& pSupportA, const Point& pB, const Point& pSupportB) {
	a = pA;
	b = pB;
	supportB = pSupportB;
	supportA = pSupportA;
}


Point BezierCurve::getCurrent(realnum  t) {
	Point result = computeBezier(a,supportA,b, supportB, t);
	return result;
}

// return a number 0..1 representing the position of t between a and b
realnum  BezierCurve::getIntervalRatio(unsigned long a, unsigned long t, unsigned long b) {
	return ((realnum )t-(realnum )a)/((realnum )b-(realnum )a);
}


realnum  BezierCurve::curveLength() {
	realnum  distance = 0.0;
	Point curr = getCurrent(0);

	// BTW: computing the length of a bezier curve in maths style is really complicated, so do it numerically
	realnum  t = 0.01; // at least 0.01mm, in order to not divide by 0 later on
	while (t<1.0) {
		Point next = getCurrent(t);
		distance += curr.distance(next);
		curr = next;
		t += 0.05;
	}

	// last node
	distance += curr.distance(b);
	return distance;
}


