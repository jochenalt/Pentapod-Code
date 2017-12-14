/*
 * BezierCurve.h
 *
 * Cubic bezier curve used to model the movement of a leg.
 * It is shaped like this (C,D are support points that are not on the curve)
 *
 * C   _____   D
 *    /     \
 *   /       \
 *  |         |
 *  |         |
 *  A         B
 *
 * Author: JochenAlt
 */

#ifndef BEZIERCURVE_H_
#define BEZIERCURVE_H_

#include "basics/point.h"
#include "basics/spatial.h"
#include "basics/util.h"

class BezierCurve  {
	public:
		BezierCurve();
		~BezierCurve() {};
		BezierCurve(const BezierCurve& par);
		void operator=(const BezierCurve& par);

		// null out start, end and support points
		void reset();

		// get start and end point of bezier curve
		Point& getStart();
		Point& getEnd();

		// define start/end and both support points
		void set(const Point& pA, const Point& pSupportA, const Point& pB, const Point& pSupportB);

		// get current point of a given bezier curve by the parameter t=[0..1]
		Point getCurrent(realnum  t);

		// returns the ratio of t's position in the interval [a..b], i.e. a=10;b=20;t=12 returns 0.2
		realnum getIntervalRatio(unsigned long a, unsigned long t, unsigned long b);

		// approximated length of the curve
		realnum curveLength();


	private:
		realnum computeBezier(realnum  a,realnum  supportA,  realnum  b, realnum  supportB, realnum  t);
		Point computeBezier(const Point& a, const Point& supportA,  const Point& b, const Point& supportB, realnum t);

		Point a; 			// starting point
		Point supportA;		// support point of a
		Point b;			// end point
		Point supportB;		// support point of b
};

#endif /* BEZIERCURVE_H_ */
