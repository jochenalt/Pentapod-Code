/*
 * BezierCurve.h
 *
 * Cubic bezier curve used to model the movement of a leg.
 * It is shaped like this (C,D are support points)
 *
 *   C _____ D
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
#include "spatial.h"
#include "Util.h"

class BezierCurve  {
	public:
		BezierCurve();
		~BezierCurve() {};

		BezierCurve(const BezierCurve& par);
		void operator=(const BezierCurve& par);

		// null out start, end and support points
		void reset();

		// get start and end point
		Point& getStart();
		Point& getEnd();

		// define start/end and support points
		void set(const Point& pA, const Point& pSupportA, const Point& pB, const Point& pSupportB);

		// get current point of a given bezier curve by the parameter t=[0..1]
		Point getCurrent(realnum  t);

		realnum getIntervalRatio(unsigned long a, unsigned long t, unsigned long b);
		realnum curveLength();


	private:
		realnum computeBezier(realnum  a,realnum  supportA,  realnum  b, realnum  supportB, realnum  t);
		Point computeBezier(const Point& a, const Point& supportA,  const Point& b, const Point& supportB, realnum t);

		Point a;
		Point supportA;
		Point b;
		Point supportB;
};

#endif /* BEZIERCURVE_H_ */
