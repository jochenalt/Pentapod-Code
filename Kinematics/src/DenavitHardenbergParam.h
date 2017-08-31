/*
 * DenavitHardenbergParam.cpp
 *
 * DenavitHardenberg parameters are required by kinematics
 * Author: JochenAlt
 */

#ifndef DENAVITHARDENBERGPARAM_H_
#define DENAVITHARDENBERGPARAM_H_

#include "setup.h"
#include "Util.h"
#include "spatial.h"

class DenavitHardenbergParams{
public:

	// Turn type defines whether alpha or theta is the turning axis
	enum TurningAxis { ALPHA, THETA };
	DenavitHardenbergParams();
	DenavitHardenbergParams(TurningAxis turnType, const realnum alphaOffset, const realnum thetaOffset, const realnum pA, const realnum pD);

	void computeDHMatrix(realnum pAngle,HomMatrix& dh) const;
	const realnum getD() const { return val_d; };
	realnum getA() const { return val_a; };
	const realnum getAlpha() const { return val_alpha; };

private:
	realnum val_a;
	realnum val_d;
	realnum val_alpha;
	realnum val_theta;

	bool turnType;

};

#endif /* DENAVITHARDENBERGPARAM_H_ */
