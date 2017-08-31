
#include "DenavitHardenbergParam.h"



DenavitHardenbergParams::DenavitHardenbergParams() {
};


// initialize with the passed Denavit Hardenberg params and precompute sin/cos
DenavitHardenbergParams::DenavitHardenbergParams(TurningAxis newTurnType, const realnum pAlpha, const realnum pTheta, const realnum pA, const realnum pD) {
		val_a = pA;
		val_d = pD;
		val_alpha = pAlpha;
		val_theta = pTheta;

    	turnType = newTurnType;
}

// use DenavitHardenberg parameter and compute the DH-Transformation matrix with a given joint angle (theta)
void DenavitHardenbergParams::computeDHMatrix(realnum pAngle,HomMatrix& dh) const {

	realnum alpha = val_alpha;
	realnum theta = val_theta;

	if (turnType == THETA)
		theta += pAngle;
	else
		alpha += pAngle;

	realnum ct = cos(theta);
	realnum st = sin(theta);
	realnum ca = cos(alpha);
	realnum sa = sin(alpha);

	realnum  a = val_a;
	realnum d = val_d;
	dh = HomMatrix(4,4,
			{ ct, 	-st*ca,  st*sa,  a*ct,
			  st, 	 ct*ca, -ct*sa,	 a*st,
			  0,	 sa,		ca,		d,
			  0,	 0,		     0,		1});
}
