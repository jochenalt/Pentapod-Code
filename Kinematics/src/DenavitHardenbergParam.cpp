
#include "DenavitHardenbergParam.h"


	DenavitHardenbergParams::DenavitHardenbergParams(const DenavitHardenbergParams& dh) {
		translate_along_x = dh.translate_along_x;
		translate_along_z = dh.translate_along_z;
		rotate_around_x = dh.rotate_around_x;
		rotate_around_z = dh.rotate_around_z;
		turnType = dh.turnType;
	};

	void DenavitHardenbergParams::operator=(const DenavitHardenbergParams& dh) {
		translate_along_x = dh.translate_along_x;
		translate_along_z = dh.translate_along_z;
		rotate_around_x = dh.rotate_around_x;
		rotate_around_z = dh.rotate_around_z;
		turnType = dh.turnType;
	};



// initialize with the passed Denavit Hardenberg params and precompute sin/cos
DenavitHardenbergParams::DenavitHardenbergParams(TurningAxis newTurnType, const realnum pAlpha, const realnum pTheta, const realnum pA, const realnum pD) {
		translate_along_x = pA;
		translate_along_z = pD;
		rotate_around_x = pAlpha;
		rotate_around_z = pTheta;
    	turnType = newTurnType;
}

// use DenavitHardenberg parameter and compute the DH-Transformation matrix with a given joint angle
void DenavitHardenbergParams::computeDHMatrix(realnum pAngle,HomMatrix& dh) const {

	realnum alpha = rotate_around_x;
	realnum theta = rotate_around_z;

	if (turnType == TURN_AROUND_Z)
		theta += pAngle;
	else
		alpha += pAngle;

	realnum ct = cos(theta);
	realnum st = sin(theta);
	realnum ca = cos(alpha);
	realnum sa = sin(alpha);

	realnum  a = translate_along_x;
	realnum d = translate_along_z;
	dh = HomMatrix(4,4,
			{ ct, 	-st*ca,  st*sa,  a*ct,
			  st, 	 ct*ca, -ct*sa,	 a*st,
			  0,	 sa,		ca,		d,
			  0,	 0,		     0,		1});
}
