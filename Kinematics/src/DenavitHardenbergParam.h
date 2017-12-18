/*
 * DenavitHardenbergParam.cpp
 *
 * DenavitHardenberg parameters are required by forward kinematics.
 * They define a joint wrt to a rotation/translation around x or z-axis.
 *
 * Author: JochenAlt
 */

#ifndef DENAVITHARDENBERGPARAM_H_
#define DENAVITHARDENBERGPARAM_H_

#include "basics/util.h"
#include "basics/spatial.h"
#include "setup.h"

class DenavitHardenbergParams{
public:

	enum TurningAxis { TURN_AROUND_X, TURN_AROUND_Z };
	DenavitHardenbergParams() {};
	DenavitHardenbergParams(const DenavitHardenbergParams& dh);
	void operator=(const DenavitHardenbergParams& dh);

	DenavitHardenbergParams(TurningAxis newTurnType, const realnum alphaOffset /* alpha is x-axis */, const realnum thetaOffset /* alpha is z-axis */, const realnum pA, const realnum pD);

	// compute transformation matrix as a result of a turning angle that contains the rotation and translation of the parameters set earlier
	void computeDHMatrix(realnum pAngle,HomogeneousMatrix& dh) const;

	// return the length of the translation along the z.axis
	const realnum getZTranslation() const { return translate_along_z; };

	// return the length of the translation along the x-axis
	realnum getXTranslation() const { return translate_along_x; };

private:
	realnum translate_along_x = 0;
	realnum translate_along_z = 0;
	realnum rotate_around_x = 0;
	realnum rotate_around_z = 0;

	TurningAxis turnType = TURN_AROUND_X;
};

#endif /* DENAVITHARDENBERGPARAM_H_ */
