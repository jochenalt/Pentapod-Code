/*
 * orientation.h
 *
 * Quarternions, EulerAngles (roll/nick/yaw), and Rotation class (x,y,z)
 *
 */

#ifndef ORIENTATION_H_
#define ORIENTATION_H_


#include "basics/serializer.h"
#include "basics/types.h"
#include "basics/point.h"


class EulerAngles;
class Quaternion;
class Rotation;

class Quaternion {
	friend class EulerAngles;
	friend class Rotation;
public:
	Quaternion() { x = 0; y = 0; z = 0;w = 0; };
	Quaternion(const Quaternion &q) { x = q.x; y = q.y; z = q.z; w = q.w; };
	Quaternion(realnum px, realnum py, realnum pz, realnum pw) { x = px; y = py; z = pz; w = pw; };

	Quaternion(const EulerAngles &e);
	Quaternion(const Rotation &r);

	realnum x;
	realnum y;
	realnum z;
	realnum w;
};

class EulerAngles {
	friend class Quaternion;
	friend class Rotation;
public:
	EulerAngles() { roll = 0; nick = 0; yaw = 0; };
	EulerAngles(realnum proll, realnum pnick, realnum pyaw) { roll = proll; nick = pnick; yaw = pyaw; };
	EulerAngles(const EulerAngles &q);
	EulerAngles(const Quaternion &q);
	EulerAngles(const Rotation &q);

	realnum roll;
	realnum nick;
	realnum yaw;
};


class Rotation {
	 friend ostream& operator<<(ostream&, const Rotation&);
	public:
		Rotation () { x = 0; y = 0; z = 0;};
		virtual ~Rotation() {};

		Rotation(angle_rad xP,angle_rad yP, angle_rad zP) {
			x = xP;
			y = yP;
			z = zP;
		}
		Rotation(const EulerAngles& eu) {
			x = eu.roll;
			y = eu.nick;
			z = eu.yaw;
		}

		Rotation(const Quaternion& q) {
			Rotation(EulerAngles(q));
		}


		Rotation(const HomogeneousVector& p) {
			x = p[X];
			y = p[Y];
			z = p[Z];
		}
		Rotation(const int p[3]) {
			x = p[X];
			y = p[Y];
			z = p[Z];
		}
		Rotation(const angle_rad p[3]) {
			x = p[X];
			y = p[Y];
			z = p[Z];
		}

		Rotation(const Rotation& p) {
			x= p.x;
			y= p.y;
			z= p.z;
		};

		Rotation getRotatedAroundZ(realnum alpha) const {
			realnum sa = sin(alpha);
			realnum ca = cos(alpha);

			Rotation rotated((ca*x - sa*y),(sa*x + ca*y),z );
			return rotated;
		}

		void rotateAroundZ(realnum alpha) {

			Rotation tmp= getRotatedAroundZ(alpha);
			Rotation result;
			result.x = tmp.x;
			result.y = tmp.y;
			result.z = tmp.z;
		}

		void moveTo(const Rotation& b, realnum dT, realnum maxAngularSpeed);

		void operator=(const Rotation& rot) {
			x = rot.x;
			y = rot.y;
			z = rot.z;
		};
		void operator+=(const Rotation& rot) {
			x += rot.x;
			y += rot.y;
			z += rot.z;
		};

		realnum& operator[] (int idx)  {
			switch (idx) {
				case X:	return x;break;
				case Y:	return y;break;
				case Z:	return z;break;
					default:
				break;
			}
			return x;
		};

		realnum operator[] (int idx)  const {
			switch (idx) {
				case X:	return x;break;
				case Y:	return y;break;
				case Z:	return z;break;
				default:
				break;
			}
			return x;
		};


		void operator-=(const Rotation& rot) {
			x -= rot.x;
			y -= rot.y;
			z -= rot.z;
		};


		void operator*=(float f) {
			x *= f;
			y *= f;
			z *= f;
		};

		void operator/=(float f) {
			x /= f;
			y /= f;
			z /= f;
		};

		Rotation operator*(const float f) const {
			Rotation result(*this);
			result.x *= f;
			result.y *= f;
			result.z *= f;
			return result;
		};

		Rotation operator/(const float f) const {
			Rotation result(*this);
			result *= (1./f);
			return result;
		};

		Rotation operator+(const Rotation& rot) const {
			Rotation result(*this);
			result += rot;
			return result;
		};

		Rotation operator-(const Rotation& rot) const {
			Rotation result(*this);
			result -= rot;
			return result;
		};
		bool operator==(const Rotation& pos) {
			return (x == pos.x) && (y == pos.y) && (z == pos.z);
		};

		bool operator!=(const Rotation& pos) {
			return !((*this) == pos);
		};

		realnum distance(const Rotation& p) const {
				return sqrt((p.x-x)*(p.x-x) + (p.y-y)*(p.y-y)+  (p.z-z)*(p.z-z));
		}

		void null() { x = 0;y = 0;z = 0; };
		bool isNull() { return (x == 0) && (y == 0) && (z == 0); };

		void limit(const Rotation &min, const Rotation &max);
		virtual std::ostream& serialize(std::ostream &out) const;
		virtual std::istream& deserialize(std::istream &in, bool &ok);

		angle_rad x;
		angle_rad y;
		angle_rad z;
};



#endif
