/*
 * spatial.h
 *
 * Spatial data structures like point, orientation, vector and matrix
 *
 * Author: JochenAlt
 */

#ifndef SPATIAL_H_
#define SPATIAL_H_


#include "core.h"

#include "basics/serializer.h"
#include "basics/point.h"
#include "basics/orientation.h"

#include "setup.h"
#include "Util.h"

// All angles of one leg
class LimbAngles : public Serializable  {
public:
	friend ostream& operator<<(ostream&, const LimbAngles&);

	LimbAngles() {
		null();
	}
	virtual ~LimbAngles() {};


	void setDefaultPosition() {
		a[0] = 0.0;
		a[1] = 0.0;
		a[2] = 0.0;
		a[3] = 0.0;
	}

	static LimbAngles getDefaultPosition() { LimbAngles ja;ja.setDefaultPosition(); return ja; };

	LimbAngles(const realnum p0, const realnum p1, const realnum p2, const realnum p3) {
		a[0] = p0;
		a[1] = p1;
		a[2] = p2;
		a[3] = p3;
	}

	LimbAngles(const LimbAngles& par) {
		for (int i = 0;i<NumberOfLimbs;i++)
			a[i] = par.a[i];
	}

	void operator=(const LimbAngles& par) {
		for (int i = 0;i<NumberOfLimbs;i++)
			a[i] = par.a[i];
	}

	bool operator==(const LimbAngles& par) {
		for (int i = 0;i<NumberOfLimbs;i++)
			if (fabs(a[i]-par.a[i]) > floatPrecision)
				return false;
		return true;
	}

	bool operator!=(const LimbAngles& pos) {
		return !((*this) == pos);
	};

	realnum& operator[](int idx) {
		if ((idx >= 0) || ( idx < NumberOfLimbs))
			return a[idx];
		static realnum dummy = 0;
		return dummy;
	}
	const realnum& operator[](int idx) const {
		if ((idx >= 0) || ( idx < NumberOfLimbs))
			return a[idx];
		static realnum dummy = 0;
		return dummy;
	}

	void null() {
		for (int i = 0;i<NumberOfLimbs;i++)
			a[i] = 0.0;
	}
	bool isNull() {
		for (int i = 0;i<NumberOfLimbs;i++)
			if (a[i] != 0.0)
				return false;
		return true;
	}

	void operator+=(const LimbAngles& pos) {
		for (int i = 0;i<NumberOfLimbs;i++)
			a[i] += pos.a[i];
	};

	void operator-=(const LimbAngles& pos) {
		for (int i = 0;i<NumberOfLimbs;i++)
			a[i] -= pos.a[i];
	};

	void operator*=(const float x) {
		for (int i = 0;i<NumberOfLimbs;i++)
			a[i] *= x;
	};
	void operator/=(const float x) {
		for (int i = 0;i<NumberOfLimbs;i++)
			a[i] /= x;
	};

	LimbAngles operator*(float x) const {
		LimbAngles result(*this);
		result *= x;
		return result;
	};

	LimbAngles operator/(float x) const {
		LimbAngles result(*this);
		result /= x;
		return result;
	};

	LimbAngles  operator+(const LimbAngles& pos) const {
		LimbAngles result(*this);
		result += pos;
		return result;
	};
	LimbAngles  operator-(const LimbAngles& pos) const {
		LimbAngles result(*this);
			result -= pos;
			return result;
		};

	virtual std::ostream& serialize(std::ostream &out) const;
	virtual std::istream& deserialize(std::istream &in, bool &ok);

	private:
		realnum a[NumberOfLimbs];
};


// An aggregation of a point, an orientation and the corresponding angles
class Pose : public Serializable  {
	public:
		friend ostream& operator<<(ostream&, const Pose&);

		Pose() {
			null();
		};
		virtual ~Pose() {};
		Pose(const Pose& pose) {
			position = pose.position;
			orientation = pose.orientation;
		};
		Pose(const Point& pPosition) {
			null();
			position = pPosition;
		}
		Pose(const Point& pPosition, const Rotation& pOrientation) {
			position = pPosition;
			orientation = pOrientation;
		};

		Pose(const Point& pPosition, const Quaternion& pQuaternion) {
			position = pPosition;
			orientation = Rotation(pQuaternion);
		};

		Pose(const Point& pPosition, const EulerAngles& pEuler) {
			position = pPosition;
			orientation = EulerAngles(pEuler);
		};

		void operator= (const Pose& pose) {
			position = pose.position;
			orientation = pose.orientation;
		}

		void null() {
			orientation.null();
			position.null();
		}

		bool isNull() const {
			return position.isNull();
		}

		void moveTo(const Pose& b, realnum dT, realnum maxSpeed, realnum maxRotateSpeed) {
			position.moveTo(b.position, dT, maxSpeed);
			orientation.moveTo(b.orientation, dT, maxRotateSpeed);
		};

		float distance(const Pose& pPose) const {
			return sqrt((pPose.position[0]-position[0])*(pPose.position[0]-position[0]) +
						(pPose.position[1]-position[1])*(pPose.position[1]-position[1]) +
						(pPose.position[2]-position[2])*(pPose.position[2]-position[2]));
		}

		float length() const{
			return sqrt(sqr(position[0]) + sqr(position[1]) + sqr(position[2]));
		}

		bool operator==(const Pose& pPose) {
			return 	(position == pPose.position &&
					orientation == pPose.orientation);
		};

		bool operator!=(const Pose& pos) {
			return !((*this) == pos);
		};

		void operator+=(const Pose& pos) {
			position += pos.position;
			orientation += pos.orientation;
		};
		void operator-=(const Pose& pos) {
			position -= pos.position;
			orientation -= pos.orientation;
		};

		void operator*=(const float x) {
			position *= x;
			orientation *= x;
		};
		void operator/=(const float x) {
			position /= x;
			orientation /= x;
		};

		Pose operator*(float x) const {
			Pose result(*this);
			result *= x;
			return result;
		};

		Pose operator/(float x) const {
			Pose result(*this);
			result /= x;
			return result;
		};
		Pose  operator+(const Pose& pos) const {
			Pose result(*this);
			result += pos;
			return result;
		};

		Pose operator-(const Pose& pos) const {
			Pose result(*this);
			result -= pos;
			return result;
		};

		// add a vector to a pose by considering the applied pose as transformation(trans and rot)
		Pose add(const Pose& add) const;
		// substract a vector from the applied pose by considering both as transformation(trans and rot)
		Pose substract(const Pose& sub) const;

		virtual std::ostream& serialize(std::ostream &out) const;
		virtual std::istream& deserialize(std::istream &in, bool &ok);

		Point position;
		Rotation orientation;
};


class StampedPose : public Serializable  {
	public:
		friend ostream& operator<<(ostream&, const StampedPose&);

		StampedPose() {
			null();
		};
		virtual ~StampedPose() {};
		StampedPose(const StampedPose& p) {
			pose = p.pose;
			timestamp = p.timestamp;
		};
		StampedPose(const Pose& pPose, const milliseconds pTimestamp) {
			pose = pPose;
			timestamp = pTimestamp;
		};


		void operator= (const StampedPose& p) {
			pose = p.pose;
			timestamp = p.timestamp;
		}

		void null() {
			pose.null();
			timestamp = 0;
		}

		bool isNull() {
			return pose.isNull();
		}

		bool operator==(const StampedPose& p) {
			return 	((pose == p.pose) &&
					(timestamp == p.timestamp));
		};

		bool operator!=(const StampedPose& p) {
			return !((*this) == p);
		};

		virtual std::ostream& serialize(std::ostream &out) const;
		virtual std::istream& deserialize(std::istream &in, bool &ok);

		Pose pose;
		milliseconds timestamp;
};


class SpatialPID {
public:
	SpatialPID() {};
	~SpatialPID() {};

	void reset();
	Rotation getPID(Rotation error, realnum p, realnum i, realnum d, const Rotation &outMax);
private:
	Rotation lastError;
	Rotation errorIntegral;
	TimeSamplerStatic pidSampler;
};

// An aggregation of a point, an orientation and the corresponding angles
class LegPose : public Serializable  {
	public:
		friend ostream& operator<<(ostream&, const LegPose&);

		LegPose() {
			null();
		};
		virtual ~LegPose() {};
		LegPose(const LegPose& pose): LegPose() {
			position = pose.position;
			orientation = pose.orientation;
			angles = pose.angles;
		};
		LegPose(const Point& pPosition) {
			null();
			position = pPosition;
		}
		LegPose(const Point& pPosition, const Rotation& pOrientation) {
			position = pPosition;
			orientation = pOrientation;
			angles.null();
		};
		LegPose(const Point& pPosition, const Rotation& pOrientation, const realnum pGripperDistance, const LimbAngles& pAngles, const Point& pTcpDeviation) {
			position = pPosition;
			orientation = pOrientation;
			angles = pAngles;
		};

		void operator= (const LegPose& pose) {
			position = pose.position;
			orientation = pose.orientation;
			angles = pose.angles;
		}

		void null() {
			orientation.null();
			position.null();
			angles.null();

		}

		bool isNull() {
			return position.isNull();
		}

		void moveTo(const LegPose& b, realnum dT, realnum maxSpeed, realnum maxRotateSpeed) {
			position.moveTo(b.position, dT, maxSpeed);
			orientation.moveTo(b.orientation, dT, maxRotateSpeed);
		};

		float distance(const LegPose& pPose) const {
			return sqrt((pPose.position[0]-position[0])*(pPose.position[0]-position[0]) +
						(pPose.position[1]-position[1])*(pPose.position[1]-position[1]) +
						(pPose.position[2]-position[2])*(pPose.position[2]-position[2]));
		}

		float length() const{
			return sqrt(sqr(position[0]) + sqr(position[1]) + sqr(position[2]));
		}

		bool operator==(const LegPose& pPose) {
			return 	(position == pPose.position &&
					orientation == pPose.orientation);
		};

		bool operator!=(const LegPose& pos) {
			return !((*this) == pos);
		};

		void operator+=(const LegPose& pos) {
			position += pos.position;
			for (int i = 0;i<3;i++)
				orientation[i] += pos.orientation[i];
		};
		void operator-=(const LegPose& pos) {
			position -= pos.position;
			for (int i = 0;i<3;i++)
				orientation[i] -= pos.orientation[i];
		};

		void operator*=(const float x) {
			position *= x;

			for (int i = 0;i<3;i++)
				orientation[i] *= x;
		};
		void operator/=(const float x) {
			position /= x;
			for (int i = 0;i<3;i++)
				orientation[i] /= x;
		};

		LegPose operator*(float x) const {
			LegPose result(*this);
			result *= x;
			return result;
		};

		LegPose operator/(float x) const {
			LegPose result(*this);
			result /= x;
			return result;
		};
		LegPose  operator+(const LegPose& pos) const {
			LegPose result(*this);
			result += pos;
			return result;
		};

		LegPose operator-(const LegPose& pos) const {
			LegPose result(*this);
			result -= pos;
			return result;
		};

		virtual std::ostream& serialize(std::ostream &out) const;
		virtual std::istream& deserialize(std::istream &in, bool &ok);

		Point position;
		Rotation orientation;
		LimbAngles angles;
};

// A
template<class T> class PentaType: public Serializable  {
public:
	PentaType();
	virtual ~PentaType() {};
	PentaType(const PentaType& par);
	void operator=(const PentaType& par);
	bool operator==(const PentaType& par);
	bool operator!=(const PentaType& pos);
	T& operator[](int idx);
	const T& operator[](int idx) const;
	void null();
	bool isNull();
	PentaType<T> getRotatedAroundZ(angle_rad ori) const;
	void rotateAroundZ(angle_rad ori) ;

	friend ostream& operator<<(ostream& os, const PentaType<T>& p) {
		os << std::setprecision(2) <<  "(" << p[0] << "," << p[1] << "," << p[2] << "," << p[3]<< "," << p[4] << ")";
	    return os;
	}
	virtual std::ostream& serialize(std::ostream &out) const;
	virtual std::istream& deserialize(std::istream &in, bool &ok);
private:
	T a[NumberOfLegs];
};

#include "spatial.inl"

typedef PentaType<Pose> PentaPoseType;
typedef PentaType<Point> PentaPointType;
typedef PentaType<LimbAngles> LegAnglesType;

// some basic vector operations
bool 	almostEqual(const Point& a, const Point& b, realnum precision);
realnum distance(const Vector& a, const Vector& b);
realnum triangleHypothenusisLength(realnum a, realnum b); 		// pythagoras
realnum triangleHeightToC(realnum a, realnum b, realnum c);		// herons law
Vector 	orthogonalVector(const Vector& a, realnum l);			// return orthogonal vector with length and z = 0
Vector 	crossProduct(const Vector& a, const Vector& b);
void 	setVectorLength(Vector &a, realnum l);

// solve equation a*sin(alpha) + b*cos(alpha) = c
// returns two solutions in general, itherwise alpha2 is qnan
void solveTrgLinearCombinationWithEqualPhase(realnum a, realnum b, realnum c, realnum &alpha1, realnum &alpha2, bool& infiniteSolutions);
void createRotationMatrix(const Rotation &r, HomMatrix& m);

void testSpatial();
#endif /* SPATIAL_H_ */
