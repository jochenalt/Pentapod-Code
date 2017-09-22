
#include "math.h"
#include <cmath>

#include "basics/point.h"
#include "basics/stringhelper.h"


ostream& operator<<(ostream& os, const Point& p)
{
	os << std::fixed << std::setprecision(1) << "P(" << p.x << "," << p.y << "," << p.z << ")";
    return os;
}


Point::Point() {
	null();
}

Point::Point(const Point& p) {
	x = p.x;
	y = p.y;
	z = p.z;
}


void Point::null() {
	x = 0.0;
	y = 0.0;
	z = 0.0;
};

Point::Point(realnum xP,realnum yP, realnum zP) {
	x = xP;
	y = yP;
	z = zP;
}

void Point::set(realnum pX, realnum pY,realnum pZ) {
	x = pX;
	y = pY;
	z = pZ;
}


bool Point::isNull() {
	return 	((abs(x) < floatPrecision) &&
			 (abs(y) < floatPrecision) &&
			 (abs(z) < floatPrecision));
};

void Point::translate(const Point& pPoint) {
	x += pPoint.x;
	y += pPoint.y;
	z += pPoint.z;
}

void Point::mirrorAt(const Point& pMirror,realnum scale) {
	x = pMirror.x + (pMirror.x-x)*scale;
	y = pMirror.y + (pMirror.y-y)*scale;
	z = pMirror.z + (pMirror.z-z)*scale;
}

void Point::mirrorAt(const Point& pMirror) {
	x = pMirror.x + (pMirror.x-x);
	y = pMirror.y + (pMirror.y-y);
	z = pMirror.z + (pMirror.z-z);
}

realnum Point::distance(const Point& pPoint) const {
	return sqrt(distanceSqr(pPoint));
}

realnum Point::distanceSqr(const Point& pPoint) const {
	return (pPoint.x-x)*(pPoint.x-x) + (pPoint.y-y)*(pPoint.y-y) +  (pPoint.z-z)*(pPoint.z-z);
}


realnum Point::length() const {
	return sqrt(x*x + y*y+ z*z);
}

realnum Point::scalarProduct(const Point& pPoint) const {
	return x*pPoint.x + y*pPoint.y + z*pPoint.z;
}

Point Point::orthogonalProjection(const Point& pLine) const {
	Point result = pLine;
	result *= scalarProduct(pLine)/ pLine.scalarProduct(pLine);
	return result;
}

Point Point::orthogonalProjection(const Point& pLineA, const Point& pLineB) const {
	Point translate(pLineA);
	Point selfTranslated=(*this);
	selfTranslated-= translate;
	Point line = pLineB;
	line -= translate;
	Point result = selfTranslated.orthogonalProjection(line);
	result += translate;
	return result;
}

Point Point::getPointOfLine(realnum ratio, const Point& target) {
	if (ratio > 1.0)
		ratio = 1.0;
	if (ratio < 0.0)
		ratio = 0.0;
	Point result(*this);
	result.x += ratio*(float)(target.x - x);
	result.y += ratio*(float)(target.y - y);
	result.z += ratio*(float)(target.z - z);
	return result;
}

Point::~Point() {

};

Point::Point(const HomVector& p) {
	x = p[X];
	y = p[Y];
	z = p[Z];
}

Point::Point(int args, const valarray<float>& vec) {
	for (int i = 0;i<args;i++)
		(*this)[i] = vec[i];
}


Point Point::getRotatedAroundZ(realnum alpha) const {
	realnum sa = sin(alpha);
	realnum ca = cos(alpha);

	Point rotated((ca*x - sa*y),(sa*x + ca*y),z );
	return rotated;
}

void Point::rotateAroundZ(realnum alpha) {
	realnum sa = sin(alpha);
	realnum ca = cos(alpha);

	Point copy(*this);
	x = ca*copy.x - sa*copy.y;
	y = sa*copy.x + ca*copy.y;
}


void Point::operator= (const Point& p) {
		x = p.x;
		y = p.y;
		z = p.z;
}

void Point::operator= (const HomVector& p) {
		x = p[X];
		y = p[Y];
		z = p[Z];
}


void Point::operator+= (const Point& p) {
	x += p.x;
	y += p.y;
	z += p.z;
}

void Point::operator-= (const Point& p) {
	x -= p.x;
	y -= p.y;
	z -= p.z;
}

void Point::operator*= (const realnum f) {
	x *= f;
	y *= f;
	z *= f;
}

void Point::operator/= (const realnum f) {
	float xrf= 1.0/f;
	x *= xrf;
	y *= xrf;
	z *= xrf;
}

Point Point::operator- (const Point& p) const{
	Point result= (*this);
	result -= p;
	return result;
}

Point Point::operator+ (const Point& p) const{
	Point result= (*this);
	result += p;
	return result;
}

Point Point::operator/ (const realnum f) const{
	Point result= (*this);
	result*= (1./f);
	return result;
}

Point Point::operator* (const realnum f) const{
	Point result= (*this);
	result*=f;
	return result;
}


bool Point::operator==(const Point& pos) {
	return ((abs(x-pos.x) < floatPrecision) && (abs(y - pos.y) <floatPrecision) && (abs(z -pos.z) < floatPrecision));
};

bool Point::operator!=(const Point& pos) {
	return !((*this) == pos);
};


realnum& Point::operator[] (int idx)  {
	switch (idx) {
		case X:	return x;break;
		case Y:	return y;break;
		case Z:	return z;break;
			default:
		break;
	}
	return x;
};

realnum Point::operator[] (int idx)  const {
	switch (idx) {
		case X:	return x;break;
		case Y:	return y;break;
		case Z:	return z;break;
		default:
		break;
	}
	return x;
};

void Point::moveTo(const Point& b, seconds dT, realnum maxSpeed) {
	realnum d = distance(b);
	realnum maxDistance = maxSpeed*dT;
	if (d  > maxDistance) {
		(*this) += (b -(*this))*maxDistance/d ;
	}
	else
		(*this) = b;
}

HomVector Point::getHomVector() const {
	HomVector result = { x,y,z,1.0 };
	return result;
}

// returns the vector, i.e. a 3-dimensional vector
Vector Point::getVector() const {
		Vector result = { x,y,z };
		return result;
}

std::ostream& Point::serialize(std::ostream &out) const {
	out << std::setprecision(4) << "{"
		<< "\"x\":" << floatToString(x,2) << ","
		<< "\"y\":" << floatToString(y,2) << ","
		<< "\"z\":" << floatToString(z,2) << "}";
	return out;
}

std::istream& Point::deserialize(std::istream &in, bool &ok) {
    if (in) {
    	parseCharacter(in, '{', ok);
    	parseString(in, ok);
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, x, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok);
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, y, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok);
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, z, ok);
    	parseCharacter(in, '}', ok);
    }
    return in;
}
