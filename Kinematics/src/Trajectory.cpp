/*
 * Trajectory.cpp
 *
 *  Created on: 27.07.2017
 *      Author: JochenAlt
 */

#include <Trajectory.h>
#include "setup.h"
#include "basics/zip.h"
#include "basics/stringhelper.h"

ostream& operator<<(ostream& os, const Trajectory& p)
{
	os << std::setprecision(2) << "(";
	for (unsigned int i = 0;i<p.path.size(); i++) {
		if (i>0)
			os << ",";
		os << "{ pose=" << p.path[i].pose << ", t=" << p.path[i].timestamp << "}";
	}
	os << ")";
	return os;
}


std::ostream& Trajectory::serialize(std::ostream &out) const {
	out << "{\"path\":";
	serializeVectorOfSerializable(path, out);
	out << ",\"n\":";
	serializePrim(out, generationNumber);
	out << "}";
	return out;
}

std::istream& Trajectory::deserialize(std::istream &in, bool &ok) {
    if (in) {
    	bool ok = true;
      	parseCharacter(in, '{', ok);
    	parseString(in, ok); // "path"
    	parseCharacter(in, ':', ok);
    	deserializeVectorOfSerializable(in, path, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "generationnumber"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, generationNumber, ok);
      	parseCharacter(in, '}', ok);
    }
    return in;
}
