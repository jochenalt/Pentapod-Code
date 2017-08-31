/*
 * Map.cpp
 *
 *  Created on: 27.07.2017
 *      Author: JochenAlt
 */

#include <Map.h>
#include "setup.h"
#include "basics/zip.h"

Map::Map() {
	null();
}

Map::~Map() {
}


void Map::null() {
	noOfGridsX = 0;
	noOfGridsY = 0;
	mapSizeY = 0;
	mapSizeX = 0;
	generationNumber = 0;
	occupancy.clear();
}

bool Map::isNull() const {
	return (noOfGridsX == 0) && (noOfGridsY == 0);
}


Map::Map(const Map& m) {
	(*this) = m;
}


std::ostream& Map::serialize(std::ostream &out) const {
	out << "{ \"w\": ";
	serializePrim(out, noOfGridsX);
	out << ",\"h\":";
	serializePrim(out, noOfGridsY);
	out << ",\"r\":";
	serializePrim(out, gridSize);
	out << ",\"n\":";
	serializePrim(out, generationNumber);
	out << ",\"og\":\"";


	int len = noOfGridsX*noOfGridsY;
	const int gridsPerByte = 4;
	int s = 0;
	int gridsInByte = 0;
	std::ostringstream occupancyGridOut;

	for (int i = 0;i<len; i++) {
		s *= gridsPerByte;
		s += (int)occupancy[i];
		gridsInByte++;
		if (gridsInByte == gridsPerByte) {
			occupancyGridOut << (char)((int)0 + s);
			s = 0;
			gridsInByte = 0;
		}
	}
	if (gridsInByte != 0) {
		occupancyGridOut << (char)((int)0 + s);
	}
	out << compressAndEncode(occupancyGridOut.str());

	out << "\"}";
	return out;
}


std::istream& Map::deserialize(std::istream &in, bool &ok) {
	if (in) {
		null();

    	parseCharacter(in, '{', ok);
    	parseString(in, ok); // "width"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, noOfGridsX, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "height"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, noOfGridsY, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "resolution"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, gridSize, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "generationnumber"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, generationNumber, ok);

    	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "og"
    	parseCharacter(in, ':', ok);
    	string occupancyStr = parseString(in, ok);
    	if (ok) {
        	setGridDimension(noOfGridsY, noOfGridsX,gridSize);
    		occupancy.clear();
    		int occupancyLen = noOfGridsX*noOfGridsY;
    		string occupancyCompressedStr = decodeAndUncompress(occupancyStr,occupancyLen/3+256);

    		occupancy.resize(occupancyLen);
			int len = occupancyCompressedStr.length();
			const int gridsPerByte = 4;
			int pos = 0;
			for (int i = 0;i < len;i++) {
				int c = (int)(occupancyCompressedStr[i]-0);
				for (int j = 0;j<gridsPerByte;j++) {
					int occPos = pos+gridsPerByte-j-1;
					if (occPos < occupancyLen) {
						occupancy[occPos] = (GridState)(c % gridsPerByte);
						c /= gridsPerByte;
					}
				}
				pos += gridsPerByte;
			}
    	}

    	parseCharacter(in, '}', ok);
	}
	return in;
}


void Map::operator=(const Map& m) {
	noOfGridsX = m.noOfGridsX;
	noOfGridsY = m.noOfGridsY;
	gridSize = m.gridSize;
	occupancy = m.occupancy;
	mapSizeX = m.mapSizeX;
	mapSizeY = m.mapSizeY;
	generationNumber = m.generationNumber;
}

void Map::setGridDimension(int pWidth, int pHeight, millimeter pGridSize) {
	noOfGridsX = pWidth;
	noOfGridsY = pHeight;
	gridSize = pGridSize;
	int len = noOfGridsX*noOfGridsY;
	occupancy.resize(len);
	for (int i = 0;i<len;i++) {
		occupancy[i] = UNKNOWN;
	}
	mapSizeX = noOfGridsX*gridSize;
	mapSizeY= noOfGridsY*gridSize;
}

Map::GridState  Map::getOccupancyByWorld(int x,int y) {
	int gridX = (x/gridSize + noOfGridsX/2);
	int gridY = (y/gridSize + noOfGridsY/2);

	unsigned  pos = gridX + noOfGridsX*gridY;
	if (pos < occupancy.size())
		return occupancy[pos];
	return FREE;
};

void Map::setOccupancyByGridCoord(int gridX,int gridY, GridState p) {
	unsigned pos = noOfGridsY*gridX + gridY;
	if (pos < occupancy.size())
		occupancy[pos] = p;
	else
		pos = 0;
};

Map::GridState Map::getOccupancyByGridCoord(int gridX,int gridY) {
	unsigned pos = noOfGridsY*gridX + gridY;
	if (pos < occupancy.size())
		return occupancy[pos];

	return FREE;
}


