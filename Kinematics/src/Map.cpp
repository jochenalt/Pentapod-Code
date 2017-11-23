/*
 * Map.cpp
 *
 *  Created on: 27.07.2017
 *      Author: JochenAlt
 */

#include <Map.h>
#include "setup.h"
#include "basics/zip.h"
#include "basics/stringhelper.h"

Map::Map() {
	null();
}


Map::~Map() {
}


void Map::null() {
	gridWidth = 0;
	gridHeight = 0;
	mapSizeY = 0;
	mapSizeX = 0;
	generationNumber = 0;
	occupancy.clear();
}

bool Map::isNull() const {
	return (gridWidth == 0) && (gridHeight == 0);
}


Map::Map(const Map& m) {
	(*this) = m;
}


std::ostream& Map::serialize(std::ostream &out) const {
	out << "{ \"w\": ";
	serializePrim(out, gridWidth);
	out << ",\"h\":";
	serializePrim(out, gridHeight);
	out << ",\"r\":";
	serializePrim(out, gridSize);
	out << ",\"n\":";
	serializePrim(out, generationNumber);
	out << ",\"og\":\"";


	int len = gridWidth*gridHeight;
	std::ostringstream occupancyGridOut;

	for (int i = 0;i<len; i++) {
		occupancyGridOut << std::setfill('0') << std::setw(2) << std::hex << occupancy[i];
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
    	deserializePrim(in, gridWidth, ok);
    	parseCharacter(in, ',', ok);
    	parseString(in, ok); // "height"
    	parseCharacter(in, ':', ok);
    	deserializePrim(in, gridHeight, ok);
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
        	setGridDimension(gridHeight, gridWidth,gridSize);
    		occupancy.clear();
    		int occupancyLen = gridWidth*gridHeight;
    		string occupancyCompressedStr = decodeAndUncompress(occupancyStr,occupancyLen*2+256);

    		occupancy.resize(occupancyLen);
			int len = occupancyCompressedStr.length();
			int pos = 0;
			string s;
			for (int i = 0;i < len;i += 2) {
				s = occupancyCompressedStr.substr(i,2);
				occupancy[pos++] = hexToInt(s);
			}
    	}

    	parseCharacter(in, '}', ok);
	}
	return in;
}


void Map::operator=(const Map& m) {
	gridWidth = m.gridWidth;
	gridHeight = m.gridHeight;
	gridSize = m.gridSize;
	occupancy = m.occupancy;
	mapSizeX = m.mapSizeX;
	mapSizeY = m.mapSizeY;
	generationNumber = m.generationNumber;
}

void Map::setGridDimension(int pWidth, int pHeight, millimeter pGridSize) {
	gridWidth = pWidth;
	gridHeight = pHeight;
	gridSize = pGridSize;
	int len = gridWidth*gridHeight;
	occupancy.resize(len);
	for (int i = 0;i<len;i++) {
		occupancy[i] = UNKNOWN;
	}
	mapSizeX = gridWidth*gridSize;
	mapSizeY= gridHeight*gridSize;
}

Map::GridState  Map::getOccupancyByWorld(int x,int y) {
	int gridX = (x/gridSize + gridWidth/2);
	int gridY = (y/gridSize + gridHeight/2);

	unsigned  pos = gridX + gridWidth*gridY;
	if (pos < occupancy.size())
		return (GridState)occupancy[pos];
	return FREE;
};

int  Map::getValueByWorld(int x,int y) {
	int gridX = (x/gridSize + gridWidth/2);
	int gridY = (y/gridSize + gridHeight/2);

	unsigned  pos = gridX + gridWidth*gridY;
	if ((pos >= 0) && (pos < occupancy.size()))
		return occupancy[pos];
	return -1;
};

void Map::setOccupancyByGridCoord(int gridX,int gridY, GridState p) {
	unsigned pos = gridHeight*gridX + gridY;
	if ((pos >= 0) && (pos < occupancy.size()))
		occupancy[pos] = p;
	else
		pos = 0;
};

Map::GridState Map::getOccupancyByGridCoord(int gridX,int gridY) {
	unsigned pos = gridHeight*gridX + gridY;
	if ((pos >= 0) && (pos < occupancy.size()))
		return (GridState)occupancy[pos];

	return FREE;
}


int Map::getValueByGridCoord(int gridX,int gridY) {
	unsigned pos = gridHeight*gridX + gridY;
	if ((pos >= 0) && (pos < occupancy.size()))
		return occupancy[pos];

	return -1;
}

