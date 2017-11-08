/*
 * Map.h
 *
 *  Created on: 27.07.2017
 *      Author: JochenAlt
 */

#ifndef MAP_H_
#define MAP_H_

#include "basics/serializer.h"
#include "basics/types.h"
#include "basics/point.h"
#include "spatial.h"

#include <vector>
#include "setup.h"


class Map  : public Serializable {
public:
	enum GridState { UNKNOWN=0, FREE = 1, OCCUPIED = 2 }; // in case of occupancy map
	Map();
	Map(const Map& map);

	void operator=(const Map& m);

	virtual ~Map();
	void null();
	bool isNull() const;
	void setGridDimension(int pWidth, int pHeight, millimeter pResolution);
	int getGridsWidth() { return gridWidth; };
	int getGridsHeight() { return gridHeight; };

	int getMapSizeX() { return mapSizeX;  };
	int getMapSizeY() { return mapSizeY;  };

	millimeter getGridSize() { return gridSize; };
	int getGenerationNumber() { return generationNumber; };
	void setGenerationNumber(int no) { generationNumber = no; };

	void setOccupancyByGridCoord(int x,int y, GridState p);
	GridState getOccupancyByWorld(int x,int y);
	GridState getOccupancyByGridCoord(int x,int y);
	int getValueByWorld(int x,int y);
	int getValueByGridCoord(int x,int y);

	virtual std::ostream& serialize(std::ostream &out) const;
	virtual std::istream& deserialize(std::istream &in, bool& ok);
	std::vector<int>& getVector() { return occupancy; };
private:
	int gridWidth;
	int gridHeight;
	int mapSizeX;
	int mapSizeY;
	int generationNumber;

	millimeter gridSize;
	std::vector<int> occupancy;
};

#endif /* MAP_H_ */
