/*
 * Map.h
 *
 * A map is not a hashmap but a 2-dimensional grid with values. It is used as a slam map containing the Occupied/Free data
 * or for costmaps containing the cost induced by the distance to any wall.
 *
 *  Created on: 27.07.2017
 *      Author: JochenAlt
 */

#ifndef MAP_H_
#define MAP_H_

#include "basics/serializer.h"
#include "basics/types.h"
#include "basics/point.h"
#include "basics/spatial.h"

#include <vector>
#include "setup.h"


class Map  : public Serializable {
public:
	enum GridState { UNKNOWN=0, FREE = 1, OCCUPIED = 2 }; // in case of occupancy map
	Map();
	Map(const Map& map);
	void operator=(const Map& m);
	virtual ~Map();

	// clear out the map
	void null();

	// true if null has been called upfront
	bool isNull() const;

	// define the width and height of the map
	void setGridDimension(int pWidth, int pHeight, millimeter pGridSize);

	// return width passed in setGridDimension
	int getGridsWidth() const { return gridWidth; };

	// return height passed in setGridDimension
	int getGridsHeight() const { return gridHeight; };

	// return gridSize*width in [mm]
	millimeter_int getMapSizeX() const { return mapSizeX;  };

	// return gridSize*height in [mm]
	millimeter_int  getMapSizeY() const { return mapSizeY;  } ;

	// return size of one grid cell
	millimeter getGridSize() const { return gridSize; };

	// return the generation number which is increased on the server side after any modification. Useful to identify a modified map.
	int getGenerationNumber() const { return generationNumber; };
	void setGenerationNumber(int no) { generationNumber = no; };

	// set value of one grid cell. 0,0 is the grid cell in top left
	void setOccupancyByGridCoord(int x,int y, GridState p);

	// return the state of that cell that contains the world coordinates (x,y). (0.0) is in the middle of the map
	GridState getOccupancyByWorld(int x,int y) const ;

	// same but with grid index
	GridState getOccupancyByGridCoord(int x,int y) const;


	int getValueByWorld(int x,int y) const;
	int getValueByGridCoord(int x,int y) const;

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
