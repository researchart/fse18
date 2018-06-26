/*
 * map.h
 *
 *  Created on: Oct 22, 2014
 *      Author: Filippo Martinoni
 *      Note: Class that receive the information from ros and saves them in his instance.
 *      	  It allows also to work on the map, like for example to inflate.
 */

#ifndef MAFILIPP_PATH_PLANNING_SRC_MAP_H_
#define MAFILIPP_PATH_PLANNING_SRC_MAP_H_

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

class Map {
public:
	// Constructors and destructors
	Map(double robotSize = 0);
	Map(nav_msgs::OccupancyGrid map, double robotSize = 0);
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
	~Map();

	// inflate the current Map by the robot's radius
	void inflate();

	// There is a wall at location x,y ?
	bool isOccupied(float x, float y);

	// Since map.data is an array, getIndex convert the matrix index (easy for calculation) in the aray index
	int getIndex(int row, int column);

	// Get the index of the cell that contain point (x,y)
	int getIndexXY(float x, float y);


	// Setters and getters
	const nav_msgs::OccupancyGrid& getMap() const;
	void setMap(const nav_msgs::OccupancyGrid& map);
	double getRobotSize() const;
	void setRobotSize(double robotSize);
	int getColumn() const;
	void setColumn(int column);
	int getRow() const;
	void setRow(int row);
	const geometry_msgs::Pose& getOrigin() const;
	void setOrigin(const geometry_msgs::Pose& origin);
	const int* getOriginPx() const;
	int getM_robotSizePx() const;
	void setM_robotSizePx(int radiusPx);
	float getResolution() const;
	void setResolution(float resolution);
	bool isUpToDate() const;
	void setUpToDate(bool upToDate);
	bool isAlreadyInflated() const;
	void setAlreadyInflated(bool alreadyInflated);

private:
	// Attributes
	double m_robotSize;
	nav_msgs::OccupancyGrid m_map;
	int nRow, nColumn;
	geometry_msgs::Pose origin;
	float resolution;
	int m_robotSizePx;
	int originPx[2];
	bool upToDate, alreadyInflated;

};


#endif /* MAFILIPP_PATH_PLANNING_SRC_MAP_H_ */
