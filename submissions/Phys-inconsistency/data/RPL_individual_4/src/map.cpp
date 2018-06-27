/*
 * map.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Filippo Martinoni
 *      Note: Implementation of map.h
 */

#include "map.h"
#include <assert.h>
#include <ros/ros.h>

// Constructor and deconstructor
Map::Map(double robotSize)
{
	assert(robotSize>=0); // Ensure that the robot size is non-negative
	m_robotSize = robotSize;
	upToDate = false;
}

Map::Map(nav_msgs::OccupancyGrid map, double robotSize)
{
	assert(robotSize>=0); // Ensure that the robot size is non-negative
	m_robotSize = robotSize;

	//Create the map
	nColumn = map.info.height;
	nRow = map.info.width;
	m_map = map;
	origin = map.info.origin;
	resolution = map.info.resolution;
	m_robotSizePx = m_robotSize/resolution;
	upToDate = true;
	alreadyInflated = false;

}

Map::~Map()
{

}

// ROS Callback
void Map::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	ROS_INFO("map callback");

	// Fill the map with the message data
	m_map = *map;
	nColumn = m_map.info.height;
	nRow = m_map.info.width;
	origin = m_map.info.origin;
	resolution = m_map.info.resolution;
	m_robotSizePx = m_robotSize/resolution;
	upToDate = true;
	ROS_INFO("Element Map correctly updated");
}

// Inflate the map
void Map::inflate()
{
	if( m_robotSizePx > 1 and upToDate and not alreadyInflated) // if < 1px else it doesn't make sense to inflate
	{
		ROS_INFO("Inflating the map");

		// Define the filter kernel
		int sizeKernel;
		bool whiteAngle = false;
		// This tell us, since we want to have an odd filter, in the case that the filter robot size in px is
		// even, we add 1 px but in compensation we take away the angle on the filter kernel

		if (m_robotSizePx % 2 == 0)
		{
			sizeKernel = m_robotSizePx + 1;
			whiteAngle = true;
		}
		else
		{
			sizeKernel = m_robotSizePx;
		}

		int kernelCalc = floor(sizeKernel/2);
		// kernelCalc represent the part of the filter that could stay out of the image (map) during the convolution

		// Create the filter kernel
		int kernel[sizeKernel][sizeKernel];

		// Populate the kernel
		for(int row = 0; row < sizeKernel; row++)
		{
			for(int column = 0; column < sizeKernel; column++)
			{
				kernel[row][column] = 100;
			}
		}

		// if robot dimension compared to the filter is small, make the angle free (see above (whiteAngle description))
		if(whiteAngle)
		{
			kernel[0][0] = 0;
			kernel[sizeKernel - 1][0] = 0;
			kernel[0][sizeKernel - 1] = 0;
			kernel[sizeKernel - 1][sizeKernel - 1] = 0;
		}


		// Create the map that will be inflated
		Map inflatedMap(m_map,m_robotSize);

		// Calculation for the inflation

		// For each px
		for(int r = 0; r < nRow; r++)
		{
			for(int c = 0; c < nColumn; c++)
			{
				// For every element of the filter
				for(int rF = -kernelCalc; rF <= kernelCalc; rF++)
				{
					for(int cF = -kernelCalc; cF <= kernelCalc; cF++)
					{
						// Check if we are in the boundaries of the image
						if((r + rF >= 0) and (r + rF <= nRow - 1) and (c + cF >= 0) and (c + cF <= nColumn - 1))
						{
							if((m_map.data[getIndex(r+rF, c+cF)] > 0) and (kernel[rF + kernelCalc][cF + kernelCalc] > 0))
							{
								inflatedMap.m_map.data[c + r*nColumn] = 100;
							}
						}
					}
				}
			}
		}
		// inflate current map
		m_map.data = inflatedMap.m_map.data;
		alreadyInflated = true;
	}
}


bool Map::isOccupied(float x, float y)
{
	int index = getIndexXY(x, y);
	return (m_map.data[index] != 0);
}

int Map::getIndexXY(float x, float y)
{
		int row, column;

		column = floor(x/resolution);
		row = floor(y/resolution);

		return getIndex(row, column);
}


// Since map.data is an array, getIndex convert the matrix index (easy for calculation) in the aray index
int Map::getIndex(int row, int column)
{
	return row*nColumn + column;
}

// Getters and Setters
void Map::setRobotSize(double robotSize)
{
	assert(robotSize >= 0); // Ensure that the robot size is non-negative
	m_robotSize = robotSize;
}

double Map::getRobotSize() const {
	return m_robotSize;
}

const nav_msgs::OccupancyGrid& Map::getMap() const {
	return m_map;
}

void Map::setMap(const nav_msgs::OccupancyGrid& map) {
	m_map = map;
}

int Map::getColumn() const {
	return nColumn;
}

void Map::setColumn(int column) {
	nColumn = column;
	m_map.info.width = column;
}

int Map::getRow() const {
	return nRow;
}

void Map::setRow(int row) {
	nRow = row;
	m_map.info.height = row;
}

const geometry_msgs::Pose& Map::getOrigin() const {
	return origin;
}

void Map::setOrigin(const geometry_msgs::Pose& origin) {
	this->origin = origin;
	m_map.info.origin = origin;
}

const int* Map::getOriginPx() const {
	return originPx;
}

int Map::getM_robotSizePx() const {
	return m_robotSizePx;
}

void Map::setM_robotSizePx(int m_robotSizePx) {
	this->m_robotSizePx = m_robotSizePx;
}

float Map::getResolution() const {
	return resolution;
}

void Map::setResolution(float resolution) {
	this->resolution = resolution;
}

bool Map::isUpToDate() const {
	return upToDate;
}

void Map::setUpToDate(bool upToDate) {
	this->upToDate = upToDate;
}

bool Map::isAlreadyInflated() const {
	return alreadyInflated;
}

void Map::setAlreadyInflated(bool alreadyInflated) {
	this->alreadyInflated = alreadyInflated;
}
