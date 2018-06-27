/*
 * Sensor.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: mafilipp
 *      Note: Implementation of the class Sensor.h
 */

#include "Sensor.h"

Sensor::Sensor(Particle * pc, int numPart, Map *map, double * cor)
{
	particleCloud = pc;
	numberOfParticle = numPart;
	mapPtr = map;
	correlation = cor;

	rangeMin = 0;
	rangeMax = 0;
	angleMin = 0;
	angleMax = 0;
	angleIncrement = 0;

	upToDate = false;
}

Sensor::~Sensor() {
}

void Sensor::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scanPtr = msg;

	if(!upToDate)
	{
		rangeMin = scanPtr->range_min;
		rangeMax = scanPtr->range_max;
		angleMin = scanPtr->angle_min;
		angleMax = scanPtr->angle_max;
		angleIncrement = scanPtr->angle_increment;
		upToDate = true;
	}
}

bool Sensor::isUpToDate() const {
	return upToDate;
}

void Sensor::setUpToDate(bool upToDate) {
	this->upToDate = upToDate;
}


void Sensor::sensorPredictionError()
{

	// First check if we get some data from the scan (sometimes can happen that all the measurement are NaN)
	bool allNaN = true;
	int idx = 0;

	for (double check = angleMin; check < angleMax; check = check + angleIncrement)
	{
		if(isnan(scanPtr->ranges[idx]))
		{
			idx = idx + 1;
		}
		else
		{
			allNaN = false;
			break;
		}
	}

	if(!allNaN)
	{
		// Get the data from the map
		double resolution = mapPtr->getResolution();
		int nColumn = mapPtr->getColumn();
		int nRow = mapPtr->getRow();

		// Iterate through all the particles
		for (int i = 0; i < numberOfParticle; i++)
		{
			idx = 0;
			int count = 0;
			double total = 0;

			// Find the distance to the wall
			double x, y, x_original, y_original, theta, upX, upY;

			x_original = particleCloud[i].getX();
			y_original = particleCloud[i].getY();
			theta = particleCloud[i].getTheta();

			// Check if they are inside the map
			if(    x > nColumn * resolution ||
				   x < 0 ||
				   y > nRow * resolution ||
				   y < 0)
			{
				correlation[i] = 0;
			}
			else
			{
				// Check if they are in a free space or "in the wall"
				if(mapPtr->isOccupied(x, y))
				{
					correlation[i] = 0;
				}
				else
				{
					// Here we are sure that the particle is inside the boundary and not in the wall

					for (double angle = angleMax; angle > angleMin; angle = angle - angleIncrement)

					{
						x = x_original;
						y = y_original;

						// Check if the scan measurement is NaN
						if(isnan(scanPtr->ranges[idx]))
						{
							idx = idx + 1;
							continue;
						}

						// Set the movement that we do during every step in x and y
						upX = cos(angle + theta) * resolution;
						upY = sin(angle + theta) * resolution;

						// Loop untill we hit a wall
						for(double dist = rangeMin; dist < rangeMax; dist = dist + resolution)
						{
							x = x + upX;
							y = y + upY;

							if(mapPtr->isOccupied(x,y))
							{
								// Calculate the error between what we should measure and what we actually measure
								total = total + std::abs(dist - scanPtr->ranges[idx]);
								count = count + 1;
								break;
							}
						}
					}
				}
			}

			// Calculate Correlation

			// Note: first we calculate the inverse of correlation:
			// If correlation[i] = 0, then we don't want to pick during the resampling the particle i,
			// but if correlation[i] != 0, then bigger is the number, bigger is the error, i.e. we want to pick the particle
			// with the smallest value

			if (count == 0)
			{
				correlation[i] = 0;
			}
			else
			{
				correlation[i] = count;
			}
		}

		// Set the "right" correlation
		double total = 0;
		for (int i = 0; i < numberOfParticle; i++)
		{
			total = total + correlation[i];
		}

		for (int i = 0; i < numberOfParticle; i++)
		{
			if(correlation[i] != 0)
			{
				correlation[i] = (total - correlation[i])/total;
			}
		}
	}
}


void Sensor::sensorPredictionMap()
{

	// First check if we get some data from the scan (sometimes can happen that all the measurement are NaN)
	bool allNaN = true;
	int idx = 0;

	for (double check = angleMax; check > angleMin; check = check - angleIncrement)
	{
		if(isnan(scanPtr->ranges[idx]))
		{
			idx = idx + 1;
		}
		else
		{
			allNaN = false;
			break;
		}
	}

	if(!allNaN)
	{
		double resolution = mapPtr->getResolution();
		int nColumn = mapPtr->getColumn();
		int nRow = mapPtr->getRow();

		for (int i = 0; i < numberOfParticle; i++)
		{
			idx = -1;
			int count = 0;
			double total = 0;

			// Find the distance to the wall
			double x, y, x_original, y_original, theta, upX, upY;

			x_original = particleCloud[i].getX();
			y_original = particleCloud[i].getY();
			theta = particleCloud[i].getTheta();

			x = x_original;
			y = y_original;

			// Check if they are inside the map
			if(    x > nColumn * resolution ||
				   x < 0 ||
				   y > nRow * resolution ||
				   y < 0)
			{
				correlation[i] = 0;
			}
			else
			{
				// Check if they are in a free space or "in the wall"
				if(mapPtr->isOccupied(x, y)) //-> I don't know why but so it make what we want to see in Rviz
				{
					correlation[i] = 0;
				}
				else
				{
					// Here we are sure that the particle is inside the boundary and not in the wall
					std::vector<std::pair<bool, bool>> maps; //Local and global Map

					// Vector that contains the index of the map cell that we already visited
					std::vector<int> alreadyUsedIndex;

					for (double angle = angleMax; angle > angleMin; angle = angle - angleIncrement)
					{
						idx = idx + 1;
						x = x_original;
						y = y_original;

						// Check if the scan measurement is NaN
						if(isnan(scanPtr->ranges[idx]))
						{
							continue;
						}

						// Set the movement that we do during every step in x and y
						upX = cos(angle + theta) * resolution;
						upY = sin(angle + theta) * resolution;

						for(double dist = rangeMin; dist < rangeMax; dist = dist + resolution)
						{
							bool endLocalMap = false;

							x = x + upX;
							y = y + upY;

							int currentIndex = mapPtr->getIndexXY(x,y);

							// Check if we already visited the cell
							bool alreadyVisited = false;

							for (std::vector<int>::iterator it = alreadyUsedIndex.begin(); it != alreadyUsedIndex.end(); ++it)
							{
								if(currentIndex == *it)
								{
									alreadyVisited = true;
									break;
								}
							}
							if(!alreadyVisited)
							{
								alreadyUsedIndex.push_back(currentIndex);

								// check global Map
								bool global, local;

								if(mapPtr->isOccupied(x,y))
								{
									global = true;
								}
								else
								{
									global = false;
								}

								// Update local map representation
								if( (dist - resolution < scanPtr->ranges[idx]) && (scanPtr->ranges[idx] < dist + resolution) )
								{
									local = true;
									endLocalMap = true;
								}
								else
								{
									local = false;
								}
								maps.push_back(std::make_pair(local,global));
							}

							if(endLocalMap)
							{
								break;
							}
						}
					}

					// Here I have all the points corresponding to the two map
					// Calculate the correlation between them
					double m, sum;
					double numerator, denominator, denominatorLocal, denominatorGlobal;

					for (std::vector<std::pair<bool, bool>>::iterator itMap = maps.begin(); itMap != maps.end(); ++itMap)
					{
						sum = sum + itMap->first + itMap->second;
					}

					m = 1/(2 * maps.size())*(sum);

					for (std::vector<std::pair<bool, bool>>::iterator itMap = maps.begin(); itMap != maps.end(); ++itMap)
					{
						numerator = numerator + (itMap->first - m) * (itMap->second - m);
						denominatorLocal = denominatorLocal + pow((itMap->first - m),2);
						denominatorGlobal = denominatorGlobal + pow((itMap->second - m),2);

					}
					denominator = sqrt(denominatorGlobal*denominatorLocal);
					double rho = numerator/denominator;
					correlation[i] = std::max(0.0, rho);
				}
			}
		}
	}
}


void Sensor::sensorPredictionCount()
{

	// First check if we get some data from the scan (sometimes can happen that all the measurement are NaN)
	bool allNaN = true;
	int idx = 0;

	for (double check = angleMin; check < angleMax; check = check + angleIncrement)
	{
		if(isnan(scanPtr->ranges[idx]))
		{
			idx = idx + 1;
		}
		else
		{
			allNaN = false;
			break;
		}
	}

	if(allNaN)
	{
		for(int i = 0; i < numberOfParticle; i++)
		{
			correlation[i] = 1;
		}
	}
	else
	{
		// Get the information from the map
		double resolution = mapPtr->getResolution();
		int nColumn = mapPtr->getColumn();
		int nRow = mapPtr->getRow();

		for (int i = 0; i < numberOfParticle; i++)
		{
			idx = -1;
			int count = 0;
			double total = 0;

			// Find the distance to the wall
			double x, y, x_original, y_original, theta, upX, upY;

			x_original = particleCloud[i].getX();
			y_original = particleCloud[i].getY();
			theta = particleCloud[i].getTheta();

			x = x_original;
			y = y_original;

			// Check if they are inside the map
			if(    x > nColumn * resolution ||
				   x < 0 ||
				   y > nRow * resolution ||
				   y < 0)
			{
				correlation[i] = 0;
			}
			else
			{
				// Check if they are in a free space or "in the wall"
				if(mapPtr->isOccupied(x, y))
				{
					correlation[i] = 0;
				}
				else
				{
					// Here we are sure that the particle is inside the boundary and not in the wall

					for (double angle = angleMax; angle > angleMin; angle = angle - 6 * angleIncrement)
					{
						idx = idx + 6;

						double range = scanPtr->ranges[idx];
						x = x_original;
						y = y_original;

						// Check if the scan measurement is NaN
						if(isnan(range))
						{
							continue;
						}

						upX = cos(angle + theta) * resolution;
						upY = sin(angle + theta) * resolution;

						for(double dist = rangeMin; dist < rangeMax; dist = dist + resolution)
						{
							x = x + upX;
							y = y + upY;

							// Calculate the reward: if we are with the laser scan inside a threshold given by the map
							// Calculation, then get +1, else +0
							if(mapPtr->isOccupied(x,y))
							{
								double distCurrent = dist - rangeMin + resolution;
								if( (distCurrent - 2*resolution < range) && (range < distCurrent + 2*resolution) )
									count = count + 1;
								break;
							}
						}
					}
					correlation[i] = count;
				}
			}
		}
	}
}
