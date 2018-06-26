/*
 * Sensor.h
 *
 *  Created on: Dec 5, 2014
 *      Author: Filippo Martinoni
 *      Note: Class that implement different algorithm for the calculation of the sensor update.
 *      	  Unfortunately none of them work :'(
 */

#ifndef MAFILIPP_PARTICLE_FILTER_SRC_SENSOR_H_
#define MAFILIPP_PARTICLE_FILTER_SRC_SENSOR_H_


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>
#include <random>
#include <iostream>
#include <chrono>


#include "map.h"
#include "Model.h"
#include "Particle.h"


class Sensor
{
private:

	sensor_msgs::LaserScan::ConstPtr scanPtr;
	Particle * particleCloud;
	int numberOfParticle;
	Map * mapPtr;
	double * correlation;

	float rangeMin;
	float rangeMax;
	float angleMin;
	float angleMax;
	float angleIncrement;

	bool upToDate;


public:
	Sensor(Particle * pc, int numPart, Map *map, double * cor);
	virtual ~Sensor();

	// Callback for the laser scan
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

	// This function calculate the sensor prediction using as correlation factor the inverse of the error between the distance
	// particle-map_wall and the laserscan
	void sensorPredictionError();

	// This function calculate the sensor prediction using as correlation the matches between grid cell in the local map of the robot
	// and the global one
	void sensorPredictionMap();

	// This function calculate the sensor prediction using as correlation factor a reward that is increased for every scan that
	// match between what the robot should see and what it actually see (thanks to the laserscan)
	void sensorPredictionCount();

	bool isUpToDate() const;
	void setUpToDate(bool upToDate);
};

#endif /* MAFILIPP_PARTICLE_FILTER_SRC_SENSOR_H_ */
