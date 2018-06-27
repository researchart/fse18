/*
 * main.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Filippo Martinoni
 *      Note: The core of the node particle_filter
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <random>
#include <iostream>
#include <chrono>


#include "map.h"
#include "Model.h"
#include "Particle.h"
#include "Sensor.h"
#include "Resampler.h"


using namespace std;

//** Helpfunction used by the program

// Trhansform degree in radian
double deg2Rad(double grad)
{
	return grad/180*M_PI;
}

// Publish the pose of more particle in order to visualize them in Rviz
geometry_msgs::PoseArray publishParticleArray(Particle * particleCloud, int numberOfParticle)
{
	geometry_msgs::PoseArray poseArray;
	geometry_msgs::Pose pose;

	poseArray.header.frame_id = "odometry_link";
	poseArray.header.stamp = ros::Time();

	// For all particle, draw in Rviz the pose
	for (int i = 0; i < numberOfParticle; i ++)
	{
		pose.position.x = particleCloud[i].getX();
		pose.position.y = particleCloud[i].getY();
		pose.position.z = 0.0;
		pose.orientation = tf::createQuaternionMsgFromYaw(particleCloud[i].getTheta());

		poseArray.poses.push_back(pose);
	}

	return poseArray;
}

// Main
int main(int argc, char **argv)
{
  //** Initialize the node
  ros::init(argc, argv, "particle_filter");

  //** Define variables
  double robotRadius = 0.08;
  int numberOfParticle;
  alpha alpha;
  int frequency;

  // Get the variable from the parameter file
  ros::NodeHandle n_paramters = ros::NodeHandle("~");

  // unfortunately ros::param doesn't know about unsigned types!
  n_paramters.param<int>("number_of_particle", numberOfParticle, 800);
  n_paramters.param<int>("frequency", frequency, 10);
  n_paramters.param<double>("alpha1", alpha.alpha1, 0.0005);
  n_paramters.param<double>("alpha2", alpha.alpha2, 0.2);
  n_paramters.param<double>("alpha3", alpha.alpha3, 0.5);
  n_paramters.param<double>("alpha4", alpha.alpha4, 0.00001);
  n_paramters.param<double>("robot_radius", robotRadius, 0.08);

  // Creating the object with which we will work
  // Map
  Map gridMap(robotRadius);
  // Particle Cloud
  Particle * particleCloud = new Particle[numberOfParticle];
  // Correlation
  double * correlation = new double[numberOfParticle];
  // Model
  Model model(particleCloud, numberOfParticle, alpha);
  // Sensor
  Sensor sensor(particleCloud, numberOfParticle, &gridMap, correlation);
  // Resampler
  Resampler resample(particleCloud, numberOfParticle, &gridMap, correlation);

  //** Initialize node
  ros::NodeHandle n;

  // Set publisher and subscriber
  ros::Subscriber mapSubscriberCallback = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &Map::mapCallback, &gridMap);
  ros::Subscriber laserSubscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &Sensor::laserCallback, &sensor);
  ros::Subscriber odomSubscriber = n.subscribe<nav_msgs::Odometry>("/thymio_driver/odometry", 10, &Model::odomCallback, &model);

  ros::Publisher particle_pose = n.advertise<geometry_msgs::PoseArray>( "/particle_pose", 0 );

  //** Initialize Element

  // First wait until we get the map
  while ( !(gridMap.isUpToDate() && sensor.isUpToDate()) )
  {
    ros::spinOnce();
  }

  // Initialize the particle with random pose
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);
  std::uniform_real_distribution<double> distribution(0.0,1.0);

  double number;

  for (int i = 0; i < numberOfParticle; i++)
  {
	  number = distribution(generator);
	  particleCloud[i].setX(gridMap.getColumn() * gridMap.getResolution() * number);

	  number = distribution(generator);
	  particleCloud[i].setY(gridMap.getRow() * gridMap.getResolution() * number);

	  number = distribution(generator);
	  particleCloud[i].setTheta( deg2Rad(number*360) );
  }

  // Note: the particle that are in a wall are removed at the first iteration by the resample algorithm!


  //** Start the algorithm

  ros::Rate loop_rate(frequency);

  ROS_INFO("Start Particle Filter");

  while (ros::ok())
  {
	  ros::spinOnce();

	  model.modelPrediction();

	  particle_pose.publish(publishParticleArray(particleCloud, numberOfParticle));

	  sensor.sensorPredictionError();

	  resample.resampleUniversal();

	  loop_rate.sleep();
  }


  return 0;
}
