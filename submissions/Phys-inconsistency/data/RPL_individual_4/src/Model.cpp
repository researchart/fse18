/*
 * Model.cpp
 *
 *  Created on: Dec 3, 2014
 *      Author: Filippo Martinoni
 *      Note: implementation of the class Model.h
 */

#include "Model.h"


Model::Model(Particle * pc, int nOp, alpha alpha)
{
	// Initialize all the parameter
	x_odom = 0;
	x_odom_old = 0;
	y_odom = 0;
	y_odom_old = 0;
	theta_odom = 0;
	theta_odom_old = 0;

	dRot1  = 0;
	dTrans = 0;
	dRot2 = 0;

	dRot1_hat = 0;
	dTrans_hat = 0;
	dRot2_hat = 0;

	x = 0;
	x_old = 0;
	y = 0;
	y_old = 0;
	theta = 0;
	theta_old = 0;

	alpha1 = alpha.alpha1;
	alpha2 = alpha.alpha2;
	alpha3 = alpha.alpha3;
	alpha4 = alpha.alpha4;

	// Connect the model with the particle cloud
	particleCloud = pc;
	numberOfParticle = nOp;

}

Model::~Model() {
}

void Model::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	x_odom = msg->pose.pose.position.x;
	y_odom = msg->pose.pose.position.y;
	theta_odom = tf::getYaw(msg->pose.pose.orientation);
}

double Model::sample(double variance)
{
	double stdDeviation = sqrt(variance);
	double number = (stdDeviation * (2* (double)rand()/RAND_MAX -1) + stdDeviation * (2* (double)rand()/RAND_MAX -1)) * sqrt(6)/2;
	number = std::max(number, -stdDeviation);
	number = std::min(number, stdDeviation);

	return number;
}

void Model::modelPrediction()
{
	dRot1 = atan2(y_odom - y_odom_old, x_odom - x_odom_old) - theta_odom_old;
	dTrans = sqrt( pow((x_odom_old - x_odom), 2) + pow((y_odom_old - y_odom), 2) );
	dRot2 = theta_odom - theta_odom_old - dRot1;

	// Calculate the noise
	for(int i = 0; i < numberOfParticle; i++)
	{
		double noise = 0;

		noise = sample(alpha1 * pow(dRot1, 2) + alpha2 * pow(dTrans, 2));
		dRot1_hat = dRot1   - noise;

		noise = sample(alpha3 * pow(dTrans, 2) + alpha4 * pow(dRot1, 2) + alpha4 * pow(dRot2, 2) );
		dTrans_hat = dTrans - noise;

		noise = sample(alpha1 * pow(dRot2, 2) + alpha2 * pow(dTrans, 2) );
		dRot2_hat = dRot2   - noise;

		x_old = particleCloud[i].getX();
		y_old = particleCloud[i].getY();
		theta_old = particleCloud[i].getTheta();

		x = x_old + dTrans_hat * cos( theta_old + dRot1_hat);
		y = y_old + dTrans_hat * sin( theta_old + dRot1_hat);
		theta = theta_old + dRot1_hat + dRot2_hat;

		particleCloud[i].setX(x);
		particleCloud[i].setY(y);
		particleCloud[i].setTheta(theta);
	}

	x_odom_old = x_odom;
	y_odom_old = y_odom;
	theta_odom_old = theta_odom;
}




