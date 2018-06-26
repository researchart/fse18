/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Commonplace Robotics GmbH
 *  http://www.commonplacerobotics.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name Commonplace Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// Created on: 	Jan 12th, 2013
// Last Update:	Jan 21st, 2013


#ifndef CPR_SLIDER_H
#define CPR_SLIDER_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <cpr_robots_comm_rs232.h>

namespace cpr_robots
{

/**
* Kinematic for mobile Platform Slider
*
* This class holds the kinematic for the mobile platform Slider from Commonplace Robotics
* It subscribes to twist messages, transforms 
* these messages to joint speeds for the four mecanum
* wheels and then forwards these joint speeds to CPRCommRS232 to
* put them on the CAN bus.
*/
class CPRSlider{

	private:
		
		int nrOfJoints_;						/** Number of joints */
		int jointID_[4];						/** The CAN ids of the joint controller */
		double length_;							/** The length of the platform (distance of the axles in m) */
		double width_;							/** The width of the platform (distance of the wheel centers in m) */
		double diameter_;						/** The diameter of the wheels in m */
		double scale_translation_;				/** Factor to compute the velocity command */
		double scale_rotation_;					/** Factor to compute the velocity command */
		double scale_fact_;						/** Final value for increment the velocity */
			
		geometry_msgs::Pose pos_current_;		/** The current position in world coordinates */
		std::vector<geometry_msgs::Twist> twist_recv_;	/** Last twist message received */
	  	ros::NodeHandle nh_;
		ros::Subscriber sub_twist_slider_;
		ros::Publisher odom_slider_pub_;
		ros::Timer serialTimer_;
		tf::TransformBroadcaster odom_broadcaster_;
		
		geometry_msgs::Twist twist_current_;	/** Last twist received */
		ros::Time twist_current_time_;			/** Time of the last twist received */
		geometry_msgs::Twist twist_sent_;		/** Last twist sent to the platform */
		ros::Time twist_sent_time_;				/** Time of the last twist sent to the platform */
		
		CPRCommRS232 serial_;					/** The interface to the hardware  */

		boost::mutex twist_mutex_;
		boost::mutex twist_sent_mutex_;

		// Node params
		std::string pPort;
		int pAvgFilterSize;
		double pMaxNoTwistTime;
		int pPrintErrors;
		double pPeriod;
		std::string pOdomFrame;
		std::string pBaseFrame;
		int pPublishOdom;
		int pPublishTf;
		
		void TwistCallback(const geometry_msgs::Twist::ConstPtr& msg); 						/** Scales the velocities and forwards them to the hardware */
		void SerialTimerCallback(const ros::TimerEvent& e);									/** A timer for sending messages through the serial port */
		void SetVelocities(double* vel); 													/** Forward the velocities to the CAN hardware */
		void InvKin(const geometry_msgs::Twist& vel_cart, double* vel_joint);		/** Computes the joint values of the mecanum wheels from the velocities (m/s rad/s) */
		void UpdatePosition(const ros::Time& time);		/** Tracks the position in world coordinates */
		void Wait(int ms);																	/** Waits for ms miliseconds. We cannot use usleep */
		void ProcessErrorCodes(int* errors);												/** Writes a message from the error codes */
		void ClearTwistFilter();															/** Clears the average twist filter */
		void ReadParams();																	/** Reads the configuration parameters */

	public:
		CPRSlider(const ros::NodeHandle nh);
		~CPRSlider();
		void ResetErrors(); 						/** Reset the errors of all four joint controllers */
		void ResetPosition(); 						/** Reset the joint controllers position to a mid position */
		void EnableMotors(); 						/** Enables the motors in the controller hardware */
		void DisableMotors();						/** Disables the controllers in the motor hardware */
};

}

#endif
