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

#include <cpr_slider.h>

namespace cpr_robots
{
	CPRSlider::CPRSlider(const ros::NodeHandle nh)
	: nh_(nh)
	{
		ROS_INFO("[CPRSlider] Initializing node CPRSlider");
		// Reading parameters
		ReadParams();

		nrOfJoints_ = 4;
		jointID_[0] = 0x08;     // Front left
		jointID_[1] = 0x06;     // Front right
		jointID_[2] = 0x02;     // Back left
		jointID_[3] = 0x04;     // Back right

		length_ = 0.3;			// Values for Slider 100 and 150
		width_ = 0.270;
		diameter_ = 0.154;		// Value for Slider 150 only

		double max_vel = 123.0;						// Velocity from -123 to 123 (one byte)
		double rpm_at_max_vel = 0.56;				// Due to gear ratio and motor characteristics 
		double mps_at_max_vel = rpm_at_max_vel * diameter_ * 3.141;
		scale_translation_ =  max_vel / mps_at_max_vel;

		double complete_rotation = width_ * 3.141;	// Rotational speed given in rad/s
		scale_rotation_ = complete_rotation / (2.0 * 3.141) * scale_translation_;		
		
		double ab = (length_ + width_) / 2.0;
		//scale_fact_ = ab * scale_rotation_;
		scale_fact_ = ab * scale_translation_;

		pos_current_.position.x = 0.0;
		pos_current_.position.y = 0.0;
		pos_current_.orientation.z = 0.0;

		twist_current_time_ = ros::Time::now();
		twist_sent_time_ = ros::Time::now();

		ClearTwistFilter();

		serial_.Connect(pPort);
		Wait(1000);

		ResetErrors();
		Wait(2);
		EnableMotors();
		Wait(2);

		serialTimer_ = nh_.createTimer(ros::Duration(pPeriod), &CPRSlider::SerialTimerCallback, this);
		sub_twist_slider_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &CPRSlider::TwistCallback, this);
		odom_slider_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
	}
	
	CPRSlider::~CPRSlider()
	{
		DisableMotors();
		serial_.SetActive(false);
		serial_.Disconnect();
	}

	void CPRSlider::ReadParams()
	{
		nh_.param<std::string>("port", pPort, "/dev/ttyUSB0");
		ROS_INFO("[CPRSlider] Using port: %s", pPort.c_str());

		nh_.param("avg_filter_size", pAvgFilterSize, 40);
		ROS_INFO("[CPRSlider] Avg filter size: %i", pAvgFilterSize);
		
		nh_.param("max_notwist_time", pMaxNoTwistTime, 5.0);
		ROS_INFO("[CPRSlider] Max no twist time: %.2f", pMaxNoTwistTime);
		
		nh_.param("print_errors", pPrintErrors, 1);
		ROS_INFO("[CPRSlider] Printing errors: %i", pPrintErrors);

		nh_.param("send_period", pPeriod, 0.025);
		ROS_INFO("[CPRSlider] Sending period: %f", pPeriod);
		
		nh_.param<std::string>("odom_frame", pOdomFrame, "odom");
		ROS_INFO("[CPRSlider] Odom Frame: %s", pOdomFrame.c_str());
		
		nh_.param<std::string>("base_frame", pBaseFrame, "base_link");
		ROS_INFO("[CPRSlider] Base Frame: %s", pBaseFrame.c_str());
		
		nh_.param("publish_odom", pPublishOdom, 1);
		ROS_INFO("[CPRSlider] Publish Odom Messages: %i", pPublishOdom);
		
		nh_.param("publish_tf", pPublishTf, 1);
		ROS_INFO("[CPRSlider] Publish Tf Messages: %i", pPublishTf);
	}

	void CPRSlider::Wait(int ms){
		using namespace boost::posix_time;
		ptime start, now;
		time_duration passed;

		start = microsec_clock::universal_time();
		now = microsec_clock::universal_time();
		passed = now - start;
		while (passed.total_milliseconds() < ms) 
		{   
			now = microsec_clock::universal_time();
			passed = now - start;
		}   
	}

	void CPRSlider::ClearTwistFilter()
	{
		twist_recv_.clear();
		geometry_msgs::Twist firstTwist;
		firstTwist.linear.x = 0.0;
		firstTwist.linear.y = 0.0;
		firstTwist.angular.z = 0.0;
		twist_recv_.push_back(firstTwist);
	}

	void CPRSlider::ResetPosition()
	{
		DisableMotors();
		Wait(2);

		int l = 2;
		unsigned char data[8] = {1, 8, 0, 0, 0, 0, 0, 0};       // CAN message to reset the position
		for(int i = 0; i < nrOfJoints_; i++)
		{
			serial_.WriteMsg(jointID_[i], l, data);
			Wait(3);
		}

		pos_current_.position.x = 0.0;
		pos_current_.position.y = 0.0;
		pos_current_.orientation.z = 0.0;

		EnableMotors();
		Wait(2);
	}

	void CPRSlider::ResetErrors()
	{
		int l = 2;
		unsigned char data[8] = {1, 6, 0, 0, 0, 0, 0, 0};       // CAN message to reset errors
		for(int i = 0; i < nrOfJoints_; i++)
		{
			serial_.WriteMsg(jointID_[i], l, data);
			Wait(3);
		}
	}
		
	void CPRSlider::EnableMotors()
	{
		int l = 2;
		unsigned char data[8] = {1, 9, 0, 0, 0, 0, 0, 0};       // CAN message to enable the motors
		for(int i = 0; i < nrOfJoints_; i++)
		{
			serial_.WriteMsg(jointID_[i], l, data);
			Wait(3);
		}
	}

	void CPRSlider::DisableMotors()
	{
		int l = 2;
		unsigned char data[8] = {1, 10, 0, 0, 0, 0, 0, 0};      // CAN message to disable the motors
		for(int i = 0; i < nrOfJoints_; i++)
		{
			serial_.WriteMsg(jointID_[i], l, data);
			Wait(3);
		}
	}

	void CPRSlider::SetVelocities(double* vel)
	{
		unsigned char data[8] = {0x05, 0, 0x51, 0, 0, 0, 0, 0};
		int velaux;
		int vels[4];

		// Moving motors
		for(int i = 0; i < nrOfJoints_; i++)
		{
			velaux = 127 + (int)vel[i];
			if (velaux > 254) velaux = 255;
			if (velaux < 1) velaux = 0;
			data[1] = (unsigned char)velaux;                              	// Velocity
			vels[i] = velaux;
			serial_.WriteMsg(jointID_[i], 3, data);
			Wait(5);
		}

		ROS_DEBUG("[CPRSlider] Setting velocities: %.2i %.2i %.2i %.2i", vels[0], vels[1], vels[2], vels[3]);
	}

	void CPRSlider::SerialTimerCallback(const ros::TimerEvent& e)
	{
		if (twist_recv_.size() > pAvgFilterSize - 1)
		{
			twist_recv_.erase(twist_recv_.begin());
		}

		geometry_msgs::Twist last_twist;
		ros::Time last_twist_time;
		twist_mutex_.lock();
		last_twist = twist_current_;
		last_twist_time = twist_current_time_;
		twist_mutex_.unlock();

		ros::Time curr_time = ros::Time::now();
		if ((curr_time - last_twist_time).toSec() < pMaxNoTwistTime)
		{
			twist_recv_.push_back(last_twist);
		}
		else
		{
			geometry_msgs::Twist zeroTwist;
			zeroTwist.linear.x = 0.0;
			zeroTwist.linear.y = 0.0;
			zeroTwist.angular.z = 0.0;
			twist_recv_.push_back(zeroTwist);
		}
		
		// Average Twist
		geometry_msgs::Twist currTwist;
		double xtotal = 0.0;
		double ytotal = 0.0;
		double thtotal = 0.0;
		for (int i = 0; i < twist_recv_.size(); i++)
		{
			xtotal += twist_recv_[i].linear.x;
			ytotal += twist_recv_[i].linear.y;
			thtotal += twist_recv_[i].angular.z;
		}
		xtotal /= twist_recv_.size();
		ytotal /= twist_recv_.size();
		thtotal /= twist_recv_.size();
		
		currTwist.linear.x = xtotal;
		currTwist.linear.y = ytotal;
		currTwist.angular.z = thtotal;

		ROS_DEBUG("[CPRSlider] Twist velocities: %.2f m/s %.2f m/s %.2f rad/s", currTwist.linear.x, currTwist.linear.y, currTwist.angular.z);

		double velocities[4];
		InvKin(currTwist, velocities);
		SetVelocities(velocities);

		// Updating position
		ros::Time currTime = ros::Time::now();
		UpdatePosition(currTime);

		twist_sent_mutex_.lock();
		twist_sent_ = currTwist;
		twist_sent_time_ = currTime;
		twist_sent_mutex_.unlock();

		// Reading responses
		unsigned char response[8];
		int length;
		int errors[4];
		double voltage;
		double current = 0.0;
		for (int i = 0; i < nrOfJoints_; i++)
		{
			serial_.GetMsg(jointID_[i] + 1, &length, response);
			errors[i] = (int)response[0];
			current += 25.0 + 7.8 * (double)response[4];
		}
		voltage = 6.37 + (0.033 * (response[6]));
		ROS_DEBUG("[CPRSlider] Error Codes: %i %i %i %i, Voltage: %f V, Current: %f mA", errors[0], errors[1], errors[2], errors[3], voltage, current);

		if (pPrintErrors) {
			ProcessErrorCodes(errors);
		}
	}

	void CPRSlider::TwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
	{
		twist_mutex_.lock();
		geometry_msgs::Twist past_twist = twist_current_;
		ros::Time past_twist_time = twist_current_time_;
		twist_current_ = *msg;
		twist_current_time_ = ros::Time::now();

		// Limiting the accelerations.
		double dt = (twist_current_time_ - past_twist_time).toSec();
		double acc = 0.01;
		if ((fabs(twist_current_.linear.x - past_twist.linear.x) / dt) > acc)
		{
			if (twist_current_.linear.x > past_twist.linear.x) twist_current_.linear.x = past_twist.linear.x + acc;
			else twist_current_.linear.x = past_twist.linear.x - acc;
		}

		if ((fabs(twist_current_.linear.y - past_twist.linear.y) / dt) > acc)
		{
			if (twist_current_.linear.y > past_twist.linear.y) twist_current_.linear.y = past_twist.linear.y + acc;
			else twist_current_.linear.y = past_twist.linear.y - acc;
		}
		
		if ((fabs(twist_current_.angular.z - past_twist.angular.z) / dt) > acc)
		{
			if (twist_current_.angular.z > past_twist.angular.z) twist_current_.angular.z = past_twist.angular.z + acc;
			else twist_current_.angular.z = past_twist.angular.z - acc;
		}
		twist_mutex_.unlock();
	}

	void CPRSlider::InvKin(const geometry_msgs::Twist& vel_cart, double* vel_joint)
	{		
		// naming:
		// The front of the slider is the empty side, the back is the side with the main switch
		// Front left: vel[0] inverted		Front right: vel[1]	(looking from behind)		
		// Back left: vel[2] inverted		Back left: vel[3]

		// forward: all four wheels 
		vel_joint[0] = -scale_translation_ * vel_cart.linear.x;
		vel_joint[1] =  scale_translation_ * vel_cart.linear.x;
		vel_joint[2] = -scale_translation_ * vel_cart.linear.x;
		vel_joint[3] =  scale_translation_ * vel_cart.linear.x;

		// traverse: 
		vel_joint[0] +=  scale_translation_ * vel_cart.linear.y;
		vel_joint[1] +=  scale_translation_ * vel_cart.linear.y;
		vel_joint[2] += -scale_translation_ * vel_cart.linear.y;
		vel_joint[3] += -scale_translation_ * vel_cart.linear.y;

		// rotate: 
		vel_joint[0] +=  scale_fact_ * vel_cart.angular.z;
		vel_joint[1] +=  scale_fact_ * vel_cart.angular.z;
		vel_joint[2] +=  scale_fact_ * vel_cart.angular.z;
		vel_joint[3] +=  scale_fact_ * vel_cart.angular.z;
	} 

	void CPRSlider::ProcessErrorCodes(int* errors)
	{
		bool resetErrors = false;
		bool enableMotors = false;

		for (int i = 0; i < nrOfJoints_; i++)
		{
			// Getting error for this motor
			unsigned char error = (unsigned char)errors[i];

			// String for each motor
			std::string motor;
			if (jointID_[i] == 0x02)
			{
				motor = "BL";
			}
			else if (jointID_[i] == 0x04)
			{
				motor = "BR";
			}
			else if (jointID_[i] == 0x06)
			{
				motor = "FR";
			}
			else
			{
				motor = "FL";
			}

			// Bit 1
			if (error & 0x01) 
			{
				ROS_ERROR("[CPRSlider] Motor: %s - Supply voltage was too low or mC got stuck.", motor.c_str());
				resetErrors = true;
			}
			// Bit 2

			if (error & 0x02)
			{
				ROS_ERROR("[CPRSlider] Motor: %s - Velocity changes too fast.", motor.c_str());
				resetErrors = true;
			}

			// Bit 3
			if (error & 0x04)
			{
				ROS_WARN("[CPRSlider] Motor: %s - Motor disabled.", motor.c_str());
				enableMotors = true;
			}

			// Bit 4
			if (error & 0x08)
			{
				ROS_ERROR("[CPRSlider] Motor: %s - Interval without command was too long.", motor.c_str());
			}

			// Bit 5
			if (error & 0x10)
			{
				ROS_ERROR("[CPRSlider] Motor: %s - Position is too far away from the set point position.", motor.c_str());
			}

			// Bit 6
			if (error & 0x20)
			{
				ROS_ERROR("[CPRSlider] Motor: %s - The sequence of the quadrature encoder pulses did not fit.", motor.c_str());
			}

			// Bit 7
			if (error & 0x40)
			{
				ROS_ERROR("[CPRSlider] Motor: %s - Current value too high.", motor.c_str());
			}

			// Bit 8
			if (error & 0x80)
			{
				ROS_ERROR("[CPRSlider] Motor: %s - CAN error.", motor.c_str());
			}
		}

		if (resetErrors) 
		{
			ResetErrors();
			Wait(2);
		}
		
		if (enableMotors)
		{
			EnableMotors();
			Wait(2);
		}

		if (resetErrors || enableMotors)
		{
			ClearTwistFilter();
		}
	}

	void CPRSlider::UpdatePosition(const ros::Time& time)
	{
		geometry_msgs::Twist twist_old;
		ros::Time twist_old_time;
		twist_sent_mutex_.lock();
		twist_old = twist_sent_;
		twist_old_time = twist_sent_time_;
		twist_sent_mutex_.unlock();

		double dt = (time - twist_old_time).toSec();
		double vx = twist_old.linear.x;
		double vy = twist_old.linear.y;
		double vth = twist_old.angular.z;
		double th_current = pos_current_.orientation.z;

		// Compute odometry in a typical way given the velocities of the robot
		double delta_x = (vx * cos(th_current) - vy * sin(th_current)) * dt;
		double delta_y = (vx * sin(th_current) + vy * cos(th_current)) * dt;
		double delta_th = vth * dt;

		pos_current_.position.x += delta_x;
		pos_current_.position.y += delta_y;
		pos_current_.orientation.z += delta_th;

		ROS_DEBUG("[CPRSlider] Current pos: %.2f %.2f %.2f\n", pos_current_.position.x, pos_current_.position.y, pos_current_.orientation.z);

		// Since all odometry is 6DOF we'll need a quaternion created from yaw		
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos_current_.orientation.z);

		if (pPublishTf)
		{		
			// First, we'll publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;

			odom_trans.header.stamp = time;
			odom_trans.header.frame_id = pOdomFrame;
			odom_trans.child_frame_id = pBaseFrame;

			odom_trans.transform.translation.x = pos_current_.position.x;
			odom_trans.transform.translation.y = pos_current_.position.y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			// Send the transform
			odom_broadcaster_.sendTransform(odom_trans);
		}

		if (pPublishOdom)
		{
			// Next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = time;
			odom.header.frame_id = pOdomFrame;

			// Set the position
			odom.pose.pose.position.x = pos_current_.position.x;
			odom.pose.pose.position.y = pos_current_.position.y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			// Set the velocity
			odom.child_frame_id = pBaseFrame;
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.angular.z = vth;

			// Publish the message
			odom_slider_pub_.publish(odom);
		}
	}
}
