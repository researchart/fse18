/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Commonplace Robotics GmbH
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
// Last Update:	


#ifndef cpr_robots_driver_ctrl_loop_H
#define cpr_robots_driver_ctrl_loop_H

#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <cpr_teleop_keyboard.h>


namespace cpr_robots{



//* Control loop for teleop
/**
* This class starts an main loop that constantly does:
* 1. Get the velocities from cCPRTeleopKeyboard
* 2. Scale them
* 3. Publish a twist message with these velocities
*/
class CPRCtrlLoop{

	private:

	geometry_msgs::Twist cmd_slider_velocities;		/**< velocities of the Slider in mm/s and rad/s */
  	
	ros::NodeHandle n_;
	ros::Publisher twist_slider_pub;			/**< publishes the velocities  */
	
	cpr_robots::CPRTeleopKeyboard kbd;			/**< forwards the user keyboard input */

	bool flag_stop_requested;				/**< stops the main loop */

	public:
	CPRCtrlLoop();
	~CPRCtrlLoop();

	/*!
    	* \brief	General initialization 
	* Initializes the publisher
	*/
	int init();

	/*!
    	* \brief	The main loop 
	* Reads from the keyboard, checks the values and publishes them
	*/
	void ctrlLoop();

	/*!
    	* \brief	Check the velocities for validity 
	* min max etc
	*/
	void checkCmdValues();


};



}



#endif
