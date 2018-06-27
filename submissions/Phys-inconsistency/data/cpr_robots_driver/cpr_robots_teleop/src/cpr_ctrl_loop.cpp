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


#include <cpr_ctrl_loop.h>



void quit(int sig)
{
  	ros::shutdown();
  	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cpr_teleopkb");
	signal(SIGINT,quit);

	cpr_robots::CPRCtrlLoop loop;
	loop.init();
	loop.ctrlLoop();
			
	return(0);
}




namespace cpr_robots{


	//****************************************************************
	CPRCtrlLoop::CPRCtrlLoop(){

	}

	//****************************************************************
	CPRCtrlLoop::~CPRCtrlLoop(){

	}

	//****************************************************************
	// init the publisher and the keyboard
	int CPRCtrlLoop::init(){

		twist_slider_pub = n_.advertise<geometry_msgs::Twist>("twistSlider", 1);

		kbd.init();

		flag_stop_requested = false;

		return 0;
	}

	//****************************************************************
	void CPRCtrlLoop::ctrlLoop()
	{
  		geometry_msgs::Twist twist_Tmp;  		

  		puts("---------------------------");
		puts("Starting Teleop Control Loop");
  		puts("---------------------------");

		double scale_Trans = 0.4;
		double scale_Ori = 0.5;

 	 	for(;;)
  		{
   		
			if(flag_stop_requested)
				break;

			ros::Duration(0.1).sleep();	// main loop with 10 Hz. 
							// Can be faster up to 50 Hz, but for the Slider this should not be necessary.
			
			// Get the keyboard input values and add them (Scaled) to the current velocities
			kbd.getVelocities(&twist_Tmp);
			cmd_slider_velocities.linear.x += scale_Trans * twist_Tmp.linear.x;
			cmd_slider_velocities.linear.y += scale_Trans * twist_Tmp.linear.y;
			cmd_slider_velocities.linear.z += scale_Trans * twist_Tmp.linear.z;
			cmd_slider_velocities.angular.x += scale_Ori * twist_Tmp.angular.x;
			cmd_slider_velocities.angular.y += scale_Ori * twist_Tmp.angular.y;
			cmd_slider_velocities.angular.z += scale_Ori * twist_Tmp.angular.z;
			   	       

			// the values are slowly decreasing when there is no keyboard input
			double dec_Trans = 0.2;
			double dec_Ori = 0.1;
			if(twist_Tmp.linear.x == 0.0){					// only if no new command 
				if(cmd_slider_velocities.linear.x >= dec_Trans)		// if bigger than bias
					cmd_slider_velocities.linear.x -= dec_Trans;	
				else if(cmd_slider_velocities.linear.x <= -dec_Trans)	// smaller than -bias
					cmd_slider_velocities.linear.x += dec_Trans;
				else
					cmd_slider_velocities.linear.x = 0.0;		// around 0
			}
			if(twist_Tmp.linear.y == 0.0){
				if(cmd_slider_velocities.linear.y >= dec_Trans)
					cmd_slider_velocities.linear.y -= dec_Trans;
				else if(cmd_slider_velocities.linear.y <= -dec_Trans)
					cmd_slider_velocities.linear.y += dec_Trans;
				else
					cmd_slider_velocities.linear.y = 0.0;
			}
			if(twist_Tmp.angular.z == 0.0){
				if(cmd_slider_velocities.angular.z >= dec_Ori)
					cmd_slider_velocities.angular.z -= dec_Ori;
				else if(cmd_slider_velocities.angular.z <= -dec_Ori)
					cmd_slider_velocities.angular.z += dec_Ori;
				else
					cmd_slider_velocities.angular.z = 0.0;
			}



 

			checkCmdValues();

			twist_slider_pub.publish(cmd_slider_velocities);


  		}

		puts("---------------------------");
		puts("Closing Control Loop");
  		puts("---------------------------");

	

	} //endof CtrlLoop


	//***********************************************************************
	// check for min and max values	
	void CPRCtrlLoop::checkCmdValues(){
		double max_Trans_X = 0.2;	// max velocity in m/s
		double max_Trans_Y = 0.1;
		double max_Ori = 1.0;		// max rotational speed in rad/s
		
		// check for upper limit
		cmd_slider_velocities.linear.x = std::min(cmd_slider_velocities.linear.x, max_Trans_X); 
		cmd_slider_velocities.linear.y = std::min(cmd_slider_velocities.linear.y, max_Trans_Y); 
		cmd_slider_velocities.linear.z = std::min(cmd_slider_velocities.linear.z, max_Trans_Y); 
		cmd_slider_velocities.angular.x = std::min(cmd_slider_velocities.angular.x, max_Ori); 
		cmd_slider_velocities.angular.y = std::min(cmd_slider_velocities.angular.y, max_Ori); 
		cmd_slider_velocities.angular.z = std::min(cmd_slider_velocities.angular.z, max_Ori); 

		// and for the lower limit
		cmd_slider_velocities.linear.x = std::max(cmd_slider_velocities.linear.x, -max_Trans_X); 
		cmd_slider_velocities.linear.y = std::max(cmd_slider_velocities.linear.y, -max_Trans_Y); 
		cmd_slider_velocities.linear.z = std::max(cmd_slider_velocities.linear.z, -max_Trans_Y); 
		cmd_slider_velocities.angular.x = std::max(cmd_slider_velocities.angular.x, -max_Ori); 
		cmd_slider_velocities.angular.y = std::max(cmd_slider_velocities.angular.y, -max_Ori); 
		cmd_slider_velocities.angular.z = std::max(cmd_slider_velocities.angular.z, -max_Ori); 
	
	}

}




