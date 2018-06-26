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


#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#include <cpr_teleop_keyboard.h>


using namespace boost::posix_time;
using namespace std;



namespace cpr_robots{


	//***************************************************************
	// loop to read from the keyboard, started in an own thread.
	void keyboardLoop(void * context )
	{

		CPRTeleopKeyboard *ctx;
		ctx = (CPRTeleopKeyboard*)context;
	    	
		int kfd = 0;
		struct termios cooked, raw;

		// get the console in raw mode
  		tcgetattr(kfd, &cooked);
  		memcpy(&raw, &cooked, sizeof(struct termios));
  		raw.c_lflag &=~ (ICANON | ECHO);
  		// Setting a new line, then end of file
  		raw.c_cc[VEOL] = 1;
  		raw.c_cc[VEOF] = 2;
  		tcsetattr(kfd, TCSANOW, &raw);

		puts("Reading from keyboard");
  		puts("---------------------------");
  		puts("Use 'WS' to move forward / backward");
  		puts("Use 'QE' to traverse left / right");
  		puts("Use 'AD' to rotate left / right");
		    
		char c;
	    	while (true)
	    	{
			// get the next event from the keyboard
    			if(read(kfd, &c, 1) < 0)
    			{
      				perror("read():");
      				exit(-1);
    			}

			switch(c)
		    	{

			case KEYCODE_W:
				ctx->cmd_twist_internal.linear.x = 0.1;
		      		break;
		    	case KEYCODE_S:
		      		ctx->cmd_twist_internal.linear.x = -0.1;
		      		break;

			case KEYCODE_Q:
				ctx->cmd_twist_internal.linear.y = 0.1;
		      		break;
		    	case KEYCODE_E:
		      		ctx->cmd_twist_internal.linear.y = -0.1;
		      		break;

			case KEYCODE_A:
				ctx->cmd_twist_internal.angular.z = 0.1;
		      		break;
		    	case KEYCODE_D:
		      		ctx->cmd_twist_internal.angular.z = -0.1;
		      		break;

		 
		    }
		

		}
		tcsetattr(kfd, TCSANOW, &cooked);

	}

	//****************************************************************
	CPRTeleopKeyboard::CPRTeleopKeyboard(){

	}

	//****************************************************************
	CPRTeleopKeyboard::~CPRTeleopKeyboard(){
		//tcsetattr(kfd, TCSANOW, &cooked);
	}

	//****************************************************************
	// starts the keyboard loop
	int CPRTeleopKeyboard::init(){
		boost::thread keyboardThread(cpr_robots::keyboardLoop, (void*)this);
		return 0;
	}


	//****************************************************************
	// forwards the velocities to the main loop
	void CPRTeleopKeyboard::getVelocities(geometry_msgs::Twist *t){

		t->linear.x = cmd_twist_internal.linear.x;
		t->linear.y = cmd_twist_internal.linear.y;
		t->linear.z = cmd_twist_internal.linear.z;
		t->angular.x = cmd_twist_internal.angular.x;
		t->angular.y = cmd_twist_internal.angular.y;
		t->angular.z = cmd_twist_internal.angular.z;
		
		cmd_twist_internal.linear.x = 0.0;
		cmd_twist_internal.linear.y = 0.0;
		cmd_twist_internal.linear.z = 0.0;
		cmd_twist_internal.angular.x = 0.0;
		cmd_twist_internal.angular.y = 0.0;
		cmd_twist_internal.angular.z = 0.0;
		
		//ROS_INFO("Keyboard vx=%.3lf vy=%.3lf rz=%.3lf", t->linear.x, t->linear.y, t->angular.z);

		return;
	}



}




