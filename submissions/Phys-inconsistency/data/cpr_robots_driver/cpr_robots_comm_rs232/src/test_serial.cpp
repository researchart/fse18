#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "cpr_robots_comm_rs232.h"

using namespace std;

int nrOfJoints;
int jointIDs[4];

void Wait(int ms){
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

void EnableMotors(CPRCommRS232& serial)
{
	int l = 2;
	unsigned char data[8] = {1, 9, 0, 0, 0, 0, 0, 0};		// CAN message to enable the motors
	for(int i = 0; i < nrOfJoints; i++)
	{
		serial.WriteMsg(jointIDs[i], l, data);
		Wait(3);
	}
}

void DisableMotors(CPRCommRS232& serial)
{
	int l = 2;
	unsigned char data[8] = {1, 10, 0, 0, 0, 0, 0, 0};		// CAN message to disable the motors
	for(int i = 0; i < nrOfJoints; i++)
	 {
		serial.WriteMsg(jointIDs[i], l, data);
		Wait(3);
	}
}


int main(int argc, char **argv)
{
	nrOfJoints = 4;
	jointIDs[0] = 0x08;		// front left
	jointIDs[1] = 0x06;		// front right
	jointIDs[2] = 0x02;		// back left
	jointIDs[3] = 0x04;		// back right

	CPRCommRS232 serial;
	serial.Connect("/dev/ttyUSB0");

	// Enabling motors
	EnableMotors(serial);

	unsigned char data[8] = {0, 0, 0, 0, 0, 0, 0, 0};	// cmd, vel, posH, posL, counter

	// Moving motors
	while (true) 
	{
		for(int i = 0; i < nrOfJoints; i++)
		{
			data[0] = 5;								// Command code for velocity control
			data[1] = 222;								// Velocity
			data[2] = 45;								// Should be counting up, the board will returing with 1 added to identify the messages
			serial.WriteMsg(jointIDs[i], 3, data);
			Wait(5);
		}
		Wait(50);
	}

	// Disabling motors
	DisableMotors(serial);
	
	serial.Disconnect();
	return 0;
}
