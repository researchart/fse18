/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-13, Commonplace Robotics GmbH
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

#include "cpr_robots_comm_rs232.h"

#include <iostream>

using namespace::boost::asio;
using namespace boost::posix_time;
using namespace std;

#define COMPORT "/dev/ttyUSB0"

CPRCommRS232::CPRCommRS232()
{
	// Init the message buffer
	for(int i = 0; i < 256; i++){
		msgBuffer[i].length = 0;
		msgBuffer[i].id = i;
		for(int j = 0; j < 8; j++)
			msgBuffer[i].data[j] = 0x80;
	}
}

bool CPRCommRS232::Connect(const std::string& portst)
{
	io_service io;
	const char *PORT = portst.c_str();
	serial_port_base::baud_rate baud_option(460800);
	active = false;

	try{
		port = new serial_port(io);
		port->open(PORT);
		port->set_option(baud_option); // set the baud rate
		active = true;
		cout << "Port " << PORT << " opened\n";
	}catch (boost::system::system_error &e){
		boost::system::error_code ec = e.code();
		cerr << "Cannot open port " << PORT << " - error code: " << ec.category().name() << std::endl;
		return false;
	}catch(std::exception e){
		cerr << "Cannot open port " << PORT << " - error code: " << e.what() << endl;
		return false;
	}

	boost::thread readThread(boost::bind(&CPRCommRS232::readLoop, this));
	return true;
}

bool CPRCommRS232::Disconnect()
{
	if(port->is_open()) 
	{
		mutex_active.lock();
		active = false;
		mutex_active.unlock();
		port->close();
		std::cout << "Port closed" << endl;
	}
	return true;
}

void CPRCommRS232::WriteMsg(int id, int length, unsigned char* data)
{
	unsigned char commands[11] = {16, 4, 4, 125, 125, 0,0,0,0,0, 19};
	
	commands[0] = id;
	commands[1] = length;
	for(int i = 0; i < 8; i++)
		commands[2+i] = data[i];

	int sum = 0;
	for(int i = 0; i < 10; i++)
		sum += commands[i];
	sum = sum % 256;
	commands[10] = sum;

	bool act;
	mutex_active.lock();
	act = active;
	mutex_active.unlock();

	if (act) {
		size_t bytes = boost::asio::write(*port, boost::asio::buffer(commands, 11));
	}
}

void CPRCommRS232::GetMsg(int id, int *length, unsigned char* data)
{
	if(id>255)
		throw std::string("Invalid message id!");
    mutex.lock();
	length[0] = msgBuffer[id].length;
	for(int i = 0; i < 8; i++)
		data[i] = msgBuffer[id].data[i];
	mutex.unlock();
}

int CPRCommRS232::EvaluateBuffer(unsigned char* buf)
{
	int i = 0;
	int mid = (int)buf[0];
	int length = (int)buf[1];
	int cs = (unsigned char)buf[10];	// Checksum byte

	int sum = 0;				        // Genereate a simple checksum bit from the
	for(i = 0; i < 10; i++)			    // Received data
		sum += (unsigned char)buf[i];
	sum = sum % 256;

	if(sum == cs){			            // Compare it with the remote generated checksum bit
	    mutex.lock();
		msgBuffer[mid].id = mid;
		msgBuffer[mid].length = length;
		for(i = 0; i < 8; i++)
			msgBuffer[mid].data[i] = buf[i+2];
		mutex.unlock();
		//cout << "Found good msg. ID: " << mid << ", Length: " << length << endl;
	}else{
		//cerr << "Found bad messages" << endl;
	}

	return 0;
}

void CPRCommRS232::SetActive(bool act)
{
	mutex_active.lock();
	active = act;
	mutex_active.unlock();
}

void CPRCommRS232::readLoop()
{
	ptime start, now;
	time_duration passed;

	unsigned char bu[11];
	unsigned char buffer[11];
	int bufferCnt = 0;

	int tmpid = 0;
	while (true)
	{

		/*
		 * Main Read loop: read one byte in a time
		 * When a known sender is found the complete CAN message of 11 bytes is read
		 */
		bool act;
		mutex_active.lock();
		act = active;
		mutex_active.unlock();
		if (act)
		{
			start = microsec_clock::universal_time();
			boost::asio::read(*(port), boost::asio::buffer(bu, 1));
			now = microsec_clock::universal_time();
			passed = now - start;

			if(passed.total_milliseconds() > 2){		// we are out of sync!!!
				bufferCnt = 0;
			}

			if(bufferCnt == 0 ){
				tmpid = (int)bu[0];
				buffer[0] = bu[0];
				bufferCnt++;

			}else{
				buffer[bufferCnt] = bu[0];
				bufferCnt++;

				if(bufferCnt == 11){
					EvaluateBuffer(buffer);
					bufferCnt = 0;
				}
			}
		}
		else
		{
			break;
		}
	}
	//std::cout << "ReadLoop: finished" << std::endl;
}
