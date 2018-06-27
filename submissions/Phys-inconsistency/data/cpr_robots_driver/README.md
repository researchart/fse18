cpr_robots_driver
=================

ROS stack to drive the CPR robots Slider and Mover4, packages cpr_robots_hwinterface and cpr_robots_teleop 
BSD license.

**Attention!** Current version does not support communication with Checksum to the USB2CAN bridge. Will be updated!

Current Version
-----------------
January 21st, 2013 - Set up

Scope
-----------------
This repository contains ROS packages to move the robot arm Mover4 and the mobile platform Slider, see www.cpr-robots.com

Package cpr_robots_teleop
-----------------
This package reads the keyboard and publishes twist messages with the cartesian velocities for the slider.

Package cpr_robots_hwinterface
-----------------
This package contains the inverse kinematics and the hardware interface to move the real robot according to the twist messages.

