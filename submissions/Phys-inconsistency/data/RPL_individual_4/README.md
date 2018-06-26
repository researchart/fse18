RPL_individual_4: Particle Filter
================

Unforunately this work is not complete, so the reader is invited to use the different function and the whole program without to much confidence.

-- Functionality
This ROS node implement a particle filter that afford the pose estimation of a differential driver robot. The node use the laser scan given by the PrimeSensor Kinect and the odometry data coming from the wheel encoder to estimate the pose of the robot. This filter also afford to solve the "global localization problem" and the "kidnappen robot problem".

Is possible to visualize all the particle in Rviz. 

The particle filter is composed by 3 main step that are continuously repeated:
- Model update
- Sensor update
- Resample

Therefore the idea behind my implementation is to create these three object as istance of three different calss that operate on the same object (particle cloud and correlation vector). Since the step are done consequently, there is no danger of dreadlock.


-- Usage
Open a terminal and type

	roslaunch particle_filter particle_filter

It is possible to modify all the global variable in the file parameters.yaml. 


-- File/directory structure
In the src and include directory:
* main: Here the core of the program run, and some helperfunction are implemented.
* Model: this class contain all the variable and method to simulate the differential driver thymio, give the odometry data.
* Sensor: this class contain all the variable and method to perform the sensor update using the particle updated by a model step simulation and the laser scan coming from the kinect.
* Resampler: this class contain all the variable and method that affor the resampling of the particle cloud.


-- Note for the instructor
I didn't use svn, but github for the commit, so if you are interested, please have a look at https://github.com/mafilipp/RPL_individual_4.


Please note:

* The sensor update functions doesn't work properly, and in order to solve the kidnapp robot problem, in the resampling algorithm some random particle should be added
* In order to compile the code, the c++11 is needed.
