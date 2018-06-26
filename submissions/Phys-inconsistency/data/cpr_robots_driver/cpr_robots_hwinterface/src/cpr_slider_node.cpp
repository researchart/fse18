#include "cpr_slider.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cpr_slider");
	ros::NodeHandle nh_private("~");
	cpr_robots::CPRSlider cpr_slider(nh_private);
	ros::spin();
	return 0;
}
