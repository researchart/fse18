#include "cpr_slider_nodelet.h"

PLUGINLIB_DECLARE_CLASS(cpr_robots_hwinterface, CPRSliderNodelet,
  CPRSliderNodelet, nodelet::Nodelet);

void CPRSliderNodelet::onInit()
{
  NODELET_INFO("Initializing CPRSlider Nodelet");
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();
  slider_ = boost::shared_ptr<cpr_robots::CPRSlider>(new cpr_robots::CPRSlider(nh_private));
}
