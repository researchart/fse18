#ifndef CPR_SLIDER_NODELET_H
#define CPR_SLIDER_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "cpr_slider.h"

class CPRSliderNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit ();

  private:
    boost::shared_ptr<cpr_robots::CPRSlider> slider_;
};

#endif
