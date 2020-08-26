#ifndef MVC_MENU_CONTROLLERS_NODELETS_HPP
#define MVC_MENU_CONTROLLERS_NODELETS_HPP

#include <memory>

#include <mvc_menu_controllers/radial_config.hpp>
#include <mvc_menu_controllers/radial_controller.hpp>
#include <mvc_menu_models/State.h>
#include <mvc_menu_models/model.hpp>
#include <nodelet/nodelet.h>
#include <ros/exception.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

namespace mvc_menu_controllers {

template < class Controller, class Config > class Nodelet : public nodelet::Nodelet {
public:
  Nodelet() {}

  virtual ~Nodelet() {}

protected:
  virtual void onInit() {
    ros::NodeHandle nh(getNodeHandle()), pnh(getPrivateNodeHandle());

    model_.reset(new mvc_menu_models::Model());
    if (!model_->setDescriptionFromParam(nh.resolveName("menu_description"))) {
      throw ros::Exception("Cannot set a model description from the param '" +
                           nh.resolveName("menu_description") + "'");
    }
    NODELET_INFO_STREAM("Menu:\n" << model_->toString());

    controller_.reset(new Controller(model_, Config::fromParamNs(pnh.getNamespace())));

    state_pub_ = nh.advertise< mvc_menu_models::State >("menu_state", 1, true);
    state_pub_.publish(model_->exportState());
    joy_sub_ = nh.subscribe("joy", 1, &Nodelet::onJoyRecieved, this);
  }

  void onJoyRecieved(const sensor_msgs::JoyConstPtr &joy) {
    // update menu state and publish
    state_pub_.publish(controller_->update(*joy));
    // NODELET_DEBUG_STREAM("Updated menu:\n" << menu_->toString());
  }

protected:
  mvc_menu_models::ModelPtr model_;
  std::shared_ptr< Controller > controller_;

  ros::Subscriber joy_sub_;
  ros::Publisher state_pub_;
};

typedef Nodelet< RadialController, RadialConfig > RadialNodelet;
} // namespace mvc_menu_controllers

#endif