#ifndef MVC_MENU_RVIZ_DISPLAY_BASE_HPP
#define MVC_MENU_RVIZ_DISPLAY_BASE_HPP

#include <memory>

#include <mvc_menu_models/State.h>
#include <mvc_menu_models/model.hpp>
#include <mvc_menu_rviz/image_overlay.hpp>
#include <mvc_menu_rviz/properties.hpp>
#include <ros/console.h>
#include <ros/exception.h>
#include <ros/subscriber.h>
#include <rviz/display.h>

namespace mvc_menu_rviz {

// base implementation of display classes except Qt's signals and slots
// because template classes cannot have any slots and signals by Qt's limitation
template < typename DrawingProperty, class PropertyControl, class ImageDrawer >
class DisplayBase : public rviz::Display {
public:
  DisplayBase() {}

  virtual ~DisplayBase() {}

protected:
  // called once on initialization
  virtual void onInitialize() {
    // allocate objects
    prop_ctl_.reset(new PropertyControl(this));
    model_.reset(new mvc_menu_models::Model());
    drawer_.reset(new ImageDrawer(model_, prop_ctl_->drawingProperty()));
    overlay_.reset(new ImageOverlay());

    // apply the initial properties
    // (except subscription. it will be executed in onEnable().)
    updateDescription(prop_ctl_->descriptionProperty());
    updateImage(prop_ctl_->drawingProperty());
    updatePosition(prop_ctl_->positionProperty());
  }

  // called when enabled
  virtual void onEnable() {
    updateSubscription(prop_ctl_->subscriptionProperty());
    overlay_->show();
  }

  // called when disabled
  virtual void onDisable() {
    overlay_->hide();
    state_sub_.shutdown();
  }

  void updateDescription(const DescriptionProperty &prop) {
    const std::string param_name(prop.param_name.toStdString());
    if (!param_name.empty() && model_->setDescriptionFromParam(param_name)) {
      state_ = model_->exportState();
      updateImage();
    }
  }

  void updateSubscription(const SubscriptionProperty &prop) {
    // unsubscribe
    state_sub_.shutdown();

    // destroy the last state from the previous session
    model_->resetState();
    state_ = model_->exportState();
    updateImage();

    // subscribe the new topic
    const std::string topic(prop.topic.toStdString());
    try {
      if (!topic.empty()) {
        state_sub_ = ros::NodeHandle().subscribe(topic, 1, &DisplayBase::updateImage, this);
      }
    } catch (const ros::Exception &error) {
      ROS_ERROR_STREAM(getName().toStdString()
                       << ": error on subscribing '" << topic << "': " << error.what());
    }
  }

  // update menu image based on the current configs
  void updateImage() {
    overlay_->setImage(drawer_->draw());
    overlay_->update();
  }

  // update menu image with the given menu state
  void updateImage(const mvc_menu_models::StateConstPtr &new_state) {
    if (state_->is_enabled != new_state->is_enabled ||
        state_->pointed_id != new_state->pointed_id ||
        state_->selected_ids != new_state->selected_ids) {
      model_->setState(*new_state);
      state_ = new_state;
      updateImage();
    }
  }

  // update menu image with the given drawing property
  void updateImage(const DrawingProperty &prop) {
    drawer_->setProperty(prop);
    updateImage();
  }

  void updatePosition(const PositionProperty &prop) {
    overlay_->setOrigin(prop.origin);
    overlay_->update();
  }

protected:
  // property control via Rviz
  std::unique_ptr< PropertyControl > prop_ctl_;
  // menu tree model
  mvc_menu_models::ModelPtr model_;
  // menu state subscriber
  ros::Subscriber state_sub_;
  mvc_menu_models::StateConstPtr state_;
  // state drawer
  std::unique_ptr< ImageDrawer > drawer_;
  // overlay on Rviz
  std::unique_ptr< ImageOverlay > overlay_;
};
} // namespace mvc_menu_rviz

#endif