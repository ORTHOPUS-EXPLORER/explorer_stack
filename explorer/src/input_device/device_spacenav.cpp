/*
 *  device_spacenav.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_explorer/device_spacenav.h"
#include "ros2_control_explorer/spacenav_config.h"

namespace input_device
{
DeviceSpacenav::DeviceSpacenav(rclcpp::Node::SharedPtr n) : Device(n)
{
  device_sub_ = n_->create_subscription<sensor_msgs::msg::Joy>("spacenav/joy", 10,
                  std::bind(&DeviceSpacenav::callbackJoy_, this, std::placeholders::_1));

  debounce_button_left_ = n_->now();
  debounce_button_right_ = n_->now();

  button_left_ = 0;
  button_right_ = 0;

  gripper_toggle_ = false;
  control_mode_select_ = 0;

  rot_x_ = 0.0;
  rot_y_ = 0.0;
  rot_z_ = 0.0;
  trans_x_ = 0.0;
  trans_y_ = 0.0;
  trans_z_ = 0.0;

  n_->get_parameter("debounce_button_time", debounce_button_time_);
  n_->get_parameter("static_trans_deadband", static_trans_deadband_);
  n_->get_parameter("static_rot_deadband_", static_rot_deadband_);
}

void DeviceSpacenav::callbackJoy_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  RCLCPP_DEBUG(n_->get_logger(), "Process buttons.");
  processButtons_(msg);
  updateGripperCmd_();
  updateControlMode_();

  RCLCPP_DEBUG_STREAM(n_->get_logger(), "Control mode select : " << control_mode_select_);

  spacenav_is_stopped_prev_ = spacenav_is_stopped_;
  double trans_scale = 1. / (1. - static_trans_deadband_);
  double rot_scale = 1. / (1. - static_rot_deadband_);

  if( trans_x_ != msg->axes[0]
    || trans_y_ != msg->axes[1]
    || trans_z_ != msg->axes[2]
    || rot_x_ != msg->axes[3]
    || rot_y_ != msg->axes[4]
    || rot_z_ != msg->axes[5] )
  {
    spacenav_is_stopped_ = false;
  }
  else
  {
    spacenav_is_stopped_ = true;
  }

  if( !(spacenav_is_stopped_ && spacenav_is_stopped_prev_) )
  {
    if (control_mode_select_==0)
    {
      trans_x_ = msg->axes[0];
      trans_y_ = msg->axes[1];
      trans_z_ = msg->axes[2];
      rot_x_ = msg->axes[3];
      rot_y_ = msg->axes[4];
      rot_z_ = msg->axes[5];
    }
    else if (control_mode_select_==1)
    {
      trans_x_ = msg->axes[0];
      trans_y_ = msg->axes[1];
      trans_z_ = msg->axes[2];
      rot_x_ = 0.0;
      rot_y_ = 0.0;
      rot_z_ = 0.0;
    }
    else if (control_mode_select_==2)
    {
      trans_x_ = 0.0;
      trans_y_ = 0.0;
      trans_z_ = 0.0;
      rot_x_ = msg->axes[3];
      rot_y_ = msg->axes[4];
      rot_z_ = msg->axes[5];
    }
    else
    {
      control_mode_select_ = 0;
    }

    // Cartesian control with the axes
    geometry_msgs::msg::TwistStamped cartesian_vel;
    cartesian_vel.header.stamp = n_->now();

    cartesian_vel.twist.linear.x = trans_x_ * abs(trans_x_) * trans_scale * trans_scale;
    cartesian_vel.twist.linear.y = trans_y_ * abs(trans_y_) * trans_scale * trans_scale;
    cartesian_vel.twist.linear.z = trans_z_ * abs(trans_z_) * trans_scale * trans_scale;
    cartesian_vel.twist.angular.x = rot_x_ * abs(rot_x_) * rot_scale * rot_scale;
    cartesian_vel.twist.angular.y = rot_y_ * abs(rot_y_) * rot_scale * rot_scale;
    cartesian_vel.twist.angular.z = rot_z_ * abs(rot_z_) * rot_scale * rot_scale;
    cartesian_cmd_pub_->publish(cartesian_vel);
  }
}

void DeviceSpacenav::processButtons_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  button_left_ = 0;
  button_right_ = 0;

  debounceButtons_(msg, SPACENAV_BUTTON_LEFT, debounce_button_left_, button_left_);
  debounceButtons_(msg, SPACENAV_BUTTON_RIGHT, debounce_button_right_, button_right_);
}

void DeviceSpacenav::debounceButtons_(const sensor_msgs::msg::Joy::SharedPtr msg, const int button_id,
                                      rclcpp::Time& debounce_timer_ptr, int& button_value_ptr)
{
  RCLCPP_DEBUG_STREAM(n_->get_logger(), "Get button ID :" << button_id);
  if ( !(msg->buttons).empty() ){
    if (msg->buttons[button_id])
    {
      RCLCPP_DEBUG(n_->get_logger(), "Check time.");
      if (n_->now() > debounce_timer_ptr)
      {
        RCLCPP_DEBUG(n_->get_logger(), "Update debounce timer.");
        debounce_timer_ptr = n_->now() + rclcpp::Duration::from_seconds(debounce_button_time_);
        button_value_ptr = msg->buttons[button_id];
      }
    }
  }
  else
  {
    RCLCPP_DEBUG(n_->get_logger(), "Buttons ptr is NULL.");
  }
  RCLCPP_DEBUG(n_->get_logger(), "Finished with button.");
}

void DeviceSpacenav::updateGripperCmd_()
{
  // Use A to toggle gripper state (open/close)
  if (button_left_ == 1 && gripper_toggle_ == false)
  {
    gripper_toggle_ = true;
    //openGripper_();
  }
  else if (button_left_ == 1 && gripper_toggle_ == true)
  {
    gripper_toggle_ = false;
    //closeGripper_();
  }
}

void DeviceSpacenav::updateControlMode_()
{
  // Use A to toggle gripper state (open/close)
  if (button_right_ == 1)
  {
    control_mode_select_ ++;
  }
  if (control_mode_select_ > 2)
  {
    control_mode_select_ = 0;
  }
}
}

using namespace input_device;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto n = rclcpp::Node::make_shared("device_spacenav", node_options);
  DeviceSpacenav device_spacenav(n);

  rclcpp::spin(n);

  rclcpp::shutdown();
  return 0;
}
