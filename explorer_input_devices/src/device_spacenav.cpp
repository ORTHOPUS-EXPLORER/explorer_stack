/*
 *  device_spacenav.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "explorer_input_devices/device_spacenav.h"

#include "explorer_input_devices/spacenav_config.h"
#include "rclcpp/rclcpp.hpp"

namespace input_device
{
DeviceSpacenav::DeviceSpacenav(rclcpp::Node::SharedPtr node)
: Device(
    node, "spacenav/joy",
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) { callback_joy_(msg); })
{
  debounce_button_left_ = get_node_()->now();
  debounce_button_right_ = get_node_()->now();

  button_left_ = 0;
  button_right_ = 0;

  control_mode_select_ = 0;

  rot_x_ = 0.0;
  rot_y_ = 0.0;
  rot_z_ = 0.0;
  trans_x_ = 0.0;
  trans_y_ = 0.0;
  trans_z_ = 0.0;

  get_node_()->get_parameter("debounce_button_time", debounce_button_time_);
  get_node_()->get_parameter("static_trans_deadband", static_trans_deadband_);
  get_node_()->get_parameter("static_rot_deadband_", static_rot_deadband_);

  select_sub_ = get_node_()->create_subscription<std_msgs::msg::Int64>(
    "/explorer_user_interfaces/rqt_armcontrol/spacemouse_select", 10,
    [this](const std_msgs::msg::Int64::SharedPtr msg) { callback_spacemouse_select_(msg); });
}

void DeviceSpacenav::callback_joy_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  RCLCPP_DEBUG(get_node_()->get_logger(), "Process buttons.");
  process_buttons_(msg);
  update_gripper_command_();

  RCLCPP_DEBUG_STREAM(get_node_()->get_logger(), "Control mode select : " << control_mode_select_);

  spacenav_is_stopped_prev_ = spacenav_is_stopped_;
  double trans_scale = 1. / (1. - static_trans_deadband_);
  double rot_scale = 1. / (1. - static_rot_deadband_);

  if (
    trans_x_ != msg->axes[0] || trans_y_ != msg->axes[1] || trans_z_ != msg->axes[2] ||
    rot_x_ != msg->axes[3] || rot_y_ != msg->axes[4] || rot_z_ != msg->axes[5])
  {
    spacenav_is_stopped_ = false;
  }
  else
  {
    spacenav_is_stopped_ = true;
  }

  if (!(spacenav_is_stopped_ && spacenav_is_stopped_prev_))
  {
    if (control_mode_select_ == 0)
    {
      trans_x_ = msg->axes[0];
      trans_y_ = msg->axes[1];
      trans_z_ = msg->axes[2];
      rot_x_ = msg->axes[3];
      rot_y_ = msg->axes[4];
      rot_z_ = msg->axes[5];
    }
    else if (control_mode_select_ == 1)
    {
      trans_x_ = msg->axes[0];
      trans_y_ = msg->axes[1];
      trans_z_ = msg->axes[2];
      rot_x_ = 0.0;
      rot_y_ = 0.0;
      rot_z_ = 0.0;
    }
    else if (control_mode_select_ == 2)
    {
      trans_x_ = 0.0;
      trans_y_ = 0.0;
      trans_z_ = 0.0;
      rot_x_ = msg->axes[3];
      rot_y_ = msg->axes[4];
      rot_z_ = msg->axes[5];
    }
    else if (control_mode_select_ == 3)
    {
      trans_x_ = msg->axes[0];
      trans_y_ = msg->axes[1];
      trans_z_ = msg->axes[2];
      rot_x_ = 0.0;
      rot_y_ = 0.0;
      rot_z_ = msg->axes[5];
    }
    else
    {
      control_mode_select_ = 0;
    }

    // Cartesian control with the axes
    geometry_msgs::msg::TwistStamped cartesian_vel;
    cartesian_vel.header.stamp = get_node_()->now();

    cartesian_vel.twist.linear.x = trans_x_ * abs(trans_x_) * trans_scale * trans_scale;
    cartesian_vel.twist.linear.y = trans_y_ * abs(trans_y_) * trans_scale * trans_scale;
    cartesian_vel.twist.linear.z = trans_z_ * abs(trans_z_) * trans_scale * trans_scale;
    cartesian_vel.twist.angular.x = rot_x_ * abs(rot_x_) * rot_scale * rot_scale;
    cartesian_vel.twist.angular.y = rot_y_ * abs(rot_y_) * rot_scale * rot_scale;
    cartesian_vel.twist.angular.z = rot_z_ * abs(rot_z_) * rot_scale * rot_scale;
    publish_cartesian_command_(cartesian_vel);
  }
}

void DeviceSpacenav::callback_spacemouse_select_(const std_msgs::msg::Int64::SharedPtr msg)
{
  control_mode_select_ = msg->data;
}

void DeviceSpacenav::process_buttons_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  button_left_ = 0;
  button_right_ = 0;

  debounce_buttons_(msg, SPACENAV_BUTTON_LEFT, debounce_button_left_, button_left_);
  debounce_buttons_(msg, SPACENAV_BUTTON_RIGHT, debounce_button_right_, button_right_);
}

void DeviceSpacenav::debounce_buttons_(
  const sensor_msgs::msg::Joy::SharedPtr &msg, const int button_id, rclcpp::Time& debounce_timer_ptr,
  int& button_value_ptr)
{
  RCLCPP_DEBUG_STREAM(get_node_()->get_logger(), "Get button ID :" << button_id);
  if (!(msg->buttons).empty())
  {
    if (msg->buttons[button_id])
    {
      RCLCPP_DEBUG(get_node_()->get_logger(), "Check time.");
      if (get_node_()->now() > debounce_timer_ptr)
      {
        RCLCPP_DEBUG(get_node_()->get_logger(), "Update debounce timer.");
        debounce_timer_ptr =
          get_node_()->now() + rclcpp::Duration::from_seconds(debounce_button_time_);
        button_value_ptr = msg->buttons[button_id];
      }
    }
  }
  else
  {
    RCLCPP_DEBUG(get_node_()->get_logger(), "Buttons ptr is NULL.");
  }
  RCLCPP_DEBUG(get_node_()->get_logger(), "Finished with button.");
}

void DeviceSpacenav::update_gripper_command_()
{
  // Use the buttons to open / close the gripper while held
  double gripper_velocity = 0.0;
  if (button_left_ == 1)
  {
    gripper_velocity = 0.5;
  }
  else if (button_right_ == 1)
  {
    gripper_velocity = -0.5;
  }

  publish_gripper_velocity_(gripper_velocity);
}

}  // namespace input_device

using namespace input_device;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("device_spacenav", node_options);
  DeviceSpacenav device_spacenav(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
