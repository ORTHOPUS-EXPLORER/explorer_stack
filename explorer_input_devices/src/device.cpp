/*
 *  device.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "explorer_input_devices/device.h"

#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace input_device
{
Device::Device(
  rclcpp::Node::SharedPtr n, const std::string& joystick_topic, JoystickCallback joystick_callback)
: n_(std::move(n))
{
  RCLCPP_DEBUG(n_->get_logger(), "Device constructor");

  const auto cartesian_command_topic = n_->declare_parameter<std::string>(
    "cartesian_command_topic", "/explorer_user_interfaces/rqt_armcontrol/input_device_velocity");
  RCLCPP_INFO(
    n_->get_logger(), "Publishing cartesian command on '%s'", cartesian_command_topic.c_str());

  cartesian_cmd_pub_ =
    n_->create_publisher<geometry_msgs::msg::TwistStamped>(cartesian_command_topic, 1);
  // command_node scales it by the speed factor and integrates it into the gripper position
  gripper_velocity_pub_ =
    n_->create_publisher<std_msgs::msg::Float64>("/command_node/gripper/velocity/commands", 10);

  device_sub_ = n_->create_subscription<sensor_msgs::msg::Joy>(
    joystick_topic, 10, std::move(joystick_callback));
}

Device::~Device() {}

const rclcpp::Node::SharedPtr& Device::get_node_() const { return n_; }

void Device::publish_cartesian_command_(const geometry_msgs::msg::TwistStamped& cartesian_cmd) const
{
  cartesian_cmd_pub_->publish(cartesian_cmd);
}

void Device::publish_gripper_velocity_(double velocity)
{
  // Repeated zeros would collide with another gripper source (e.g. the virtual joystick), only
  // the first one is sent to stop the gripper right away.
  if (velocity == 0.0 && last_gripper_velocity_ == 0.0)
  {
    return;
  }
  last_gripper_velocity_ = velocity;

  gripper_velocity_pub_->publish(std_msgs::msg::Float64().set__data(velocity));
}
}  // namespace input_device
