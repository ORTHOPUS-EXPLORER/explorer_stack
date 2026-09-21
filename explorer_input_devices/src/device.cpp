/*
 *  device.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "explorer_input_devices/device.h"

#include <algorithm>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace input_device
{
Device::Device(
  rclcpp::Node::SharedPtr n, const std::string& joystick_topic, JoystickCallback joystick_callback)
: n_(std::move(n))
{
  RCLCPP_DEBUG(n_->get_logger(), "Device constructor");

  cartesian_cmd_pub_ = n_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/explorer_user_interfaces/rqt_armcontrol/input_device_velocity", 1);
  gripper_command_pub_ =
    n_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_controller/commands", 1);

  device_sub_ = n_->create_subscription<sensor_msgs::msg::Joy>(
    joystick_topic, 10, std::move(joystick_callback));

  // Half open
  gripper_command_.data = {0.5};
}

Device::~Device() {}

const rclcpp::Node::SharedPtr& Device::get_node_() const { return n_; }

void Device::publish_cartesian_command_(const geometry_msgs::msg::TwistStamped& cartesian_cmd) const
{
  cartesian_cmd_pub_->publish(cartesian_cmd);
}

void Device::update_gripper_command_(double velocity, double dt)
{
  double& position = gripper_command_.data[0];

  position = std::clamp(position + velocity * dt, 0.0, 1.0);

  gripper_command_pub_->publish(gripper_command_);
}
}  // namespace input_device
