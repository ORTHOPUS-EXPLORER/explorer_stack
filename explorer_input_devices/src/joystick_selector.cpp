/*
 *  joystick_selector.cpp
 *  Copyright (C) 2025 Orthopus
 *  All rights reserved.
 */
#include "explorer_input_devices/joystick_selector.h"

#include <chrono>
#include <cmath>
#include <utility>

namespace input_device
{
JoystickSelector::JoystickSelector(rclcpp::Node::SharedPtr node) : node_(std::move(node))
{
  RCLCPP_INFO(node_->get_logger(), "JoystickSelector constructor");

  // ROS Parameters
  node_->declare_parameter<std::string>(
    "physical_command_topic", "/explorer_input_devices/joystick/physical/velocity/commands");
  node_->declare_parameter<std::string>(
    "virtual_command_topic", "/explorer_input_devices/joystick/virtual/velocity/commands");
  node_->declare_parameter<std::string>(
    "selected_command_topic", "/command_node/robot/velocity/commands");
  node_->declare_parameter<double>("input_timeout_seconds", 0.5);
  node_->declare_parameter<double>("activity_threshold", 1e-3);
  node_->declare_parameter<double>("watchdog_rate_hz", 20.0);

  input_timeout_seconds_ = node_->get_parameter("input_timeout_seconds").as_double();
  activity_threshold_ = node_->get_parameter("activity_threshold").as_double();

  // Init command time variables
  const rclcpp::Time start_time = node_->now();
  physical_joystick_.last_command_time = start_time;
  virtual_joystick_.last_command_time = start_time;

  // Topic(s) used
  const auto physical_topic = node_->get_parameter("physical_command_topic").as_string();
  const auto virtual_topic = node_->get_parameter("virtual_command_topic").as_string();
  const auto selected_topic = node_->get_parameter("selected_command_topic").as_string();

  // Publisher(s)
  selected_command_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>(selected_topic, 10);

  // Subscriber(s)
  physical_command_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    physical_topic, 10,
    [this](const geometry_msgs::msg::TwistStamped& command)
    { on_command_(physical_joystick_, SelectedInput::PHYSICAL, command); });

  virtual_command_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    virtual_topic, 10,
    [this](const geometry_msgs::msg::TwistStamped& command)
    { on_command_(virtual_joystick_, SelectedInput::VIRTUAL, command); });

  RCLCPP_INFO(
    node_->get_logger(), "Physical '%s' takes priority over virtual '%s', forwarded to '%s'",
    physical_topic.c_str(), virtual_topic.c_str(), selected_topic.c_str());

  const double watchdog_rate_hz = node_->get_parameter("watchdog_rate_hz").as_double();
  watchdog_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / watchdog_rate_hz), [this]() { watchdog_callback_(); });
}

void JoystickSelector::on_command_(
  InputState& input, SelectedInput input_identity, const geometry_msgs::msg::TwistStamped& command)
{
  const rclcpp::Time now = node_->now();
  const std::lock_guard<std::mutex> lock(mutex_);

  input.last_command = command;
  input.has_command = true;
  input.last_command_time = now;

  const SelectedInput selected = select_input_(now);
  log_selection_change_(selected);

  if (selected == input_identity)
  {
    selected_command_pub_->publish(command);
  }
}

void JoystickSelector::watchdog_callback_()
{
  const rclcpp::Time now = node_->now();
  const std::lock_guard<std::mutex> lock(mutex_);


  const SelectedInput selected = select_input_(now);
  log_selection_change_(selected);

  // Publish "hold" position when no input selected
  if (selected == SelectedInput::NONE)
  {
    publish_zero_velocity_();
  }
}

JoystickSelector::SelectedInput JoystickSelector::select_input_(const rclcpp::Time& now) const
{

  if (!is_stale_(physical_joystick_, now) && is_moving_(physical_joystick_))
  {
    return SelectedInput::PHYSICAL;
  }

  if (!is_stale_(virtual_joystick_, now) && is_moving_(virtual_joystick_))
  {
    return SelectedInput::VIRTUAL;
  }

  return SelectedInput::NONE;
}

bool JoystickSelector::is_stale_(const InputState& input, const rclcpp::Time& now) const
{
  if (!input.has_command)
  {
    return true;
  }
  return (now - input.last_command_time).seconds() > input_timeout_seconds_;
}

bool JoystickSelector::is_moving_(const InputState& input) const
{
  return input.has_command && !is_zero_velocity_(input.last_command);
}

bool JoystickSelector::is_zero_velocity_(const geometry_msgs::msg::TwistStamped& command) const
{
  const auto& linear = command.twist.linear;
  const auto& angular = command.twist.angular;

  return std::fabs(linear.x) <= activity_threshold_ && std::fabs(linear.y) <= activity_threshold_ &&
         std::fabs(linear.z) <= activity_threshold_ &&
         std::fabs(angular.x) <= activity_threshold_ &&
         std::fabs(angular.y) <= activity_threshold_ && std::fabs(angular.z) <= activity_threshold_;
}

void JoystickSelector::publish_zero_velocity_()
{
  geometry_msgs::msg::TwistStamped zero_command;
  zero_command.header.stamp = node_->now();
  selected_command_pub_->publish(zero_command);
}

void JoystickSelector::log_selection_change_(SelectedInput selected)
{
  if (selected == selected_input_)
  {
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Joystick control: %s -> %s", to_string(selected_input_).c_str(),
    to_string(selected).c_str());

  selected_input_ = selected;
}

std::string JoystickSelector::to_string(SelectedInput selected)
{
  switch (selected)
  {
    case SelectedInput::NONE:
      return "none";
    case SelectedInput::PHYSICAL:
      return "physical";
    case SelectedInput::VIRTUAL:
      return "virtual";
  }
  return "unknown";
}
}  // namespace input_device

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("joystick_selector");
  input_device::JoystickSelector joystick_selector(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
