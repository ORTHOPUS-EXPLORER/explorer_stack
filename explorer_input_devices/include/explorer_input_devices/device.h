/*
 *  device.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_INPUT_DEVICES_DEVICE_H
#define EXPLORER_INPUT_DEVICES_DEVICE_H

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

namespace input_device
{
/**
  * \brief Abstract interface for devices implementation
  *
  * This class prepares the topics expected by the input devices 
  */
class Device
{
public:
  using JoystickCallback = std::function<void(const sensor_msgs::msg::Joy::SharedPtr)>;

  /**
    * \param n                    ROS Node.
    * \param joystick_topic       Joystick topic this device listens to.
    * \param joystick_callback    Invoked for every Joystick message received.
    */
  Device(
    rclcpp::Node::SharedPtr n, const std::string& joystick_topic,
    JoystickCallback joystick_callback);
  virtual ~Device() = 0;

protected:
  /**
    * \brief Get ROS Node const ref.
    */
  [[nodiscard]] const rclcpp::Node::SharedPtr& get_node_() const;

  void publish_cartesian_command_(const geometry_msgs::msg::TwistStamped& cartesian_cmd) const;

  /**
    * \brief Drive the gripper from a velocity.
    *
    * Speed scaling is done afterward, only publish unscaled velocities .
    * Repeated zero velocities are dropped, only the stop is sent.
    *
    * \param velocity  Normalized gripper velocity; sign is open/close, magnitude is speed.
    */
  void publish_gripper_velocity_(double velocity);

private:
  rclcpp::Node::SharedPtr n_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_velocity_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr device_sub_;

  // Last gripper velocity sent (needed to prevent repeated zeros)
  double last_gripper_velocity_ = 0.0;
};
}  // namespace input_device

#endif
