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
#include "std_msgs/msg/float64_multi_array.hpp"

/* TODO Improve multiple device handling. Currently, all devices could publish in the same topics which is nor safe nor expected behavior */
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
    * Devices produce a gripper velocity, the velocity is integrated over dt into a position
    * command, clamped to [0, 1], and published.
    *
    * \param velocity  Normalized gripper velocity; sign is open/close, magnitude is speed.
    * \param dt        Seconds elapsed since the previous call.
    */
  void update_gripper_command_(double velocity, double dt);

private:
  rclcpp::Node::SharedPtr n_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr device_sub_;

  // Integrated gripper position, value sent to the gripper controller
  std_msgs::msg::Float64MultiArray gripper_command_;
};
}  // namespace input_device

#endif
