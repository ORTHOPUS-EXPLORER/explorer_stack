/*
 *  joystick_selector.h
 *  Copyright (C) 2025 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_INPUT_DEVICES_JOYSTICK_SELECTOR_H
#define EXPLORER_INPUT_DEVICES_JOYSTICK_SELECTOR_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"

namespace input_device
{
/**
  * \brief Chooses between the physical and the virtual joystick command.
  *
  * Both joysticks publish a Cartesian velocity on their own topic; this class forwards
  * exactly one of them to the unified topic command_node listens to.
  *
  * Priority is in favour of the hardware the user is holding. 
  * Whenever the physical joystick asks for motion the virtual one is ignored.
  *
  * Commands are forwarded as they arrives but a stalled or disconnected joystick stops the arm.
  */
class JoystickSelector
{
public:
  /**
    * \param node ROS node
    */
  explicit JoystickSelector(rclcpp::Node::SharedPtr node);

private:
  /// Represent a joystick input
  struct InputState
  {
    geometry_msgs::msg::TwistStamped last_command;
    bool has_command = false;
    rclcpp::Time last_command_time;
  };

  /// Represents which input is currently applied
  enum class SelectedInput
  {
    NONE,
    PHYSICAL,
    VIRTUAL
  };

  /// Callback called on input command and forward its command if currently selected.
  void on_command_(
    InputState& input, SelectedInput input_identity,
    const geometry_msgs::msg::TwistStamped& command);

  /// Internal thread callback that updates currently selected device
  void watchdog_callback_();

  /// Apply the priority rules for the current instant. The caller holds the lock.
  [[nodiscard]] SelectedInput select_input_(const rclcpp::Time& now) const;

  /// Indicates whether a given input is considerated stale (no command received for a while)
  [[nodiscard]] bool is_stale_(const InputState& input, const rclcpp::Time& now) const;

  /// Indicates whether a given input is publishing non zero command
  [[nodiscard]] bool is_moving_(const InputState& input) const;

  [[nodiscard]] bool is_zero_velocity_(const geometry_msgs::msg::TwistStamped& command) const;

  void publish_zero_velocity_();

  /// Logs every change of control, so a bag shows who was driving and when.
  void log_selection_change_(SelectedInput selected);

  [[nodiscard]] static std::string to_string(SelectedInput selected);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr physical_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr virtual_command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr selected_command_pub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // How long an input received is "eligible" to be transmitted.
  double input_timeout_seconds_;
  // Velocity magnitude above which an input counts as being used.
  double activity_threshold_;

  // Guards everything below: written from both subscriptions and the watchdog timer.
  mutable std::mutex mutex_;
  InputState physical_joystick_;
  InputState virtual_joystick_;
  SelectedInput selected_input_ = SelectedInput::NONE;
};
}  // namespace input_device

#endif
