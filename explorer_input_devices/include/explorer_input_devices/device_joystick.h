/*
 *  device_joystick.h
 *  Copyright (C) 2025 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_INPUT_DEVICES_DEVICE_JOYSTICK_H
#define EXPLORER_INPUT_DEVICES_DEVICE_JOYSTICK_H

#include <atomic>
#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "explorer_input_devices/button_handler.h"
#include "explorer_input_devices/device.h"
#include "explorer_input_devices/mode_config.h"
#include "explorer_msgs/msg/control_frame_selection.hpp"
#include "explorer_msgs/srv/set_speed_level.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace input_device
{
/**
 * \brief Physical joystick device
 *
 * Turns a 2-axis / 1-button joystick into the Cartesian command the robot manager
 * expects, using button action meaning described by the mode YAML file.
 */
class DeviceJoystick : public Device
{
public:
  explicit DeviceJoystick(rclcpp::Node::SharedPtr n);

private:
  // ROS callbacks
  void callback_joy_(const sensor_msgs::msg::Joy& msg);
  void callback_x_current_(const geometry_msgs::msg::Pose& msg);
  void callback_retract_status_(const std_msgs::msg::String& msg);

  // Internal timer callback
  void timer_callback_();

  // Ask command_node to shift its speed level by delta.
  void request_speed_level_change_(int delta);

  // Publish the trajectory velocity.
  void publish_trajectory_velocity_(double velocity);

  // Execute behavior based on axis information
  void execute_behavior_(const AxisInfo& axis);

  // Read joystick axis value
  float read_axis_value_(const AxisInfo& axis_info);

  void reset_velocities_();

  void complex_calculation_(const double rotation_speed_scale);

  // Behavior implementations
  void cartesian_linear_(const AxisInfo& axis_info);
  void cartesian_rotation_(const AxisInfo& axis_info);
  void joint_direct_(const AxisInfo& axis_info);
  void change_speed_(const AxisInfo& axis_info);
  void drink_(const AxisInfo& axis_info);
  void gripper_(const AxisInfo& axis_info);
  void complex_(const AxisInfo& axis_info);
  void trajectory_control_(const AxisInfo& axis_info);

  // Publishers
  rclcpp::Publisher<explorer_msgs::msg::ControlFrameSelection>::SharedPtr frame_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_name_pub_;
  // True while the joystick is handled (as long as one button/joystick is used)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  // Drive the trajectory retract/home mode
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trajectory_velocity_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr x_current_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr retract_status_sub_;

  rclcpp::Client<explorer_msgs::srv::SetSpeedLevel>::SharedPtr set_speed_level_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  ButtonHandler button_handler_;

  ModeData data_;
  std::string current_mode_name_;
  std::unordered_map<ControlName, std::function<void(AxisInfo)>> control_behaviors_;

  // Joystick state variables
  mutable std::mutex mutex_axis_;

  // Raw joystick values (before smoothing)
  float axis_1_raw_ RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = 0.0f;
  float axis_2_raw_ RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = 0.0f;
  bool button_pressed_ RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = false;

  // Raw stick deflection above which the joystick counts as handled
  float activity_deadzone_;

  // Smoothed joystick values (computed once per timer cycle)
  float axis_1_smoothed_ = 0.0f;
  float axis_2_smoothed_ = 0.0f;

  int button_threshold_ms_;
  double sampling_period_;

  float joy_prec_ = 0.0;
  float speed_change_threshold_;

  bool complex_mode_ = false;
  double v_x_ = 0.0;
  double v_y_ = 0.0;
  double rotation_speed_scale_ = 1.0;

  // Modified upon command_node's retract_status changes
  std::atomic<bool> locked_{true};

  // Velocity messages
  geometry_msgs::msg::TwistStamped cartesian_vel_;
  // Gripper velocity for the current cycle, relayed to command_node
  double gripper_vel_ = 0.0;
  // Trajectory velocity for the current cycle, and the last one sent (prevent repeated zeros)
  double trajectory_vel_ = 0.0;
  double last_trajectory_vel_ = 0.0;

  explorer_msgs::msg::ControlFrameSelection frame_id_;

  geometry_msgs::msg::Pose x_current_;
};
}  // namespace input_device

#endif
