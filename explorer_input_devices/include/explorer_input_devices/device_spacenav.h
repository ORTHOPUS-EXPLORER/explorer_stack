/*
 *  device_spacenav.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_INPUT_DEVICES_DEVICE_SPACENAV_H
#define EXPLORER_INPUT_DEVICES_DEVICE_SPACENAV_H

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int64.hpp"

#include <explorer_input_devices/device.h>

namespace input_device
{
  using SelectCallback = std::function<void(const std_msgs::msg::Int64::SharedPtr)>;

/**
 * \brief Handle spacenav input device to control the robot
 *
 * Process spacenav_node topic (spacenav/joy) to control robot. Gripper control
 * is performed using services provided by niryo ros stack */
class DeviceSpacenav : public Device
{
public:
  DeviceSpacenav(rclcpp::Node::SharedPtr node);

private:
  int control_mode_select_;
  double debounce_button_time_;
  rclcpp::Time debounce_button_left_, debounce_button_right_;
  int button_left_, button_right_;
  double static_trans_deadband_, static_rot_deadband_;
  double trans_x_, trans_y_, trans_z_, rot_x_, rot_y_, rot_z_;

  // The spacemouse callback is event driven, so the gripper integration in Device is
  // fed the measured interval between two joy messages.
  rclcpp::Time last_gripper_update_;

  // Control mode selection is specific to the spacemouse, so it is owned here rather
  // than by Device.
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr select_sub_;

  bool spacenav_is_stopped_ = false;
  bool spacenav_is_stopped_prev_ = false;

  void callback_joy_(const sensor_msgs::msg::Joy::SharedPtr msg);
  void callback_spacemouse_select_(const std_msgs::msg::Int64::SharedPtr msg);
  void process_buttons_(const sensor_msgs::msg::Joy::SharedPtr msg);
  void debounce_buttons_(const sensor_msgs::msg::Joy::SharedPtr &msg, const int button_id, rclcpp::Time& debounce_timer_ptr,
                        int& button_value_ptr);
  void update_gripper_command_();
  void update_control_mode_();
};
}
#endif