/*
 *  device_spacenav.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_DEVICE_SPACENAV_H
#define CARTESIAN_CONTROLLER_DEVICE_SPACENAV_H

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <ros2_control_explorer/device.h>

namespace input_device
{
/**
 * \brief Handle spacenav input device to control the robot
 *
 * Process spacenav_node topic (spacenav/joy) to control robot. Gripper control
 * is performed using services provided by niryo ros stack */
class DeviceSpacenav : public Device
{
public:
  DeviceSpacenav(rclcpp::Node::SharedPtr n);

private:
  bool gripper_toggle_;
  int control_mode_select_;
  double debounce_button_time_;
  rclcpp::Time debounce_button_left_, debounce_button_right_;
  int button_left_, button_right_;
  double static_trans_deadband_, static_rot_deadband_;
  double trans_x_, trans_y_, trans_z_, rot_x_, rot_y_, rot_z_;

  bool spacenav_is_stopped_ = false;
  bool spacenav_is_stopped_prev_ = false;

  void callbackJoy_(const sensor_msgs::msg::Joy::SharedPtr msg);
  void processButtons_(const sensor_msgs::msg::Joy::SharedPtr msg);
  void debounceButtons_(const sensor_msgs::msg::Joy::SharedPtr msg, const int button_id, rclcpp::Time& debounce_timer_ptr,
                        int& button_value_ptr);
  void updateGripperCmd_();
  void updateControlMode_();
};
}
#endif