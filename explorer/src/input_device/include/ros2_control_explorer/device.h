/*
 *  device.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_DEVICE_H
#define CARTESIAN_CONTROLLER_DEVICE_H

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

/* TODO Improve multiple device handling. Currently, all device publish at the
same time in the same topic device_sub_ which is nor safe nor expected behavior */
namespace input_device
{
/**
  * \brief Abstract interface for devices implementation
  *
  * This class prepares the geometry_msgs/TwistStamped topic expected by
  * the robot manager for cartesian control
  */
class Device
{
public:
  Device(rclcpp::Node::SharedPtr n);
  virtual ~Device() = 0;

protected:
  rclcpp::Node::SharedPtr n_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr device_sub_;
  /*
  ros::ServiceClient learning_mode_client_;
  ros::ServiceClient change_tool_srv_;
  ros::ServiceClient open_gripper_srv_;
  ros::ServiceClient close_gripper_srv_;

  void initializeServices_();
  void requestLearningMode(int state);
  void setGripperId_();
  void openGripper_();
  void closeGripper_();
  */
};
}

#endif