/*
 *  device.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_explorer/device.h"

namespace input_device
{
Device::Device(rclcpp::Node::SharedPtr n) : n_(n)
{
  RCLCPP_DEBUG(n_->get_logger(), "Device constructor");

  cartesian_cmd_pub_ = n_->create_publisher<geometry_msgs::msg::TwistStamped>
                        ("/ros2_control_explorer/input_device_velocity", 1);
  //initializeServices_();
}

Device::~Device()
{
}
/*
void Device::initializeServices_()
{
  ros::service::waitForService("/niryo_one/activate_learning_mode");
  ros::service::waitForService("/niryo_one/change_tool");
  ros::service::waitForService("/niryo_one/tools/open_gripper");
  ros::service::waitForService("/niryo_one/tools/close_gripper");

  learning_mode_client_ = n_.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/activate_learning_mode");
  change_tool_srv_ = n_.serviceClient<niryo_one_msgs::SetInt>("niryo_one/change_tool");
  open_gripper_srv_ = n_.serviceClient<niryo_one_msgs::OpenGripper>("niryo_one/tools/open_gripper");
  close_gripper_srv_ = n_.serviceClient<niryo_one_msgs::CloseGripper>("niryo_one/tools/close_gripper");
}

void Device::requestLearningMode(int state)
{
  niryo_one_msgs::SetInt learning_mode;
  learning_mode.request.value = state;
  if (!learning_mode_client_.call(learning_mode))
  {
    ROS_WARN("Could not set learning mode. Service call failed.");
  }
}

void Device::setGripperId_()
{
  niryo_one_msgs::SetInt gripper_id;
  gripper_id.request.value = 12;
  change_tool_srv_.call(gripper_id);  // gripper 2
}

void Device::openGripper_()
{
  niryo_one_msgs::OpenGripper open_gripper;
  open_gripper.request.id = 12;
  open_gripper.request.open_position = 640;
  open_gripper.request.open_speed = 300;
  open_gripper.request.open_hold_torque = 128;
  open_gripper_srv_.call(open_gripper);
}

void Device::closeGripper_()
{
  niryo_one_msgs::CloseGripper close_gripper;
  close_gripper.request.id = 12;
  close_gripper.request.close_position = 400;
  close_gripper.request.close_speed = 300;
  close_gripper.request.close_hold_torque = 128;
  close_gripper.request.close_max_torque = 1023;
  close_gripper_srv_.call(close_gripper);
}
*/
}