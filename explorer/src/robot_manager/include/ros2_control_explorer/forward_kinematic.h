/*
 *  forward_kinematic.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_FORWARD_KINEMATIC_H
#define CARTESIAN_CONTROLLER_FORWARD_KINEMATIC_H

#include "rclcpp/rclcpp.hpp"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ros2_control_explorer/types/joint_position.h"
#include "ros2_control_explorer/types/space_position.h"

#include "Eigen/Dense"

namespace space_control
{
class ForwardKinematic
{
public:
  ForwardKinematic(rclcpp::Node::SharedPtr n, const int joint_number);
  void init(const std::string end_effector_link);
  void reset();
  void resolveForwardKinematic();
  void setQCurrent(const JointPosition& q_current);
  void getXCurrent(SpacePosition& x_current);

protected:
private:
  rclcpp::Node::SharedPtr n_;
  bool init_flag_;
  int joint_number_;

  std::string end_effector_link_;
  SpacePosition x_current_;
  JointPosition q_current_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
};
}
#endif
