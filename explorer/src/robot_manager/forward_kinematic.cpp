/*
 *  forward_kinematic.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_explorer/forward_kinematic.h"

namespace space_control
{
ForwardKinematic::ForwardKinematic(rclcpp::Node::SharedPtr n, const int joint_number)
  : n_(n)
  , joint_number_(joint_number)
  , x_current_()
  , q_current_(joint_number)
{
  robot_model_loader::RobotModelLoader robot_model_loader(n_);
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();
  end_effector_link_ = "";
}

void ForwardKinematic::init(const std::string end_effector_link)
{
  end_effector_link_ = end_effector_link;
  init_flag_ = true;
}

void ForwardKinematic::reset()
{
  init_flag_ = true;
}

void ForwardKinematic::resolveForwardKinematic()
{
  /* Set kinemtic state of the robot to the previous joint positions computed */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();

  /* Get the cartesian state of the tool_link frame */
  const Eigen::Affine3d& end_effector_state =
      kinematic_state_->getGlobalLinkTransform(kinematic_state_->getLinkModel(end_effector_link_));

  /* Use rotation matrix directly - no discontinuities! */
  Eigen::Matrix3d rotation_matrix = end_effector_state.linear();
  
  /* Store position */
  x_current_.position.x() = end_effector_state.translation()[0];
  x_current_.position.y() = end_effector_state.translation()[1];
  x_current_.position.z() = end_effector_state.translation()[2];
  
  /* Set orientation using rotation matrix (eliminates quaternion discontinuities) */
  x_current_.orientation.setRotationMatrix(rotation_matrix);
}

void ForwardKinematic::setQCurrent(const JointPosition& q_current)
{
  q_current_ = q_current;
}

void ForwardKinematic::getXCurrent(SpacePosition& x_current)
{
  x_current = x_current_;
}
}
