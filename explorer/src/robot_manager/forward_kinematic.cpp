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

  /* Convert rotation matrix to quaternion */
  Eigen::Quaterniond conv_quat(end_effector_state.linear());
  /* Warning : During the convertion in quaternion, sign could change as there are tow quaternion definitions possible
   * (q and -q) for the same rotation. The following code ensure quaternion continuity between to occurence of this
   * method call
   */
  if (init_flag_)
  {
    init_flag_ = false;
  }
  else
  {
    /* Detect if a discontinuity happened between new quaternion and the previous one */
    double diff_norm =
        sqrt(pow(conv_quat.w() - x_current_.orientation.w(), 2) + pow(conv_quat.x() - x_current_.orientation.x(), 2) +
             pow(conv_quat.y() - x_current_.orientation.y(), 2) + pow(conv_quat.z() - x_current_.orientation.z(), 2));
    if (diff_norm > 1)
    {
      RCLCPP_WARN_STREAM(n_->get_logger(), "ForwardKinematic - A discontinuity has been detected during quaternion conversion.");
      /* If discontinuity happened, change sign of the quaternion */
      conv_quat.w() = -conv_quat.w();
      conv_quat.x() = -conv_quat.x();
      conv_quat.y() = -conv_quat.y();
      conv_quat.z() = -conv_quat.z();
    }
    else
    {
      /* Else, do nothing and keep quaternion sign */
    }
  }

  x_current_.position.x() = end_effector_state.translation()[0];
  x_current_.position.y() = end_effector_state.translation()[1];
  x_current_.position.z() = end_effector_state.translation()[2];
  x_current_.orientation.w() = conv_quat.w();
  x_current_.orientation.x() = conv_quat.x();
  x_current_.orientation.y() = conv_quat.y();
  x_current_.orientation.z() = conv_quat.z();
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
