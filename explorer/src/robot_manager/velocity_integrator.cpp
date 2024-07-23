/*
 *  velocity_integrator.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_explorer/velocity_integrator.h"

namespace space_control
{
VelocityIntegrator::VelocityIntegrator(rclcpp::Node::SharedPtr n, const int joint_number)
: n_(n)
, joint_number_(joint_number)
, q_current_(joint_number)
{
  RCLCPP_DEBUG_STREAM(n_->get_logger(), "VelocityIntegrator constructor");
  sampling_period_ = 0;
}

void VelocityIntegrator::init(const double sampling_period)
{
  sampling_period_ = sampling_period;
}

void VelocityIntegrator::integrate(const JointVelocity& dq_input, JointPosition& q_output)
{
  for (int i = 0; i < joint_number_; i++)
  {
    q_output[i] = q_current_[i] + dq_input[i] * sampling_period_;
  }
}

void VelocityIntegrator::setQCurrent(const JointPosition& q_current)
{
  q_current_ = q_current;
}
}
