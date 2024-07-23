/*
 *  velocity_integrator.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_INTEGRATOR_H
#define CARTESIAN_CONTROLLER_INTEGRATOR_H

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "ros2_control_explorer/types/joint_position.h"
#include "ros2_control_explorer/types/joint_velocity.h"

namespace space_control
{
class VelocityIntegrator
{
public:
  VelocityIntegrator(rclcpp::Node::SharedPtr n, const int joint_number);
  void init(const double sampling_period);
  void integrate(const JointVelocity& dq_input, JointPosition& q_output);
  void setQCurrent(const JointPosition& q_current);

protected:
private:
  rclcpp::Node::SharedPtr n_;
  int joint_number_;
  double sampling_period_;
  JointPosition q_current_;
};
}
#endif
