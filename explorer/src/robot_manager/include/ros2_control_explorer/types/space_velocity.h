/*
 *  space_velocity.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_SPACE_VELOCITY_H
#define CARTESIAN_CONTROLLER_SPACE_VELOCITY_H

#include "rclcpp/rclcpp.hpp"

#include "ros2_control_explorer/types/space_base.h"

namespace space_control
{
class SpaceVelocity : public SpaceBase
{
public:
  SpaceVelocity() : SpaceBase(){};
  SpaceVelocity(double (&raw_data)[7]) : SpaceBase(raw_data){};

private:
};
}
#endif
