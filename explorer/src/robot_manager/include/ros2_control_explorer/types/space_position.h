/*
 *  space_position.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_SPACE_POSITION_H
#define CARTESIAN_CONTROLLER_SPACE_POSITION_H

#include "rclcpp/rclcpp.hpp"

#include "ros2_control_explorer/types/space_base.h"

namespace space_control
{
class SpacePosition : public SpaceBase
{
public:
  SpacePosition() : SpaceBase(){};
  SpacePosition(double (&raw_data)[7]) : SpaceBase(raw_data){};

private:
};
}
#endif
