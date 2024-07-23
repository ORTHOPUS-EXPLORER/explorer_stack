/*
 *  joint_position.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_JOINT_POSITION_H
#define CARTESIAN_CONTROLLER_JOINT_POSITION_H

#include "rclcpp/rclcpp.hpp"

namespace space_control
{
/**
* \brief Joint position vector
*/
class JointPosition : public std::vector<double>
{
public:
  JointPosition(int joint_number) : joint_number_(joint_number)
  {
    resize(joint_number, 0.0);
  };

  int getJointNumber()
  {
    return joint_number_;
  };

  /* Overload ostream << operator */
  friend std::ostream& operator<<(std::ostream& os, const JointPosition& sp)
  {
    using namespace std;
    copy(sp.begin(), sp.end(), ostream_iterator<double>(os, ", "));
    return os;
  };

private:
  int joint_number_;
};
}
#endif
