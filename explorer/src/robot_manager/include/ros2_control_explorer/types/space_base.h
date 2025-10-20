/*
 *  space_base.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_SPACE_BASE_H
#define CARTESIAN_CONTROLLER_SPACE_BASE_H

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"

// Eigen
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 7, 1> Vector7d;

namespace space_control
{
/**
* \brief Parent class of coordinate space position or velocity.
*/
class SpaceBase
{
public:
  SpaceBase() : position(0, 0, 0), orientation(0, 0, 0, 0){};

  SpaceBase(double (&raw_data)[7])
  {
    for (int i = 0; i < 7; i++)
    {
      (*this)[i] = raw_data[i];
    }
  };

  SpaceBase(const geometry_msgs::msg::Pose p)
  {
    position.x() = p.position.x;
    position.y() = p.position.y;
    position.z() = p.position.z;
    orientation.w() = p.orientation.w;
    orientation.x() = p.orientation.x;
    orientation.y() = p.orientation.y;
    orientation.z() = p.orientation.z;
  };

  virtual ~SpaceBase() = 0;

  class Positiond : public Eigen::Vector3d
  {
  public:
    using Eigen::Vector3d::Vector3d;
    inline double x(void) const
    {
      return m_storage.data()[0];
    };
    inline double y(void) const
    {
      return m_storage.data()[1];
    };
    inline double z(void) const
    {
      return m_storage.data()[2];
    };

    inline double& x(void)
    {
      return m_storage.data()[0];
    };
    inline double& y(void)
    {
      return m_storage.data()[1];
    };
    inline double& z(void)
    {
      return m_storage.data()[2];
    };
  };

  class Orientationd : public Eigen::Quaterniond
  {
  public:
    using Eigen::Quaterniond::Quaterniond;
    Eigen::Vector4d toVector() const
    {
      Eigen::Vector4d vec;
      vec(0) = w();
      vec(1) = x();
      vec(2) = y();
      vec(3) = z();
      return vec;
    };
  };

  bool isAllZero() const
  {
    bool ret = true;
    for (int i = 0; i < 7; i++)
    {
      if ((*this)[i] != 0.0)
      {
        ret = false;
        break;
      }
    }
    return ret;
  };

  Positiond getPosition() const
  {
    return position;
  };
  void setPosition(const Positiond& p)
  {
    position = p;
  };

  Orientationd getOrientation() const
  {
    return orientation;
  };

  void setOrientation(const Orientationd& q)
  {
    orientation = q;
  };

  Vector7d getRawVector() const
  {
    Vector7d ret;
    ret(0) = position.x();
    ret(1) = position.y();
    ret(2) = position.z();
    ret(3) = orientation.w();
    ret(4) = orientation.x();
    ret(5) = orientation.y();
    ret(6) = orientation.z();
    return ret;
  };

  void setRawVector(const Vector7d& raw_vect)
  {
    position.x() = raw_vect(0);
    position.y() = raw_vect(1);
    position.z() = raw_vect(2);
    orientation.w() = raw_vect(3);
    orientation.x() = raw_vect(4);
    orientation.y() = raw_vect(5);
    orientation.z() = raw_vect(6);
  };

  /* Write raw data operator */
  double& operator[](int i)
  {
    if (i >= 0 && i < 3)
    {
      return position[i];
    }
    else if (i == 3)
    {
      return orientation.w();
    }
    else if (i == 4)
    {
      return orientation.x();
    }
    else if (i == 5)
    {
      return orientation.y();
    }
    else if (i == 6)
    {
      return orientation.z();
    }
  };

  /* Read raw data operator */
  double operator[](int i) const
  {
    if (i >= 0 && i < 3)
    {
      return position[i];
    }
    else if (i == 3)
    {
      return orientation.w();
    }
    else if (i == 4)
    {
      return orientation.x();
    }
    else if (i == 5)
    {
      return orientation.y();
    }
    else if (i == 6)
    {
      return orientation.z();
    }
    else
    {
      return 0.0;
    }
  };

  /* ostream << operator */
  friend std::ostream& operator<<(std::ostream& os, const SpaceBase& sp)
  {
    return os << "position : [" << sp.position(0) << ", " << sp.position(1) << ", " << sp.position(2)
              << "] orientation :[" << sp.orientation.w() << ", " << sp.orientation.x() << ", " << sp.orientation.y()
              << ", " << sp.orientation.z() << "]";
  };

  Positiond position;
  Orientationd orientation;
};
}
#endif
