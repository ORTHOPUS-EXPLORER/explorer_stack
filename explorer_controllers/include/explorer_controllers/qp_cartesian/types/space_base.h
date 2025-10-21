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
  private:
    mutable Eigen::Matrix3d rotation_matrix_;
    mutable bool matrix_valid_;
    mutable bool quat_valid_;
    
  public:
    using Eigen::Quaterniond::Quaterniond;
    
    // Constructor from rotation matrix (preferred internal representation)
    Orientationd(const Eigen::Matrix3d& R) : rotation_matrix_(R), matrix_valid_(true), quat_valid_(false) {
      updateQuaternionFromMatrix();
    }
    
    // Override assignment operators to maintain matrix representation
    Orientationd& operator=(const Eigen::Quaterniond& q) {
      Eigen::Quaterniond::operator=(q);
      quat_valid_ = true;
      matrix_valid_ = false;
      updateMatrixFromQuaternion();
      return *this;
    }
    
    // Get rotation matrix (always continuous, no discontinuities)
    const Eigen::Matrix3d& toRotationMatrix() const {
      if (!matrix_valid_) {
        updateMatrixFromQuaternion();
      }
      return rotation_matrix_;
    }
    
    // Set from rotation matrix (preferred method)
    void setRotationMatrix(const Eigen::Matrix3d& R) {
      rotation_matrix_ = R;
      matrix_valid_ = true;
      quat_valid_ = false;
      updateQuaternionFromMatrix();
    }
    
    // Continuous quaternion update (maintains sign consistency)
    void updateQuaternionContinuous(const Eigen::Quaterniond& prev_quat) {
      if (!quat_valid_) {
        updateQuaternionFromMatrix();
      }
      
      // Ensure quaternion continuity using dot product
      double dot_product = this->w() * prev_quat.w() + 
                          this->x() * prev_quat.x() + 
                          this->y() * prev_quat.y() + 
                          this->z() * prev_quat.z();
      
      if (dot_product < 0.0) {
        // Flip quaternion for continuity
        this->w() = -this->w();
        this->x() = -this->x();
        this->y() = -this->y();
        this->z() = -this->z();
      }
    }
    
    Eigen::Vector4d toVector() const
    {
      Eigen::Vector4d vec;
      vec(0) = w();
      vec(1) = x();
      vec(2) = y();
      vec(3) = z();
      return vec;
    }
    
  private:
    void updateMatrixFromQuaternion() const {
      if (!quat_valid_) return;
      rotation_matrix_ = this->toRotationMatrix();
      matrix_valid_ = true;
    }
    
    void updateQuaternionFromMatrix() const {
      if (!matrix_valid_) return;
      
      // Convert rotation matrix to quaternion using Eigen's method
      Eigen::Quaterniond q(rotation_matrix_);
      
      // Update quaternion components (const_cast needed for mutable update)
      const_cast<Orientationd*>(this)->w() = q.w();
      const_cast<Orientationd*>(this)->x() = q.x();
      const_cast<Orientationd*>(this)->y() = q.y();
      const_cast<Orientationd*>(this)->z() = q.z();
      
      quat_valid_ = true;
    }
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
