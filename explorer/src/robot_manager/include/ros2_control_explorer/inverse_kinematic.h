/*
 *  inverse_kinematic.h
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#ifndef CARTESIAN_CONTROLLER_INVERSE_KINEMATIC_H
#define CARTESIAN_CONTROLLER_INVERSE_KINEMATIC_H

#include "rclcpp/rclcpp.hpp"

#include "ros2_control_explorer/types/joint_position.h"
#include "ros2_control_explorer/types/joint_velocity.h"
#include "ros2_control_explorer/types/space_position.h"
#include "ros2_control_explorer/types/space_velocity.h"

// MoveIt!
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"

// QPOASES
#include "qpOASES/qpOASES.hpp"

// Eigen
#include "Eigen/Dense"

using namespace Eigen;
typedef Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRowMaj;

namespace space_control
{
/**
* \brief Compute inverse kinematic
*
* This class computes inverse kinematic based on quadratic programming (QP). It relies on MoveIt RobotState and
* RobotModel to compute jacobian and qpOASES project (https://projects.coin-or.org/qpOASES/) to solve the QP
* optimization problem of the following form :
*     min   1/2*x'Hx + x'g
*     s.t.  lb  <=  x <= ub
*           lbA <= Ax <= ubA
*/
class InverseKinematic
{
public:
  enum class ControlFrame
  {
    World,
    Tool
  };

  InverseKinematic(rclcpp::Node::SharedPtr n, const int joint_number);
  void init(const std::string end_effector_link, const double sampling_period);
  void reset();
  void resolveInverseKinematic(JointVelocity& dq_computed,
                          const SpaceVelocity& dx_desired, const SpacePosition& x_desired,
                          bool path_tracking_mode);
  void setQCurrent(const JointPosition& q_current);
  void setXCurrent(const SpacePosition& x_current);
  void setPositionControlFrame(const ControlFrame frame);
  void setOrientationControlFrame(const ControlFrame frame);
  const ControlFrame& getPositionControlFrame() const;
  const ControlFrame& getOrientationControlFrame() const;

protected:
private:
  rclcpp::Node::SharedPtr n_;
  int joint_number_;
  const int space_dimension_;
  double sampling_period_;
  bool qp_init_required_; /*!< Flag to track the first iteration of QP solver */
  bool jacobian_init_flag_;

  ControlFrame position_ctrl_frame_;
  ControlFrame orientation_ctrl_frame_;

  Quaterniond jacobian_quat_prev_;
  std::vector<double> gamma_weight_vec;

  JointPosition q_current_;  /*!< Current joint position */
  SpacePosition x_current_;  /*!< Current space position */
  VectorXd x_current_eigen_; /*!< Copy of x_current in eigen format (use in matrix computation) */
  VectorXd x_des;            /*!< TODO */

  std::string end_effector_link_;

  Vector3d pos_snap_;
  double eps_inf_pos[3];

  JointPosition dq_lower_limit_; /*!< Joint lower velocity bound (vector lb) */
  JointPosition dq_upper_limit_; /*!< Joint upper velocity bound (vector ub) */

  JointPosition q_lower_limit_; /*!< Joint lower limit used in lower constraints bound vector lbA */
  JointPosition q_upper_limit_; /*!< Joint upper limit used in upper constraints bound vector ubA */
  std::vector<int> q_has_limit_;
  JointVelocity prev_dq_computed_;

  double x_min_limit[7]; /*!< Space min limit used in lower constraints bound vector lbA */
  double x_max_limit[7]; /*!< Space max limit used in upper constraints bound vector ubA */

  MatrixXd alpha_weight_;   /*!< Diagonal matrix which contains weight for space velocity minimization */
  MatrixXd beta_weight_;    /*!< Diagonal matrix which contains weight for joint velocity minimization */
  MatrixXd gamma_pos_weight_;   /*!< Diagonal matrix which contains weight for space position minimization */
  MatrixXd gamma_or_weight_;   /*!< Diagonal matrix which contains weight for space position minimization */
  MatrixXd lambda_weight_;   /*!< TODO */

  qpOASES::SQProblem* QP_; /*!< QP solver instance pointer */

  moveit::core::RobotModelPtr kinematic_model_;      /*!< MoveIt RobotModel pointer */
  moveit::core::RobotStatePtr kinematic_state_;     /*!< MoveIt RobotState pointer */
  moveit::core::JointModelGroup* joint_model_group_; /*!< MoveIt JointModelGroup pointer */

  Quaterniond quat_des;
  bool flag_pos_save[3];
  bool flag_orient_save[3];
  Vector4d Qsnap[3];
  Quaterniond r_snap_;
  Quaterniond r_snap_conj_;
  Matrix4d Rs_conj_;

  // Define the quaternion conjugate matrix such as r_conj = conjugate_mat * r
  const Matrix4d CONJ_MAT = (Matrix4d() << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1).finished();

  const double eps_pos = 0.001;
  const double eps_orientation = 0.001;
  const double inf = 10.0;

  void setAlphaWeight_(const std::vector<double>& alpha_weight);
  void setBetaWeight_(const std::vector<double>& beta_weight);
  void setGammaWeight_(const std::vector<double>& gamma_weight);
  void setLambdaWeight_(const std::vector<double>& lambda_weight);
  void setDqBounds_(const JointVelocity& dq_bound);

  void checkConstraintsDebug_(std::vector<double>& ubA, std::vector<double>& lbA, const MatrixXdRowMaj& A);

  void computeVelocityInFrame_(SpaceVelocity& dx_desired_in_frame, const SpaceVelocity& dx_desired, const Matrix3d& R_0to1);
  /*
  void computeHessianMatrix(MatrixXd& hessian, const MatrixXd& jacobian);
  void computeGradientVector(VectorXd& g, const MatrixXd& jacobian,
                                const SpaceVelocity& dx_des, const SpacePosition& x_snap);
  void computeConstrainMatrix(MatrixXdRowMaj& A, const MatrixXd& jacobian, const Matrix3d& R_0to1_transpose, const Matrix4d& Rs_conj);
  void computeConstrainVectors(std::vector<double>& lbA, std::vector<double>& ubA,
                                                const double (&x_min_limit)[7], const double (&x_max_limit)[7]);
  */
  void computeObjectives_(MatrixXd& hessian, VectorXd& g,
                                const SpaceVelocity& dx_des, const SpacePosition& x_des,
                                const MatrixXd& jacobian, bool path_tracking);
  void computeConstraints_(MatrixXdRowMaj& A, std::vector<double>& lbA,
                          std::vector<double>& ubA, const MatrixXd& jacobian);
  /**
  * \brief Compute jacobian
  *
  * This is an updated version of MoveIt (RobotState class) implementation with quaternion discontinuity handling.
  */
  bool getJacobian_(const moveit::core::RobotStatePtr kinematic_state, const moveit::core::JointModelGroup* group,
                    const moveit::core::LinkModel* link, const Vector3d& reference_point_position,
                    MatrixXd& jacobian, bool use_quaternion_representation);

  Matrix4d xR_(Quaterniond& quat);
  Matrix4d Rx_(Quaterniond& quat);
};
}
#endif
