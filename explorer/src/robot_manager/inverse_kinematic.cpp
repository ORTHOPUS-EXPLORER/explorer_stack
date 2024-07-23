/*
 *  inverse_kinematic.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_explorer/inverse_kinematic.h"

//#include <eigen_conversions/eigen_msg.h>
#include <math.h>

namespace space_control
{
InverseKinematic::InverseKinematic(rclcpp::Node::SharedPtr n, const int joint_number)
  : n_(n)
  , joint_number_(joint_number)
  , space_dimension_(7)
  , sampling_period_(0)
  , qp_init_required_(true)
  , jacobian_init_flag_(true)
  , position_ctrl_frame_(ControlFrame::World)
  , orientation_ctrl_frame_(ControlFrame::World)
  , q_current_(joint_number)
  , x_current_()
  , dq_lower_limit_(joint_number)
  , dq_upper_limit_(joint_number)
  , q_lower_limit_(joint_number)
  , q_upper_limit_(joint_number)
  , q_has_limit_(joint_number)
  , prev_dq_computed_(joint_number)
{
  //rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  RCLCPP_DEBUG_STREAM(n_->get_logger(), "InverseKinematic constructor");

  std::vector<double> alpha_weight_vec;
  std::vector<double> beta_weight_vec;
  std::vector<double> gamma_weight_vec;

  n_->get_parameter("alpha_weight", alpha_weight_vec);
  n_->get_parameter("beta_weight", beta_weight_vec);
  n_->get_parameter("gamma_weight", gamma_weight_vec);
  //n_->get_parameter("lambda_weight", lambda_weight_);
  //n_->get_parameter("q_natural", q_natural_);

  setAlphaWeight_(alpha_weight_vec);
  setBetaWeight_(beta_weight_vec);
  setGammaWeight_(gamma_weight_vec);
  //setLambdaWeight_(lambda_weight_vec);

  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Setting up bounds on joint position (q) of the QP");
  for(int i=0; i<joint_number_; i++){
    n_->get_parameter("j"+std::to_string(i+1)+".limits", q_has_limit_[i]);
    if (q_has_limit_[i] == 1)
    {
      n_->get_parameter("j"+std::to_string(i+1)+".min", q_lower_limit_[i]);
      n_->get_parameter("j"+std::to_string(i+1)+".max", q_upper_limit_[i]);
    }
    else
    {
      q_lower_limit_[i] = 0;
      q_upper_limit_[i] = 0;
    }
    RCLCPP_DEBUG_STREAM(n_->get_logger(),"J" << i+1 << " - min:" << q_lower_limit_[i] << " max:" << q_upper_limit_[i]);
  }

  /*
   * To limit the joint velocity, we define a constraint for the QP optimisation. Assuming a commun limit of dq_max for
   * all joints, we could write the lower and upper bounds as :
   *        (-dq_max) < dq < dq_max
   * so :
   *      lb = -dq_max
   *      ub = dq_max
   */
  RCLCPP_DEBUG(n_->get_logger(),"Setting up bounds on joint velocity (dq) of the QP");
  double dq_max;
  n_->get_parameter("joint_max_vel", dq_max);
  JointVelocity limit = JointVelocity(joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    limit[i] = dq_max;
  }
  setDqBounds_(limit);

  std::string model_group_name;
  n_->get_parameter("model_group_name", model_group_name);
  RCLCPP_DEBUG(n_->get_logger(),"model group name %s",  model_group_name.c_str());
  robot_model_loader::RobotModelLoader robot_model_loader(n_);
  kinematic_model_ = robot_model_loader.getModel();
  kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model_->getJointModelGroup(model_group_name);
}

void InverseKinematic::init(const std::string end_effector_link, const double sampling_period)
{
  end_effector_link_ = end_effector_link;
  sampling_period_ = sampling_period;
}

void InverseKinematic::setAlphaWeight_(const std::vector<double>& alpha_weight)
{
  // Minimize cartesian velocity (dx) weight
  alpha_weight_ = MatrixXd::Identity(space_dimension_, space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    alpha_weight_(i, i) = alpha_weight[i];
  }

  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set alpha weight to : \n" << alpha_weight_ << "\n");
}

void InverseKinematic::setBetaWeight_(const std::vector<double>& beta_weight)
{
  // Minimize joint velocity (dq) weight
  beta_weight_ = MatrixXd::Identity(joint_number_, joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    beta_weight_(i, i) = beta_weight[i];
  }
  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set beta weight to : \n" << beta_weight_ << "\n");
}

void InverseKinematic::setGammaWeight_(const std::vector<double>& gamma_weight)
{
  // Minimize cartesian position drift on non-driven axes
  gamma_pos_weight_ = Matrix3d::Identity();
  gamma_or_weight_ = Matrix4d::Identity();
  for (int i = 0; i < 3; i++)
  {
    gamma_pos_weight_(i, i) = gamma_weight[i];
  }
  for (int i = 0; i < 4; i++)
  {
    gamma_or_weight_(i, i) = gamma_weight[i+3];
  }
  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set gamma position weight to : \n" << gamma_pos_weight_ << "\n");
  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set gamma orientation weight to : \n" << gamma_or_weight_ << "\n");
}

void InverseKinematic::setLambdaWeight_(const std::vector<double>& lambda_weight)
{
  // Minimize cartesian position drift on non-driven axes
  lambda_weight_ = MatrixXd::Identity(space_dimension_, space_dimension_);
  for (int i = 0; i < joint_number_; i++)
  {
    lambda_weight_(i, i) = lambda_weight[i];
  }
  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set lambda weight to : \n" << lambda_weight_ << "\n");
}

void InverseKinematic::setQCurrent(const JointPosition& q_current)
{
  q_current_ = q_current;
}

void InverseKinematic::setXCurrent(const SpacePosition& x_current)
{
  x_current_ = x_current;
  /* As most of computation in this class are eigen matrix operation,
  the current state is also copied in an eigen vector */
  x_current_eigen_ = VectorXd(space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    x_current_eigen_(i) = x_current[i];
  }
}

void InverseKinematic::setPositionControlFrame(const ControlFrame frame)
{
  position_ctrl_frame_ = frame;
}

void InverseKinematic::setOrientationControlFrame(const ControlFrame frame)
{
  orientation_ctrl_frame_ = frame;
}

const InverseKinematic::ControlFrame& InverseKinematic::getPositionControlFrame() const
{
  return position_ctrl_frame_;
}

const InverseKinematic::ControlFrame& InverseKinematic::getOrientationControlFrame() const
{
  return orientation_ctrl_frame_;
}

void InverseKinematic::setDqBounds_(const JointVelocity& dq_bound)
{
  for (unsigned int i = 0; i < dq_bound.size(); i++)
  {
    dq_lower_limit_[i] = -dq_bound[i];
    dq_upper_limit_[i] = dq_bound[i];
  }
}

void InverseKinematic::reset()
{
  /* Initialize a flag used to init QP if required */
  qp_init_required_ = true;
  jacobian_init_flag_ = true;
}

void InverseKinematic::resolveInverseKinematic(JointVelocity& dq_computed,
                        const SpaceVelocity& dx_desired, const SpacePosition& x_desired,
                        bool path_tracking_mode)
{
  /*
   * Setup IK inputs according to selected control frame
   * Note : Depending on position_ctrl_frame_ and orientation_ctrl_frame_, the dx_desired parameter could be interpreted
   * as tool frame or world frame setpoint
   */

  Matrix3d R_0to1 = Matrix3d::Identity();
  Matrix3d R_0to1_transpose = Matrix3d::Identity();

  if (position_ctrl_frame_ == ControlFrame::World)
  {
    R_0to1_transpose = Matrix3d::Identity();
  }
  else if (position_ctrl_frame_ == ControlFrame::Tool)
  {
    R_0to1 = x_current_.getOrientation().toRotationMatrix();
    R_0to1_transpose = R_0to1.transpose();
  }
  else
  {
    RCLCPP_ERROR(n_->get_logger(), "This control frame is not already handle !");
  }
  SpaceVelocity dx_desired_in_frame;
  computeVelocityInFrame_(dx_desired_in_frame, dx_desired, R_0to1_transpose);

  /*********************************************************/

  /* Set kinematic state of the robot to the current joint positions */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();

  /* Get jacobian */
  Vector3d reference_point_position(0.0, 0.0, 0.0);
  MatrixXd jacobian;
  getJacobian_(kinematic_state_, joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
               reference_point_position, jacobian, true);
  //kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
  //             reference_point_position, jacobian, true);

  /*
   * qpOASES solves QPs of the following form :
   * [1]    min   1/2*x'Hx + x'g
   *        s.t.  lb  <=  x <= ub
   *             lbA <= Ax <= ubA
   * where :
   *      - x' is the transpose of x
   *      - H is the hesiian matrix
   *      - g is the gradient vector
   *      - lb and ub are respectively the lower and upper bound constraint vectors
   *      - lbA and ubA are respectively the lower and upper inequality constraint vectors
   *      - A is the constraint matrix
   *
   * .:!:. NOTE : the symbol ' refers to the transpose of a matrix
   *
   * === Context ==============================================================================================
   * Usually, to resolve inverse kinematic, people use the well-known inverse jacobian formula :
   *        dX = J.dq     =>      dq = J^-1.dX
   * where :
   *      - dX is the cartesian velocity
   *      - dq is the joint velocity
   *      - J the jacobian
   * Unfortunatly, this method lacks when robot is in singular configuration and can lead to unstabilities.
   * ==========================================================================================================
   *
   * In our case, to resolve the inverse kinematic, we use a QP to minimize the joint velocity (dq). This minimization
   * intends to follow a cartesian velocity (dX_des) and position (X_des) trajectory while minimizing joint
   * velocity (dq) :
   *        min(dq) (1/2 * || dX - dX_des ||_alpha^2 + 1/2 * || dq ||_beta^2 + 1/2 * || X - X_des ||_gamma^2)
   * where :
   *      - alpha is the cartesian velocity weight matrix
   *      - beta is the joint velocity weight matrix
   *      - gamma is the cartesian position weight matrix
   *
   * Knowing that dX = J.dq, we have :
   * [2]    min(dq) (1/2 * || J.dq - dX_des ||_alpha^2 + 1/2 * || dq ||_beta^2 + 1/2 * || X - X_des ||_gamma^2)
   *
   * In order to reduce X, we perform a Taylor development (I also show how I develop this part):
   * [3]      1/2 * || X - X_des ||_gamma^2 = 1/2 * || X_0 + T.dX - X_des ||_gamma^2
   *                                      = 1/2 * || X_0 + T.J.dq - X_des ||_gamma^2
   *                                      = 1/2 * (X_0'.gamma.X_0 + dq'.J'.gamma.J.dq*T^2 + X_des'.gamma.X_des)
   *                                      + dq'.J'.gamma.X_0*T - X_des'.gamma.X_0 - X_des'.gamma.J.dq*T
   * where :
   *      - X_0 is the initial position
   *
   * Then, after developing the rest of the equation [2] as shown in [3]:
   *        min(dq) (1/2 * dq'.(J'.alpha.J + beta + J'.gamma.J*T^2).dq
   *               + dq'(-J'.alpha.dX_des + (J'.gamma.X_0 - J'.gamma.Xdes)*T))
   *
   * After identification with [1], we have :
   *        H = J'.alpha.J + beta + J'.gamma.J*T^2
   *        g = -J'.alpha.dX_des + (J'.gamma.X_0 - J'.gamma.Xdes)*T
   *
   *
   * In order to limit the joint position, we define a inequality constraint for the QP optimisation.
   * Taylor developpement of joint position is :
   *        q = q_0 + dq*T
   * with
   *      - q_0 the initial joint position.
   *
   * So affine inequality constraint could be written as :
   *        q_min < q < q_max
   *        q_min < q0 + dq*T < q_max
   *        (q_min-q0) < dq.T < (q_max-q0)
   *
   * so :
   *        A = I.T
   *        lbA = q_min-q0
   *        ubA = q_max-q0
   * where :
   *      - I is the identity matrix
   *
   */

  if (qp_init_required_)
  {
   for (int i = 0; i < 3; i++)
   {
     flag_pos_save[i] = true;
   }
   pos_snap_ = x_current_.getPosition();
   r_snap_ = x_current_.getOrientation();
  }

  for (int i = 0; i < 3; i++)
  {
   if (dx_desired[i] != 0)
   {
     flag_pos_save[i] = true;
   }
   else
   {
     if (flag_pos_save[i] == true)
     {
       flag_pos_save[i] = false;
       pos_snap_ = x_current_.getPosition();
     }
   }
  }
  for (int i = 0; i < 3; i++)
  {
    if (dx_desired[4 + i] != 0.0)
    {
      flag_orient_save[i] = true;
    }
    else
    {
      if (flag_orient_save[i] == true)
      {
        flag_orient_save[i] = false;
        r_snap_ = x_current_.getOrientation();
      }
    }
  }

  double x_snap_vect[] = {pos_snap_[0], pos_snap_[1], pos_snap_[2], r_snap_.w(), r_snap_.x(), r_snap_.y(), r_snap_.z()};
  SpacePosition x_snap(x_snap_vect);

  SpacePosition x_des_objective = path_tracking_mode ? x_desired : x_snap;

  MatrixXd hessian;
  VectorXd g;
  computeObjectives_(hessian, g, dx_desired_in_frame, x_des_objective, jacobian, path_tracking_mode);
  // RCLCPP_DEBUG_STREAM(n_->get_logger(),"Hessian Matrix:\n" << hessian);
  // RCLCPP_DEBUG_STREAM(n_->get_logger(),"Gradient Vector\n:" << g);


  MatrixXdRowMaj A;
  std::vector<double> lbA(joint_number_);
  std::vector<double> ubA(joint_number_);
  computeConstraints_(A, lbA, ubA, jacobian);

  RCLCPP_DEBUG_STREAM(n_->get_logger(), "x_snap = " << x_snap);
  // checkConstraintsDebug_(ubA, lbA, A);

  /* Solve QP */
  qpOASES::real_t xOpt[6];
  qpOASES::int_t nWSR = 10;
  qpOASES::returnValue qp_return = qpOASES::SUCCESSFUL_RETURN;
  if (qp_init_required_)
  {
    /* Initialize QP solver */
    QP_ = new qpOASES::SQProblem(joint_number_, joint_number_);// + 6);
    qpOASES::Options options;
    //Options options;
    options.setToReliable();
    // options.enableInertiaCorrection = qpOASES::BT_TRUE;
    options.printLevel = qpOASES::PL_NONE;
    QP_->setOptions(options);

    qp_return = QP_->init(hessian.data(), g.data(), A.data(), dq_lower_limit_.data(), dq_upper_limit_.data(),
                          lbA.data(), ubA.data(), nWSR, 0);
    qp_init_required_ = false;
  }
  else
  {
    qp_return = QP_->hotstart(hessian.data(), g.data(), A.data(), dq_lower_limit_.data(), dq_upper_limit_.data(),
                          lbA.data(), ubA.data(), nWSR, 0);
  }

  // enableInertiaCorrection ? ?

  if (qp_return == qpOASES::SUCCESSFUL_RETURN)
  {
    RCLCPP_DEBUG_STREAM(n_->get_logger(),"qpOASES : succesfully return");

    /* Get solution of the QP */
    QP_->getPrimalSolution(xOpt);
    for (int i=0; i<joint_number_; i++){
      dq_computed[i] = xOpt[i];
      prev_dq_computed_[i] = dq_computed[i];
    }
  }
  else
  {
    RCLCPP_ERROR(n_->get_logger(), "qpOASES : Failed with code : %d !", qp_return);
    const char* error_msg = qpOASES::MessageHandling::getErrorCodeMessage(qp_return);
    RCLCPP_ERROR(n_->get_logger(), "qpOASES : Failed with message : %s", error_msg);
    for (int i=0; i<joint_number_; i++){
      dq_computed[i] = 0.0;
    }
  }

  if (qp_return != qpOASES::SUCCESSFUL_RETURN && qp_return != qpOASES::RET_MAX_NWSR_REACHED)
  {
    reset();
    exit(0);  // TODO improve error handling. Crash of the application is neither safe nor beautiful
  }
}



void InverseKinematic::computeVelocityInFrame_(SpaceVelocity& dx_desired_in_frame, const SpaceVelocity& dx_desired, const Matrix3d& R_0to1)
{
  /* Compute the desired linear velocity according to the selected control frame :
   *       p0 = R_0to1 * p1
   * with :
   *     - p0 : the linear velocity in world frame
   *     - p1 : the linear velocity in tool frame
   *     - R_0to1 : the rotation matrix of the tool link in the world frame
   */
  dx_desired_in_frame.setPosition(R_0to1 * dx_desired.getPosition());

  /* Compute the desired andular velocity according to the selected control frame into a quaternion velocity
   * using the formula :
   *       r_dot = 1/2 * omega * r_c
   * with :
   *     - r_dot : the orientation velocity in quaternion representation
   *     - r_c : the current orientation in quaternion representation
   *     - omega : the angular velocity in rad/s
   *
   * Warning : "omega * r_c" is quaternion product !
   *
   * Note : velocity in tool frame can be written as :
   *       r_dot = 1/2 * r_c * omega
   */

  // The omega vector (store in dx_desired) can be consider as a quaternion with a scalar part equal to zero
  Quaterniond r_half_omega(0.0, 0.5 * dx_desired.orientation.x(), 0.5 * dx_desired.orientation.y(),
                                  0.5 * dx_desired.orientation.z());
  Quaterniond r_dot;
  if (orientation_ctrl_frame_ == ControlFrame::World)
  {
    r_dot = r_half_omega * x_current_.getOrientation();
  }
  else if (orientation_ctrl_frame_ == ControlFrame::Tool)
  {
    r_dot = x_current_.getOrientation() * r_half_omega;
  }
  else
  {
    RCLCPP_ERROR(n_->get_logger(), "This control frame is not already handle !");
  }
  dx_desired_in_frame.setOrientation(r_dot);
}

void InverseKinematic::computeObjectives_(MatrixXd& hessian, VectorXd& g,
                              const SpaceVelocity& dx_des, const SpacePosition& x_des,
                              const MatrixXd& jacobian, bool path_tracking)
{
  Matrix3d pos_controlled_mat = Matrix3d::Identity();
  Matrix4d or_controlled_mat = Matrix4d::Identity();
  or_controlled_mat(0, 0) = 0.0;
  for(int i=0; i<space_dimension_; i++)
  {
    if( dx_des[i] != 0 && !path_tracking )
    {
      if( i<3 )
      {
        pos_controlled_mat(i,i) = 0.0;
      }
      else
      {
        or_controlled_mat(i-3,i-3) = 0.0;
      }
    }
  }

  MatrixXd dx_controlled_mat = MatrixXd::Identity(space_dimension_, space_dimension_);

  MatrixXd jacob_pos = jacobian.topRows(3);
  MatrixXd jacob_or = jacobian.bottomRows(4);
  Matrix4d Rdes_conj;
  Quaterniond x_des_quat = x_des.getOrientation();
  if (orientation_ctrl_frame_ == ControlFrame::World)
  {
    Rdes_conj = Rx_(x_des_quat) * CONJ_MAT;
  }
  else if (orientation_ctrl_frame_ == ControlFrame::Tool)
  {
    Rdes_conj = xR_(x_des_quat) * CONJ_MAT;
  }

  hessian = jacobian.transpose() * alpha_weight_ * dx_controlled_mat * jacobian
          + jacob_pos.transpose() * gamma_pos_weight_ * pos_controlled_mat * jacob_pos * sampling_period_ * sampling_period_
          + jacob_or.transpose() * Rdes_conj.transpose() * gamma_or_weight_ * or_controlled_mat * Rdes_conj * jacob_or * sampling_period_ * sampling_period_
          + beta_weight_;
  //hessian = (jacobian.transpose() * (alpha_weight_ + gamma_weight_ * ctrl_axes_mat) * jacobian)
  //          + beta_weight_
  //          + lambda_weight_ * ;

  g = -jacobian.transpose() * alpha_weight_ * dx_controlled_mat * dx_des.getRawVector()
      + jacob_pos.transpose() * gamma_pos_weight_ * pos_controlled_mat * (x_current_.getPosition() - x_des.getPosition()) * sampling_period_
      + jacob_or.transpose() * Rdes_conj.transpose() * gamma_or_weight_ * or_controlled_mat * Rdes_conj * x_current_.getOrientation().toVector() * sampling_period_;
  //    + jacobian.transpose() * gamma_weight_ * ctrl_axes_mat * sampling_period_ * delta_X_current_snap.getRawVector()
  //    + lambda_weight_ * sampling_period_ * q_natural_.getRawVector();
}

void InverseKinematic::computeConstraints_(MatrixXdRowMaj& A, std::vector<double>& lbA,
                                          std::vector<double>& ubA, const MatrixXd& jacobian)
{
  A.resize(joint_number_, joint_number_);

  A.topLeftCorner(joint_number_, joint_number_) = MatrixXd::Identity(joint_number_, joint_number_) * sampling_period_;

  for (int i=0; i < joint_number_; i++){
    if (q_has_limit_[i] == 1){
      lbA[i] = (q_lower_limit_[i] - q_current_[i]);
      ubA[i] = (q_upper_limit_[i] - q_current_[i]);
    }
    else
    {
      lbA[i] = -inf;
      ubA[i] = inf;
      A(i, i) = 0.0;
    }
  }
}



bool InverseKinematic::getJacobian_(const moveit::core::RobotStatePtr kinematic_state, const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* link,
                                    const Vector3d& reference_point_position, MatrixXd& jacobian,
                                    bool use_quaternion_representation)
{
  //BOOST_VERIFY(checkLinkTransforms());

  if (!group->isChain())
  {
    RCLCPP_ERROR(n_->get_logger(), "The group '%s' is not a chain. Cannot compute Jacobian.", group->getName().c_str());
    return false;
  }

  if (!group->isLinkUpdated(link->getName()))
  {
    RCLCPP_ERROR(n_->get_logger(), "Link name '%s' does not exist in the chain '%s' or is not a child for this chain",
                 link->getName().c_str(), group->getName().c_str());
    return false;
  }

  const moveit::core::JointModel* root_joint_model = group->getJointModels()[0];  // group->getJointRoots()[0];
  const moveit::core::LinkModel* root_link_model = root_joint_model->getParentLinkModel();
  // getGlobalLinkTransform() returns a valid isometry by contract
  Isometry3d reference_transform =
      root_link_model ? kinematic_state->getGlobalLinkTransform(root_link_model).inverse() : Isometry3d::Identity();
  int rows = use_quaternion_representation ? 7 : 6;
  int columns = group->getVariableCount();
  jacobian = MatrixXd::Zero(rows, columns);

  // getGlobalLinkTransform() returns a valid isometry by contract
  Isometry3d link_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);  // valid isometry
  Vector3d point_transform = link_transform * reference_point_position;

  /*
  RCLCPP_DEBUG(LOGGER, "Point from reference origin expressed in world coordinates: %f %f %f",
           point_transform.x(),
           point_transform.y(),
           point_transform.z());
  */

  Vector3d joint_axis;
  Isometry3d joint_transform;

  while (link)
  {
    
    // RCLCPP_DEBUG(n_->get_logger(), "Link: %s, %f %f %f",link->getName().c_str(),
    //          kinematic_state->getGlobalLinkTransform(link).translation().x(),
    //          kinematic_state->getGlobalLinkTransform(link).translation().y(),
    //          kinematic_state->getGlobalLinkTransform(link).translation().z());
    // RCLCPP_DEBUG(n_->get_logger(), "Joint: %s",link->getParentJointModel()->getName().c_str());
    
    const moveit::core::JointModel* pjm = link->getParentJointModel();
    if (pjm->getVariableCount() > 0)
    {
      if (!group->hasJointModel(pjm->getName()))
      {
        link = pjm->getParentLinkModel();
        continue;
      }
      unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());
      // getGlobalLinkTransform() returns a valid isometry by contract
      joint_transform = reference_transform * kinematic_state->getGlobalLinkTransform(link);  // valid isometry
      if (pjm->getType() == moveit::core::JointModel::REVOLUTE)
      {
        joint_axis = joint_transform.linear() * static_cast<const moveit::core::RevoluteJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) =
            jacobian.block<3, 1>(0, joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index) = jacobian.block<3, 1>(3, joint_index) + joint_axis;
      }
      else if (pjm->getType() == moveit::core::JointModel::PRISMATIC)
      {
        joint_axis = joint_transform.linear() * static_cast<const moveit::core::PrismaticJointModel*>(pjm)->getAxis();
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
      }
      else if (pjm->getType() == moveit::core::JointModel::PLANAR)
      {
        joint_axis = joint_transform * Vector3d(1.0, 0.0, 0.0);
        jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
        joint_axis = joint_transform * Vector3d(0.0, 1.0, 0.0);
        jacobian.block<3, 1>(0, joint_index + 1) = jacobian.block<3, 1>(0, joint_index + 1) + joint_axis;
        joint_axis = joint_transform * Vector3d(0.0, 0.0, 1.0);
        jacobian.block<3, 1>(0, joint_index + 2) = jacobian.block<3, 1>(0, joint_index + 2) +
                                                   joint_axis.cross(point_transform - joint_transform.translation());
        jacobian.block<3, 1>(3, joint_index + 2) = jacobian.block<3, 1>(3, joint_index + 2) + joint_axis;
      }
      else
        RCLCPP_ERROR(n_->get_logger(), "Unknown type of joint in Jacobian computation");
    }
    if (pjm == root_joint_model)
      break;
    link = pjm->getParentLinkModel();
  }
  if (use_quaternion_representation)
  {
    /* Convert rotation matrix to quaternion */
    Quaterniond conv_quat(link_transform.linear());

    /* Warning : During the convertion in quaternion, sign could change as there are tow quaternion definitions possible
     * (q and -q) for the same rotation. The following code ensure quaternion continuity between to occurence of this
     * method call
     */
    if (jacobian_init_flag_)
    {
      jacobian_init_flag_ = false;
    }
    else
    {
      /* Detect if a discontinuity happened between new quaternion and the previous one */
      double diff_norm =
          sqrt(pow(conv_quat.w() - jacobian_quat_prev_.w(), 2) + pow(conv_quat.x() - jacobian_quat_prev_.x(), 2) +
               pow(conv_quat.y() - jacobian_quat_prev_.y(), 2) + pow(conv_quat.z() - jacobian_quat_prev_.z(), 2));
      if (diff_norm > 1)
      {
        RCLCPP_WARN_STREAM(n_->get_logger(), "InverseKinematic - A discontinuity has been detected during quaternion conversion.");
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
    jacobian_quat_prev_ = conv_quat;

    //double w = conv_quat.w(), x = conv_quat.x(), y = conv_quat.y(), z = conv_quat.z();
    // MatrixXd quaternion_update_matrix(4, 3);

    /* d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
     *        [x]           [  w  z -y ]    [ omega_2 ]
     *        [y]           [ -z  w  x ]    [ omega_3 ]
     *        [z]           [  y -x  w ]
     */
    // quaternion_update_matrix << -x, -y, -z, w, z, -y, -z, w, x, y, -x, w;

    // Vector4d omega;
    // omega(0) = 0.0;
    // omega.bottomLeftCorner(3, 1) =
    jacobian.block(3, 0, 4, columns) = 0.5 * xR_(conv_quat).block(0, 1, 4, 3) * jacobian.block(3, 0, 3, columns);
  }
  return true;
}

void InverseKinematic::checkConstraintsDebug_(std::vector<double>& ubA, std::vector<double>& lbA, const MatrixXdRowMaj& A)
{
  VectorXd dq_init_vect = Eigen::Map<VectorXd>(prev_dq_computed_.data(), prev_dq_computed_.size());
  VectorXd ubA_eigen = Eigen::Map<VectorXd>(ubA.data(), ubA.size());
  VectorXd lbA_eigen = Eigen::Map<VectorXd>(lbA.data(), lbA.size());
  VectorXd Adq = A*dq_init_vect;
  MatrixXd ConstraintMat(Adq.rows(), Adq.cols()+2);
  ConstraintMat << lbA_eigen, Adq, ubA_eigen;

  for(unsigned int i=0; i < ubA.size(); i++){
    if (lbA[i] >= Adq[i] || ubA[i] <= Adq[i]){
      double diff = std::max( lbA[i]-Adq[i], Adq[i]-ubA[i] );
      RCLCPP_WARN_STREAM(n_->get_logger(), "Constrain n°" << i << " infeasible ! delta=" << diff);
    }
  }
  RCLCPP_DEBUG_STREAM(n_->get_logger(), "Constraints check matrix : \n" << ConstraintMat );
}


/* Quaternion post-product matrix */
Matrix4d InverseKinematic::xR_(Quaterniond& quat)
{
  double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  Matrix4d ret_mat;
  ret_mat << w, -x, -y, -z, x, w, z, -y, y, -z, w, x, z, y, -x, w;
  return ret_mat;
}

/* Quaternion pre-product matrix */
Matrix4d InverseKinematic::Rx_(Quaterniond& quat)
{
  double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
  Matrix4d ret_mat;
  ret_mat << w, -x, -y, -z, x, w, -z, y, y, z, w, -x, z, -y, x, w;
  return ret_mat;
}
}
