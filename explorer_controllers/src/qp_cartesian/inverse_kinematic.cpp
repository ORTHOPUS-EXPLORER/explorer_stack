/*
 *  inverse_kinematic.cpp
 *  Copyright (C) 2022 Orthopus
 *  All rights reserved.
 */
#include "rclcpp/rclcpp.hpp"

#include "explorer_controllers/qp_cartesian/inverse_kinematic.h"

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

  n_->get_parameter("alpha_multiplier", alpha_multiplier);
  n_->get_parameter("beta_multiplier", beta_multiplier);
  n_->get_parameter("gamma_multiplier", gamma_multiplier);
  n_->get_parameter("joint_centering_multiplier", joint_centering_multiplier);
  
  // Control frame parameters
  std::string position_frame_str, orientation_frame_str;
  if (!n_->get_parameter("position_control_frame", position_frame_str)) {
    position_frame_str = "World";  // Default to World frame
    RCLCPP_INFO(n_->get_logger(), "Using default position_control_frame: %s", position_frame_str.c_str());
  }
  if (!n_->get_parameter("orientation_control_frame", orientation_frame_str)) {
    orientation_frame_str = "World";  // Default to World frame
    RCLCPP_INFO(n_->get_logger(), "Using default orientation_control_frame: %s", orientation_frame_str.c_str());
  }
  
  // Set control frames from parameters
  if (position_frame_str == "Tool") {
    position_ctrl_frame_ = ControlFrame::Tool;
  } else if (position_frame_str == "DrinkSmall") {
    position_ctrl_frame_ = ControlFrame::DrinkSmall;
  } else if (position_frame_str == "DrinkBig") {
    position_ctrl_frame_ = ControlFrame::DrinkBig;
  } else {
    position_ctrl_frame_ = ControlFrame::World;
  }
  
  if (orientation_frame_str == "Tool") {
    orientation_ctrl_frame_ = ControlFrame::Tool;
  } else if (orientation_frame_str == "DrinkSmall") {
    orientation_ctrl_frame_ = ControlFrame::DrinkSmall;
  } else if (orientation_frame_str == "DrinkBig") {
    orientation_ctrl_frame_ = ControlFrame::DrinkBig;
  } else {
    orientation_ctrl_frame_ = ControlFrame::World;
  }
  
  RCLCPP_INFO(n_->get_logger(), "Control frames - Position: %s, Orientation: %s", 
              position_frame_str.c_str(), orientation_frame_str.c_str());
  
  // Joint centering threshold parameter
  if (!n_->get_parameter("j5_alignment_threshold", j5_alignment_threshold_)) {
    j5_alignment_threshold_ = 0.2;  // Default: ~11 degrees
    RCLCPP_INFO(n_->get_logger(), "Using default j5_alignment_threshold: %.3f rad", j5_alignment_threshold_);
  }
  
  // Movement detection threshold parameter for J4-J6 centering
  if (!n_->get_parameter("movement_detection_threshold_centering", movement_detection_threshold_centering_)) {
    movement_detection_threshold_centering_ = 1e-6;  // Default: very small threshold for any intentional movement
    RCLCPP_INFO(n_->get_logger(), "Using default movement_detection_threshold_centering: %.2e", movement_detection_threshold_centering_);
  }

  // Adaptive snap parameters for drift prevention
  if (!n_->get_parameter("enable_adaptive_snap", enable_adaptive_snap_)) {
    enable_adaptive_snap_ = true;
    RCLCPP_INFO(n_->get_logger(), "Using default enable_adaptive_snap: true");
  } else {
    RCLCPP_INFO(n_->get_logger(), "Loaded enable_adaptive_snap: %s",
                enable_adaptive_snap_ ? "true" : "false");
  }

  if (!n_->get_parameter("adaptive_snap_threshold_pos", adaptive_snap_threshold_pos_)) {
    adaptive_snap_threshold_pos_ = 0.01;
    RCLCPP_INFO(n_->get_logger(), "Using default adaptive_snap_threshold_pos: %.3f m/s",
                adaptive_snap_threshold_pos_);
  } else {
    RCLCPP_INFO(n_->get_logger(), "Loaded adaptive_snap_threshold_pos: %.3f m/s",
                adaptive_snap_threshold_pos_);
  }

  if (!n_->get_parameter("adaptive_snap_threshold_or", adaptive_snap_threshold_or_)) {
    adaptive_snap_threshold_or_ = 0.01;
    RCLCPP_INFO(n_->get_logger(), "Using default adaptive_snap_threshold_or: %.3f",
                adaptive_snap_threshold_or_);
  } else {
    RCLCPP_INFO(n_->get_logger(), "Loaded adaptive_snap_threshold_or: %.3f",
                adaptive_snap_threshold_or_);
  }

  if (!n_->get_parameter("adaptive_snap_cycles", adaptive_snap_cycles_required_)) {
    adaptive_snap_cycles_required_ = 50;  // Default 500ms @ 100Hz
    RCLCPP_INFO(n_->get_logger(), "Using default adaptive_snap_cycles: %d cycles",
                adaptive_snap_cycles_required_);
  } else {
    RCLCPP_INFO(n_->get_logger(), "Loaded adaptive_snap_cycles: %d cycles",
                adaptive_snap_cycles_required_);
  }

  if (!n_->get_parameter("gamma_suppression_cycles", gamma_suppression_cycles_)) {
    gamma_suppression_cycles_ = 100;  // Default 1000ms @ 100Hz - give robot time to settle
    RCLCPP_INFO(n_->get_logger(), "Using default gamma_suppression_cycles: %d cycles",
                gamma_suppression_cycles_);
  } else {
    RCLCPP_INFO(n_->get_logger(), "Loaded gamma_suppression_cycles: %d cycles",
                gamma_suppression_cycles_);
  }

  if (!n_->get_parameter("snap_input_threshold", snap_input_threshold_)) {
    snap_input_threshold_ = 1e-3;  // Default: 0.001 m/s or rad/s
    RCLCPP_INFO(n_->get_logger(), "Using default snap_input_threshold: %.2e",
                snap_input_threshold_);
  } else {
    RCLCPP_INFO(n_->get_logger(), "Loaded snap_input_threshold: %.2e",
                snap_input_threshold_);
  }

  // Initialize counters and flags
  adaptive_snap_counter_pos_ = 0;
  adaptive_snap_counter_or_ = 0;
  gamma_suppression_counter_pos_ = 0;
  gamma_suppression_counter_or_ = 0;
  adaptive_snap_triggered_pos_ = false;
  adaptive_snap_triggered_or_ = false;

  n_->get_parameter("alpha_weight", alpha_weight_vec);
  n_->get_parameter("beta_weight", beta_weight_vec);
  n_->get_parameter("gamma_weight", gamma_weight_vec);
  n_->get_parameter("joint_centering_weight", joint_centering_weight_vec);
  //n_->get_parameter("lambda_weight", lambda_weight_);
  //n_->get_parameter("q_natural", q_natural_);

  setAlphaWeight_(alpha_weight_vec, alpha_multiplier);
  setBetaWeight_(beta_weight_vec, beta_multiplier);
  setGammaWeight_(gamma_weight_vec, gamma_multiplier);
  setJointCenteringWeight_(joint_centering_weight_vec, joint_centering_multiplier);
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

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(n_);
  
  auto callback_alpha_weight = [this](const rclcpp::Parameter & p) {
    alpha_weight_vec = p.as_double_array();
    setAlphaWeight_(alpha_weight_vec, alpha_multiplier);
  };

  auto callback_alpha_multiplier = [this](const rclcpp::Parameter & p) {
    alpha_multiplier = p.as_int();
    setAlphaWeight_(alpha_weight_vec, alpha_multiplier);
  };

  auto callback_beta_weight = [this](const rclcpp::Parameter & p) {
    beta_weight_vec = p.as_double_array();
    setBetaWeight_(beta_weight_vec, beta_multiplier);
  };

  auto callback_beta_multiplier = [this](const rclcpp::Parameter & p) {
    beta_multiplier = p.as_int();
    setBetaWeight_(beta_weight_vec, beta_multiplier);
  };

  auto callback_gamma_weight = [this](const rclcpp::Parameter & p) {
    gamma_weight_vec = p.as_double_array();
    setGammaWeight_(gamma_weight_vec, gamma_multiplier);
  };

  auto callback_gamma_multiplier = [this](const rclcpp::Parameter & p) {
    gamma_multiplier = p.as_int();
    setGammaWeight_(gamma_weight_vec, gamma_multiplier);
  };

  auto callback_joint_centering_weight = [this](const rclcpp::Parameter & p) {
    joint_centering_weight_vec = p.as_double_array();
    setJointCenteringWeight_(joint_centering_weight_vec, joint_centering_multiplier);
  };

  auto callback_joint_centering_multiplier = [this](const rclcpp::Parameter & p) {
    joint_centering_multiplier = p.as_int();
    setJointCenteringWeight_(joint_centering_weight_vec, joint_centering_multiplier);
  };

  auto callback_position_control_frame = [this](const rclcpp::Parameter & p) {
    std::string frame_str = p.as_string();
    if (frame_str == "Tool") {
      position_ctrl_frame_ = ControlFrame::Tool;
    } else if (frame_str == "DrinkSmall") {
      position_ctrl_frame_ = ControlFrame::DrinkSmall;
    } else if (frame_str == "DrinkBig") {
      position_ctrl_frame_ = ControlFrame::DrinkBig;
    } else {
      position_ctrl_frame_ = ControlFrame::World;
    }
    RCLCPP_INFO(n_->get_logger(), "Position control frame changed to: %s", frame_str.c_str());
  };

  auto callback_orientation_control_frame = [this](const rclcpp::Parameter & p) {
    std::string frame_str = p.as_string();
    if (frame_str == "Tool") {
      orientation_ctrl_frame_ = ControlFrame::Tool;
    } else if (frame_str == "DrinkSmall") {
      orientation_ctrl_frame_ = ControlFrame::DrinkSmall;
    } else if (frame_str == "DrinkBig") {
      orientation_ctrl_frame_ = ControlFrame::DrinkBig;
    } else {
      orientation_ctrl_frame_ = ControlFrame::World;
    }
    RCLCPP_INFO(n_->get_logger(), "Orientation control frame changed to: %s", frame_str.c_str());
  };

  auto callback_j2_limits = [this](const rclcpp::Parameter & p) {
    q_upper_limit_[1] = p.as_double();
    RCLCPP_INFO(n_->get_logger(), "J2 upper limit changed to: %.3f", q_upper_limit_[1]);
  };

  auto callback_j3_limits = [this](const rclcpp::Parameter & p) {
    q_upper_limit_[2] = p.as_double();
    RCLCPP_INFO(n_->get_logger(), "J3 upper limit changed to: %.3f", q_upper_limit_[2]);
  };

  cb_handle_alpha_weight = param_subscriber_->add_parameter_callback("alpha_weight", callback_alpha_weight);
  cb_handle_alpha_multiplier = param_subscriber_->add_parameter_callback("alpha_multiplier", callback_alpha_multiplier);
  cb_handle_beta_weight = param_subscriber_->add_parameter_callback("beta_weight", callback_beta_weight);
  cb_handle_beta_multiplier = param_subscriber_->add_parameter_callback("beta_multiplier", callback_beta_multiplier);
  cb_handle_gamma_weight = param_subscriber_->add_parameter_callback("gamma_weight", callback_gamma_weight);
  cb_handle_gamma_multiplier = param_subscriber_->add_parameter_callback("gamma_multiplier", callback_gamma_multiplier);
  cb_handle_joint_centering_weight = param_subscriber_->add_parameter_callback("joint_centering_weight", callback_joint_centering_weight);
  cb_handle_joint_centering_multiplier = param_subscriber_->add_parameter_callback("joint_centering_multiplier", callback_joint_centering_multiplier);
  cb_handle_position_control_frame = param_subscriber_->add_parameter_callback("position_control_frame", callback_position_control_frame);
  cb_handle_orientation_control_frame = param_subscriber_->add_parameter_callback("orientation_control_frame", callback_orientation_control_frame);
  cb_handle_j2_limits = param_subscriber_->add_parameter_callback("j2.max", callback_j2_limits);
  cb_handle_j3_limits = param_subscriber_->add_parameter_callback("j3.max", callback_j3_limits);
}

void InverseKinematic::init(const std::string end_effector_link, const double sampling_period)
{
  end_effector_link_ = end_effector_link;
  sampling_period_ = sampling_period;
}

void InverseKinematic::setAlphaWeight_(const std::vector<double>& alpha_weight, const int alpha_multiplier)
{
  // Minimize cartesian velocity (dx) weight
  alpha_weight_ = MatrixXd::Identity(space_dimension_, space_dimension_);
  for (int i = 0; i < space_dimension_; i++)
  {
    alpha_weight_(i, i) = alpha_multiplier * alpha_weight[i];
  }

  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set alpha weight to : \n" << alpha_weight_ << "\n");
}

void InverseKinematic::setBetaWeight_(const std::vector<double>& beta_weight, const int beta_multiplier)
{
  // Minimize joint velocity (dq) weight
  beta_weight_ = MatrixXd::Identity(joint_number_, joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    beta_weight_(i, i) = beta_multiplier * beta_weight[i];
  }
  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set beta weight to : \n" << beta_weight_ << "\n");
}

void InverseKinematic::setGammaWeight_(const std::vector<double>& gamma_weight, const int gamma_multiplier)
{
  // Minimize cartesian position drift on non-driven axes
  gamma_pos_weight_ = Matrix3d::Identity();
  gamma_or_weight_ = Matrix4d::Identity();
  for (int i = 0; i < 3; i++)
  {
    gamma_pos_weight_(i, i) = gamma_multiplier * gamma_weight[i];
  }
  for (int i = 0; i < 4; i++)
  {
    gamma_or_weight_(i, i) = gamma_multiplier * gamma_weight[i+3];
  }
  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set gamma position weight to : \n" << gamma_pos_weight_ << "\n");
  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set gamma orientation weight to : \n" << gamma_or_weight_ << "\n");
}

void InverseKinematic::setJointCenteringWeight_(const std::vector<double>& joint_centering_weight, const int joint_centering_multiplier)
{
  // Joint centering weight to keep redundant joints near zero
  joint_centering_weight_ = MatrixXd::Identity(joint_number_, joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    joint_centering_weight_(i, i) = joint_centering_multiplier * joint_centering_weight[i];
  }
  RCLCPP_DEBUG_STREAM(n_->get_logger(),"Set joint centering weight to : \n" << joint_centering_weight_ << "\n");
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
  
  /* Reset adaptive snap state */
  adaptive_snap_counter_pos_ = 0;
  adaptive_snap_counter_or_ = 0;
  gamma_suppression_counter_pos_ = 0;
  gamma_suppression_counter_or_ = 0;
  adaptive_snap_triggered_pos_ = false;
  adaptive_snap_triggered_or_ = false;
}

void InverseKinematic::resolveInverseKinematic(JointVelocity& dq_computed,
                        const SpaceVelocity& dx_desired, const SpacePosition& x_desired,
                        bool path_tracking_mode, bool wheelchair)
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
  else if (position_ctrl_frame_ == ControlFrame::DrinkSmall)
  {
    R_0to1 = x_current_.getOrientation().toRotationMatrix();
    R_0to1_transpose = R_0to1.transpose();
  }
  else if (position_ctrl_frame_ == ControlFrame::DrinkBig)
  {
    R_0to1 = x_current_.getOrientation().toRotationMatrix();
    R_0to1_transpose = R_0to1.transpose();
  }
  else
  {
    RCLCPP_ERROR(n_->get_logger(), "This control frame is not already handle !");
  }
  SpaceVelocity dx_desired_in_frame;
  computeVelocityInFrame_(dx_desired_in_frame, dx_desired, R_0to1);

  /*********************************************************/

  /* Set kinematic state of the robot to the current joint positions */
  kinematic_state_->setVariablePositions(q_current_);
  kinematic_state_->updateLinkTransforms();

  /* Get jacobian for the appropriate control frame */
  Vector3d reference_point_position(0.0, 0.0, 0.0);
  MatrixXd jacobian;
  
  // Determine which frame to use for Jacobian computation based on control frame
  std::string jacobian_frame = end_effector_link_;  // Default to tool0
  if (orientation_ctrl_frame_ == ControlFrame::DrinkSmall || position_ctrl_frame_ == ControlFrame::DrinkSmall) {
    jacobian_frame = "drink_small";
  } else if (orientation_ctrl_frame_ == ControlFrame::DrinkBig || position_ctrl_frame_ == ControlFrame::DrinkBig) {
    jacobian_frame = "drink_big";
  }
  
  // Check if the selected frame exists, fallback to tool0 if not
  try {
    getJacobian_(kinematic_state_, joint_model_group_, kinematic_state_->getLinkModel(jacobian_frame),
                 reference_point_position, jacobian, true);
    
    if (jacobian_frame != end_effector_link_) {
      RCLCPP_DEBUG_THROTTLE(n_->get_logger(), *n_->get_clock(), 10000, 
                            "Using Jacobian computed for frame: %s", jacobian_frame.c_str());
    }
  } catch (const std::exception& e) {
    // Fallback to tool0 if drink frame not found
    RCLCPP_WARN_THROTTLE(n_->get_logger(), *n_->get_clock(), 10000, 
                         "Frame '%s' not found, falling back to %s: %s", 
                         jacobian_frame.c_str(), end_effector_link_.c_str(), e.what());
    getJacobian_(kinematic_state_, joint_model_group_, kinematic_state_->getLinkModel(end_effector_link_),
                 reference_point_position, jacobian, true);
  }
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

  // Threshold for detecting user input (accounts for smoothing decay)
  // Uses configurable snap_input_threshold_ parameter
  //
  // KEY FIX FOR SMOOTHING: We CONTINUOUSLY update the snap position while input is above threshold.
  // This ensures that when input finally drops below threshold (due to smoothing decay),
  // the snap is already at the robot's current position - eliminating the jump-back.

  for (int i = 0; i < 3; i++)
  {
   if (std::abs(dx_desired[i]) > snap_input_threshold_)
   {
     flag_pos_save[i] = true;
     // CONTINUOUSLY update snap while user is commanding - this is the key fix!
     // As the smoothed input decays, we keep the snap at the latest position.
     pos_snap_ = x_current_.getPosition();
   }
   else
   {
     if (flag_pos_save[i] == true)
     {
       flag_pos_save[i] = false;
       // Final snap update when input drops below threshold
       // (Should be very close to current position due to continuous updates above)
       pos_snap_ = x_current_.getPosition();
       
       // Reset adaptive snap counter when user stops commanding
       adaptive_snap_counter_pos_ = 0;
       
       // If adaptive snap was triggered during this forcing period, activate gamma suppression NOW
       // This prevents the robot from drifting back when joystick is released
       if (adaptive_snap_triggered_pos_) {
         gamma_suppression_counter_pos_ = gamma_suppression_cycles_;
         // Also suppress orientation since they're in conflict
         gamma_suppression_counter_or_ = gamma_suppression_cycles_;
         r_snap_ = x_current_.getOrientation();  // Update orientation snap too
         adaptive_snap_triggered_pos_ = false;  // Reset flag
         adaptive_snap_triggered_or_ = false;   // Reset flag
         RCLCPP_WARN(n_->get_logger(),
                     "Joystick released after adaptive snap - updating pos AND orientation snap, activating BOTH gamma suppressions for %d cycles",
                     gamma_suppression_cycles_);
       }
     }
   }
  }
  for (int i = 0; i < 3; i++)
  {
    if (std::abs(dx_desired[4 + i]) > snap_input_threshold_)
    {
      flag_orient_save[i] = true;
      // CONTINUOUSLY update snap while user is commanding - this is the key fix!
      // As the smoothed input decays, we keep the snap at the latest orientation.
      r_snap_ = x_current_.getOrientation();
    }
    else
    {
      if (flag_orient_save[i] == true)
      {
        flag_orient_save[i] = false;
        // Final snap update when input drops below threshold
        // (Should be very close to current orientation due to continuous updates above)
        r_snap_ = x_current_.getOrientation();
        
        // Reset adaptive snap counter when user stops commanding
        adaptive_snap_counter_or_ = 0;
        
        // If adaptive snap was triggered during this forcing period, activate gamma suppression NOW
        // This prevents the robot from drifting back when joystick is released
        if (adaptive_snap_triggered_or_) {
          gamma_suppression_counter_or_ = gamma_suppression_cycles_;
          // Also suppress position since they're in conflict
          gamma_suppression_counter_pos_ = gamma_suppression_cycles_;
          pos_snap_ = x_current_.getPosition();  // Update position snap too
          adaptive_snap_triggered_or_ = false;   // Reset flag
          adaptive_snap_triggered_pos_ = false;  // Reset flag
          RCLCPP_WARN(n_->get_logger(),
                      "Joystick released after adaptive snap - updating pos AND orientation snap, activating BOTH gamma suppressions for %d cycles",
                      gamma_suppression_cycles_);
        }
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
  computeConstraints_(A, lbA, ubA, jacobian, wheelchair);

  // for(int i=4; i< 6; i++){
  //           RCLCPP_DEBUG_STREAM(n_->get_logger(), "lba = " << lbA[i]);
  //       }
  //  for(int i=4; i< 6; i++){
  //           RCLCPP_DEBUG_STREAM(n_->get_logger(), "uba = " << ubA[i]);
  //       }
  //RCLCPP_DEBUG_STREAM(n_->get_logger(), "x_snap = " << x_snap);
  // checkConstraintsDebug_(ubA, lbA, A);

  /* Solve QP */
  qpOASES::real_t xOpt[6];
  qpOASES::int_t nWSR = 100;  // Increased from 10 to 100 to allow more working set recalculations
  qpOASES::returnValue qp_return = qpOASES::SUCCESSFUL_RETURN;
  if (qp_init_required_)
  {
    /* Initialize QP solver */
    QP_ = new qpOASES::SQProblem(joint_number_, joint_number_);// + 6);
    qpOASES::Options options;
    //Options options;
    options.setToReliable();
    // Enable inertia correction to improve numerical stability
    options.enableInertiaCorrection = qpOASES::BT_TRUE;
    // Enable regularisation to handle ill-conditioned problems
    options.enableRegularisation = qpOASES::BT_TRUE;
    // Set termination tolerance for better convergence
    options.terminationTolerance = 1e-8;
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
    //RCLCPP_DEBUG_STREAM(n_->get_logger(),"qpOASES : succesfully return");

    /* Get solution of the QP */
    QP_->getPrimalSolution(xOpt);
    for (int i=0; i<joint_number_; i++){
      dq_computed[i] = xOpt[i];
      prev_dq_computed_[i] = dq_computed[i];
    }
  }
  else if (qp_return == qpOASES::RET_MAX_NWSR_REACHED)
  {
    RCLCPP_WARN_THROTTLE(n_->get_logger(), *n_->get_clock(), 1000, 
                         "qpOASES : Maximum number of working set recalculations reached, using best solution found");
    
    /* Get the best solution found even if not fully converged */
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
    
    /* Use previous solution or zero velocity as fallback */
    for (int i=0; i<joint_number_; i++){
      dq_computed[i] = prev_dq_computed_[i] * 0.5; // Reduce previous velocity by half as conservative fallback
    }
  }

  /* Only reset on serious errors, not on max iterations reached */
  if (qp_return != qpOASES::SUCCESSFUL_RETURN &&
      qp_return != qpOASES::RET_MAX_NWSR_REACHED &&
      qp_return != qpOASES::RET_HOTSTART_STOPPED_INFEASIBILITY)
  {
    RCLCPP_WARN(n_->get_logger(), "QP solver failed with serious error, resetting solver state");
    reset();
    // Don't exit, just reset and continue - this allows recovery
    // exit(0);  // TODO improve error handling. Crash of the application is neither safe nor beautiful
  }

  /* Adaptive snap: Detect when QP cannot achieve commanded velocity and update snap positions */
  if (enable_adaptive_snap_) {
    // Compute achieved Cartesian velocity from joint velocities
    VectorXd dq_computed_vector(joint_number_);
    for (int i = 0; i < joint_number_; i++) {
      dq_computed_vector(i) = dq_computed[i];
    }
    VectorXd dx_achieved = jacobian * dq_computed_vector;

    // Split into position and orientation components
    Vector3d dx_pos_achieved = dx_achieved.head<3>();
    Vector3d dx_pos_desired = dx_desired_in_frame.getPosition();
    double pos_error = (dx_pos_achieved - dx_pos_desired).norm();

    Vector4d dx_or_achieved = dx_achieved.tail<4>();
    // Convert orientation part of dx_des (angular velocity in quaternion form)
    VectorXd dx_des_raw = dx_desired_in_frame.getRawVector();
    Vector4d dx_or_desired = dx_des_raw.tail<4>();
    double or_error = (dx_or_achieved - dx_or_desired).norm();

    // Detect sacrifice conditions (sustained over multiple cycles)
    // CRITICAL: Only trigger when user is ACTIVELY commanding (dx > threshold)
    // If dx_desired ≈ 0, any movement is from gamma term drift, not user input
    bool user_commanding_pos = (dx_pos_desired.norm() > 1e-3);  // 1mm/s threshold for "user is commanding"
    bool user_commanding_or = (dx_or_desired.norm() > 1e-3);

    bool pos_sacrifice = (pos_error > adaptive_snap_threshold_pos_ && user_commanding_pos);
    bool or_sacrifice = (or_error > adaptive_snap_threshold_or_ && user_commanding_or);

    // DIAGNOSTIC: ALWAYS log to understand what's happening
    RCLCPP_WARN_THROTTLE(n_->get_logger(), *n_->get_clock(), 500,
                         "ADAPTIVE SNAP DEBUG: dx_pos_cmd_norm=%.6f, dx_or_cmd_norm=%.6f, "
                         "pos_error=%.4f, or_error=%.4f, user_cmd_pos=%d, user_cmd_or=%d, "
                         "pos_sacrifice=%d, or_sacrifice=%d",
                         dx_pos_desired.norm(), dx_or_desired.norm(),
                         pos_error, or_error,
                         user_commanding_pos, user_commanding_or,
                         pos_sacrifice, or_sacrifice);

    // Increment counters for sustained detection
    if (pos_sacrifice) {
      adaptive_snap_counter_pos_++;
      RCLCPP_INFO_THROTTLE(n_->get_logger(), *n_->get_clock(), 500,
                           "Position sacrifice detected: counter=%d/%d, error=%.4f > threshold=%.4f",
                           adaptive_snap_counter_pos_, adaptive_snap_cycles_required_,
                           pos_error, adaptive_snap_threshold_pos_);
    } else {
      adaptive_snap_counter_pos_ = 0;
    }

    if (or_sacrifice) {
      adaptive_snap_counter_or_++;
      RCLCPP_INFO_THROTTLE(n_->get_logger(), *n_->get_clock(), 500,
                           "Orientation sacrifice detected: counter=%d/%d, error=%.4f > threshold=%.4f",
                           adaptive_snap_counter_or_, adaptive_snap_cycles_required_,
                           or_error, adaptive_snap_threshold_or_);
    } else {
      adaptive_snap_counter_or_ = 0;
    }

    // Update snap positions only after sustained adaptive snap
    if (adaptive_snap_counter_pos_ >= adaptive_snap_cycles_required_) {
      // Position is being sacrificed - mark that adaptive snap occurred
      // We do NOT immediately update pos_snap_ here because:
      // 1. The user is still forcing, so position will keep changing
      // 2. We want to capture the FINAL position when joystick is released
      // Instead, set flag so snap is updated to current position on release
      adaptive_snap_triggered_pos_ = true;
      
      // ALSO trigger orientation snap since pos/or are in conflict
      // This prevents orientation from pulling the robot back after release
      adaptive_snap_triggered_or_ = true;
      
      // Reset counter to prevent continuous re-triggering
      adaptive_snap_counter_pos_ = 0;

      RCLCPP_WARN(n_->get_logger(),
                  "ADAPTIVE SNAP TRIGGERED: Position sacrifice sustained for %d cycles - will update pos AND orientation snap on joystick release",
                  adaptive_snap_cycles_required_);
    }

    if (adaptive_snap_counter_or_ >= adaptive_snap_cycles_required_) {
      // Orientation is being sacrificed - mark that adaptive snap occurred
      // Same logic as position: wait for joystick release to capture final orientation
      adaptive_snap_triggered_or_ = true;
      
      // ALSO trigger position snap since pos/or are in conflict
      // This prevents position from pulling the robot back after release
      adaptive_snap_triggered_pos_ = true;
      
      // Reset counter to prevent continuous re-triggering
      adaptive_snap_counter_or_ = 0;

      RCLCPP_WARN(n_->get_logger(),
                  "ADAPTIVE SNAP TRIGGERED: Orientation sacrifice sustained for %d cycles - will update pos AND orientation snap on joystick release",
                  adaptive_snap_cycles_required_);
    }
  }
}



void InverseKinematic::computeVelocityInFrame_(SpaceVelocity& dx_desired_in_frame, const SpaceVelocity& dx_desired, const Matrix3d& R_0to1)
{
  /* Compute the desired linear velocity according to the selected control frame :
   *       p0 = R_0to1 * p1
   * with :
   *     - p0 : the linear velocity in world frame
   *     - p1 : the linear velocity in tool frame
   *     - R_0to1 : the rotation matrix FROM tool TO world frame (tool->world transformation)
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
  else if (orientation_ctrl_frame_ == ControlFrame::DrinkSmall)
  {
    r_dot = x_current_.getOrientation() * r_half_omega;
  }
  else if (orientation_ctrl_frame_ == ControlFrame::DrinkBig)
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
    if( std::abs(dx_des[i]) > snap_input_threshold_ && !path_tracking )
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

  // Gamma suppression after adaptive snap: disable gamma term to prevent drift
  // This allows robot to settle at the new snap position without being pulled by residual errors
  // CRITICAL: Only decrement counter when user is NOT commanding (dx_des ≈ 0)
  // This ensures suppression happens AFTER joystick release, not during forcing
  if (enable_adaptive_snap_) {
    double dx_norm = dx_des.getRawVector().norm();
    bool user_commanding = dx_norm > 1e-6;

    if (gamma_suppression_counter_pos_ > 0) {
      if (!user_commanding) {
        // Suppress position gamma term (only when not commanding)
        pos_controlled_mat.setZero();
        gamma_suppression_counter_pos_--;

        RCLCPP_INFO_THROTTLE(n_->get_logger(), *n_->get_clock(), 200,
                             "GAMMA SUPPRESSION ACTIVE: Position gamma suppressed, countdown=%d, dx_norm=%.6f",
                             gamma_suppression_counter_pos_, dx_norm);

        if (gamma_suppression_counter_pos_ == 0) {
          // CRITICAL: Update snap to current position AFTER suppression ends
          // This ensures the robot stays at its settled position
          pos_snap_ = x_current_.getPosition();
          RCLCPP_WARN(n_->get_logger(), "Position gamma suppression ended - snap updated to current position, resuming normal control");
        }
      } else {
        RCLCPP_WARN_THROTTLE(n_->get_logger(), *n_->get_clock(), 500,
                             "Gamma suppression pending (user still commanding): counter=%d, dx_norm=%.6f",
                             gamma_suppression_counter_pos_, dx_norm);
      }
    }

    if (gamma_suppression_counter_or_ > 0) {
      if (!user_commanding) {
        // Suppress orientation gamma term (only when not commanding)
        or_controlled_mat.setZero();
        gamma_suppression_counter_or_--;

        if (gamma_suppression_counter_or_ == 0) {
          // CRITICAL: Update snap to current orientation AFTER suppression ends
          // This ensures the robot stays at its settled orientation
          r_snap_ = x_current_.getOrientation();
          RCLCPP_INFO(n_->get_logger(), "Orientation gamma suppression ended - snap updated to current orientation, resuming normal control");
        }
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
  else if (orientation_ctrl_frame_ == ControlFrame::DrinkSmall)
  {
    Rdes_conj = xR_(x_des_quat) * CONJ_MAT;
  }
  else if (orientation_ctrl_frame_ == ControlFrame::DrinkBig)
  {
    Rdes_conj = xR_(x_des_quat) * CONJ_MAT;
  }
  else
  {
    // Default to world frame if unknown
    Rdes_conj = Rx_(x_des_quat) * CONJ_MAT;
  }

  // Conditional joint centering: only active when J5 is near zero (J4 and J6 aligned) AND robot is being moved
  // Strategy: When J5≈0, J4 and J6 act as a single rotation. We want to:
  // 1. Maintain J4+J6 (preserve end-effector orientation) 
  // 2. Minimize |J4-J6| (center both joints toward equal values)
  // 3. Only apply when robot is actively being controlled to avoid unwanted movement
  
  // Check if robot is being actively controlled by looking at desired velocity magnitude
  double velocity_magnitude = dx_des.getRawVector().norm();
  bool robot_is_moving = velocity_magnitude > movement_detection_threshold_centering_;
  
  double j5_angle = std::abs(q_current_[4]);  // Joint 5 (index 4, 0-based)
  double centering_scale = 0.0;
  
  if (j5_angle < j5_alignment_threshold_ && robot_is_moving) {
    // Linear scaling: full centering when perfectly aligned, reduces as J5 moves away
    // Only active when robot is being moved to prevent unwanted motion during rest
    centering_scale = 1.0 - (j5_angle / j5_alignment_threshold_);
  }
  
  // Create smart centering matrix that only penalizes J4-J6 difference, not their sum
  MatrixXd smart_centering_weight = MatrixXd::Zero(joint_number_, joint_number_);
  if (centering_scale > 0.0) {
    // Copy base joint centering weights
    smart_centering_weight = joint_centering_weight_ * centering_scale;
    
    // For J4 and J6 (indices 3 and 5): modify to preserve J4+J6 while minimizing J4-J6
    // Instead of penalizing J4 and J6 individually, we'll use a different approach in the gradient
    // The hessian can stay the same but we'll modify the gradient to be smarter
  }
  
  // Log centering activity for debugging
  if (centering_scale > 0.1) {  // Only log when significantly active
    double j4_plus_j6 = q_current_[3] + q_current_[5];
    double j4_minus_j6 = q_current_[3] - q_current_[5];
    /*RCLCPP_DEBUG_THROTTLE(n_->get_logger(), *n_->get_clock(), 2000,
                          "Smart centering ACTIVE: J5=%.3f, scale=%.3f, vel_mag=%.6f, J4=%.3f, J6=%.3f, sum=%.3f, diff=%.3f", 
                          q_current_[4], centering_scale, velocity_magnitude, q_current_[3], q_current_[5], j4_plus_j6, j4_minus_j6);*/
  }
  
  hessian = jacobian.transpose() * alpha_weight_ * dx_controlled_mat * jacobian
          + jacob_pos.transpose() * gamma_pos_weight_ * pos_controlled_mat * jacob_pos * sampling_period_ * sampling_period_
          + jacob_or.transpose() * Rdes_conj.transpose() * gamma_or_weight_ * or_controlled_mat * Rdes_conj * jacob_or * sampling_period_ * sampling_period_
          + beta_weight_
          + smart_centering_weight * sampling_period_ * sampling_period_;  // Smart joint centering
  //hessian = (jacobian.transpose() * (alpha_weight_ + gamma_weight_ * ctrl_axes_mat) * jacobian)
  //          + beta_weight_
  //          + lambda_weight_ * ;

  // Convert current joint positions to Eigen vector for matrix operations
  VectorXd q_current_eigen = VectorXd::Zero(joint_number_);
  for (int i = 0; i < joint_number_; i++)
  {
    q_current_eigen(i) = q_current_[i];
  }

  // Smart centering gradient: preserve J4+J6, minimize |J4-J6|
  VectorXd smart_centering_gradient = VectorXd::Zero(joint_number_);
  if (centering_scale > 0.0) {
    // For joints other than J4 and J6, use normal centering (drive toward zero)
    for (int i = 0; i < joint_number_; i++) {
      if (i != 3 && i != 5) {  // Not J4 or J6
        smart_centering_gradient(i) = joint_centering_weight_(i,i) * q_current_eigen(i) * centering_scale;
      }
    }
    
    // For J4 and J6: only penalize their difference, not their sum
    // Goal: minimize (J4-J6)^2 while allowing (J4+J6) to vary freely
    // Gradient of (J4-J6)^2 w.r.t J4 = 2*(J4-J6)
    // Gradient of (J4-J6)^2 w.r.t J6 = -2*(J4-J6)
    double j4_minus_j6 = q_current_[3] - q_current_[5];
    double centering_strength = joint_centering_weight_(3,3) * centering_scale;  // Use J4's weight
    
    smart_centering_gradient(3) = centering_strength * j4_minus_j6;      // Push J4 toward J6
    smart_centering_gradient(5) = -centering_strength * j4_minus_j6;     // Push J6 toward J4
  }

  g = -jacobian.transpose() * alpha_weight_ * dx_controlled_mat * dx_des.getRawVector()
      + jacob_pos.transpose() * gamma_pos_weight_ * pos_controlled_mat * (x_current_.getPosition() - x_des.getPosition()) * sampling_period_
      + jacob_or.transpose() * Rdes_conj.transpose() * gamma_or_weight_ * or_controlled_mat * Rdes_conj * x_current_.getOrientation().toVector() * sampling_period_
      + smart_centering_gradient * sampling_period_;  // Smart centering gradient
  //    + jacobian.transpose() * gamma_weight_ * ctrl_axes_mat * sampling_period_ * delta_X_current_snap.getRawVector()
  //    + lambda_weight_ * sampling_period_ * q_natural_.getRawVector();
}

void InverseKinematic::computeConstraints_(MatrixXdRowMaj& A, std::vector<double>& lbA,
                                          std::vector<double>& ubA, const MatrixXd& jacobian, bool wheelchair)
{
  A.resize(joint_number_, joint_number_);

  A.topLeftCorner(joint_number_, joint_number_) = MatrixXd::Identity(joint_number_, joint_number_) * sampling_period_;

  for (int i=0; i < joint_number_; i++){
    if (q_has_limit_[i] == 1){
      if (wheelchair){
        lbA[i] = (q_lower_limit_[i] - q_current_[i+8]);
        ubA[i] = (q_upper_limit_[i] - q_current_[i+8]);
      }
      else{
        lbA[i] = (q_lower_limit_[i] - q_current_[i]);
        ubA[i] = (q_upper_limit_[i] - q_current_[i]);
      }

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
    /* Use rotation matrix directly - no quaternion discontinuities! */
    Eigen::Matrix3d rotation_matrix = link_transform.linear();
    
    /* Convert to quaternion only for Jacobian computation */
    Quaterniond conv_quat(rotation_matrix);
    
    /* Apply quaternion continuity correction only if needed for external interfaces */
    if (jacobian_init_flag_)
    {
      jacobian_init_flag_ = false;
      jacobian_quat_prev_ = conv_quat;
    }
    else
    {
      /* Ensure quaternion continuity using dot product (more robust than euclidean distance) */
      double dot_product = conv_quat.w() * jacobian_quat_prev_.w() + 
                          conv_quat.x() * jacobian_quat_prev_.x() + 
                          conv_quat.y() * jacobian_quat_prev_.y() + 
                          conv_quat.z() * jacobian_quat_prev_.z();
      
      if (dot_product < 0.0) {
        // Flip quaternion for continuity
        conv_quat.w() = -conv_quat.w();
        conv_quat.x() = -conv_quat.x();
        conv_quat.y() = -conv_quat.y();
        conv_quat.z() = -conv_quat.z();
      }
      jacobian_quat_prev_ = conv_quat;
    }

    /* d/dt ( [w] ) = 1/2 * [ -x -y -z ]  * [ omega_1 ]
     *        [x]           [  w  z -y ]    [ omega_2 ]
     *        [y]           [ -z  w  x ]    [ omega_3 ]
     *        [z]           [  y -x  w ]
     */

    /* Compute quaternion Jacobian from angular velocity Jacobian */
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
