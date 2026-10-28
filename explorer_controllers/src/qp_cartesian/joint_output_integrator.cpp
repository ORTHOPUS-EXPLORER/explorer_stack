#include "explorer_controllers/qp_cartesian/joint_output_integrator.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cstdlib>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <stdexcept>

namespace space_control
{
namespace
{
// Other URDF joints (gripper...) are locked out of the model
const std::vector<std::string> kControlledPointControlledJointNames = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

std::string controlled_point_run_xacro(const std::string& xacro_path)
{
  const std::string output_path = "/tmp/explorer_joint_output_integrator_controlled_point.urdf";
  const std::string command =
    std::string("xacro ") + xacro_path + " use_POC2:=true" + " > " + output_path;

  const int status = std::system(command.c_str());
  if (status != 0)
  {
    throw std::runtime_error("Failed to process Xacro file via 'xacro': " + xacro_path);
  }
  return output_path;
}

std::string controlled_point_resolve_urdf_path(const std::string& input_path)
{
  if (input_path.empty())
  {
    throw std::runtime_error("No input path supplied.");
  }

  const std::string extension = input_path.substr(input_path.find_last_of('.') + 1);
  if (extension == "xacro")
  {
    return controlled_point_run_xacro(input_path);
  }
  return input_path;
}
}  // namespace

JointOutputIntegrator::JointOutputIntegrator(rclcpp::Node::SharedPtr n)
: n_(n), sampling_period_(0.02), init(false), q_lower_limit_(7), q_upper_limit_(7), q_has_limit_(6)
{
  auto debug_enabled = n_->get_parameter("debug").as_bool();
  if (debug_enabled)
  {
    if (
      rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) !=
      RCUTILS_RET_OK)
    {
      throw std::runtime_error("Couldn't set logger level to DEBUG.");
    }
  }

  // init node parameters
  auto controller_position_topic_name =
    n_->get_parameter("controller_position_topic_name").as_string();
  if (controller_position_topic_name.empty())
  {
    throw std::runtime_error("Parameter 'controller_position_topic_name' is required");
  }
  auto controller_gripper_position_topic_name =
    n_->get_parameter("controller_gripper_position_topic_name").as_string();
  if (controller_gripper_position_topic_name.empty())
  {
    throw std::runtime_error("Parameter 'controller_gripper_position_topic_name' is required");
  }

  // Params may already be declared from overrides (see main())
  if (!n_->has_parameter("position_error_saturation_enable"))
  {
    n_->declare_parameter<bool>("position_error_saturation_enable", false);
  }
  position_error_saturation_enable_ =
    n_->get_parameter("position_error_saturation_enable").as_bool();

  // Default threshold: 15 degrees, expressed in rad since joint values are in rad.
  if (!n_->has_parameter("position_error_saturation_threshold"))
  {
    n_->declare_parameter<double>("position_error_saturation_threshold", 15.0 * M_PI / 180.0);
  }
  position_error_saturation_threshold_ =
    n_->get_parameter("position_error_saturation_threshold").as_double();

  // Persistent mode: off by default (i.e. "spring back" to the original setpoint once the
  // error goes back under the threshold).
  if (!n_->has_parameter("position_error_saturation_persistent"))
  {
    n_->declare_parameter<bool>("position_error_saturation_persistent", false);
  }
  position_error_saturation_persistent_ =
    n_->get_parameter("position_error_saturation_persistent").as_bool();

  // "Controlled point" marker: the cartesian tool0 position corresponding to the position
  // command this node publishes (see publish_controlled_point_marker_()).
  if (!n_->has_parameter("controlled_point_urdf_path"))
  {
    n_->declare_parameter<std::string>(
      "controlled_point_urdf_path",
      "/root/explorer_ws/explorer_stack/explorer_description/urdf/explorer.urdf.xacro");
  }
  controlled_point_urdf_path_ = n_->get_parameter("controlled_point_urdf_path").as_string();

  if (!n_->has_parameter("controlled_point_end_effector_frame"))
  {
    n_->declare_parameter<std::string>("controlled_point_end_effector_frame", "tool0");
  }
  controlled_point_end_effector_frame_ =
    n_->get_parameter("controlled_point_end_effector_frame").as_string();

  // Must match the URDF root link
  if (!n_->has_parameter("controlled_point_marker_frame_id"))
  {
    n_->declare_parameter<std::string>("controlled_point_marker_frame_id", "world");
  }
  controlled_point_marker_frame_id_ =
    n_->get_parameter("controlled_point_marker_frame_id").as_string();

  std::string controlled_point_marker_topic;
  if (!n_->has_parameter("controlled_point_marker_topic"))
  {
    n_->declare_parameter<std::string>(
      "controlled_point_marker_topic",
      "/explorer_controllers/joint_output_integrator/controlled_point_marker");
  }
  controlled_point_marker_topic = n_->get_parameter("controlled_point_marker_topic").as_string();
  controlled_point_marker_pub_ =
    n_->create_publisher<visualization_msgs::msg::Marker>(controlled_point_marker_topic, 10);

  controlled_point_kinematics_ready_ = false;
  controlled_point_kinematics_load_attempted_ = false;

  // Allow all three position_error_saturation_* params above to be changed live without restarting the node.
  on_set_parameters_callback_handle_ = n_->add_on_set_parameters_callback(
    std::bind(&JointOutputIntegrator::on_parameter_change_, this, std::placeholders::_1));

  //init settings
  dq_output_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  gripper_dq_output_.data = {0.0};
  q_command_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  gripper_q_command_.data = {0.0};
  q_measured_.fill(0.0);

  joint_name = {"joint_1", "joint_2", "joint_3",           "joint_4",
                "joint_5", "joint_6", "right_finger_joint"};

  for (int i = 0; i < 6; i++)
  {
    n_->get_parameter("j" + std::to_string(i + 1) + ".limits", q_has_limit_[i]);
    if (q_has_limit_[i] == 1)
    {
      n_->get_parameter("j" + std::to_string(i + 1) + ".min", q_lower_limit_[i]);
      n_->get_parameter("j" + std::to_string(i + 1) + ".max", q_upper_limit_[i]);
    }
    else
    {
      q_lower_limit_[i] = 0;
      q_upper_limit_[i] = 0;
    }
    RCLCPP_DEBUG_STREAM(
      n_->get_logger(),
      "J" << i + 1 << " - min:" << q_lower_limit_[i] << " max:" << q_upper_limit_[i]);
  }
  q_lower_limit_[6] = 0.0;
  q_upper_limit_[6] = 1.0;

  //init suscriber
  current_pos_sub_ = n_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&JointOutputIntegrator::callback_current_pos_, this, std::placeholders::_1));
  dq_output_sub_ = n_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/explorer_controllers/qp_solving/dq_output", 10,
    std::bind(&JointOutputIntegrator::callback_dq_output_, this, std::placeholders::_1));
  gripper_dq_output_sub_ = n_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/explorer_user_interfaces/rqt_armcontrol/input_gripper_velocity", 10,
    std::bind(&JointOutputIntegrator::callback_gripper_dq_output_, this, std::placeholders::_1));

  //init publisher
  joints_command_pub_ =
    n_->create_publisher<std_msgs::msg::Float64MultiArray>(controller_position_topic_name, 10);
  gripper_command_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>(
    controller_gripper_position_topic_name, 10);

  timer_ = n_->create_wall_timer(20ms, std::bind(&JointOutputIntegrator::timer_callback, this));
}

void JointOutputIntegrator::callback_current_pos_(const sensor_msgs::msg::JointState& msg)
{
  // Once init, only keep track of the measured position (used by the position error saturation)
  if (init)
  {
    for (int i = 0; i < 7; i++)
    {
      q_measured_[i] = msg.position[joint_order_[i]];
    }
    return;
  }

  unsigned int j = 0;
  //Get the order of the joint state for the simulation with the wheelchair
  for (unsigned int i = 0; i <= joint_name.size(); i++)
  {
    j = 0;
    while (joint_name[i] != msg.name[j] && j < msg.position.size())
    {
      j++;
    }
    if (joint_name[i] == msg.name[j])
    {
      RCLCPP_DEBUG_STREAM(n_->get_logger(), joint_name[i] << ": " << j);
      joint_order_[i] = j;
    }
  }

  for (int i = 0; i < 6; i++)
  {
    q_command_.data[i] = msg.position[joint_order_[i]];
  }
  gripper_q_command_.data[0] = msg.position[joint_order_[6]];
  for (int i = 0; i < 7; i++)
  {
    q_measured_[i] = msg.position[joint_order_[i]];
  }
  init = true;
}

void JointOutputIntegrator::callback_dq_output_(const std_msgs::msg::Float64MultiArray& msg)
{
  dq_output_.data = msg.data;
}

void JointOutputIntegrator::callback_gripper_dq_output_(const std_msgs::msg::Float64MultiArray& msg)
{
  gripper_dq_output_.data = msg.data;
}

rcl_interfaces::msg::SetParametersResult JointOutputIntegrator::on_parameter_change_(
  const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& param : parameters)
  {
    if (param.get_name() == "position_error_saturation_enable")
    {
      position_error_saturation_enable_ = param.as_bool();
    }
    else if (param.get_name() == "position_error_saturation_threshold")
    {
      double threshold = param.as_double();
      if (threshold < 0.0)
      {
        result.successful = false;
        result.reason = "position_error_saturation_threshold must be >= 0";
        continue;
      }
      position_error_saturation_threshold_ = threshold;
    }
    else if (param.get_name() == "position_error_saturation_persistent")
    {
      position_error_saturation_persistent_ = param.as_bool();
    }
  }
  return result;
}

void JointOutputIntegrator::timer_callback()
{
  if (!init)
  {
    return;
  }
  for (int i = 0; i < 6; i++)
  {
    q_command_.data[i] = q_command_.data[i] + dq_output_.data[i] * sampling_period_;
    q_command_.data[i] = std::clamp(q_command_.data[i], q_lower_limit_[i], q_upper_limit_[i]);
  }

  // index magic number 6 used: => gripper value
  gripper_q_command_.data[0] =
    gripper_q_command_.data[0] + gripper_dq_output_.data[0] * sampling_period_;
  gripper_q_command_.data[0] =
    std::clamp(gripper_q_command_.data[0], q_lower_limit_[6], q_upper_limit_[6]);

  // Position error saturation (gripper excluded)
  std_msgs::msg::Float64MultiArray q_command_out = q_command_;
  if (position_error_saturation_enable_)
  {
    for (int i = 0; i < 6; i++)
    {
      double clamped = std::clamp(
        q_command_.data[i], q_measured_[i] - position_error_saturation_threshold_,
        q_measured_[i] + position_error_saturation_threshold_);
      q_command_out.data[i] = clamped;
      if (position_error_saturation_persistent_)
      {
        q_command_.data[i] = clamped;
      }
    }
  }

  joints_command_pub_->publish(q_command_out);
  gripper_command_pub_->publish(gripper_q_command_);

  publish_controlled_point_marker_(q_command_out);
}

bool JointOutputIntegrator::ensure_controlled_point_kinematics_ready_()
{
  if (controlled_point_kinematics_ready_)
  {
    return true;
  }
  if (controlled_point_kinematics_load_attempted_)
  {
    // Already tried and failed: don't retry (and re-log) every control cycle.
    return false;
  }
  controlled_point_kinematics_load_attempted_ = true;

  try
  {
    const std::string resolved_path =
      controlled_point_resolve_urdf_path(controlled_point_urdf_path_);
    pinocchio::Model full_model;
    pinocchio::urdf::buildModel(resolved_path, full_model, false);

    std::vector<pinocchio::JointIndex> joints_to_lock;
    for (pinocchio::JointIndex joint_id = 1;
         joint_id < static_cast<pinocchio::JointIndex>(full_model.njoints); ++joint_id)
    {
      const bool is_controlled =
        std::find(
          kControlledPointControlledJointNames.begin(), kControlledPointControlledJointNames.end(),
          full_model.names[joint_id]) != kControlledPointControlledJointNames.end();
      if (!is_controlled)
      {
        joints_to_lock.push_back(joint_id);
      }
    }
    pinocchio::buildReducedModel(
      full_model, joints_to_lock, pinocchio::neutral(full_model), controlled_point_model_);

    if (
      static_cast<std::size_t>(controlled_point_model_.njoints - 1) !=
      kControlledPointControlledJointNames.size())
    {
      throw std::runtime_error(
        "Expected joint_1..joint_6 in the URDF, found a different set of controlled joints.");
    }
    if (!controlled_point_model_.existFrame(controlled_point_end_effector_frame_))
    {
      throw std::runtime_error(
        "Unknown end effector frame '" + controlled_point_end_effector_frame_ + "'.");
    }

    controlled_point_data_ = pinocchio::Data(controlled_point_model_);
    controlled_point_kinematics_ready_ = true;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(
      n_->get_logger(),
      "Controlled point marker: unable to load Pinocchio model from '%s': %s. Marker publishing "
      "disabled.",
      controlled_point_urdf_path_.c_str(), ex.what());
    return false;
  }
  return true;
}

void JointOutputIntegrator::publish_controlled_point_marker_(
  const std_msgs::msg::Float64MultiArray& q_command_out)
{
  if (!ensure_controlled_point_kinematics_ready_())
  {
    return;
  }

  Eigen::VectorXd q = Eigen::VectorXd::Zero(controlled_point_model_.nq);
  for (int i = 0; i < 6; i++)
  {
    q[i] = q_command_out.data[static_cast<std::size_t>(i)];
  }

  pinocchio::forwardKinematics(controlled_point_model_, controlled_point_data_, q);
  pinocchio::updateFramePlacements(controlled_point_model_, controlled_point_data_);
  const Eigen::Vector3d x_setpoint =
    controlled_point_data_
      .oMf[controlled_point_model_.getFrameId(controlled_point_end_effector_frame_)]
      .translation();

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = controlled_point_marker_frame_id_;
  marker.header.stamp = n_->now();
  marker.ns = "controlled_point";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = x_setpoint.x();
  marker.pose.position.y = x_setpoint.y();
  marker.pose.position.z = x_setpoint.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.03;
  marker.color.g = 1.0f;
  marker.color.a = 1.0f;
  controlled_point_marker_pub_->publish(marker);
}
}  // namespace space_control

using namespace space_control;
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto n = rclcpp::Node::make_shared("joint_output_integrator", node_options);

  JointOutputIntegrator joint_output_integrator(n);

  rclcpp::spin(n);
  rclcpp::shutdown();
  return 0;
}
