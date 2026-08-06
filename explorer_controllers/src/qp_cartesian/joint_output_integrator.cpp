#include "explorer_controllers/qp_cartesian/joint_output_integrator.h"

#include <algorithm>

namespace space_control
{
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

  // Position error saturation: disabled by default so behaviour is unchanged unless explicitly enabled.
  // Params may already be auto-declared from overrides (see automatically_declare_parameters_from_overrides
  // in main()), so only declare them here (with their default) if that didn't happen.
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
  // error goes back under the threshold), matching the behaviour before this param existed.
  if (!n_->has_parameter("position_error_saturation_persistent"))
  {
    n_->declare_parameter<bool>("position_error_saturation_persistent", false);
  }
  position_error_saturation_persistent_ =
    n_->get_parameter("position_error_saturation_persistent").as_bool();

  // Allow all three params above to be changed live (e.g. `ros2 param set`) without restarting the node.
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

  // By default q_command_ itself is left untouched above (unchanged behaviour): only the
  // published command is offset, and only on joints/cycles where the error actually
  // exceeds the threshold, so the desired position never gets farther than the threshold
  // away from the measured one. If position_error_saturation_persistent_ is set, the
  // clamped value is also written back into q_command_ so the offset "sticks" instead of
  // springing back once the error goes back under the threshold.
  // The gripper is published separately and is not concerned by this saturation.
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
