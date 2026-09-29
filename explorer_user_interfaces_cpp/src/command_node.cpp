#include "explorer_user_interfaces_cpp/command_node.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace space_control
{
namespace
{
// Longest gap between two gripper velocity messages to be considered "continuous".
// If this value is reached we considers it's a new "set" of gripper inputs.
constexpr double MAX_GRIPPER_INTERVAL_SECONDS = 0.1;

// A trajectory velocity not refreshed within this delay falls back to 0 (hold position), so a
// source that vanishes while moving do not stays on last command sent.
constexpr double TRAJECTORY_VELOCITY_TIMEOUT_SECONDS = 0.3;

// Cartesian velocity magnitude above which a command asks for motion (same as joystick_selector)
constexpr double CARTESIAN_ACTIVITY_THRESHOLD = 1e-3;
}  // namespace

CommandNode::CommandNode(rclcpp::Node::SharedPtr n) : n_(n), controller_manager_wrapper_(n)
{
  RCLCPP_INFO(n->get_logger(), "CommandNode constructor");

  n_->declare_parameter<double>("sampling_period", 0.01);
  n_->declare_parameter<std::vector<std::string>>(
    "default_controller_name_list", std::vector<std::string>());

  sampling_period_ = n_->get_parameter("sampling_period").as_double();

  default_controller_name_list_ =
    n_->get_parameter("default_controller_name_list").as_string_array();
  if (default_controller_name_list_.empty())
  {
    throw std::runtime_error("Parameter 'default_controller_name_list' is required");
  }

  n_->declare_parameter<std::string>("trajectory_file", "");
  n_->declare_parameter<bool>("active_trajectory", true);
  n_->declare_parameter<bool>("use_qp_inria", false);

  if (n_->get_parameter("active_trajectory").as_bool())
  {
    RCLCPP_INFO(n_->get_logger(), "Active trajectory control mode enabled.");

    std::string trajectory_file;
    n_->get_parameter("trajectory_file", trajectory_file);

    if (!trajectory_manager_.load_trajectory(trajectory_file))
    {
      rclcpp::shutdown();
      return;
    }
  }

  n_->get_parameter("use_qp_inria", use_qp_inria_);

  n_->declare_parameter<int>("min_speed_level", 1);
  n_->declare_parameter<int>("max_speed_level", 4);
  n_->declare_parameter<int>("default_speed_level", 2);
  n_->declare_parameter<double>("speed_level_multiplier", 0.25);
  n_->declare_parameter<std::string>(
    "cartesian_velocity_output_topic",
    "/explorer_user_interfaces/rqt_armcontrol/input_device_velocity");

  min_speed_level_ = static_cast<int>(n_->get_parameter("min_speed_level").as_int());
  max_speed_level_ = static_cast<int>(n_->get_parameter("max_speed_level").as_int());
  if (min_speed_level_ > max_speed_level_)
  {
    throw std::runtime_error("Parameter 'min_speed_level' must not exceed 'max_speed_level'");
  }
  speed_level_ = std::clamp(
    static_cast<int>(n_->get_parameter("default_speed_level").as_int()), min_speed_level_,
    max_speed_level_);
  speed_level_multiplier_ = n_->get_parameter("speed_level_multiplier").as_double();

  // Half open
  gripper_command_.data = {0.5};

  // Initialize subscribers and publishers
  q_current_sub_ = n_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&CommandNode::callback_q_current_, this, std::placeholders::_1));
  cartesian_vel_sub_ = n_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "command_node/robot/velocity/commands", 10,
    std::bind(&CommandNode::callback_cartesian_velocity_, this, std::placeholders::_1));
  gripper_vel_sub_ = n_->create_subscription<std_msgs::msg::Float64>(
    "command_node/gripper/velocity/commands", 10,
    std::bind(&CommandNode::callback_gripper_velocity_, this, std::placeholders::_1));
  trajectory_vel_sub_ = n_->create_subscription<std_msgs::msg::Float64>(
    "command_node/trajectory/velocity/commands", 10,
    std::bind(&CommandNode::callback_trajectory_velocity_, this, std::placeholders::_1));
  last_trajectory_vel_time_ = n_->now();

  trajectory_pub_ = n_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory_controller/joint_trajectory", 10);
  reset_qp_solving_pub_ =
    n_->create_publisher<std_msgs::msg::Bool>("/command_node/reset_qp_solving", 10);
  // Latched: only published on change, late subscribers still get the current status
  retract_status_pub_ = n_->create_publisher<std_msgs::msg::String>(
    "command_node/retract_status", rclcpp::QoS(1).transient_local());
  // Latched: only published on change, late subscribers still get the current level
  speed_level_pub_ = n_->create_publisher<std_msgs::msg::Int32>(
    "command_node/speed_level", rclcpp::QoS(1).transient_local());
  cartesian_vel_pub_ = n_->create_publisher<geometry_msgs::msg::TwistStamped>(
    n_->get_parameter("cartesian_velocity_output_topic").as_string(), 10);
  gripper_command_pub_ =
    n_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_controller/commands", 10);

  set_speed_level_srv_ = n_->create_service<explorer_msgs::srv::SetSpeedLevel>(
    "command_node/set_speed_level",
    std::bind(
      &CommandNode::callback_set_speed_level_, this, std::placeholders::_1,
      std::placeholders::_2));

  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(n_, "qp_solving");

  publish_retract_status_();
  speed_level_pub_->publish(std_msgs::msg::Int32().set__data(speed_level_));

  // Timer callback
  timer_ = n_->create_wall_timer(
    std::chrono::duration<double>(sampling_period_),
    std::bind(&CommandNode::timer_callback_, this));
}

CommandNode::~CommandNode()
{
  if (switch_thread_.joinable())
  {
    switch_thread_.join();
  }
}

double CommandNode::speed_factor_() const { return speed_level_multiplier_ * speed_level_; }

void CommandNode::callback_cartesian_velocity_(const geometry_msgs::msg::TwistStamped& msg)
{
  const bool moving = std::fabs(msg.twist.linear.x) > CARTESIAN_ACTIVITY_THRESHOLD ||
                      std::fabs(msg.twist.linear.y) > CARTESIAN_ACTIVITY_THRESHOLD ||
                      std::fabs(msg.twist.linear.z) > CARTESIAN_ACTIVITY_THRESHOLD ||
                      std::fabs(msg.twist.angular.x) > CARTESIAN_ACTIVITY_THRESHOLD ||
                      std::fabs(msg.twist.angular.y) > CARTESIAN_ACTIVITY_THRESHOLD ||
                      std::fabs(msg.twist.angular.z) > CARTESIAN_ACTIVITY_THRESHOLD;

  // Cartesian motion needs the default controller, switch back to it on demand
  if (moving)
  {
    setTrajectoryMode_(false);
  }

  // Until the default controller is back, hold
  if (trajectory_mode_ || controller_state_ != ControllerState::DEFAULT_CONTROLLER)
  {
    geometry_msgs::msg::TwistStamped hold;
    hold.header = msg.header;
    cartesian_vel_pub_->publish(hold);
    return;
  }

  const double factor = speed_factor_();

  geometry_msgs::msg::TwistStamped scaled = msg;
  scaled.twist.linear.x *= factor;
  scaled.twist.linear.y *= factor;
  scaled.twist.linear.z *= factor;
  scaled.twist.angular.x *= factor;
  scaled.twist.angular.y *= factor;
  scaled.twist.angular.z *= factor;

  cartesian_vel_pub_->publish(scaled);
}

void CommandNode::callback_trajectory_velocity_(const std_msgs::msg::Float64& msg)
{
  last_trajectory_vel_time_ = n_->now();
  trajectory_velocity_input_.store(msg.data * speed_factor_());

  // Moving along the trajectory needs joint_trajectory_controller, switch to it on demand.
  // Releasing the input only holds the position, the switch back is left to the next Cartesian
  // command.
  if (msg.data != 0.0)
  {
    setTrajectoryMode_(true);
  }
}

void CommandNode::callback_gripper_velocity_(const std_msgs::msg::Float64& msg)
{
  const rclcpp::Time now = n_->now();
  double dt = 0.0;
  if (last_gripper_vel_time_)
  {
    // Computes time gap for "continuous" input or set to 0.0 for a new set of inputs
    const double gap = (now - *last_gripper_vel_time_).seconds();
    dt = gap > MAX_GRIPPER_INTERVAL_SECONDS ? 0.0 : gap;
  }
  last_gripper_vel_time_ = now;

  // Ensure default controller is enabled
  if (msg.data != 0.0)
  {
    setTrajectoryMode_(false);
  }

  // Until gripper_controller is back, hold
  if (trajectory_mode_ || controller_state_ != ControllerState::DEFAULT_CONTROLLER)
  {
    return;
  }

  double& position = gripper_command_.data[0];
  position = std::clamp(position + msg.data * speed_factor_() * dt, 0.0, 1.0);

  gripper_command_pub_->publish(gripper_command_);
}

void CommandNode::publish_retract_status_()
{
  if (last_retract_status_ == trajectory_manager_.get_status())
  {
    return;
  }
  last_retract_status_ = trajectory_manager_.get_status();;

  retract_status_pub_->publish(std_msgs::msg::String().set__data(to_string(trajectory_manager_.get_status())));
}

void CommandNode::callback_set_speed_level_(
  const std::shared_ptr<explorer_msgs::srv::SetSpeedLevel::Request> request,
  std::shared_ptr<explorer_msgs::srv::SetSpeedLevel::Response> response)
{
  const int requested = request->relative ? speed_level_ + request->level : request->level;
  const int previous = speed_level_;

  speed_level_ = std::clamp(requested, min_speed_level_, max_speed_level_);

  if (speed_level_ != previous)
  {
    RCLCPP_INFO(n_->get_logger(), "Speed level changed to %d", speed_level_);
    speed_level_pub_->publish(std_msgs::msg::Int32().set__data(speed_level_));
  }

  response->success = true;
  response->level = speed_level_;
  response->message = requested == speed_level_
                        ? "Speed level set"
                        : "Requested speed level " + std::to_string(requested) +
                            " clamped to [" + std::to_string(min_speed_level_) + ", " +
                            std::to_string(max_speed_level_) + "]";
}

void CommandNode::setTrajectoryMode_(bool enable)
{
  // Nothing to do if trajectory disabled or no transition
  if (!trajectory_manager_.is_enabled() || trajectory_mode_.exchange(enable) == enable)
  {
    return;
  }

  if (!enable)
  {
    // Do not carry a stale rate into the next retract.
    trajectory_velocity_input_.store(0.0);
  }

  RCLCPP_INFO(n_->get_logger(), "Trajectory mode %s", enable ? "requested" : "released");
}

void CommandNode::callback_q_current_(const sensor_msgs::msg::JointState& msg)
{
  current_pos_ = msg;
  if (init_) return;

  mode_ = joint_mode_resolver_.detect_mode(msg.name, n_->get_logger(), "[command_node]");
  if (
    mode_ != space_control::JointMode::INVALID &&
    joint_mode_resolver_.build_joint_order(
      mode_, msg.name, joint_order_, n_->get_logger(), "[command_node]"))
  {
    // In FULL mode, wheelchair joints are added in first, so the Explorer robot joints range starts after them
    // In EXPLORER mode it starts at 0
    explorer_joint_offset_ = (mode_ == space_control::JointMode::FULL)
                               ? joint_mode_resolver_.expected_names_wheelchair().size()
                               : 0;
    init_ = true;
    RCLCPP_INFO(n_->get_logger(), "[command_node] Init done.");
  }
}

void CommandNode::handle_controller_state_()
{
  auto vec_string_to_string = [](std::vector<std::string>& str_vec) -> std::string
  {
    const char* const delimiter = ", ";

    std::ostringstream output_str;
    std::copy(
      str_vec.begin(), str_vec.end(), std::ostream_iterator<std::string>(output_str, delimiter));
    return output_str.str();
  };

  switch (controller_state_)
  {
    case ControllerState::DEFAULT_CONTROLLER:
      reset_qp_solving_pub_->publish(std_msgs::msg::Bool().set__data(false));
      if (trajectory_mode_)
      {
        RCLCPP_INFO(n_->get_logger(), "→ SWITCHING TO TRAJECTORY MODE");
        controller_state_ = ControllerState::SWITCHING_TO_TRAJ;
      }
      break;

    case ControllerState::SWITCHING_TO_TRAJ:
      if (!switch_in_progress_)
      {
        switch_in_progress_ = true;
        auto future = controller_manager_wrapper_.switch_controller_async(
          default_controller_name_list_, {"joint_trajectory_controller"});

        if (switch_thread_.joinable())
        {
          switch_thread_.join();
        }
        // Callback when the switch completes
        switch_thread_ = std::thread(
          [this, future = std::move(future)]() mutable
          {
            try
            {
              bool success = future.get();  // Bloque ici, mais dans un thread séparé
              if (success)
              {
                RCLCPP_INFO(n_->get_logger(), "Switched to joint_trajectory_controller");
                controller_state_ = ControllerState::TRAJECTORY;
              }
              else
              {
                RCLCPP_ERROR(n_->get_logger(), "Failed to switch to joint_trajectory_controller");
                controller_state_ = ControllerState::DEFAULT_CONTROLLER;
              }
            }
            catch (const std::exception& e)
            {
              RCLCPP_ERROR(n_->get_logger(), "Exception during controller switch: %s", e.what());
              controller_state_ = ControllerState::DEFAULT_CONTROLLER;
            }
            switch_in_progress_ = false;
          });
      }
      break;

    case ControllerState::TRAJECTORY:
      if (!trajectory_mode_)
      {
        RCLCPP_INFO(
          n_->get_logger(), "→ RESTORING DEFAULT CONTROLLER(S): %s",
          vec_string_to_string(default_controller_name_list_).c_str());
        controller_state_ = ControllerState::RESTORING_DEFAULT_CONTROLLER;
      }
      break;

    case ControllerState::RESTORING_DEFAULT_CONTROLLER:
      if (!switch_in_progress_)
      {
        switch_in_progress_ = true;

        trajectory_manager_.reset();
        reset_qp_solving_pub_->publish(std_msgs::msg::Bool().set__data(true));

        auto future = controller_manager_wrapper_.switch_controller_async(
          {"joint_trajectory_controller"}, default_controller_name_list_);

        if (switch_thread_.joinable())
        {
          switch_thread_.join();
        }
        switch_thread_ = std::thread(
          [this, vec_string_to_string, future = std::move(future)]() mutable
          {
            try
            {
              bool success = future.get();
              if (success)
              {
                controller_state_ = ControllerState::DEFAULT_CONTROLLER;
                RCLCPP_INFO(
                  n_->get_logger(), "→ DEFAULT CONTROLLER(S) RESTORED: Switched to %s",
                  vec_string_to_string(default_controller_name_list_).c_str());
              }
              else
              {
                RCLCPP_WARN(
                  n_->get_logger(), "Failed to switch to %s",
                  vec_string_to_string(default_controller_name_list_).c_str());
                controller_state_ = ControllerState::TRAJECTORY;
              }
            }
            catch (const std::exception& e)
            {
              RCLCPP_ERROR(n_->get_logger(), "Exception during controller switch: %s", e.what());
              controller_state_ = ControllerState::TRAJECTORY;
            }
            switch_in_progress_ = false;
          });
      }
      break;
  }
}

void CommandNode::modifyTargetNodeParameter_(
  const std::string& param_name, const rclcpp::ParameterValue& value)
{
  if (!param_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(n_->get_logger(), "Parameter service of qp_solving not available");
    return;
  }

  rclcpp::Parameter param(param_name, value);

  param_client_->set_parameters(
    {param},
    [this,
     param_name](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
    {
      const auto results = future.get();

      if (!results.empty() && results[0].successful)
      {
        RCLCPP_INFO(n_->get_logger(), "Parameter %s modified successfully", param_name.c_str());
      }
      else
      {
        RCLCPP_ERROR(n_->get_logger(), "Failed to modify parameter %s", param_name.c_str());
      }
    });
}

void CommandNode::getDoubleParameter_(const std::string& param_name, std::optional<double>& value)
{
  // Check if parameters client is ready to accept requests
  if (
    !param_client_->service_is_ready() ||
    // Check if this parameter name call is already in pending
    std::find(
      parameter_name_list_in_pending_.begin(), parameter_name_list_in_pending_.end(), param_name) !=
      parameter_name_list_in_pending_.end())
  {
    return;
  }
  RCLCPP_INFO(n_->get_logger(), "Initializing %s from qp_solving (async)", param_name.c_str());

  parameter_name_list_in_pending_.push_back(param_name);

  param_client_->get_parameters(
    {param_name},
    [this, param_name, &value](std::shared_future<std::vector<rclcpp::Parameter>> future)
    {
      const auto& params = future.get();
      // Delete this parameter from the parameter pending request list
      auto parameter_it = std::find(
        parameter_name_list_in_pending_.begin(), parameter_name_list_in_pending_.end(), param_name);
      if (parameter_it != parameter_name_list_in_pending_.end())
      {
        parameter_name_list_in_pending_.erase(parameter_it);
      }

      if (params.empty())
      {
        RCLCPP_WARN(n_->get_logger(), "%s not available yet ", param_name.c_str());
        return;
      }

      if (params[0].get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        RCLCPP_WARN(n_->get_logger(), "%s is not a double", param_name.c_str());
        return;
      }

      value = params[0].as_double();

      RCLCPP_INFO(n_->get_logger(), "%s initialized to %.3f", param_name.c_str(), *value);
    });
}

void CommandNode::update_trajectory_()
{
  // Nothing to do if :
  // - trajectory disabled
  // - not in trajectory_mode
  // - joint_trajectory_controller not active
  if (!trajectory_manager_.is_enabled() || !trajectory_mode_ || controller_state_ != ControllerState::TRAJECTORY)
  {
    return;
  }

  if (!init_ || joint_order_.size() < explorer_joint_offset_ + 7)
  {
    RCLCPP_ERROR_THROTTLE(
      n_->get_logger(), *n_->get_clock(), 1000,
      "[command_node] update_trajectory_ called before joint_order_ is ready, skipping cycle");
    return;
  }

  for (size_t i = 0; i < 7; ++i)
  {
    size_t idx = joint_order_[explorer_joint_offset_ + i];
    if (idx >= current_pos_.position.size())
    {
      RCLCPP_ERROR_THROTTLE(
        n_->get_logger(), *n_->get_clock(), 1000,
        "[command_node] update_trajectory_: out of range joint index %zu (current_pos_ has %zu "
        "positions), skipping this index",
        idx, current_pos_.position.size());
      // TODO continue even with partial joints or return ?
      continue;
    }
    q_current_[i] = current_pos_.position[idx];
  }

  trajectory_manager_.update(q_current_, static_cast<float>(trajectory_velocity_input_.load()));

  auto trajectory = trajectory_manager_.get_trajectory();
  if (trajectory)
  {
    trajectory_pub_->publish(trajectory.value());
    publish_retract_status_();
  }
}

void CommandNode::timer_callback_()
{
  // Check if last trajectory velocity input is older than timeout
  if ((n_->now() - last_trajectory_vel_time_).seconds() > TRAJECTORY_VELOCITY_TIMEOUT_SECONDS)
  {
    // Force stationary (input seems not active anymore for now)
    trajectory_velocity_input_.store(0.0);
  }

  // TODO delete when deleting old QP (confusing)
  if (!use_qp_inria_)
  {
    if (!j2_max_cached_ || !j2_operational_max_cached_)
    {
      getDoubleParameter_("j2.max", j2_max_cached_);
      getDoubleParameter_("j2.operational_max", j2_operational_max_cached_);
      return;
    }
    else if (!j3_max_cached_ || !j3_operational_max_cached_)
    {
      getDoubleParameter_("j3.max", j3_max_cached_);
      getDoubleParameter_("j3.operational_max", j3_operational_max_cached_);
      return;
    }
    else if (limits_initialized_ == false)
    {
      j2_max_ = *j2_max_cached_;
      j2_operational_max_ = *j2_operational_max_cached_;

      j3_max_ = *j3_max_cached_;
      j3_operational_max_ = *j3_operational_max_cached_;

      actual_j2_limit_ = j2_max_;
      actual_j3_limit_ = j3_max_;
      limits_initialized_ = true;
    }
  }

  handle_controller_state_();

  update_trajectory_();

  // TODO delete when deleting old QP (confusing)
  if (!use_qp_inria_ && trajectory_manager_.is_enabled())
  {
    if (trajectory_manager_.get_status() != RetractStatus::READY)
    {
      if (actual_j2_limit_ != j2_max_)
      {
        modifyTargetNodeParameter_("j2.max", rclcpp::ParameterValue(j2_max_));
        actual_j2_limit_ = j2_max_;
      }

      if (actual_j3_limit_ != j3_max_)
      {
        modifyTargetNodeParameter_("j3.max", rclcpp::ParameterValue(j3_max_));
        actual_j3_limit_ = j3_max_;
      }
    }
    else if (trajectory_manager_.get_status() == RetractStatus::READY)
    {
      if (!init_ || joint_order_.size() < explorer_joint_offset_ + 3)
      {
        RCLCPP_ERROR(
          n_->get_logger(),
          "[command_node] timer_callback_: joint_order_ not ready, skipping j2/j3 operational "
          "limit update");
        return;
      }

      size_t j2_index = joint_order_[explorer_joint_offset_ + 1];
      size_t j3_index = joint_order_[explorer_joint_offset_ + 2];
      if (j2_index >= current_pos_.position.size() || j3_index >= current_pos_.position.size())
      {
        RCLCPP_ERROR(
          n_->get_logger(),
          "[command_node] timer_callback_: joint index out of range, skipping j2/j3 "
          "operational limit update");
        return;
      }
      if (
        current_pos_.position[j2_index] < j2_operational_max_ &&
        actual_j2_limit_ != j2_operational_max_)
      {
        modifyTargetNodeParameter_("j2.max", rclcpp::ParameterValue(j2_operational_max_));
        actual_j2_limit_ = j2_operational_max_;
      }

      if (
        current_pos_.position[j3_index] < j3_operational_max_ &&
        actual_j3_limit_ != j3_operational_max_)
      {
        modifyTargetNodeParameter_("j3.max", rclcpp::ParameterValue(j3_operational_max_));
        actual_j3_limit_ = j3_operational_max_;
      }
    }
  }
}

}  // namespace space_control

using namespace space_control;
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  auto n = rclcpp::Node::make_shared("command_node", node_options);

  CommandNode command_node(n);

  rclcpp::spin(n);
  rclcpp::shutdown();
  return 0;
}
