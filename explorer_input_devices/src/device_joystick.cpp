/*
 *  device_joystick.cpp
 *  Copyright (C) 2025 Orthopus
 *  All rights reserved.
 */
#include "explorer_input_devices/device_joystick.h"

#include <algorithm>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

namespace input_device
{
DeviceJoystick::DeviceJoystick(rclcpp::Node::SharedPtr n)
: Device(n, "joy", [this](const sensor_msgs::msg::Joy::SharedPtr msg) { callback_joy_(*msg); })
{
  RCLCPP_INFO(get_node_()->get_logger(), "DeviceJoystick constructor");

  get_node_()->declare_parameter<int>("button_threshold_ms", button_default_threshold_ms);
  get_node_()->declare_parameter<double>("speed_change_threshold", 0.95);
  get_node_()->declare_parameter<double>("speed_level_multiplier", 0.25);
  get_node_()->declare_parameter<double>("sampling_period", 0.01);
  get_node_()->declare_parameter<std::string>("mode_file", "");
  get_node_()->declare_parameter<std::string>(
    "end_effector_pose_topic", "/explorer_controllers/qp_solving/x_current");

  button_threshold_ms_ = get_node_()->get_parameter("button_threshold_ms").as_int();
  speed_change_threshold_ = get_node_()->get_parameter("speed_change_threshold").as_double();
  speed_level_multiplier_ = get_node_()->get_parameter("speed_level_multiplier").as_double();
  sampling_period_ = get_node_()->get_parameter("sampling_period").as_double();

  button_handler_.init(button_threshold_ms_);

  // Map control behaviors to corresponding functions
  control_behaviors_ = {
    {ControlName::CARTESIAN_X, [this](const AxisInfo& axis) { cartesian_linear_(axis); }},
    {ControlName::CARTESIAN_Y, [this](const AxisInfo& axis) { cartesian_linear_(axis); }},
    {ControlName::CARTESIAN_Z, [this](const AxisInfo& axis) { cartesian_linear_(axis); }},
    {ControlName::ROTATION_X, [this](const AxisInfo& axis) { cartesian_rotation_(axis); }},
    {ControlName::ROTATION_Y, [this](const AxisInfo& axis) { cartesian_rotation_(axis); }},
    {ControlName::ROTATION_Z, [this](const AxisInfo& axis) { cartesian_rotation_(axis); }},
    {ControlName::JOINT_6, [this](const AxisInfo& axis) { joint_direct_(axis); }},
    {ControlName::CHANGE_SPEED, [this](const AxisInfo& axis) { change_speed_(axis); }},
    {ControlName::DRINK, [this](const AxisInfo& axis) { drink_(axis); }},
    {ControlName::GRIPPER, [this](const AxisInfo& axis) { gripper_(axis); }},
    {ControlName::COMPLEX_X, [this](const AxisInfo& axis) { complex_(axis); }},
    {ControlName::COMPLEX_Y, [this](const AxisInfo& axis) { complex_(axis); }},
    {ControlName::TRAJECTORY_CONTROL,
     [this](const AxisInfo& axis) { trajectory_control_(axis); }}};

  // Get the value of the mode_file parameter
  std::string mode_file;
  get_node_()->get_parameter("mode_file", mode_file);

  // Load mode configuration from YAML file.
  std::string config_error;
  const auto loaded = load_mode_data(mode_file, config_error);
  if (!loaded)
  {
    RCLCPP_FATAL(get_node_()->get_logger(), "%s. Shutting down node.", config_error.c_str());
    rclcpp::shutdown();
    return;
  }
  data_ = *loaded;

  std::unordered_set<ControlName> valid_control_names;
  for (const auto& kv : control_behaviors_) valid_control_names.insert(kv.first);

  if (!validate_mode_data(data_, valid_control_names, config_error))
  {
    RCLCPP_FATAL(
      get_node_()->get_logger(), "%s. YAML configuration validation failed. Shutting down node.",
      config_error.c_str());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(get_node_()->get_logger(), "YAML mode configuration validated successfully");

  current_mode_name_ = data_.initial_mode;

  x_current_sub_ = get_node_()->create_subscription<geometry_msgs::msg::Pose>(
    get_node_()->get_parameter("end_effector_pose_topic").as_string(), 10,
    [this](const geometry_msgs::msg::Pose& msg) { callback_x_current_(msg); });
  retract_status_sub_ = get_node_()->create_subscription<std_msgs::msg::String>(
    "command_node/retract_status", 10,
    [this](const std_msgs::msg::String& msg) { callback_retract_status_(msg); });

  frame_id_pub_ = get_node_()->create_publisher<explorer_msgs::msg::ControlFrameSelection>(
    "/explorer_controllers/command_node/control_frame_selection", 10);
  mode_name_pub_ =
    get_node_()->create_publisher<std_msgs::msg::String>("command_node/mode_name", 10);
  speed_level_pub_ =
    get_node_()->create_publisher<std_msgs::msg::Int32>("command_node/speed_level", 10);
  set_trajectory_mode_client_ =
    get_node_()->create_client<std_srvs::srv::SetBool>("command_node/set_trajectory_mode");

  reset_velocities_();

  timer_ = get_node_()->create_wall_timer(
    std::chrono::duration<double>(sampling_period_), [this]() { timer_callback_(); });
}

void DeviceJoystick::callback_joy_(const sensor_msgs::msg::Joy& msg)
{
  if (msg.axes.size() < 2)
  {
    RCLCPP_WARN_THROTTLE(
      get_node_()->get_logger(), *get_node_()->get_clock(), 1000, "Joystick has insufficient axes");
    return;
  }
  else if (msg.buttons.size() < 1)
  {
    RCLCPP_WARN_THROTTLE(
      get_node_()->get_logger(), *get_node_()->get_clock(), 1000,
      "Joystick has insufficient buttons");
    return;
  }
  {
    std::lock_guard<std::mutex> lock_axis(mutex_axis_);
    // Store raw joystick values (smoothing is applied per-axis in readAxisValue)
    axis_1_raw_ = msg.axes[0];
    axis_2_raw_ = msg.axes[1];
  }

  button_handler_.update(msg.buttons[0]);
}

void DeviceJoystick::callback_x_current_(const geometry_msgs::msg::Pose& msg) { x_current_ = msg; }

void DeviceJoystick::callback_retract_status_(const std_msgs::msg::String& msg)
{
  // "ready" is the only state in which the arm may be driven in Cartesian space.
  locked_.store(msg.data != "ready");
}

void DeviceJoystick::request_trajectory_mode_(bool enable)
{
  if (!set_trajectory_mode_client_->service_is_ready())
  {
    RCLCPP_ERROR(
      get_node_()->get_logger(),
      "command_node/set_trajectory_mode unavailable, cannot %s trajectory mode",
      enable ? "request" : "release");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;

  set_trajectory_mode_client_->async_send_request(
    request,
    [this, enable](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
    {
      const auto& response = future.get();
      if (!response->success)
      {
        RCLCPP_ERROR(
          get_node_()->get_logger(), "Failed to %s trajectory mode: %s",
          enable ? "request" : "release", response->message.c_str());
      }
    });
}

bool DeviceJoystick::mode_has_trajectory_control_(const std::string& mode_name) const
{
  const auto mode_it = data_.button_modes_map.find(mode_name);
  if (mode_it == data_.button_modes_map.end())
  {
    return false;
  }

  for (const auto& axis : mode_it->second.axes)
  {
    if (axis.control_name == ControlName::TRAJECTORY_CONTROL)
    {
      return true;
    }
  }
  return false;
}

void DeviceJoystick::timer_callback_()
{
  // Step 1: Get the current mode to know which smoothing alphas to use
  ButtonMode mode = data_.button_modes_map[current_mode_name_];

  // Step 2: Read raw values and apply smoothing for each axis ONCE
  // Find the smoothing alpha for each joystick axis from the current mode config
  float alpha_ax1 = 1.0f;  // default: no smoothing
  float alpha_ax2 = 1.0f;
  for (const auto& axis : mode.axes)
  {
    if (axis.joystick_axis == JoystickAxis::AX1)
    {
      alpha_ax1 = axis.smoothing_alpha;
    }
    else if (axis.joystick_axis == JoystickAxis::AX2)
    {
      alpha_ax2 = axis.smoothing_alpha;
    }
  }

  // Step 3: Atomically read raw values and apply smoothing
  {
    std::lock_guard<std::mutex> lock_axis(mutex_axis_);
    axis_1_smoothed_ = alpha_ax1 * axis_1_raw_ + (1.0f - alpha_ax1) * axis_1_smoothed_;
    axis_2_smoothed_ = alpha_ax2 * axis_2_raw_ + (1.0f - alpha_ax2) * axis_2_smoothed_;
  }

  // Step 4: Reset velocities
  reset_velocities_();

  complex_mode_ = false;

  // Step 5: Execute control behaviors for each axis (uses pre-smoothed values)
  for (const auto& axis : mode.axes)
  {
    execute_behavior_(axis);
  }

  if (complex_mode_ && !locked_)
  {
    complex_calculation_(rotation_speed_scale_);
  }

  // Step 6: Publish the computed velocities
  publish_cartesian_command_(cartesian_vel_);
  frame_id_pub_->publish(frame_id_);

  // Handle mode switching based on button clicks
  const std::string previous_mode_name = current_mode_name_;
  if (button_handler_.is_short_click() && mode.buttons.short_click != "")
  {
    current_mode_name_ = mode.buttons.short_click;
    // Reset smoothed values when switching modes to avoid artifacts
    axis_1_smoothed_ = 0.0f;
    axis_2_smoothed_ = 0.0f;
  }
  else if (button_handler_.is_long_click() && mode.buttons.long_click != "")
  {
    current_mode_name_ = mode.buttons.long_click;
    // Reset smoothed values when switching modes to avoid artifacts
    axis_1_smoothed_ = 0.0f;
    axis_2_smoothed_ = 0.0f;
  }

  if (current_mode_name_ != previous_mode_name)
  {
    const bool was_trajectory = mode_has_trajectory_control_(previous_mode_name);
    const bool is_trajectory = mode_has_trajectory_control_(current_mode_name_);
    if (was_trajectory != is_trajectory)
    {
      request_trajectory_mode_(is_trajectory);
    }
  }

  mode_name_pub_->publish(std_msgs::msg::String().set__data(current_mode_name_));
  speed_level_pub_->publish(std_msgs::msg::Int32().set__data(speed_level_));
  // Every cycle, including with velocity 0 outside the gripper mode, so the controller
  // keeps being commanded to the held position.
  update_gripper_command_(gripper_vel_, sampling_period_);
}

void DeviceJoystick::execute_behavior_(const AxisInfo& axis)
{
  if (control_behaviors_.count(axis.control_name))
  {
    control_behaviors_[axis.control_name](axis);
  }
}

// Read joystick axis value (smoothing already applied at start of timer)
float DeviceJoystick::read_axis_value_(const AxisInfo& axis_info)
{
  float smoothed_value = 0.0f;

  // Simply return the pre-smoothed value for the requested axis
  if (axis_info.joystick_axis == JoystickAxis::AX1)
  {
    smoothed_value = axis_1_smoothed_;
  }
  else if (axis_info.joystick_axis == JoystickAxis::AX2)
  {
    smoothed_value = axis_2_smoothed_;
  }

  double deadzone = 0.0;
  if (axis_info.params.count("deadzone"))
  {
    deadzone = axis_info.params.at("deadzone");
  }

  if (std::abs(smoothed_value) < deadzone)
  {
    smoothed_value = 0.0f;  // Zero out values within the deadzone
  }
  else
  {
    // Rescale values outside the deadzone to the range [0, 1]
    if (smoothed_value > 0)
    {
      smoothed_value = (smoothed_value - deadzone) / (1.0 - deadzone);
    }
    else
    {
      smoothed_value = (smoothed_value + deadzone) / (1.0 - deadzone);
    }
  }

  // Apply direction, scale and speed factor
  float value = smoothed_value * axis_info.direction * axis_info.scale * speed_factor_;

  return value;
}

// Reset Cartesian velocities to zero
void DeviceJoystick::reset_velocities_()
{
  cartesian_vel_.twist.linear = geometry_msgs::msg::Vector3();
  cartesian_vel_.twist.angular = geometry_msgs::msg::Vector3();
  gripper_vel_ = 0.0;
}

void DeviceJoystick::complex_calculation_(const double rotation_speed_scale)
{
  double omega_z = 0.0;
  double x_E = x_current_.position.x;
  double y_E = x_current_.position.y;
  double denom = x_E * x_E + y_E * y_E;
  if (denom > 1e-6)
  {
    omega_z = (x_E * v_y_ - y_E * v_x_) / denom;
  }
  cartesian_vel_.twist.angular.z = omega_z * rotation_speed_scale;
}

// Behavior implementations
void DeviceJoystick::cartesian_linear_(const AxisInfo& axis_info)
{
  if (locked_)
  {
    return;
  }
  // Determine joystick axis value
  float value = 0.0;

  value = read_axis_value_(axis_info);

  // Assign to the appropriate Cartesian linear velocity component
  if (axis_info.control_name == ControlName::CARTESIAN_X)
  {
    cartesian_vel_.twist.linear.x = value;
  }
  else if (axis_info.control_name == ControlName::CARTESIAN_Y)
  {
    cartesian_vel_.twist.linear.y = value;
  }
  else if (axis_info.control_name == ControlName::CARTESIAN_Z)
  {
    cartesian_vel_.twist.linear.z = value;
  }
}

void DeviceJoystick::cartesian_rotation_(const AxisInfo& axis_info)
{
  if (locked_)
  {
    return;
  }
  // Determine joystick axis value
  float value = 0.0;

  value = read_axis_value_(axis_info);

  // Assign to the appropriate Cartesian angular velocity component
  if (axis_info.control_name == ControlName::ROTATION_X)
  {
    cartesian_vel_.twist.angular.x = value;
  }
  else if (axis_info.control_name == ControlName::ROTATION_Y)
  {
    cartesian_vel_.twist.angular.y = value;
  }
  else if (axis_info.control_name == ControlName::ROTATION_Z)
  {
    cartesian_vel_.twist.angular.z = value;
  }

  frame_id_.orientation_control_frame = 1;
}

void DeviceJoystick::joint_direct_(const AxisInfo& axis_info)
{
  if (locked_)
  {
    return;
  }
  // joint_6 is expressed as a tool-frame rotation rather than a joint velocity
  if (axis_info.control_name == ControlName::JOINT_6)
  {
    cartesian_vel_.twist.angular.x = read_axis_value_(axis_info);
    // Set control frame to end-effector for joint 6 control
    frame_id_.orientation_control_frame = 1;
  }
}

void DeviceJoystick::change_speed_(const AxisInfo& axis_info)
{
  // Get min/max speed levels from params (default: 1 to 4)
  int min_level = 1;
  int max_level = 4;
  if (axis_info.params.count("min_speed_level"))
  {
    min_level = static_cast<int>(axis_info.params.at("min_speed_level"));
  }
  if (axis_info.params.count("max_speed_level"))
  {
    max_level = static_cast<int>(axis_info.params.at("max_speed_level"));
  }

  // Determine joystick axis value (use raw values for instant threshold detection)
  float value = 0.0;
  std::lock_guard<std::mutex> lock_axis(mutex_axis_);
  if (axis_info.joystick_axis == JoystickAxis::AX1)
  {
    value = axis_1_raw_;
  }
  else if (axis_info.joystick_axis == JoystickAxis::AX2)
  {
    value = axis_2_raw_;
  }

  value *= axis_info.direction * axis_info.scale;

  // Change speed level based on joystick movement
  if (value > speed_change_threshold_ && joy_prec_ <= speed_change_threshold_)
  {
    speed_level_ += 1;
    if (speed_level_ > max_level)
    {
      speed_level_ = max_level;
    }
  }
  else if (value < -speed_change_threshold_ && joy_prec_ >= -speed_change_threshold_)
  {
    speed_level_ -= 1;
    if (speed_level_ < min_level)
    {
      speed_level_ = min_level;
    }
  }

  joy_prec_ = value;
  speed_factor_ = speed_level_multiplier_ * speed_level_;
}

void DeviceJoystick::drink_(const AxisInfo& axis_info)
{
  if (locked_)
  {
    return;
  }
  // Determine joystick axis value
  float value = 0.0;

  value = read_axis_value_(axis_info);

  // Assign to the appropriate joint velocity component for drinking action
  cartesian_vel_.twist.angular.x = value;

  // Set control frame to end-effector for drinking action
  frame_id_.orientation_control_frame = 3;
}

void DeviceJoystick::gripper_(const AxisInfo& axis_info)
{
  if (locked_)
  {
    return;
  }
  // Determine joystick axis value
  float value = 0.0;

  value = read_axis_value_(axis_info);

  // Assign to gripper velocity; Device integrates it into the position command.
  gripper_vel_ = value;
}

void DeviceJoystick::complex_(const AxisInfo& axis_info)
{
  if (locked_)
  {
    return;
  }
  // Placeholder for complex behavior implementation
  float value = 0.0;

  complex_mode_ = true;

  value = read_axis_value_(axis_info);

  // Assign to appropriate complex mode variables
  if (axis_info.control_name == ControlName::COMPLEX_X)
  {
    v_x_ = value;
    cartesian_vel_.twist.linear.x = value;
  }
  if (axis_info.control_name == ControlName::COMPLEX_Y)
  {
    v_y_ = value;
    cartesian_vel_.twist.linear.y = value;
  }

  if (axis_info.params.count("rotation_speed_scale"))
  {
    rotation_speed_scale_ = static_cast<double>(axis_info.params.at("rotation_speed_scale"));
  }

  frame_id_.orientation_control_frame = 0;
}

void DeviceJoystick::trajectory_control_(const AxisInfo& axis_info)
{
  cartesian_vel_.twist.linear.z = read_axis_value_(axis_info);
}
}  // namespace input_device

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("device_joystick");
  input_device::DeviceJoystick device(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
