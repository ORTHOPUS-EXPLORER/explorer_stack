// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "explorer_command_controllers/explorer_custom_controller.hpp"

#include <algorithm>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <controller_interface/helpers.hpp>
#include <cstddef>
#include <hardware_interface/loaned_state_interface.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <vector>

// https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/controller_chaining.html
/* It goes:
  - on_init
  - on_configure
  ------ Those gets repeated several times, no idea why but it seems normal------------
  - command_interface_configuration
  - state_interface_configuration
  -------------------------------------------------------------------------------------
  - on_activate
  - command_interface_configuration
*/

namespace explorer_command_controllers
{
namespace
{
std::string build_interface_name(
  const std::string& joint_name, const orthopus::JointVariableType interface_joint_type)
{
  return joint_name + "/" + orthopus::JointVariableType_to_string(interface_joint_type);
}
}  // namespace
static const auto NaN = std::numeric_limits<double>::quiet_NaN();

controller_interface::CallbackReturn CustomController::on_init()
{
  if (
    rcutils_logging_set_logger_level(
      get_node()->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
    RCLCPP_DEBUG(get_node()->get_logger(), "Set LOG_LEVEL to Debug");

  RCLCPP_DEBUG(get_node()->get_logger(), "on_init");
  print_joint_srv_ = get_node()->create_service<std_srvs::srv::Empty>(
    "~/printJoints", [this](
                       const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>) { this->print_joints_(); });

  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception thrown during init stage with message: " << e.what() << std::endl;
    return controller_interface::CallbackReturn::ERROR;
  }

  // Config : check if joints are given
  params_ = param_listener_->get_params();
  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Config: check that size of joints / settings matches
  auto sz = params_.joints.size();
  if (sz != params_.settings.joints_map.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' and 'settings' parameters sizes do not match");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Pre built joints_ vector
  joints_.reserve(sz);
  for (const auto& [joint_name, settings] : params_.settings.joints_map)
  {
    joints_.emplace_back(joint_name, settings);
    RCLCPP_INFO(get_node()->get_logger(), "on_configure: Register '%s'", joint_name.c_str());
  }

  clock_ = get_node()->get_clock();

  return controller_interface::CallbackReturn::SUCCESS;
}

void CustomController::init_ros_subscribers_()
{
  // Inner lambda: check if all command data are finite number
  auto is_command_finite =
    [this](const std::vector<double>& data, orthopus::JointVariableType command_type)
  {
    if (!std::all_of(
          data.begin(), data.end(), [](const auto& value) { return std::isfinite(value); }))
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *clock_, 1000,
        "%s command received with at least one value that is NaN, message dropped.",
        orthopus::JointVariableType_to_string(command_type));
      return false;
    }
    return true;
  };
  // Inner lambda: check if command given contains at least one value for each joint controlled
  auto is_command_size_supported =
    [this](const std::vector<double>& data, orthopus::JointVariableType command_type)
  {
    if (data.size() < joints_.size())
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *clock_, 1000,
        "%s command received with size '%ld' but required at least size '%ld', message dropped.",
        orthopus::JointVariableType_to_string(command_type), data.size(), joints_.size());
      return false;
    };
    return true;
  };

  // Topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS().keep_last(1).best_effort();

  effort_commands_subscriber_ = get_node()->create_subscription<SubscriptionMsg>(
    "~/effort_commands", subscribers_qos,
    [this, is_command_finite, is_command_size_supported](const SubscriptionMsg::SharedPtr msg)
    {
      auto command_type = orthopus::JointVariableType::EFFORT;
      if (
        !is_command_finite(msg->data, command_type) ||
        !is_command_size_supported(msg->data, command_type))
        return;

      auto command = ControllerInputCommand{msg->data, get_node()->now()};
      effort_commands_buffer_rt_.writeFromNonRT(std::make_shared<ControllerInputCommand>(command));
    });
  position_commands_subscriber_ = get_node()->create_subscription<SubscriptionMsg>(
    "~/position_commands", subscribers_qos,
    [this, is_command_finite, is_command_size_supported](const SubscriptionMsg::SharedPtr msg)
    {
      auto command_type = orthopus::JointVariableType::POSITION;
      if (
        !is_command_finite(msg->data, command_type) ||
        !is_command_size_supported(msg->data, command_type))
        return;

      auto command = ControllerInputCommand{msg->data, get_node()->now()};
      position_commands_buffer_rt_.writeFromNonRT(
        std::make_shared<ControllerInputCommand>(command));
    });
  velocity_commands_subscriber_ = get_node()->create_subscription<SubscriptionMsg>(
    "~/velocity_commands", subscribers_qos,
    [this, is_command_finite, is_command_size_supported](const SubscriptionMsg::SharedPtr msg)
    {
      auto command_type = orthopus::JointVariableType::VELOCITY;
      if (
        !is_command_finite(msg->data, command_type) ||
        !is_command_size_supported(msg->data, command_type))
        return;

      auto command = ControllerInputCommand{msg->data, get_node()->now()};
      velocity_commands_buffer_rt_.writeFromNonRT(
        std::make_shared<ControllerInputCommand>(command));
    });
}

controller_interface::CallbackReturn CustomController::on_configure(const rclcpp_lifecycle::State&)
{
  init_ros_subscribers_();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CustomController::generate_interface_configuration_(
  const char* interface_type) const
{
  controller_interface::InterfaceConfiguration interface_config = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
  };

  for (const auto& j : joints_)
  {
    if (std::strcmp(interface_type, "state") == 0)
    {
      // Loop for each state interface for this joint
      for (const auto& [interface_joint_type, _] : j.joint_state_map)
      {
        auto interface_name = build_interface_name(j.name, interface_joint_type);
        interface_config.names.emplace_back(interface_name);
        RCLCPP_INFO(
          get_node()->get_logger(), "%s_interface_configuration: Will try to claim '%s'",
          interface_type, interface_name.c_str());
      }
    }
    else
    {
      // Loop for each state interface for this joint
      for (const auto& [interface_joint_type, _] : j.joint_command_map)
      {
        auto interface_name = build_interface_name(j.name, interface_joint_type);
        interface_config.names.emplace_back(interface_name);
        RCLCPP_INFO(
          get_node()->get_logger(), "%s_interface_configuration: Will try to claim '%s'",
          interface_type, interface_name.c_str());
      }
    }
  }

  return interface_config;
}

controller_interface::InterfaceConfiguration CustomController::command_interface_configuration()
  const
{
  return generate_interface_configuration_("command");
}

controller_interface::InterfaceConfiguration CustomController::state_interface_configuration() const
{
  return generate_interface_configuration_("state");
}

void CustomController::print_joints_() const
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Joints");
  for (const auto& j : joints_)
  {
    RCLCPP_DEBUG(get_node()->get_logger(), "- %s:", j.name.c_str());
    for (const auto& state_type_object : j.joint_state_map)
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "  State (%s) : %p, %f",
        JointVariableType_to_string(state_type_object.first),
        (void*)state_type_object.second.interface, state_type_object.second.state);
    }
    for (const auto& command_type_object : j.joint_command_map)
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "  Command (%s) : %p, %f",
        JointVariableType_to_string(command_type_object.first),
        (void*)command_type_object.second.interface, command_type_object.second.command);
    }
  }
}

// // Useful only when controllers are chained to this controller
// std::vector<hardware_interface::CommandInterface> CustomController::on_export_reference_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> reference_interfaces;

//   return {};
// }

// bool CustomController::on_set_chained_mode(bool)
// {
//   RCLCPP_DEBUG(get_node()->get_logger(), "on_set_chained_mode");
//   // Always accept switch to/from chained mode
//   return true;
// }

controller_interface::CallbackReturn CustomController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (auto& joint : joints_)
  {
    for (auto& joint_state_object : joint.joint_state_map)
    {
      if (joint_state_object.second.interface == nullptr)
      {
        auto wanted_interface_name = build_interface_name(joint.name, joint_state_object.first);
        // Assign corresponding interface if found
        if (
          auto interface = std::find_if(
            state_interfaces_.begin(), state_interfaces_.end(),
            [wanted_interface_name](const hardware_interface::LoanedStateInterface& interface)
            { return interface.get_name() == wanted_interface_name; });
          interface != state_interfaces_.end())
        {
          joint_state_object.second.interface = &(*interface);
        }
        else
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "State interface '%s' not found for joint '%s'",
            JointVariableType_to_string(joint_state_object.first), joint.name.c_str());
        }
      }
    }

    for (auto& joint_command_object : joint.joint_command_map)
    {
      if (joint_command_object.second.interface == nullptr)
      {
        auto wanted_interface_name = build_interface_name(joint.name, joint_command_object.first);
        // Assign corresponding interface if found
        if (
          auto interface = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
            [wanted_interface_name](const hardware_interface::LoanedCommandInterface& interface)
            { return interface.get_name() == wanted_interface_name; });
          interface != command_interfaces_.end())
        {
          joint_command_object.second.interface = &(*interface);
        }
        else
        {
          RCLCPP_ERROR(
            get_node()->get_logger(), "Command interface '%s' not found for joint '%s'",
            JointVariableType_to_string(joint_command_object.first), joint.name.c_str());
        }
      }
    }
  }

  // reset command buffers if a command came through callback when controller was inactive
  effort_commands_buffer_rt_.reset();
  position_commands_buffer_rt_.reset();
  velocity_commands_buffer_rt_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CustomController::on_deactivate(const rclcpp_lifecycle::State&)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// controller_interface::return_type CustomController::update_reference_from_subscribers(
//   const rclcpp::Time&, const rclcpp::Duration&)
// {
//   return controller_interface::return_type::OK;
// }

controller_interface::return_type CustomController::update(
  const rclcpp::Time&, const rclcpp::Duration&)
{
  const auto clock = *(get_node()->get_clock());

  // Read input commands
  auto effort_command = effort_commands_buffer_rt_.readFromRT();
  auto position_command = position_commands_buffer_rt_.readFromRT();
  auto velocity_command = velocity_commands_buffer_rt_.readFromRT();

  size_t index = 0;
  for (auto& joint : joints_)
  {
    // Applies input(s) command(s)
    // apply_joint_input_command_(joint, index, orthopus::JointVariableType::EFFORT, effort_command);
    apply_joint_input_command_(
      joint, index, orthopus::JointVariableType::POSITION, position_command);
    // apply_joint_input_command_(
    //   joint, index, orthopus::JointVariableType::VELOCITY, velocity_command);
    index++;

    // Write commands
    write_effort_(joint);
    write_position_(joint);
    write_velocity_(joint);
  }
  return controller_interface::return_type::ERROR;
}

void CustomController::apply_joint_input_command_(
  ControllerJoint& joint, size_t joint_index, orthopus::JointVariableType command_type,
  const std::shared_ptr<ControllerInputCommand>* input_command)
{
  const std::shared_ptr<const rclcpp_lifecycle::LifecycleNode> node = get_node();

  if (input_command && *input_command)
  {
    try
    {
      joint.joint_command_map.at(command_type).command = (*input_command)->data[joint_index];
    }
    catch (const std::out_of_range& ex)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *clock_, 1000,
        "Couldn't find joint interface type '%s' for joint '%ld'",
        JointVariableType_to_string(command_type), joint_index);
    }
  }
}

bool CustomController::is_command_ready_to_be_written_(
  const ControllerJoint& joint, orthopus::JointVariableType command_type)
{
  ControllerJointControl joint_command_control;
  try
  {
    joint_command_control = joint.joint_command_map.at(command_type);
  }
  catch (std::out_of_range& exception)
  {
    return false;
  }

  auto command_is_NaN = std::isnan(joint_command_control.command);
  if (!joint_command_control.interface || command_is_NaN)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *clock_, 1000,
      "Joint %s interface null (%d) or command is NaN (%d) for joint '%s'",
      orthopus::JointVariableType_to_string(command_type), (bool)!joint_command_control.interface,
      command_is_NaN, joint.name.c_str());
    return false;
  };
  return true;
}

void CustomController::write_effort_(ControllerJoint& joint)
{
  auto command_type = orthopus::JointVariableType::EFFORT;
  if (!is_command_ready_to_be_written_(joint, command_type)) return;
  auto joint_effort_command_control = joint.joint_command_map[command_type];

  joint_effort_command_control.interface->set_value(joint_effort_command_control.command);
  joint_effort_command_control.previous_command = joint_effort_command_control.command;
}

void CustomController::write_position_(ControllerJoint& joint)
{
  auto command_type = orthopus::JointVariableType::POSITION;
  if (!is_command_ready_to_be_written_(joint, command_type)) return;

  auto joint_position_command_control = joint.joint_command_map[command_type];

  if (joint.settings.enable_limits)
  {
    // Ensure 'v' stays in range [cmd_min, cmd_max]
    joint_position_command_control.command = std::clamp(
      joint_position_command_control.command, joint.settings.cmd_min, joint.settings.cmd_max);

    const auto step_max = joint.settings.step_max;
    {
      auto diff =
        joint_position_command_control.command - joint_position_command_control.previous_command;
      if (std::abs(diff) > step_max)
      {
        // Returns the value settings.step_max but with the sign of diff.
        auto ndiff = std::copysign(joint.settings.step_max, diff);
        joint_position_command_control.command =
          joint_position_command_control.previous_command + ndiff;
      }
    }
  }

  joint_position_command_control.interface->set_value(joint_position_command_control.command);
  joint_position_command_control.previous_command = joint_position_command_control.command;
}

void CustomController::write_velocity_(ControllerJoint& joint)
{
  auto command_type = orthopus::JointVariableType::VELOCITY;
  if (!is_command_ready_to_be_written_(joint, command_type)) return;
  auto joint_velocity_command_control = joint.joint_command_map[command_type];

  joint_velocity_command_control.interface->set_value(joint_velocity_command_control.command);
  joint_velocity_command_control.previous_command = joint_velocity_command_control.command;
}

}  // namespace explorer_command_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  explorer_command_controllers::CustomController, controller_interface::ControllerInterface)