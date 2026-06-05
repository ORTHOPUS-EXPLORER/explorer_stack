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

#ifndef EXPLORER_COMMAND_CONTROLLERS__CUSTOM_CONTROLLER_HPP_
#define EXPLORER_COMMAND_CONTROLLERS__CUSTOM_CONTROLLER_HPP_

#include <map>
#include <memory>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "explorer_command_controllers/visibility_control.h"
#include "explorer_custom_controller_parameters.hpp"  // generated
#include "orthopus_vesc/common.hpp"
#include "orthopus_vesc/target.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"

namespace explorer_command_controllers
{
class CustomController : public controller_interface::ControllerInterface
{
public:
  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  CustomController() = default;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  // EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  // controller_interface::return_type update_reference_from_subscribers(
  //   const rclcpp::Time& time, const rclcpp::Duration& period) override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // override methods from ChainableControllerInterface
  // std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  // bool on_set_chained_mode(bool chained_mode) override;

private:
  using Params = explorer_custom_controller::Params;
  using ParamListener = explorer_custom_controller::ParamListener;
  using SubscriptionMsg = std_msgs::msg::Float64MultiArray;

  struct ControllerInputCommand
  {
    std::vector<double> data;
    rclcpp::Time timestamp;
  };

  struct ControllerJointControl
  {
    hardware_interface::LoanedCommandInterface* interface = nullptr;
    double command = std::numeric_limits<double>::quiet_NaN();
    double previous_command = std::numeric_limits<double>::quiet_NaN();
  };

  struct ControllerJointState
  {
    hardware_interface::LoanedStateInterface* interface = nullptr;
    double state = std::numeric_limits<double>::quiet_NaN();
  };

  struct ControllerJoint
  {
    ControllerJoint(
      const std::string& name, const struct Params::Settings::MapJoints& settings,
      orthopus::JointVariableType mode, bool simulation_enabled)
    : name(name), settings(settings), mode(mode)
    {
      if (simulation_enabled)
      {
        auto command_interface_mode = mode;
        // Only claim Position command interface in simulation mode (cannot claim both 3 interfaces with gazebo)
        if (mode == orthopus::JointVariableType::EFFORT)
        {
          command_interface_mode = orthopus::JointVariableType::POSITION;
        }
        joint_command_map[command_interface_mode] = ControllerJointControl();
        joint_state_map[command_interface_mode] = ControllerJointState();
        return;
      }

      for (const auto& command_interface_name : settings.command_interface_names)
      {
        auto command_interface_type =
          orthopus::JointVariableType_from_string(command_interface_name);
        joint_command_map[command_interface_type] = ControllerJointControl();
      }

      for (const auto& state_interface_name : settings.state_interface_names)
      {
        auto state_interface_type = orthopus::JointVariableType_from_string(state_interface_name);
        joint_state_map[state_interface_type] = ControllerJointState();
      }
    }

    const std::string& name;
    const struct Params::Settings::MapJoints& settings;
    orthopus::JointVariableType mode;
    std::map<orthopus::JointVariableType, ControllerJointControl> joint_command_map;
    std::map<orthopus::JointVariableType, ControllerJointState> joint_state_map;
  };

  bool set_joint_mode_(const std::string&, const std::string&) const;
  bool set_impedance_config_(const std::string& joint_name, double damping, double stiffness) const;
  void init_ros_subscribers_();
  bool apply_joint_input_command_(
    ControllerJoint&, size_t, orthopus::JointVariableType,
    const std::shared_ptr<ControllerInputCommand>*);
  std::string build_interface_name_(const std::string&, orthopus::JointVariableType) const;
  void write_effort_(ControllerJoint&);
  void write_position_(ControllerJoint&);
  void write_velocity_(ControllerJoint&);
  bool is_command_ready_to_be_written_(const ControllerJoint&, orthopus::JointVariableType);

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Subscribers objects
  rclcpp::Subscription<SubscriptionMsg>::SharedPtr effort_commands_subscriber_;
  rclcpp::Subscription<SubscriptionMsg>::SharedPtr position_commands_subscriber_;
  rclcpp::Subscription<SubscriptionMsg>::SharedPtr velocity_commands_subscriber_;

  // Real time buffers : store data coming from topics
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerInputCommand>>
    effort_commands_buffer_rt_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerInputCommand>>
    position_commands_buffer_rt_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerInputCommand>>
    velocity_commands_buffer_rt_;

  // Associate interface / command for each joint
  std::vector<ControllerJoint> joints_;
  // ROS node clock
  rclcpp::Clock::SharedPtr clock_;

  // DEBUG
  void print_joints_() const;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr print_joint_srv_;
};

}  // namespace explorer_command_controllers

#endif  // EXPLORER_COMMAND_CONTROLLERS__CUSTOM_CONTROLLER_HPP_
