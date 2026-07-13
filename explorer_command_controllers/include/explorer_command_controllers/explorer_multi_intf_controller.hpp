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

#ifndef EXPLORER_COMMAND_CONTROLLERS__MULTI_INTERFACE_CONTROLLER_HPP_
#define EXPLORER_COMMAND_CONTROLLERS__MULTI_INTERFACE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "explorer_command_controllers/visibility_control.h"
#include "explorer_multi_intf_controller_parameters.hpp"  // generated
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"
namespace explorer_command_controllers
{

class MultiIntfController : public controller_interface::ChainableControllerInterface
{
public:
  using CmdType = std_msgs::msg::Float64MultiArray;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  MultiIntfController();

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  //EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  //controller_interface::CallbackReturn on_cleanup(
  //  const rclcpp_lifecycle::State & previous_state) override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

  using Params = explorer_multi_intf_controller::Params;
  using ParamListener = explorer_multi_intf_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  rclcpp::Subscription<CmdType>::SharedPtr refs_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> refs_rt_b_;

private:
  using joint_t = struct
  {
    const std::string& name;
    const struct Params::Settings::MapJoints& settings;
    hardware_interface::LoanedStateInterface* state_if;
    double state_v;
    hardware_interface::LoanedCommandInterface* cmd_if;
    double cmd_v, cmd_v_p;
    hardware_interface::LoanedCommandInterface* ref_if;
    double ref_v;
  };

  // callback for topic interface
  EXPLORER_COMMAND_CONTROLLERS_LOCAL
  void reference_callback(const std::shared_ptr<CmdType> msg);
  void print_joints();

  std::vector<joint_t> joints_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr print_joint_srv_;
};

}  // namespace explorer_command_controllers

#endif  // EXPLORER_COMMAND_CONTROLLERS__MULTI_INTERFACE_CONTROLLER_HPP_
