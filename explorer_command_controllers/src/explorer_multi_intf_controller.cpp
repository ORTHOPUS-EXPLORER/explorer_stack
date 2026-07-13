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

#include "explorer_command_controllers/explorer_multi_intf_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

// https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/controller_chaining.html
/* It goes:
  - on_init
  - on_configure
  - command_interface_configuration
  - state_interface_configuration
  - command_interface_configuration
  - command_interface_configuration
  - command_interface_configuration
  - state_interface_configuration
  - command_interface_configuration
  - state_interface_configuration
  - on_activate
  - command_interface_configuration
*/

namespace explorer_command_controllers
{
static const auto NaN = std::numeric_limits<double>::quiet_NaN();

MultiIntfController::MultiIntfController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn MultiIntfController::on_init()
{
  const auto& log = get_node()->get_logger();
  if (rcutils_logging_set_logger_level(log.get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
    RCLCPP_DEBUG(log, "Set LOG_LEVEL to Debug");

  RCLCPP_DEBUG(log, "on_init");
  print_joint_srv_ = get_node()->create_service<std_srvs::srv::Empty>(
    "~/printJoints", [this](
                       const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>) { this->print_joints(); });

  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception thrown during init stage with message: " << e.what() << std::endl;
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiIntfController::on_configure(
  const rclcpp_lifecycle::State&)
{
  const auto& log = get_node()->get_logger();
  // if(previous_state != rclcpp_lifecycle::State::Unconfigured)
  //{
  //   RCLCPP_ERROR(get_node()->get_logger(), "on_configure:: Already
  //   configured"); return controller_interface::CallbackReturn::ERROR;
  // }
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(log, "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  auto sz = params_.joints.size();
  if (sz != params_.settings.joints_map.size())
  {
    RCLCPP_ERROR(log, "'joints' and 'settings' parameters sizes do not match");
    return controller_interface::CallbackReturn::ERROR;
  }

  joints_.reserve(sz);
  for (const auto& [joint_name, settings] : params_.settings.joints_map)
  {
    joints_.emplace_back(
      joint_t{joint_name, settings, nullptr, 0.0, nullptr, 0.0, 0.0, nullptr, 0.0});
    RCLCPP_INFO(log, "on_configure: Register '%s'", joint_name.c_str());
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS().keep_last(1).best_effort();

  refs_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/refs", subscribers_qos,
    [this](const CmdType::SharedPtr msg) { refs_rt_b_.writeFromNonRT(msg); });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MultiIntfController::command_interface_configuration()
  const
{
  const auto& log = get_node()->get_logger();
  // RCLCPP_DEBUG(log, "command_interface_configuration");
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& j : joints_)
  {
    const auto intf_name = j.name + "/" + j.settings.command;
    command_interfaces_config.names.emplace_back(intf_name);
    // RCLCPP_INFO(log, "command_interface_configuration: Will try to claim
    // '%s'", intf_name.c_str());
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MultiIntfController::state_interface_configuration()
  const
{
  const auto& log = get_node()->get_logger();
  // RCLCPP_DEBUG(log, "state_interface_configuration");
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& j : joints_)
  {
    const auto intf_name = j.name + "/" + j.settings.state;
    state_interfaces_config.names.emplace_back(intf_name);
    // RCLCPP_INFO(log, "state_interface_configuration: Will try to attach to
    // '%s'", intf_name.c_str());
  }
  return state_interfaces_config;
}

void MultiIntfController::print_joints()
{
  const auto& log = get_node()->get_logger();
  RCLCPP_DEBUG(log, "Joints");
  for (const auto& j : joints_)
  {
    RCLCPP_DEBUG(log, "- %s:", j.name.c_str());
    RCLCPP_DEBUG(log, "  State  : %p, %f", (void*)j.state_if, j.state_v);
    RCLCPP_DEBUG(log, "  Command: %p, %f", (void*)j.cmd_if, j.cmd_v);
    RCLCPP_DEBUG(log, "  Ref    : %p, %f", (void*)j.ref_if, j.ref_v);
  }
}

std::vector<hardware_interface::CommandInterface>
MultiIntfController::on_export_reference_interfaces()
{
  const auto& log = get_node()->get_logger();
  RCLCPP_DEBUG(log, "on_export_reference_interfaces");
  size_t new_sz = 1;
  reference_interfaces_.resize(new_sz, NaN);

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(new_sz);

  size_t index = 0;
  reference_interfaces.emplace_back(
    get_node()->get_name(), "mynode/ctrl", &reference_interfaces_[index]);

  return reference_interfaces;
}

bool MultiIntfController::on_set_chained_mode(bool)
{
  const auto& log = get_node()->get_logger();
  RCLCPP_DEBUG(log, "on_set_chained_mode");
  // Always accept switch to/from chained mode
  return true;
}

controller_interface::CallbackReturn MultiIntfController::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto& log = get_node()->get_logger();
  RCLCPP_DEBUG(log, "on_activate");
  for (auto& j : joints_)
  {
    j.state_v = NaN;
    if (j.state_if == nullptr)
    {
      const auto intf_name = j.name + "/" + j.settings.state;
      for (auto& intf : state_interfaces_)
      {
        if (intf.get_name() == intf_name)
        {
          j.state_if = &intf;
          break;
        }
        // throw error !
      }
    }
    j.cmd_v = NaN;
    if (j.cmd_if == nullptr)
    {
      const auto intf_name = j.name + "/" + j.settings.command;
      for (auto& intf : command_interfaces_)
      {
        if (intf.get_name() == intf_name)
        {
          j.cmd_if = &intf;
          break;
        }
        // throw error !
      }
    }
  }
  // printJoints();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiIntfController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  const auto& log = get_node()->get_logger();
  RCLCPP_DEBUG(log, "on_deactivate");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MultiIntfController::update_reference_from_subscribers(
  const rclcpp::Time&, const rclcpp::Duration&)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type MultiIntfController::update_and_write_commands(
  const rclcpp::Time&, const rclcpp::Duration&)
{
  const auto& log = get_node()->get_logger();
  const auto clock = *(get_node()->get_clock());

  // Read state
  {
    for (auto& j : joints_)
    {
      if (!j.state_if) return controller_interface::return_type::ERROR;

      j.state_v = j.state_if->get_value();
      // First loop probably, init ref
      if (std::isnan(j.cmd_v))
      {
        auto v = j.settings.cmd_init_from_state ? j.state_v : j.settings.cmd_init_value;
        RCLCPP_WARN(log, "[%s] Ref is NaN. Init with %f", j.name.c_str(), v);
        j.cmd_v = v;
        j.cmd_v_p = j.cmd_v;
      }
    }
  }

  auto refs = refs_rt_b_.readFromRT();
  if (refs && (*refs))  // external command received
  {
    if ((*refs)->data.size() != joints_.size())
    {
      RCLCPP_ERROR_THROTTLE(
        log, clock, 1000, "refs size (%zu) does not match number of joints (%zu)",
        (*refs)->data.size(), joints_.size());
      return controller_interface::return_type::ERROR;
    }

    size_t i = 0;
    for (auto& j : joints_)
    {
      auto v = (*refs)->data[i++] * j.settings.ref_mult;
      v = j.state_v;
      j.cmd_v = v;
    }
  }

  // Write commands
  for (auto& j : joints_)
  {
    auto v = j.cmd_v;
    if (j.cmd_if && !std::isnan(v))
    {
      if (j.settings.enable_limits)
      {
        // v = std::min(std::max(v, j.settings.cmd_min), j.settings.cmd_max);
        if (v > j.settings.cmd_max)
        {
          // RCLCPP_ERROR_THROTTLE(log, clock, 100,
          //    "[%s] Ref is above cmd_max: %f > %f. Capping to
          //    cmd_max.",j.name.c_str(), v, j.settings.cmd_max);
          v = j.cmd_v = j.settings.cmd_max;
        }
        else if (v < j.settings.cmd_min)
        {
          // RCLCPP_ERROR_THROTTLE(log, clock, 100,
          //  "[%s] Ref is below cmd_min %f < %f. Capping to
          //  cmd_min.",j.name.c_str(), v, j.settings.cmd_min);
          v = j.cmd_v = j.settings.cmd_min;
        }

        const auto step_max = j.settings.step_max;
        {
          auto diff = v - j.cmd_v_p;
          if (std::abs(diff) > step_max)
          {
            auto ndiff = std::copysign(j.settings.step_max, diff);
            // RCLCPP_ERROR_THROTTLE(log, clock, 100,
            //  "[%s] CMD step is over step_max: %f > %f. Capping to step_max.
            //  cmd_p: %f cmd_v: %f, new_cmd_v %f",j.name.c_str(),
            //  std::abs(diff), j.settings.step_max, j.cmd_v_p, j.cmd_v,
            //  j.cmd_v_p + ndiff);
            v = j.cmd_v_p + ndiff;
          }
        }
      }
      j.cmd_if->set_value(v);
      j.cmd_v_p = v;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace explorer_command_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  explorer_command_controllers::MultiIntfController,
  controller_interface::ChainableControllerInterface)
