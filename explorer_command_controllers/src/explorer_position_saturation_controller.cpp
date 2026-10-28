#include <algorithm>
#include <cmath>
#include <explorer_command_controllers/explorer_position_saturation_controller.hpp>
#include <limits>
#include <memory>

// Chain: qontrol_explorer -> PositionSaturationController -> CustomController
// or:    output_integrator -(topic)-> PositionSaturationController -> CustomController

namespace explorer_command_controllers
{
controller_interface::CallbackReturn PositionSaturationController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception thrown during init stage with message: " << e.what() << std::endl;
    return controller_interface::CallbackReturn::ERROR;
  }

  params_ = param_listener_->get_params();
  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  reference_interfaces_.resize(params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  saturation_ = std::make_unique<PositionSaturation>(params_.joints.size(), saturation_config_());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionSaturationController::on_configure(
  const rclcpp_lifecycle::State&)
{
  params_ = param_listener_->get_params();
  saturation_->set_config(saturation_config_());

  position_commands_subscriber_ = get_node()->create_subscription<SubscriptionMsg>(
    "~/position/commands", rclcpp::SystemDefaultsQoS().keep_last(1).best_effort(),
    [this](const SubscriptionMsg::ConstSharedPtr& msg)
    {
      if (is_chained_.load()) return;

      if (msg->data.size() < params_.joints.size())
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Position command received with size '%ld' but required at least size '%ld', message "
          "dropped.",
          msg->data.size(), params_.joints.size());
        return;
      }
      if (!std::all_of(
            msg->data.begin(), msg->data.end(), [](double value) { return std::isfinite(value); }))
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Position command received with at least one value that is NaN, message dropped.");
        return;
      }
      position_commands_buffer_rt_.writeFromNonRT(msg);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PositionSaturationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration interface_config = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
  };
  for (const auto& joint_name : params_.joints)
  {
    interface_config.names.emplace_back(
      params_.chained_controller_name + "/" + joint_name + "/" + params_.command_interface_name);
  }
  return interface_config;
}

controller_interface::InterfaceConfiguration
PositionSaturationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration interface_config = {
    controller_interface::interface_configuration_type::INDIVIDUAL,
  };
  for (const auto& joint_name : params_.joints)
  {
    interface_config.names.emplace_back(joint_name + "/" + params_.state_interface_name);
  }
  return interface_config;
}

std::vector<hardware_interface::CommandInterface>
PositionSaturationController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> exported_interfaces;
  exported_interfaces.reserve(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); i++)
  {
    exported_interfaces.emplace_back(
      get_node()->get_name(), params_.joints[i] + "/" + params_.reference_interface_name,
      &reference_interfaces_[i]);
  }
  return exported_interfaces;
}

controller_interface::CallbackReturn PositionSaturationController::on_activate(
  const rclcpp_lifecycle::State&)
{
  reset_();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionSaturationController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  reset_();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PositionSaturationController::on_set_chained_mode(bool chained_mode)
{
  is_chained_.store(chained_mode);
  // Input source changes: previous reference and offsets are meaningless
  reset_();
  return true;
}

controller_interface::return_type PositionSaturationController::update_reference_from_subscribers(
  const rclcpp::Time&, const rclcpp::Duration&)
{
  auto command = *position_commands_buffer_rt_.readFromRT();
  if (command)
  {
    std::copy_n(command->data.begin(), reference_interfaces_.size(), reference_interfaces_.begin());
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type PositionSaturationController::update_and_write_commands(
  const rclcpp::Time&, const rclcpp::Duration&)
{
  // Saturation parameters can be changed live (threshold validated by the parameter library)
  if (param_listener_->try_get_params(params_))
  {
    saturation_->set_config(saturation_config_());
  }

  bool write_failed = false;
  for (size_t i = 0; i < params_.joints.size(); i++)
  {
    // A NaN command (no reference yet) is ignored by CustomController
    const double command =
      saturation_->compute(i, reference_interfaces_[i], state_interfaces_[i].get_optional());

    if (!command_interfaces_[i].set_value(command))
    {
      write_failed = true;
    }
  }

  if (write_failed)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Unable to write at least one command interface this cycle.");
  }
  return controller_interface::return_type::OK;
}

void PositionSaturationController::reset_()
{
  // Ignore last topic command, wait for a new one
  position_commands_buffer_rt_.reset();
  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());
  saturation_->reset();
}

PositionSaturationConfig PositionSaturationController::saturation_config_() const
{
  PositionSaturationConfig config;
  config.enable = params_.position_error_saturation_enable;
  config.threshold = params_.position_error_saturation_threshold;
  config.persistent = params_.position_error_saturation_persistent;
  return config;
}

}  // namespace explorer_command_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  explorer_command_controllers::PositionSaturationController,
  controller_interface::ChainableControllerInterface)
