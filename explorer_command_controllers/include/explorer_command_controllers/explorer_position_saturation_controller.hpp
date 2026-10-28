#ifndef EXPLORER_COMMAND_CONTROLLERS__POSITION_SATURATION_CONTROLLER_HPP_
#define EXPLORER_COMMAND_CONTROLLERS__POSITION_SATURATION_CONTROLLER_HPP_

#include <atomic>
#include <controller_interface/chainable_controller_interface.hpp>
#include <explorer_command_controllers/explorer_position_saturation_controller_parameters.hpp>  // generated
#include <memory>
#include <realtime_tools/realtime_buffer.hpp>
#include <string>
#include <vector>

#include "explorer_command_controllers/position_saturation.hpp"
#include "explorer_command_controllers/visibility_control.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace explorer_command_controllers
{
/**
 * Chainable controller placed between a position source and CustomController. The source is
 * either an upstream chained controller (qontrol_controller) writing the reference interfaces, or
 * a topic (~/position/commands, e.g. output_integrator) when not in chained mode.
 *
 * It limits the distance between the position command and the measured joint position, which
 * happens when the robot is moved by hand while in impedance ("soft") mode. The saturation itself
 * is done by PositionSaturation; this class only moves values between ROS interfaces and it.
 */
class PositionSaturationController : public controller_interface::ChainableControllerInterface
{
public:
  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  PositionSaturationController() = default;

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

  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;
  EXPLORER_COMMAND_CONTROLLERS_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  bool on_set_chained_mode(bool chained_mode) override;

private:
  using Params = explorer_position_saturation_controller::Params;
  using ParamListener = explorer_position_saturation_controller::ParamListener;
  using SubscriptionMsg = std_msgs::msg::Float64MultiArray;

  void reset_();
  [[nodiscard]] PositionSaturationConfig saturation_config_() const;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::unique_ptr<PositionSaturation> saturation_;

  // Position commands from topic, only used when not in chained mode
  rclcpp::Subscription<SubscriptionMsg>::SharedPtr position_commands_subscriber_;
  realtime_tools::RealtimeBuffer<SubscriptionMsg::ConstSharedPtr> position_commands_buffer_rt_;
  std::atomic<bool> is_chained_{false};
};

}  // namespace explorer_command_controllers

#endif  // EXPLORER_COMMAND_CONTROLLERS__POSITION_SATURATION_CONTROLLER_HPP_
