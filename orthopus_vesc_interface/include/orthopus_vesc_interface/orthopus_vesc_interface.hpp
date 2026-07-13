#pragma once

#include <realtime_tools/realtime_publisher.h>

#include <memory>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "orthopus_vesc/host.hpp"
#include "orthopus_vesc_interface/visibility_control.h"
#include "orthopus_vesc_interfaces/msg/config.hpp"
#include "orthopus_vesc_interfaces/msg/state.hpp"
#include "orthopus_vesc_interfaces/srv/cmd.hpp"
#include "orthopus_vesc_interfaces/srv/dev.hpp"
#include "orthopus_vesc_interfaces/srv/set_mode.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace orthopus_ros
{
class VESCInterface : public hardware_interface::ActuatorInterface
{
public:
  VESCInterface() = default;
  virtual ~VESCInterface() = default;

  RCLCPP_SHARED_PTR_DEFINITIONS(VESCInterface)

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ORTHOPUS_ROS_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string>& start_if, const std::vector<std::string>& stop_if) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_if, const std::vector<std::string>& stop_if) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  struct printMsg_t
  {
    rclcpp::Time time;
    std::string str;
  };

private:
  void print_hardware_info_(const hardware_interface::HardwareInfo& info);
  void print_component_info_(const hardware_interface::ComponentInfo& info, size_t i);
  void print_interface_info_(const hardware_interface::InterfaceInfo& info, size_t i);
  void print_transmission_info_(const hardware_interface::TransmissionInfo& info, size_t i);
  void print_parameters_(const std::unordered_map<std::string, std::string>& params);
  void callback_config_(const orthopus_vesc_interfaces::msg::Config& msg);
  CallbackReturn wait_can_data_();
  void init_refs_();

  rclcpp::Time print_buf_start_{0};
  rclcpp::Duration print_buf_duration_ = rclcpp::Duration::from_seconds(0);
  std::vector<printMsg_t> print_buf_;

  //std::vector<hardware_interface::CommandInterface> _command_interfaces;
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  std::shared_ptr<orthopus::VESCHost> vesc_host_{nullptr};
  std::shared_ptr<orthopus::VESCTarget> vesc_dev_{nullptr};
  std::string default_mode_{
    "off"};  // Default mode to apply on activation (backward compatible: "off" if not specified)
  std::string name_;
  bool is_virtual_can_used_ = false;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<orthopus_vesc_interfaces::srv::Dev>::SharedPtr dev_srv_;
  rclcpp::Service<orthopus_vesc_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<orthopus_vesc_interfaces::srv::Cmd>::SharedPtr cmd_srv_;
  rclcpp::Publisher<orthopus_vesc_interfaces::msg::State>::SharedPtr state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<orthopus_vesc_interfaces::msg::State>>
    state_rtpub_;
  rclcpp::Subscription<orthopus_vesc_interfaces::msg::Config>::SharedPtr config_sub_;
};

}  // namespace orthopus_ros
