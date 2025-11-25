#pragma once

#include "orthopus_vesc_interface/visibility_control.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "control_msgs/msg/dynamic_joint_state.hpp"

#include "orthopus_vesc_interfaces/srv/dev.hpp"
#include "orthopus_vesc_interfaces/srv/cmd.hpp"
#include "orthopus_vesc_interfaces/srv/set_mode.hpp"

#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include "orthopus_vesc_interfaces/msg/state.hpp"

#include "orthopus_vesc/host.hpp"

namespace orthopus_ros
{
class VESCInterface
    : public hardware_interface::ActuatorInterface
{
public:
  VESCInterface() = default;
  virtual ~VESCInterface() = default;

  RCLCPP_SHARED_PTR_DEFINITIONS(VESCInterface)

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ORTHOPUS_ROS_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_if, const std::vector<std::string> &stop_if) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start_if, const std::vector<std::string> &stop_if) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  std::string _name;
  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::Service<orthopus_vesc_interfaces::srv::Dev>::SharedPtr _dev_srv;
  rclcpp::Service<orthopus_vesc_interfaces::srv::SetMode>::SharedPtr  _set_mode_srv;
  rclcpp::Service<orthopus_vesc_interfaces::srv::Cmd>::SharedPtr  _cmd_srv;
  rclcpp::Publisher<orthopus_vesc_interfaces::msg::State>::SharedPtr  _state_pub;
  std::unique_ptr<realtime_tools::RealtimePublisher<orthopus_vesc_interfaces::msg::State>> _state_rtpub;
    
  typedef struct
  {
    rclcpp::Time time;
    std::string str;
  } printMsg_t;
  rclcpp::Time            _print_buf_start{0};
  rclcpp::Duration        _print_buf_duration = rclcpp::Duration::from_seconds(0);
  std::vector<printMsg_t> _print_buf;

  //std::vector<hardware_interface::CommandInterface> _command_interfaces;
  std::vector<hardware_interface::StateInterface> _state_interfaces;

  std::shared_ptr<orthopus::VESCHost>   _vesc_host{nullptr};
  std::shared_ptr<orthopus::VESCTarget> _vesc_dev{nullptr};

  void printHardwareInfo(const hardware_interface::HardwareInfo& info);
  void printComponentInfo(const hardware_interface::ComponentInfo& info, size_t i);
  void printInterfaceInfo(const hardware_interface::InterfaceInfo& info, size_t i);
  void printTransmissionInfo(const hardware_interface::TransmissionInfo& info, size_t i);
  void printParameters(const std::unordered_map<std::string, std::string>& params);

};

} // namespace orthopus_ros
