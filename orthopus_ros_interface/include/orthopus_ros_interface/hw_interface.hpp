#ifndef ORTHOPUS_ROS_HW_INTERFACE_HPP_
#define ORTHOPUS_ROS_HW_INTERFACE_HPP_

#include "orthopus_ros_interface/visibility_control.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "control_msgs/msg/dynamic_joint_state.hpp"
namespace orthopus_ros_interface
{

class HwInterface;
class HwInterfaceComm
  : public rclcpp::Node
{
public:
  HwInterfaceComm(HwInterface& parent);

private:
  HwInterface& _parent;

  // From ros_explorer_bridge
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr                  _expl_meas_sub;
  //realtime_tools::RealtimeBuffer<std::shared_ptr<control_msgs::msg::DynamicJointState>>  _expl_meas_msg_buf;
  //std::shared_ptr<control_msgs::msg::DynamicJointState>                                  _expl_meas_msg;

  // To ros_explorer_bridge
  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr                  _expl_ref_pub;
  rclcpp::Time                                                                        _expl_ref_pub_time;
  control_msgs::msg::DynamicJointState                                                _expl_ref_msg;
  rclcpp::TimerBase::SharedPtr                                                        _expl_ref_pub_tmr;

};

class HwInterface
  : public hardware_interface::ActuatorInterface
{
public:
  HwInterface() = default;
  virtual ~HwInterface();

  RCLCPP_SHARED_PTR_DEFINITIONS(HwInterface)

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ORTHOPUS_ROS_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type  prepare_command_mode_switch(const std::vector<std::string>& start_if, const std::vector<std::string>& stop_if) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type  perform_command_mode_switch(const std::vector<std::string>& start_if, const std::vector<std::string>& stop_if) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ORTHOPUS_ROS_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::string               _name;

  typedef struct
  {
    double pos;
    double vel;
    double acc;
    double eff;
  } hw_state_t;

  typedef struct
  {
    double            pos, ppos;
    std::atomic<bool> nwpos;
    double            vel, pvel;
    std::atomic<bool> nwvel;
    double            acc, pacc;
    std::atomic<bool> nwacc;
    double            eff, peff;
    std::atomic<bool> nweff;
  } hw_ref_t;

  typedef struct
  {
    std::string name;
    bool        enable;
    hw_state_t  state;
    hw_ref_t    ref;
  } hw_joint_t;

  static constexpr size_t N_JOINTS = 2;
  std::array<hw_joint_t, N_JOINTS> _hw_joints;

  //std::vector<hardware_interface::CommandInterface> _command_interfaces;
  std::vector<hardware_interface::StateInterface>   _state_interfaces;

  std::shared_ptr<HwInterfaceComm> _comm;
  rclcpp::executors::SingleThreadedExecutor _executor;  //Executor needed to subscriber
  std::thread _comm_th;
};

}  // namespace orthopus_ros_interface

#endif  // ORTHOPUS_ROS_HW_INTERFACE_HPP_
