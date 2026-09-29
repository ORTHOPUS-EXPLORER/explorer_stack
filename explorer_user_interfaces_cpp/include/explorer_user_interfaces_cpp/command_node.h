#ifndef EXPLORER_USER_INTERFACES_CPP_COMMAND_NODE_H
#define EXPLORER_USER_INTERFACES_CPP_COMMAND_NODE_H

#include <ctime>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "atomic"
#include "explorer_joint_utils/joint_mode_resolver.h"
#include "explorer_msgs/srv/set_speed_level.hpp"
#include "explorer_user_interfaces_cpp/controller_manager_wrapper.h"
#include "explorer_user_interfaces_cpp/trajectory_manager.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono;

namespace space_control
{
/**
 * \brief Robot-side command executor
 *
 * Owns the trajectory manager and deals with the controller switch, driven by the inputs:
 *   - [topic] command_node/trajectory/velocity/commands moves along the trajectory (hold-to-run,
 *     falls back to hold when not refreshed); a non-zero value switches to the joint_trajectory_controller.
 *   - [topic] command_node/retract_status reports trajectory progress
 *
 * Also acts as the gateway between input devices and the robot, owning the speed level:
 *   - [service] command_node/set_speed_level sets or shifts the speed level;
 *   - [topic] command_node/speed_level reports it;
 *   - [topic] command_node/robot/velocity/commands is scaled by the speed factor and relayed to the
 *     controllers;
 *   - [topic] command_node/gripper/velocity/commands is scaled by the speed factor, integrated into a
 *     position and relayed to the gripper controller.
 */
class CommandNode
{
public:
  CommandNode(rclcpp::Node::SharedPtr n);
  ~CommandNode();

protected:
private:
  rclcpp::Node::SharedPtr n_;

  TrajectoryManager trajectory_manager_;
  ControllerManagerWrapper controller_manager_wrapper_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr q_current_sub_;
  // Cartesian velocity command input
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_vel_sub_;
  // Normalized gripper velocity input; sign is open/close, magnitude is speed
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_vel_sub_;
  // Normalized trajectory velocity input; sign is deploy (+) / retract (-), magnitude is speed.
  // A non-zero value switches to trajectory mode on its own.
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr trajectory_vel_sub_;

  // Publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_qp_solving_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr retract_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_level_pub_;
  // Speed-scaled relays of the input device commands
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;

  // Services
  rclcpp::Service<explorer_msgs::srv::SetSpeedLevel>::SharedPtr set_speed_level_srv_;

  rclcpp::AsyncParametersClient::SharedPtr param_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  double sampling_period_;

  bool use_qp_inria_;

  enum class ControllerState
  {
    DEFAULT_CONTROLLER,  // default controller used (forward_position_controller, explorer_custom_controller ... ?)
    SWITCHING_TO_TRAJ,
    TRAJECTORY,  // joint_trajectory_controller
    RESTORING_DEFAULT_CONTROLLER
  };

  // Written from main thread and the detached switch_thread_
  std::atomic<ControllerState> controller_state_{ControllerState::DEFAULT_CONTROLLER};

  // Is trajectory mode enabled, only changed by setTrajectoryMode_()
  std::atomic<bool> trajectory_mode_{false};

  // Trajectory movement input, speed-scaled value of the trajectory velocity topic.
  std::atomic<double> trajectory_velocity_input_{0.0};
  // Reception time of the last trajectory velocity, the input falls back to 0 (hold) when stale
  rclcpp::Time last_trajectory_vel_time_;

  std::atomic<bool> switch_in_progress_{false};

  // Speed level, the speed factor applied to relayed commands is multiplier * level
  int min_speed_level_;
  int max_speed_level_;
  int speed_level_;
  double speed_level_multiplier_;

  // Integrated gripper position, value sent to the gripper controller
  std_msgs::msg::Float64MultiArray gripper_command_;
  // Reception time of the previous gripper velocity, the integration step is measured from it
  std::optional<rclcpp::Time> last_gripper_vel_time_;

  // Holds the controller-switch thread spawned in handle_controller_state_()
  std::thread switch_thread_;

  sensor_msgs::msg::JointState current_pos_;

  std::array<double, 7> q_current_;

  bool init_{false};

  space_control::JointModeResolver joint_mode_resolver_;

  std::vector<size_t> joint_order_;
  space_control::JointMode mode_;
  // Index of the first joint of Explorer robot (> 0 in case of wheelchair stack)
  size_t explorer_joint_offset_ = 0;

  std::optional<double> j2_max_cached_;
  std::optional<double> j2_operational_max_cached_;
  std::optional<double> j3_max_cached_;
  std::optional<double> j3_operational_max_cached_;
  std::vector<std::string> parameter_name_list_in_pending_;

  double j2_max_;
  double j2_operational_max_;
  double j3_max_;
  double j3_operational_max_;

  double actual_j2_limit_;
  double actual_j3_limit_;

  bool limits_initialized_ = false;

  std::vector<std::string> default_controller_name_list_;

  void callback_q_current_(const sensor_msgs::msg::JointState& msg);

  void callback_cartesian_velocity_(const geometry_msgs::msg::TwistStamped& msg);

  void callback_gripper_velocity_(const std_msgs::msg::Float64& msg);

  void callback_trajectory_velocity_(const std_msgs::msg::Float64& msg);

  void callback_set_speed_level_(
    const std::shared_ptr<explorer_msgs::srv::SetSpeedLevel::Request> request,
    std::shared_ptr<explorer_msgs::srv::SetSpeedLevel::Response> response);

  [[nodiscard]] double speed_factor_() const;

  // Enable/disable joint_trajectory_controller
  void setTrajectoryMode_(bool enable);

  // Drive the trajectory progress via trajectory_velocity_input_, while trajectory mode is enabled.
  void update_trajectory_();

  void handle_controller_state_();

  void modifyTargetNodeParameter_(
    const std::string& param_name, const rclcpp::ParameterValue& value);

  void getDoubleParameter_(const std::string& param_name, std::optional<double>& value);

  void timer_callback_();
};

}  // namespace space_control

#endif
