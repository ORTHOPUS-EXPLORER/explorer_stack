#ifndef JOINT_OUTPUT_INTEGRATOR_H
#define JOINT_OUTPUT_INTEGRATOR_H

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "explorer_controllers/qp_cartesian/types/joint_position.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pinocchio/multibody.hpp>

using namespace std::chrono_literals;

namespace space_control
{
class JointOutputIntegrator
{
public:
  JointOutputIntegrator(rclcpp::Node::SharedPtr n);

private:
  rclcpp::Node::SharedPtr n_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dq_output_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_dq_output_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::Float64MultiArray q_command_;
  std_msgs::msg::Float64MultiArray gripper_q_command_;
  std_msgs::msg::Float64MultiArray dq_output_;
  std_msgs::msg::Float64MultiArray gripper_dq_output_;

  double sampling_period_;
  bool init;
  std::vector<std::string> joint_name;
  std::array<int, 7> joint_order_;
  std::string controller_position_topic_name_;

  JointPosition q_lower_limit_; /*!< Joint lower limit used in lower constraints bound vector lbA */
  JointPosition q_upper_limit_; /*!< Joint upper limit used in upper constraints bound vector ubA */
  std::vector<int> q_has_limit_;

  /*!< Measured joint positions, ordered like joint_name */
  std::array<double, 7> q_measured_;

  /*!< When true, the published command is clamped so it never gets farther than
     position_error_saturation_threshold_ from the measured position (see timer_callback) */
  bool position_error_saturation_enable_;
  /*!< Max allowed distance (rad) between the published command and the measured position when
     position_error_saturation_enable_ is true */
  double position_error_saturation_threshold_;
  /*!< When false (default), the clamp only affects the published command: once the error goes
     back under the threshold, the setpoint "springs back" to where the integrator (q_command_)
     actually is. When true, the clamped value is also written back into q_command_, so the
     offset is kept permanently ("sticky") instead of being released. Only relevant when
     position_error_saturation_enable_ is true. */
  bool position_error_saturation_persistent_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;

  // --- "Controlled point" marker: the cartesian position corresponding to q_command_out, i.e.
  // where the tool should be if the robot exactly tracked the position command this node just
  // published. Computed here (rather than downstream, e.g. in gravity_compensation_node, from
  // a subscription to controller_position_topic_name_) so it doesn't depend on any other node
  // guessing the right topic name -- it's derived directly from the command this node sends.
  pinocchio::Model controlled_point_model_;
  pinocchio::Data controlled_point_data_;
  bool controlled_point_kinematics_ready_;
  /*!< Load is attempted only once */
  bool controlled_point_kinematics_load_attempted_;
  std::string controlled_point_urdf_path_;
  std::string controlled_point_end_effector_frame_;
  /*!< Must match the URDF root link */
  std::string controlled_point_marker_frame_id_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr controlled_point_marker_pub_;

  bool ensure_controlled_point_kinematics_ready_();
  void publish_controlled_point_marker_(const std_msgs::msg::Float64MultiArray& q_command_out);

  void callback_current_pos_(const sensor_msgs::msg::JointState& msg);
  void callback_dq_output_(const std_msgs::msg::Float64MultiArray& msg);
  void callback_gripper_dq_output_(const std_msgs::msg::Float64MultiArray& msg);
  void timer_callback();
  rcl_interfaces::msg::SetParametersResult on_parameter_change_(
    const std::vector<rclcpp::Parameter>& parameters);
};
}  // namespace space_control
#endif
