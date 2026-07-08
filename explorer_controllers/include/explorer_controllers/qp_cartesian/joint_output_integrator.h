
#ifndef JOINT_OUTPUT_INTEGRATOR_H
#define JOINT_OUTPUT_INTEGRATOR_H

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "explorer_controllers/qp_cartesian/types/joint_position.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

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

  void callback_current_pos_(const sensor_msgs::msg::JointState& msg);
  void callback_dq_output_(const std_msgs::msg::Float64MultiArray& msg);
  void callback_gripper_dq_output_(const std_msgs::msg::Float64MultiArray& msg);
  void timer_callback();
};
}  // namespace space_control
#endif