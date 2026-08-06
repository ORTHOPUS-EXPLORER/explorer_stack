
#ifndef JOINT_OUTPUT_INTEGRATOR_H
#define JOINT_OUTPUT_INTEGRATOR_H

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "explorer_controllers/qp_cartesian/types/joint_position.h"

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

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

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    
        rclcpp::TimerBase::SharedPtr timer_;

        std_msgs::msg::Float64MultiArray q_command_;
        std_msgs::msg::Float64MultiArray dq_output_;

        double sampling_period_;
        bool init;
        std::vector<std::string> joint_name;
        int joint_order[7];
        std::string controller_position_topic_name_;

        JointPosition q_lower_limit_; /*!< Joint lower limit used in lower constraints bound vector lbA */
        JointPosition q_upper_limit_; /*!< Joint upper limit used in upper constraints bound vector ubA */
        std::vector<int> q_has_limit_;

        /*!< Latest measured joint positions (from /joint_states), reordered to match joint_name/joint_order */
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

        /*!< Handle for the dynamic parameter callback (must be kept alive) allowing
             position_error_saturation_enable_/_threshold_/_persistent_ to be changed at runtime,
             e.g. via `ros2 param set`, without restarting the node. */
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

        void callback_current_pos_(const sensor_msgs::msg::JointState & msg);
        void callback_dq_output(const std_msgs::msg::Float64MultiArray & msg);
        void timer_callback();
        rcl_interfaces::msg::SetParametersResult on_parameter_change_(const std::vector<rclcpp::Parameter> & parameters);
    };
}
#endif 