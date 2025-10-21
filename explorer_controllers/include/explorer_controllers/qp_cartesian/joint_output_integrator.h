
#ifndef JOINT_OUTPUT_INTEGRATOR_H
#define JOINT_OUTPUT_INTEGRATOR_H

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "explorer_controllers/qp_cartesian/types/joint_position.h"

#include <chrono>
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

        JointPosition q_lower_limit_; /*!< Joint lower limit used in lower constraints bound vector lbA */
        JointPosition q_upper_limit_; /*!< Joint upper limit used in upper constraints bound vector ubA */
        std::vector<int> q_has_limit_;

        void callback_current_pos_(const sensor_msgs::msg::JointState & msg);
        void callback_dq_output(const std_msgs::msg::Float64MultiArray & msg);
        void timer_callback();
    };
}
#endif 