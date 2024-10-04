
#ifndef OUTPUT_INTEGRATOR_H
#define OUTPUT_INTEGRATOR_H

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace space_control
{
    class OutputIntegrator
    {
        public:
        OutputIntegrator(rclcpp::Node::SharedPtr n);

        private:
        rclcpp::Node::SharedPtr n_;
    
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dq_output_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_pos_sub_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    
        rclcpp::TimerBase::SharedPtr timer_;

        std_msgs::msg::Float64MultiArray q_command_;
        std_msgs::msg::Float64MultiArray dq_output_;
        std_msgs::msg::Float64 gripper_pos_;

        double sampling_period_;

        void callback_dq_output(const std_msgs::msg::Float64MultiArray & msg);
        void callback_gripper_pos(const std_msgs::msg::Float64 & msg);
        void timer_callback();
    };
}
#endif 