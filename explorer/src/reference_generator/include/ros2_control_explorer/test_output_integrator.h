
#ifndef TEST_OUTPUT_INTEGRATOR_H
#define TEST_OUTPUT_INTEGRATOR_H

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace space_control
{
    class TestOutputIntegrator
    {
        public:
        TestOutputIntegrator(rclcpp::Node::SharedPtr n);

        private:
        rclcpp::Node::SharedPtr n_;

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_sub_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dq_output_pub_;

        void callback_input(const geometry_msgs::msg::TwistStamped & msg);
    };
}
#endif 