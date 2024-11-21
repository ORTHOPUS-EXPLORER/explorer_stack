
#ifndef JOYSTICK_INPUT_H
#define JOYSTICK_INPUT_H

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <functional>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace space_control
{
    class JoystickInput
    {
        public:
        JoystickInput(rclcpp::Node::SharedPtr n);

        private:
        rclcpp::Node::SharedPtr n_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dq_pub_;

        std_msgs::msg::Float64MultiArray dq_;
        
        double scale;
    
        void callback_input(const sensor_msgs::msg::Joy & msg);
    };
}
#endif 