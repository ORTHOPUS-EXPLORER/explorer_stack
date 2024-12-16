
#ifndef INPUT_INTEGRATOR_H
#define INPUT_INTEGRATOR_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "std_msgs/msg/float64.hpp"


#include <tf2/LinearMath/Quaternion.h>

#include "ros2_control_explorer/types/joint_position.h"
#include "ros2_control_explorer/types/joint_velocity.h"
#include "ros2_control_explorer/types/space_position.h"
#include "ros2_control_explorer/types/space_velocity.h"

#include "custom_interfaces/srv/pose.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace space_control
{
    class InputIntegrator
    {
        public:
        InputIntegrator(rclcpp::Node::SharedPtr n);

        private:
        rclcpp::Node::SharedPtr n_;
    
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_sub_;
        
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr linear_speed_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angular_speed_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr x_desired_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr dx_desired_pub_;

        rclcpp::Client<custom_interfaces::srv::Pose>::SharedPtr x_init_client_;

        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::TwistStamped dx_input_;

        geometry_msgs::msg::Pose x_init_pose_;

        SpacePosition x_desired_;
        SpacePosition x_init_;
        SpaceVelocity dx_desired_;

        double max_vel_;
        double max_vel_orientation_;
        double sampling_period_;

        bool error_;
        bool end_init_;
        int call_service_attempt_;
        int init_attempt_;
        bool success_init_;

        void callback_input(const geometry_msgs::msg::TwistStamped & msg);
        void callback_linear_speed(const std_msgs::msg::Float64 & msg);
        void callback_angular_speed(const std_msgs::msg::Float64 & msg);
        void timer_callback();
        void send_input();

    };
}
#endif 