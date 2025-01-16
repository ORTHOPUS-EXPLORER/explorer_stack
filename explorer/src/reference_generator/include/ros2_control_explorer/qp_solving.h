
#ifndef QP_SOLVING_H
#define QP_SOLVING_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "ros2_control_explorer/inverse_kinematic.h"
#include "ros2_control_explorer/forward_kinematic.h"

#include "ros2_control_explorer/types/joint_position.h"
#include "ros2_control_explorer/types/joint_velocity.h"
#include "ros2_control_explorer/types/space_position.h"
#include "ros2_control_explorer/types/space_velocity.h"

#include "custom_interfaces/srv/pose.hpp"
#include "custom_interfaces/srv/float64.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace space_control
{
    class QPSolving
    {
        public:
        QPSolving(rclcpp::Node::SharedPtr n);

        private:
        rclcpp::Node::SharedPtr n_;

        InverseKinematic ik_;
        ForwardKinematic fk_;
    
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_pos_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr  x_input_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr  dx_input_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  q_command_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_pressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr x_des_updated_sub_;
        
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dq_output_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr x_current_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_current_debug_pub_;
        
        rclcpp::Service<custom_interfaces::srv::Pose>::SharedPtr x_init_service_;
        rclcpp::Service<custom_interfaces::srv::Float64>::SharedPtr q_init_service_;

        rclcpp::TimerBase::SharedPtr timer_;
        
        JointPosition q_current_;
        JointVelocity dq_desired_;

        SpacePosition x_current_;
        SpacePosition x_input_;
        SpaceVelocity dx_input_;
        SpacePosition x_desired_;
        SpaceVelocity dx_desired_;

        std_msgs::msg::Float64MultiArray dq_output_;
        std_msgs::msg::Float64MultiArray q_current_debug;
        std_msgs::msg::Float64MultiArray q_command_prec_;
        sensor_msgs::msg::JointState current_pos_;

        geometry_msgs::msg::Pose x_init_;

        double sampling_period_;
        bool init;
        bool wheelchair;
        bool first_use;
        bool go_home;

        int joint_order[20];
        std::vector<double> q_init_;

        void callback_current_pos_(const sensor_msgs::msg::JointState & msg);
        void callback_q_command_prec_(const std_msgs::msg::Float64MultiArray & msg);
        void callback_dx_input_(const geometry_msgs::msg::Pose & msg);
        void callback_x_input_(const geometry_msgs::msg::Pose & msg);
        void callback_home_pressed_(const std_msgs::msg::Bool & msg);
        void callback_x_des_updated_(const std_msgs::msg::Bool & msg);
        void timer_callback();
        void callback_x_init_(const std::shared_ptr<custom_interfaces::srv::Pose::Request> req, std::shared_ptr<custom_interfaces::srv::Pose::Response> res);
        void callback_q_init_(const std::shared_ptr<custom_interfaces::srv::Float64::Request> req, std::shared_ptr<custom_interfaces::srv::Float64::Response> res);
        void send_output();
        void publishDebugTopic_();

    };
}
#endif 