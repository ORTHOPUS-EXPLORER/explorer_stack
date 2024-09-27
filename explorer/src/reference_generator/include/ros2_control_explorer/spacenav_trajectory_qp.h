
#ifndef SPACENAV_TRAJECTORY_QP_H
#define SPACENAV_TRAJECTORY_QP_H

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "ros2_control_explorer/inverse_kinematic.h"
#include "ros2_control_explorer/forward_kinematic.h"
#include "ros2_control_explorer/velocity_integrator.h"

#include "ros2_control_explorer/types/joint_position.h"
#include "ros2_control_explorer/types/joint_velocity.h"
#include "ros2_control_explorer/types/space_position.h"
#include "ros2_control_explorer/types/space_velocity.h"

#include <chrono>
#include <functional>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace space_control
{
    class SpacenavTrajectoryQP
    {
        public:
        SpacenavTrajectoryQP(rclcpp::Node::SharedPtr n);

        private:
        rclcpp::Node::SharedPtr n_;
        InverseKinematic ik_;
        ForwardKinematic fk_;
        VelocityIntegrator vi_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr trajectory_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_pos_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr x_current_debug_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr x_desired_debug_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr dx_desired_debug_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr q_command_debug_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_current_debug_pub_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_pos_sub;
                        
        JointPosition q_command_;
        JointPosition q_command_explorer_;
        JointPosition q_current_;
        JointVelocity dq_desired_;

        SpacePosition x_current_;
        SpacePosition x_desired_;
        SpaceVelocity dx_desired_;

        sensor_msgs::msg::JointState current_pos_;
        sensor_msgs::msg::JointState commands_debug_;
        std_msgs::msg::Float64MultiArray q_current_debug;
        
        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_prec;

        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;

        float max_vel_;
        double sampling_period_;
        bool trajectory_tracking ;
        bool init;

        int joint_order[18];

        void callback_trajectory(const geometry_msgs::msg::TwistStamped & msg);
        void callback_current_pos_(const sensor_msgs::msg::JointState & msg);
        void callback_gripper(const std_msgs::msg::Bool & msg);
        void callback_gripper_pos(const std_msgs::msg::Float64 & msg);
        void timer_callback();
        void send_Command();
        void publishDebugTopic_();

    };
}
#endif 