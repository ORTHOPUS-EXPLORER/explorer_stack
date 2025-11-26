#ifndef QP_SOLVING_H
#define QP_SOLVING_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "explorer_controllers/qp_cartesian/inverse_kinematic.h"
#include "explorer_controllers/qp_cartesian/forward_kinematic.h"

#include "explorer_controllers/qp_cartesian/types/joint_position.h"
#include "explorer_controllers/qp_cartesian/types/joint_velocity.h"
#include "explorer_controllers/qp_cartesian/types/space_position.h"
#include "explorer_controllers/qp_cartesian/types/space_velocity.h"

#include "explorer_msgs/srv/pose.hpp"
#include "explorer_msgs/srv/float64.hpp"
#include "explorer_msgs/msg/control_frame_selection.hpp"

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
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_released_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_pressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_released_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J1_zero_pressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J1_zero_released_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J2_zero_pressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J2_zero_released_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J3_zero_pressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J3_zero_released_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J4_zero_pressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J4_zero_released_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J5_zero_pressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J5_zero_released_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J6_zero_pressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr J6_zero_released_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr x_des_updated_sub_;
        rclcpp::Subscription<explorer_msgs::msg::ControlFrameSelection>::SharedPtr control_frame_sub_;
        
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dq_output_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr x_current_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_current_debug_pub_;
        
        rclcpp::Service<explorer_msgs::srv::Pose>::SharedPtr x_init_service_;
        rclcpp::Service<explorer_msgs::srv::Float64>::SharedPtr q_init_service_;

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
        bool go_zero;
        bool go_J1_zero;
        bool go_J2_zero;
        bool go_J3_zero;
        bool go_J4_zero;
        bool go_J5_zero;
        bool go_J6_zero;

        enum class Mode { INVALID, EXPLORER, FULL };

        std::vector<std::string> expected_names_explorer = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "left_external_rod_joint_mimic", "left_fingertip_joint_mimic", "left_finger_joint_mimic", "right_external_rod_joint_mimic", "right_fingertip_joint_mimic", "right_finger_joint"};
        std::vector<std::string> expected_names_wheelchair = {"left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint", "left_wheel_joint", "right_wheel_joint", "left_right_head_joint", "up_down_head_joint"};

        std::vector<size_t> joint_order;
        Mode mode;
        std::vector<double> q_init_;

        void callback_current_pos_(const sensor_msgs::msg::JointState & msg);
        void callback_q_command_prec_(const std_msgs::msg::Float64MultiArray & msg);
        void callback_dx_input_(const geometry_msgs::msg::Pose & msg);
        void callback_x_input_(const geometry_msgs::msg::Pose & msg);
        void callback_home_pressed_(const std_msgs::msg::Bool & msg);
        void callback_home_released_(const std_msgs::msg::Bool & msg);
        void callback_x_des_updated_(const std_msgs::msg::Bool & msg);
        void callback_zero_pressed_(const std_msgs::msg::Bool & msg);
        void callback_zero_released_(const std_msgs::msg::Bool & msg);
        void callback_J1_zero_released_(const std_msgs::msg::Bool & msg);
        void callback_J1_zero_pressed_(const std_msgs::msg::Bool & msg);
        void callback_J2_zero_released_(const std_msgs::msg::Bool & msg);
        void callback_J2_zero_pressed_(const std_msgs::msg::Bool & msg);
        void callback_J3_zero_released_(const std_msgs::msg::Bool & msg);
        void callback_J3_zero_pressed_(const std_msgs::msg::Bool & msg);
        void callback_J4_zero_released_(const std_msgs::msg::Bool & msg);
        void callback_J4_zero_pressed_(const std_msgs::msg::Bool & msg);
        void callback_J5_zero_released_(const std_msgs::msg::Bool & msg);
        void callback_J5_zero_pressed_(const std_msgs::msg::Bool & msg);
        void callback_J6_zero_released_(const std_msgs::msg::Bool & msg);
        void callback_J6_zero_pressed_(const std_msgs::msg::Bool & msg);
        void callback_control_frame_selection_(const explorer_msgs::msg::ControlFrameSelection::SharedPtr msg);
        void timer_callback();
        void callback_x_init_(const std::shared_ptr<explorer_msgs::srv::Pose::Request> req, std::shared_ptr<explorer_msgs::srv::Pose::Response> res);
        void callback_q_init_(const std::shared_ptr<explorer_msgs::srv::Float64::Request> req, std::shared_ptr<explorer_msgs::srv::Float64::Response> res);
        void send_output();
        void publishDebugTopic_();

    };
}
#endif