
#ifndef SPACENAV_TRAJECTORY_IK_H
#define SPACENAV_TRAJECTORY_IK_H

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trac_ik/trac_ik.hpp>

#include <chrono>
#include <functional>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace space_control
{
    class SpacenavTrajectory
    {
        public:
        SpacenavTrajectory(rclcpp::Node::SharedPtr n);

        private:
        rclcpp::Node::SharedPtr n_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr trajectory_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
        
        geometry_msgs::msg::TwistStamped twist_command;

        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;

        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_prec;

        float max_vel;

        KDL::Tree robot_tree;
        KDL::Chain chain;

        KDL::Frame desired_pose;
        KDL::Frame actual_pose;

        std::shared_ptr<TRAC_IK::TRAC_IK> tracik_solver_;
        std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

        KDL::JntArray q_actual;
        KDL::JntArray q_desired;
        KDL::JntArray dq_desired;
        
        void callback(const geometry_msgs::msg::TwistStamped & msg);
        void callback_pos(const sensor_msgs::msg::JointState & msg);
        void timer_callback();

    };
}
#endif 