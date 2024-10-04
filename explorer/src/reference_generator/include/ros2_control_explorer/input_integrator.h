
#ifndef INPUT_INTEGRATOR_H
#define INPUT_INTEGRATOR_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>


#include <tf2/LinearMath/Quaternion.h>

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
    class InputIntegrator
    {
        public:
        InputIntegrator(rclcpp::Node::SharedPtr n);

        private:
        rclcpp::Node::SharedPtr n_;
    
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr  x_init_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr x_desired_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr dx_desired_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::TwistStamped dx_input_;

        SpacePosition x_desired_;
        SpacePosition x_init_;
        SpaceVelocity dx_desired_;

        float max_vel_;
        float max_vel_orientation_;
        double sampling_period_;

        void callback_input(const geometry_msgs::msg::TwistStamped & msg);
        void callback_x_init(const geometry_msgs::msg::Pose & msg);
        void timer_callback();
        void send_input();

    };
}
#endif 