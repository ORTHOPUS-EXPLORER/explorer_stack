
#include "ros2_control_explorer/spacenav_trajectory_qp.h"

namespace space_control
{
    SpacenavTrajectoryQP::SpacenavTrajectoryQP(rclcpp::Node::SharedPtr n)
    : n_(n)
    , ik_(n, 6)
    , fk_(n, 6)
    , vi_(n, 6)
    , sampling_period_(0.0)
    , q_command_(6)
    , q_current_(6)
    , q_meas_(6)
    , dq_desired_(6)
    , x_current_()
    , x_desired_()
    , dx_desired_()
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        max_vel_ = 0.05;
        sampling_period_ = 0.001;

        trajectory_point_msg.positions.resize(6);
        trajectory_point_msg.velocities.resize(6);
        trajectory_point_prec.positions.resize(6);
        trajectory_point_prec.velocities.resize(6);

        ik_.init("tool0", sampling_period_);
        fk_.init("tool0");
        vi_.init(sampling_period_);

        trajectory_tracking = true;

        command_pub_ = n_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/explorer_controller/joint_trajectory", 10);
        trajectory_sub_ = n_->create_subscription<geometry_msgs::msg::TwistStamped>("/ros2_control_explorer/input_device_velocity", 10, std::bind(&SpacenavTrajectoryQP::callback, this, std::placeholders::_1));
        
        timer_ = n_->create_wall_timer(1ms, std::bind(&SpacenavTrajectoryQP::timer_callback, this));

        x_current_debug_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>("/explorer_controller/debug/x_current", 10);
        x_desired_debug_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>("/explorer_controller/debug/x_desired", 10);
        dx_desired_debug_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>("/explorer_controller/debug/dx_desired", 10);

    }

    void SpacenavTrajectoryQP::callback(const geometry_msgs::msg::TwistStamped & msg)
    {
        dx_desired_.position.x() = (msg.twist.linear.x * max_vel_);
        dx_desired_.position.y() = (msg.twist.linear.y * max_vel_);
        dx_desired_.position.z() = (msg.twist.linear.z * max_vel_);

        dx_desired_.orientation.w() = (0.0);
        dx_desired_.orientation.x() = (msg.twist.angular.x);
        dx_desired_.orientation.y() = (msg.twist.angular.y);
        dx_desired_.orientation.z() = (msg.twist.angular.z);
    }

    void SpacenavTrajectoryQP::timer_callback()
    {
        q_current_ = q_command_;

        //RCLCPP_INFO(n_->get_logger(), "=== Start FK computation...");
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Input joint position :");
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "q_current_           : " << q_current_);
        fk_.setQCurrent(q_current_);
        fk_.resolveForwardKinematic();
        fk_.getXCurrent(x_current_);
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Forward kinematic computes space position : ");
        //RCLCPP_DEBUG_STREAM(n_->get_logger(), "x_current_           : " << x_current_);

        // RCLCPP_INFO(n_->get_logger(), "=== Start IK computation...");
        ik_.setQCurrent(q_current_);
        ik_.setXCurrent(x_current_);
        ik_.resolveInverseKinematic(dq_desired_, dx_desired_, x_desired_, trajectory_tracking);
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Inverse kinematic computes joint velocity :");
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "dq_desired_          : " << dq_desired_);

        // RCLCPP_INFO(n_->get_logger(), "=== Integrate joint velocity...");
        vi_.setQCurrent(q_current_);
        vi_.integrate(dq_desired_, q_command_);
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Velocity integrator computes joint position :");
        //RCLCPP_DEBUG_STREAM(n_->get_logger(), "q_command_           : " << q_command_);

        send_Command();
        publishDebugTopic_();

    }

    void SpacenavTrajectoryQP::send_Command()
    {
        trajectory_msgs::msg::JointTrajectory trajectory_msg;

        
        trajectory_msg.header.stamp = n_->now();
        
        trajectory_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        // copy to trajectory_point_msg
        std::memcpy(
        trajectory_point_msg.positions.data(), q_command_.data(),
        trajectory_point_msg.positions.size() * sizeof(double));
        std::memcpy(
        trajectory_point_msg.velocities.data(), dq_desired_.data(),
        trajectory_point_msg.velocities.size() * sizeof(double));

        trajectory_point_prec.time_from_start.sec = 0;
        trajectory_point_prec.time_from_start.nanosec = 0;
        trajectory_point_msg.time_from_start.nanosec = 100000000;

        trajectory_msg.points.push_back(trajectory_point_prec);
        trajectory_msg.points.push_back(trajectory_point_msg);

        command_pub_->publish(trajectory_msg);
                        
        trajectory_point_prec = trajectory_point_msg;

    }

    void SpacenavTrajectoryQP::publishDebugTopic_()
    {

    // debug current space position (result of forward kinematic)
    geometry_msgs::msg::Pose x_current_pose;
    x_current_pose.position.x = x_current_.position.x();
    x_current_pose.position.y = x_current_.position.y();
    x_current_pose.position.z = x_current_.position.z();
    x_current_pose.orientation.w = x_current_.orientation.w();
    x_current_pose.orientation.x = x_current_.orientation.x();
    x_current_pose.orientation.y = x_current_.orientation.y();
    x_current_pose.orientation.z = x_current_.orientation.z();
    x_current_debug_pub_->publish(x_current_pose);

    // debug current space position (result of forward kinematic)
    geometry_msgs::msg::Pose x_desired_pose;
    x_desired_pose.position.x = x_desired_.position.x();
    x_desired_pose.position.y = x_desired_.position.y();
    x_desired_pose.position.z = x_desired_.position.z();
    x_desired_pose.orientation.w = x_desired_.orientation.w();
    x_desired_pose.orientation.x = x_desired_.orientation.x();
    x_desired_pose.orientation.y = x_desired_.orientation.y();
    x_desired_pose.orientation.z = x_desired_.orientation.z();
    x_desired_debug_pub_->publish(x_desired_pose);

    // debug space velocity which is sent to inverse kinematic solver
    geometry_msgs::msg::Pose dx_desired_pose;
    dx_desired_pose.position.x = dx_desired_.position.x();
    dx_desired_pose.position.y = dx_desired_.position.y();
    dx_desired_pose.position.z = dx_desired_.position.z();
    dx_desired_pose.orientation.w = dx_desired_.orientation.w();
    dx_desired_pose.orientation.x = dx_desired_.orientation.x();
    dx_desired_pose.orientation.y = dx_desired_.orientation.y();
    dx_desired_pose.orientation.z = dx_desired_.orientation.z();
    dx_desired_debug_pub_->publish(dx_desired_pose);
    }


}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto n = rclcpp::Node::make_shared("spacenav_trajectory_qp", node_options);

    SpacenavTrajectoryQP spacenav_trajectory_qp(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}