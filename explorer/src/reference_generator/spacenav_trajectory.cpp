
#include "ros2_control_explorer/spacenav_trajectory.h"

namespace space_control
{
    SpacenavTrajectory::SpacenavTrajectory(rclcpp::Node::SharedPtr n)
    : n_(n)
    {
        auto robot_param = rclcpp::Parameter();
        n_->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
        n_->get_parameter("robot_description", robot_param);
        auto robot_description = robot_param.as_string();

        kdl_parser::treeFromString(robot_description, robot_tree);
        robot_tree.getChain("world", "tool0", chain);

        joint_positions = KDL::JntArray(chain.getNrOfJoints());
        joint_velocities = KDL::JntArray(chain.getNrOfJoints());

        max_vel = 0.05;

        ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain, 0.0000001);

        trajectory_point_msg.positions.resize(chain.getNrOfJoints());
        trajectory_point_msg.velocities.resize(chain.getNrOfJoints());
        trajectory_point_prec.positions.resize(chain.getNrOfJoints());
        trajectory_point_prec.velocities.resize(chain.getNrOfJoints());

        command_pub_ = n_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/explorer_controller/joint_trajectory", 10);
        trajectory_sub_ = n_->create_subscription<geometry_msgs::msg::TwistStamped>("/ros2_control_explorer/input_device_velocity", 10, std::bind(&SpacenavTrajectory::callback, this, std::placeholders::_1));
        timer_ = n_->create_wall_timer(1ms, std::bind(&SpacenavTrajectory::timer_callback, this));
    }

    void SpacenavTrajectory::callback(const geometry_msgs::msg::TwistStamped & msg)
    {
        twist_command = msg;
    }

    void SpacenavTrajectory::timer_callback()
    {
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        
        trajectory_msg.header.stamp = n_->now();
        
        for (size_t i = 0; i < chain.getNrOfSegments(); i++)
        {
            auto joint = chain.getSegment(i).getJoint();
            if (joint.getType() != KDL::Joint::Fixed)
            {
            trajectory_msg.joint_names.push_back(joint.getName());
            }
        }
        

        auto twist = KDL::Twist(); 

        twist.vel.x(twist_command.twist.linear.x * max_vel);
        twist.vel.y(twist_command.twist.linear.y * max_vel);
        twist.vel.z(twist_command.twist.linear.z * max_vel);
        twist.rot.x(twist_command.twist.angular.x);
        twist.rot.y(twist_command.twist.angular.y);
        twist.rot.z(twist_command.twist.angular.z);
        
        ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);

        // copy to trajectory_point_msg
        std::memcpy(
        trajectory_point_msg.positions.data(), joint_positions.data.data(),
        trajectory_point_msg.positions.size() * sizeof(double));
        std::memcpy(
        trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
        trajectory_point_msg.velocities.size() * sizeof(double));

        // integrate joint velocities
        joint_positions.data += joint_velocities.data * 0.001;

        trajectory_point_prec.time_from_start.sec = 0;
        trajectory_point_prec.time_from_start.nanosec = 0;
        trajectory_point_msg.time_from_start.nanosec = 100000000;

        trajectory_msg.points.push_back(trajectory_point_prec);
        trajectory_msg.points.push_back(trajectory_point_msg);

        command_pub_->publish(trajectory_msg);
                        
        trajectory_point_prec = trajectory_point_msg;
    }

}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto n = rclcpp::Node::make_shared("spacenav_trajectory", node_options);

    SpacenavTrajectory spacenav_trajectory(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}