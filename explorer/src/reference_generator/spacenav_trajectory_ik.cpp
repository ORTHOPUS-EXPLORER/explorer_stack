
#include "ros2_control_explorer/spacenav_trajectory_ik.h"


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

        
        q_actual = KDL::JntArray(chain.getNrOfJoints());
        q_desired = KDL::JntArray(chain.getNrOfJoints());
        dq_desired = KDL::JntArray(chain.getNrOfJoints());

        max_vel = 0.08;

        tracik_solver_ = std::make_shared<TRAC_IK::TRAC_IK>("world", "tool0", robot_description, 0.01, 1e-5);
        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);

        q_actual.data[0] = 0.0;
        q_actual.data[1] = 0.0;
        q_actual.data[2] = 0.0;
        q_actual.data[3] = 0.0;
        q_actual.data[4] = 0.0;
        q_actual.data[5] = 0.0;

        q_desired.data[0] = 0.0;
        q_desired.data[1] = 0.0;
        q_desired.data[2] = 0.0;
        q_desired.data[3] = 0.0;
        q_desired.data[4] = 0.0;
        q_desired.data[5] = 0.0;

        fk_solver_->JntToCart(q_actual, desired_pose);
    
        KDL::JntArray ll, ul; //lower joint limits, upper joint limits
        bool valid =  tracik_solver_->getKDLChain(chain);

        if (!valid)
        {
            RCLCPP_ERROR(n->get_logger(),"There was no valid KDL chain found");
            return;
        }

        valid = tracik_solver_->getKDLLimits(ll, ul);

        if (!valid)
        {
            RCLCPP_ERROR(n->get_logger(),"There were no valid KDL joint limits found");
            return;
        }

        assert(chain.getNrOfJoints() == ll.data.size());
        assert(chain.getNrOfJoints() == ul.data.size());

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
        int rc;
                
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

        desired_pose.Integrate(twist, 1000);
            
        rc = tracik_solver_->CartToJnt(q_actual, desired_pose, q_desired);
            
        // if (rc >= 0)
        // {
        //     RCLCPP_INFO(n_->get_logger(),"succes");
        // }
        // else
        // {
        //     RCLCPP_INFO(n_->get_logger(),"echec code : %d", rc);
        // }

        // copy to trajectory_point_msg
        std::memcpy(
        trajectory_point_msg.positions.data(), q_desired.data.data(),
        trajectory_point_msg.positions.size() * sizeof(double));

        trajectory_point_prec.time_from_start.sec = 0;
        trajectory_point_prec.time_from_start.nanosec = 0;
        trajectory_point_msg.time_from_start.nanosec = 100000000;

        trajectory_msg.points.push_back(trajectory_point_prec);
        trajectory_msg.points.push_back(trajectory_point_msg);

        command_pub_->publish(trajectory_msg);
                    
        trajectory_point_prec = trajectory_point_msg;

        q_actual.data[0] = q_desired.data[0];
        q_actual.data[1] = q_desired.data[1];
        q_actual.data[2] = q_desired.data[2];
        q_actual.data[3] = q_desired.data[3];
        q_actual.data[4] = q_desired.data[4];
        q_actual.data[5] = q_desired.data[5];
        
       
        
    }

}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto n = rclcpp::Node::make_shared("spacenav_trajectory_ik", node_options);

    SpacenavTrajectory spacenav_trajectory(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}