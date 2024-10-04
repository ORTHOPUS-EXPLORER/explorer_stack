#include "ros2_control_explorer/test_output_integrator.h"

namespace space_control
{
    TestOutputIntegrator::TestOutputIntegrator(rclcpp::Node::SharedPtr n)
    : n_(n)
    {
        input_sub_ = n_->create_subscription<geometry_msgs::msg::TwistStamped>("/ros2_control_explorer/input_device_velocity", 10, std::bind(&TestOutputIntegrator::callback_input, this, std::placeholders::_1));

        dq_output_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/dq_output", 10);
    }

    void TestOutputIntegrator::callback_input(const geometry_msgs::msg::TwistStamped & msg)
    {   
        std_msgs::msg::Float64MultiArray dq_output;

        dq_output.data= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        dq_output.data[0] = msg.twist.linear.x;
        dq_output.data[1] = msg.twist.linear.y;
        dq_output.data[2] = msg.twist.linear.z;
        dq_output.data[3] = msg.twist.angular.x;
        dq_output.data[4] = msg.twist.angular.y;
        dq_output.data[5] = msg.twist.angular.z;

        dq_output_pub_->publish(dq_output);
    }
}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    
    auto n = rclcpp::Node::make_shared("test_output_integrator", node_options);

    TestOutputIntegrator test_output_integrator(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
