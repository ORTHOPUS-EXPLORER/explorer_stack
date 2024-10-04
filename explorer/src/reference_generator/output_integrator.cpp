#include "ros2_control_explorer/output_integrator.h"

namespace space_control
{
    OutputIntegrator::OutputIntegrator(rclcpp::Node::SharedPtr n)
    : n_(n)
    {
        
        //init settings
        sampling_period_ = 0.01;

        dq_output_.data= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gripper_pos_.data = 0.0;
        q_command_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //init suscriber
        dq_output_sub_ = n_->create_subscription<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/dq_output", 10, std::bind(&OutputIntegrator::callback_dq_output, this, std::placeholders::_1));
        gripper_pos_sub_ =  n_->create_subscription<std_msgs::msg::Float64>("/ros2_control_explorer/input_gripper_position", 10, std::bind(&OutputIntegrator::callback_gripper_pos, this, std::placeholders::_1));

        //init publisher
        command_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        timer_ = n_->create_wall_timer(10ms, std::bind(&OutputIntegrator::timer_callback, this));

    }

    void OutputIntegrator::callback_dq_output(const std_msgs::msg::Float64MultiArray & msg)
    {   
       dq_output_.data = msg.data;   
    }

    void OutputIntegrator::callback_gripper_pos(const std_msgs::msg::Float64 & msg)
    {   
        gripper_pos_.data = msg.data;
    }

    void OutputIntegrator::timer_callback()
    {
        for(int i=0; i< 6; i++){
           q_command_.data[i] = q_command_.data[i] + dq_output_.data[i] * sampling_period_;
        }
        
        q_command_.data[6] = gripper_pos_.data;

        command_pub_->publish(q_command_);
    }


}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    
    auto n = rclcpp::Node::make_shared("output_integrator", node_options);

    OutputIntegrator output_integrator(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
