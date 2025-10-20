#include "explorer_input_devices/xbox_gamepad_joint.h"

namespace space_control
{
    JoystickInput::JoystickInput(rclcpp::Node::SharedPtr n)
    : n_(n)
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        dq_.data = {0.0 ,0.0 ,0.0, 0.0, 0.0, 0.0, 0.0};

        scale = 0.5;
        //init suscribers
        joy_sub_ = n_->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoystickInput::callback_input, this, std::placeholders::_1));
    
        //init publishers
        dq_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/dq_output", 10);

    }

    void JoystickInput::callback_input(const sensor_msgs::msg::Joy & msg)
    {   
        if(msg.axes[2] != 1){
            dq_.data[0] = -((msg.axes[2]-1)/2) * scale;
        }
        else if(msg.axes[5] != 1){
            dq_.data[0] =  ((msg.axes[5]-1)/2) * scale;
        }
        else{
            dq_.data[0] =  0.0;
        }
        dq_.data[1] =  msg.axes[1] * scale;
        dq_.data[2] =  msg.axes[4] * scale;
        dq_.data[3] =  msg.axes[0] * scale;
        dq_.data[4] =  -msg.axes[3] * scale;
        dq_.data[5] =  -msg.axes[7] * scale;
        dq_.data[6] =  -msg.axes[6] * scale;

        dq_pub_->publish(dq_);
    }
}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    
    auto n = rclcpp::Node::make_shared("joystick_input", node_options);

    JoystickInput joistick_input(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
