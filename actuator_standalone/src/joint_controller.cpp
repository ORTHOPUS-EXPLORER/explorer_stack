#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointControl final
{
public:
  JointControl(rclcpp::Node::SharedPtr n)
  : n_(n)
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        q_command_.data = {0.0};
        dq_.data = {0.0};
        q_init = 0.0;
        x=0.0;

        scale = 0.5;
        init = false; 

        sampling_period_ = 0.02;

        command_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        joy_sub_ = n_->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JointControl::callback, this, std::placeholders::_1));

        current_pos_sub_ = n_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JointControl::callback_current_pos_, this, std::placeholders::_1));

        timer_ = n_->create_wall_timer(20ms, std::bind(&JointControl::timer_callback, this));
    }

    void callback(sensor_msgs::msg::Joy msg){

       if(msg.axes[2] != 1){
            dq_.data[0] = -((msg.axes[2]-1)/2) * scale;
        }
        else if(msg.axes[5] != 1){
            dq_.data[0] =  ((msg.axes[5]-1)/2) * scale;
        }
        else{
            dq_.data[0] =  0.0;
        }
    }

     void callback_current_pos_(const sensor_msgs::msg::JointState & msg)
    {   
        if(init ==false){
            q_command_.data[0] = msg.position[0];
            q_init = q_command_.data[0];
            init = true;
        }
    }

    void timer_callback()
    {
        if(init == true){
            //q_command_.data[0] = q_command_.data[0] + dq_.data[0] * sampling_period_;
            q_command_.data[0]= 0.5*cos(x)+q_init;

            command_pub_->publish(q_command_);

            x = x + 0.04;
            if(x >= (3.14*2)){
                x = 0.0;
            }
            //command_pub_->publish(q_command_);
        }
    }

    
    rclcpp::Node::SharedPtr n_;
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_pos_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Float64MultiArray dq_;
    std_msgs::msg::Float64MultiArray q_command_;

    double sampling_period_;

    double scale;
    bool init ;
    double q_init;
    double x;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto n = rclcpp::Node::make_shared("joint_controller", node_options);

    JointControl joint_controller(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}