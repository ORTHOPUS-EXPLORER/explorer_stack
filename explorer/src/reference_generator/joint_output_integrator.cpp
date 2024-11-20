#include "ros2_control_explorer/joint_output_integrator.h"

namespace space_control
{
    JointOutputIntegrator::JointOutputIntegrator(rclcpp::Node::SharedPtr n)
    : n_(n)
    , q_lower_limit_(6)
    , q_upper_limit_(6)
    , q_has_limit_(6)
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        //init settings
        sampling_period_ = 0.01;
        init = false;

        dq_output_.data= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gripper_pos_.data = 0.0;
        q_command_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        RCLCPP_DEBUG_STREAM(n_->get_logger(),"init joint_name");
        joint_name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        RCLCPP_DEBUG_STREAM(n_->get_logger(),"lecture paramètre");
        for(int i=0; i<6; i++){
            n_->get_parameter("j"+std::to_string(i+1)+".limits", q_has_limit_[i]);
            if (q_has_limit_[i] == 1)
            {
                n_->get_parameter("j"+std::to_string(i+1)+".min", q_lower_limit_[i]);
                n_->get_parameter("j"+std::to_string(i+1)+".max", q_upper_limit_[i]);
            }
            else
            {
                q_lower_limit_[i] = 0;
                q_upper_limit_[i] = 0;
            }
            RCLCPP_DEBUG_STREAM(n_->get_logger(),"J" << i+1 << " - min:" << q_lower_limit_[i] << " max:" << q_upper_limit_[i]);
        }

        //init suscriber
        current_pos_sub_ = n_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JointOutputIntegrator::callback_current_pos_, this, std::placeholders::_1));
        dq_output_sub_ = n_->create_subscription<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/dq_output", 10, std::bind(&JointOutputIntegrator::callback_dq_output, this, std::placeholders::_1));
        gripper_pos_sub_ =  n_->create_subscription<std_msgs::msg::Float64>("/ros2_control_explorer/input_gripper_position", 10, std::bind(&JointOutputIntegrator::callback_gripper_pos, this, std::placeholders::_1));

        //init publisher
        command_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        
        timer_ = n_->create_wall_timer(10ms, std::bind(&JointOutputIntegrator::timer_callback, this));

    }

    void JointOutputIntegrator::callback_current_pos_(const sensor_msgs::msg::JointState & msg)
    {   
        int j = 0;

        if(init ==false){

            //Get the order of the joint state for the simulation with the wheelchair
            for (int i=0; i< 6; i++){
                j=0;
                while (joint_name[i]!=msg.name[j] && j<msg.position.size() ){
                    j++;
                }
                if(joint_name[i]== msg.name[j]){
                    RCLCPP_DEBUG_STREAM(n_->get_logger(),joint_name[i] << ": " << j);
                    joint_order[i] = j;
                }
            }

            for(int i=0; i< 6; i++){
                    q_command_.data[i] = msg.position[joint_order[i]];
            }
            init = true;
        }
    }

    void JointOutputIntegrator::callback_dq_output(const std_msgs::msg::Float64MultiArray & msg)
    {   
        dq_output_.data = msg.data;   
    }

    void JointOutputIntegrator::callback_gripper_pos(const std_msgs::msg::Float64 & msg)
    {   
        gripper_pos_.data = msg.data;
    }

    void JointOutputIntegrator::timer_callback()
    {
        if(init == true){
            for(int i=0; i< 6; i++){
                q_command_.data[i] = q_command_.data[i] + dq_output_.data[i] * sampling_period_;
                //RCLCPP_DEBUG_STREAM(n_->get_logger(),"q_command ["<< i <<"]: " << q_command_.data[i]);
                if(q_command_.data[i] <= q_lower_limit_[i]){
                    q_command_.data[i] = q_lower_limit_[i];
                }
                else if(q_command_.data[i] >= q_upper_limit_[i]){
                    q_command_.data[i] = q_upper_limit_[i];
                }
            }
              
            q_command_.data[6] = gripper_pos_.data;

            command_pub_->publish(q_command_);
        }
    }
}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto n = rclcpp::Node::make_shared("joint_output_integrator", node_options);

    JointOutputIntegrator joint_output_integrator(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
