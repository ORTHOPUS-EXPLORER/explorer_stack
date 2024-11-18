#include "ros2_control_explorer/output_integrator.h"

namespace space_control
{
    OutputIntegrator::OutputIntegrator(rclcpp::Node::SharedPtr n)
    : n_(n)
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        //init settings
        sampling_period_ = 0.01;
        init = false;
        error_ = false;
        call_service_attempt_ = 0;
        init_attempt_ = 0;
        success_init_ = false;


        dq_output_.data= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gripper_pos_.data = 0.0;
        q_command_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        q_init_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //init suscriber
        dq_output_sub_ = n_->create_subscription<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/dq_output", 10, std::bind(&OutputIntegrator::callback_dq_output, this, std::placeholders::_1));
        gripper_pos_sub_ =  n_->create_subscription<std_msgs::msg::Float64>("/ros2_control_explorer/input_gripper_position", 10, std::bind(&OutputIntegrator::callback_gripper_pos, this, std::placeholders::_1));

        q_init_client_ = n_->create_client<custom_interfaces::srv::Float64>("/ros2_control_explorer/q_init");

        //init publisher
        command_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        auto request = std::make_shared<custom_interfaces::srv::Float64::Request>();

        while(init_attempt_< 5 && call_service_attempt_< 10 && success_init_ == false){
        request->ready = true;
        
            while (!q_init_client_->wait_for_service(1s) && error_ == false && call_service_attempt_ < 10) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    error_ = true;

                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                call_service_attempt_ += 1;
            }

            if(call_service_attempt_ < 10){
                RCLCPP_INFO_ONCE(rclcpp::get_logger("rclcpp"), "service available");
                auto result = q_init_client_->async_send_request(request);
                if (rclcpp::spin_until_future_complete(n_, result) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto copy_result = result.get();
                    if(copy_result.get()->code_error == 0){
                        q_init_ = copy_result.get()->data;
                        success_init_ = true;
                    }
                    else{
                        init_attempt_ += 1;
                    }
                
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
                }
            }
            else{
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, use initialised values");
            }
        }
        if( init_attempt_>= 5){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialise joints positions");
            exit(0);
        }

        for(int i=0; i< 6; i++){
            q_command_.data[i] = q_init_[i];
        }

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
           //RCLCPP_DEBUG_STREAM(n_->get_logger(),"q_command ["<< i <<"]: " << q_command_.data[i]);
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
