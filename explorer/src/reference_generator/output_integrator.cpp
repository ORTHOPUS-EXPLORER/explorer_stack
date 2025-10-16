#include "ros2_control_explorer/output_integrator.h"

namespace space_control
{
    OutputIntegrator::OutputIntegrator(rclcpp::Node::SharedPtr n)
    : n_(n)
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        //init settings
        sampling_period_ = 0.02;
        error_ = false;
        call_service_attempt_ = 0;
        init_attempt_ = 0;
        success_init_ = false;


        dq_output_.data= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gripper_vel_.data = 0.0;
        q_command_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5};

        q_init_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        go_home = false;
        go_zero = false;
        go_J1_zero = false;
        go_J2_zero = false;
        go_J3_zero = false;
        go_J4_zero = false;
        go_J5_zero = false;
        go_J6_zero = false;


        //init suscriber
        dq_output_sub_ = n_->create_subscription<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/dq_output", 10, std::bind(&OutputIntegrator::callback_dq_output, this, std::placeholders::_1));
        gripper_pos_sub_ =  n_->create_subscription<std_msgs::msg::Float64>("/ros2_control_explorer/input_gripper_velocity", 10, std::bind(&OutputIntegrator::callback_gripper_vel, this, std::placeholders::_1));
        home_pressed_sub_ =  n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/home_pressed", 10, std::bind(&OutputIntegrator::callback_home, this, std::placeholders::_1));
        zero_pressed_sub_ =  n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/zero_pressed", 10, std::bind(&OutputIntegrator::callback_zero, this, std::placeholders::_1));
        J1_zero_pressed_sub_ =  n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/J1_zero_pressed", 10, std::bind(&OutputIntegrator::callback_J1_zero, this, std::placeholders::_1));
        J2_zero_pressed_sub_ =  n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/J2_zero_pressed", 10, std::bind(&OutputIntegrator::callback_J2_zero, this, std::placeholders::_1));
        J3_zero_pressed_sub_ =  n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/J3_zero_pressed", 10, std::bind(&OutputIntegrator::callback_J3_zero, this, std::placeholders::_1));
        J4_zero_pressed_sub_ =  n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/J4_zero_pressed", 10, std::bind(&OutputIntegrator::callback_J4_zero, this, std::placeholders::_1));
        J5_zero_pressed_sub_ =  n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/J5_zero_pressed", 10, std::bind(&OutputIntegrator::callback_J5_zero, this, std::placeholders::_1));
        J6_zero_pressed_sub_ =  n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/J6_zero_pressed", 10, std::bind(&OutputIntegrator::callback_J6_zero, this, std::placeholders::_1));

        q_init_client_ = n_->create_client<custom_interfaces::srv::Float64>("/ros2_control_explorer/q_init");

        //init publisher
        command_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        auto request = std::make_shared<custom_interfaces::srv::Float64::Request>();

        while(init_attempt_< 100000 && call_service_attempt_< 100000 && success_init_ == false){
        request->ready = true;
        
            while (!q_init_client_->wait_for_service(1s) && error_ == false && call_service_attempt_ < 100000) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    error_ = true;

                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                call_service_attempt_ += 1;
            }

            if(call_service_attempt_ <= 100000){
                RCLCPP_INFO_ONCE(rclcpp::get_logger("rclcpp"), "service available");
                auto result = q_init_client_->async_send_request(request);
                if (rclcpp::spin_until_future_complete(n_, result) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto copy_result = result.get();
                    if(copy_result.get()->code_error == 0){
                        q_init_ = copy_result.get()->data;
                        success_init_ = true;
                        RCLCPP_INFO(n_->get_logger(), "output_integrator_initialized");
                    }
                    else{
                        init_attempt_ += 1;
                        //RCLCPP_INFO_STREAM(n_->get_logger(), "init_attempt : " << init_attempt_);
                        //RCLCPP_INFO(n_->get_logger(), "new init_attempt ");
                    }
                
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
                }
            }
            else{
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, use initialised values");
            }
        }
        if( init_attempt_>= 100000){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialise joints positions");
            exit(0);
        }

        for(int i=0; i< 6; i++){
            q_command_.data[i] = q_init_[i];
        }

        timer_ = n_->create_wall_timer(20ms, std::bind(&OutputIntegrator::timer_callback, this));

    }

    void OutputIntegrator::callback_dq_output(const std_msgs::msg::Float64MultiArray & msg)
    {   
       dq_output_.data = msg.data;   
    }

    void OutputIntegrator::callback_gripper_vel(const std_msgs::msg::Float64 & msg)
    {   
        gripper_vel_.data = msg.data;
    }

    void OutputIntegrator::callback_home(const std_msgs::msg::Bool & msg)
    {   
        go_home = msg.data;
    }

    void OutputIntegrator::callback_zero(const std_msgs::msg::Bool & msg)
    {   
        go_zero = msg.data;
    }
    
    void OutputIntegrator::callback_J1_zero(const std_msgs::msg::Bool & msg)
    {   
        go_J1_zero = msg.data;
    }
    
    void OutputIntegrator::callback_J2_zero(const std_msgs::msg::Bool & msg)
    {   
        go_J2_zero = msg.data;
    }

    void OutputIntegrator::callback_J3_zero(const std_msgs::msg::Bool & msg)
    {   
        go_J3_zero = msg.data;
    }
    
    void OutputIntegrator::callback_J4_zero(const std_msgs::msg::Bool & msg)
    {   
        go_J4_zero = msg.data;
    }

    void OutputIntegrator::callback_J5_zero(const std_msgs::msg::Bool & msg)
    {   
        go_J5_zero = msg.data;
    }
    
    void OutputIntegrator::callback_J6_zero(const std_msgs::msg::Bool & msg)
    {   
        go_J6_zero = msg.data;
    }


    void OutputIntegrator::timer_callback()
    {
        if(go_home == true){
            home(); 
        }

        if(go_zero == true){
            zero(); 
        }

        if(go_J1_zero == true){
            if(q_command_.data[0] < -0.001){
                if(q_command_.data[0] + 0.5 * sampling_period_ <= 0.0){
                    dq_output_.data[0] = 0.5;
                }
                else if (q_command_.data[0] + 0.5 * sampling_period_ > 0.0) {
                    dq_output_.data[0] =  q_command_.data[0]/sampling_period_;
                }
            }else if(q_command_.data[0] > 0.001){
                if(q_command_.data[0] - 0.5 * sampling_period_ >= 0.0){
                    dq_output_.data[0] = - 0.5;
                }
                else if (q_command_.data[0] - 0.5 * sampling_period_ < 0.0) {
                    dq_output_.data[0] =  -q_command_.data[0]/sampling_period_;
                }
            }else{
                dq_output_.data[0] = 0.0;
            } 
        }

        if(go_J2_zero == true){
            if(q_command_.data[1] < -0.001){
                if(q_command_.data[1] + 0.5 * sampling_period_ <= 0.0){
                    dq_output_.data[1] = 0.5;
                }
                else if (q_command_.data[1] + 0.5 * sampling_period_ > 0.0) {
                    dq_output_.data[1] =  q_command_.data[1]/sampling_period_;
                }
            }else if(q_command_.data[1] > 0.001){
                if(q_command_.data[1] - 0.5 * sampling_period_ >= 0.0){
                    dq_output_.data[1] = - 0.5;
                }
                else if (q_command_.data[1] - 0.5 * sampling_period_ < 0.0) {
                    dq_output_.data[1] =  -q_command_.data[1]/sampling_period_;
                }
            }else{
                dq_output_.data[1] = 0.0;
            } 
        }
        if(go_J3_zero == true){
            if(q_command_.data[2] < -0.001){
                if(q_command_.data[2] + 0.5 * sampling_period_ <= 0.0){
                    dq_output_.data[2] = 0.5;
                }
                else if (q_command_.data[2] + 0.5 * sampling_period_ > 0.0) {
                    dq_output_.data[2] =  q_command_.data[2]/sampling_period_;
                }
            }else if(q_command_.data[2] > 0.001){
                if(q_command_.data[2] - 0.5 * sampling_period_ >= 0.0){
                    dq_output_.data[2] = - 0.5;
                }
                else if (q_command_.data[2] - 0.5 * sampling_period_ < 0.0) {
                    dq_output_.data[2] =  -q_command_.data[2]/sampling_period_;
                }
            }else{
                dq_output_.data[2] = 0.0;
            } 
        }

        if(go_J4_zero == true){
            if(q_command_.data[3] < -0.001){
                if(q_command_.data[3] + 0.5 * sampling_period_ <= 0.0){
                    dq_output_.data[3] = 0.5;
                }
                else if (q_command_.data[3] + 0.5 * sampling_period_ > 0.0) {
                    dq_output_.data[3] =  q_command_.data[3]/sampling_period_;
                }
            }else if(q_command_.data[3] > 0.001){
                if(q_command_.data[3] - 0.5 * sampling_period_ >= 0.0){
                    dq_output_.data[3] = - 0.5;
                }
                else if (q_command_.data[3] - 0.5 * sampling_period_ < 0.0) {
                    dq_output_.data[3] =  -q_command_.data[3]/sampling_period_;
                }
            }else{
                dq_output_.data[3] = 0.0;
            } 
        }
        if(go_J5_zero == true){
            if(q_command_.data[4] < -0.001){
                if(q_command_.data[4] + 0.5 * sampling_period_ <= 0.0){
                    dq_output_.data[4] = 0.5;
                }
                else if (q_command_.data[4] + 0.5 * sampling_period_ > 0.0) {
                    dq_output_.data[4] =  q_command_.data[4]/sampling_period_;
                }
            }else if(q_command_.data[4] > 0.001){
                if(q_command_.data[4] - 0.5 * sampling_period_ >= 0.0){
                    dq_output_.data[4] = - 0.5;
                }
                else if (q_command_.data[4] - 0.5 * sampling_period_ < 0.0) {
                    dq_output_.data[4] =  -q_command_.data[4]/sampling_period_;
                }
            }else{
                dq_output_.data[4] = 0.0;
            } 
        }

        if(go_J6_zero == true){
            if(q_command_.data[5] < -0.001){
                if(q_command_.data[5] + 0.5 * sampling_period_ <= 0.0){
                    dq_output_.data[5] = 0.5;
                }
                else if (q_command_.data[5] + 0.5 * sampling_period_ > 0.0) {
                    dq_output_.data[5] =  q_command_.data[5]/sampling_period_;
                }
            }else if(q_command_.data[5] > 0.001){
                if(q_command_.data[5] - 0.5 * sampling_period_ >= 0.0){
                    dq_output_.data[5] = - 0.5;
                }
                else if (q_command_.data[5] - 0.5 * sampling_period_ < 0.0) {
                    dq_output_.data[5] =  -q_command_.data[5]/sampling_period_;
                }
            }else{
                dq_output_.data[5] = 0.0;
            } 
        }

        for(int i=0; i< 6; i++){
                q_command_.data[i] = q_command_.data[i] + dq_output_.data[i] * sampling_period_;
                //RCLCPP_DEBUG_STREAM(n_->get_logger(),"q_command ["<< i <<"]: " << q_command_.data[i]);
        }
        
        q_command_.data[6] = q_command_.data[6] + gripper_vel_.data * sampling_period_;
        if(q_command_.data[6]<= 0.0){
            q_command_.data[6] = 0.0;
        }
        else if(q_command_.data[6]>= 1.0){
            q_command_.data[6] = 1.0;
        }

        command_pub_->publish(q_command_);
    }

    void OutputIntegrator::home()
    {
        double x[6] = {0.0 , 0.436332, 1.48353, 0.0, -0.523599, 0.0};

        if(q_command_.data[5] < (x[5] - 0.001)){
            if(q_command_.data[5] + 0.5 * sampling_period_ <= x[5]){
                dq_output_.data[5] = 0.5;
            }
            else if (q_command_.data[5] + 0.5 * sampling_period_ > x[5]) {
                dq_output_.data[5] = (x[5] - q_command_.data[5])/sampling_period_;
            }
        }else if(q_command_.data[5] > (x[5] + 0.001)){
            if(q_command_.data[5] - 0.5 * sampling_period_ >= x[5]){
                dq_output_.data[5] = - 0.5;
            }
            else if (q_command_.data[5] - 0.5 * sampling_period_ < x[5]) {
                dq_output_.data[5] = (x[5] - q_command_.data[5])/(sampling_period_);
            }
        }else{
            dq_output_.data[5] = 0.0;
        }

        for(int i=4; i >= 0; i--){
            if(q_command_.data[i+1] <= (x[i+1] + 0.001) && q_command_.data[i+1] >= (x[i+1] - 0.001)){
                if(q_command_.data[i] < (x[i] - 0.001)){
                    if(q_command_.data[i] + 0.5 * sampling_period_ <= x[i]){
                        dq_output_.data[i] = 0.5;
                    }
                    else if (q_command_.data[i] + 0.5 * sampling_period_ > x[i]) {
                        dq_output_.data[i] =  (x[i] - q_command_.data[i])/sampling_period_;
                    }
                }else if(q_command_.data[i] > (x[i] + 0.001)){
                    if(q_command_.data[i] - 0.5 * sampling_period_ >= x[i]){
                        dq_output_.data[i] = - 0.5;
                    }
                    else if (q_command_.data[i] - 0.5 * sampling_period_ < x[i] ) {
                        dq_output_.data[i] =  (x[i] - q_command_.data[i])/(sampling_period_);
                    }
                }else{
                    dq_output_.data[i] = 0.0;
                }
            }
            else{
                dq_output_.data[i] = 0.0;
            }
        }
    }

    void OutputIntegrator::zero()
    {
        if(q_command_.data[5] < -0.001){
            if(q_command_.data[5] + 0.5 * sampling_period_ <= 0.0){
                dq_output_.data[5] = 0.5;
            }
            else if (q_command_.data[5] + 0.5 * sampling_period_ > 0.0) {
                dq_output_.data[5] = -q_command_.data[5]/sampling_period_;
            }
        }else if(q_command_.data[5] > 0.001){
            if(q_command_.data[5] - 0.5 * sampling_period_ >= 0.0){
                dq_output_.data[5] = - 0.5;
            }
            else if (q_command_.data[5] - 0.5 * sampling_period_ < 0.0) {
                dq_output_.data[5] = -q_command_.data[5]/sampling_period_;
            }
        }else{
            dq_output_.data[5] = 0.0;
        }

        for(int i=4; i >= 0; i--){
            if(q_command_.data[i+1] <= 0.001 && q_command_.data[i+1] >= -0.001){
                if(q_command_.data[i] < -0.001){
                    if(q_command_.data[i] + 0.5 * sampling_period_ <= 0.0){
                        dq_output_.data[i] = 0.5;
                    }
                    else if (q_command_.data[i] + 0.5 * sampling_period_ > 0.0) {
                        dq_output_.data[i] = -q_command_.data[i]/sampling_period_;
                    }
                }else if(q_command_.data[i] > 0.001){
                    if(q_command_.data[i] - 0.5 * sampling_period_ >= 0.0){
                        dq_output_.data[i] = - 0.5;
                    }
                    else if (q_command_.data[i] - 0.5 * sampling_period_ < 0.0) {
                        dq_output_.data[i] = -q_command_.data[i]/sampling_period_;
                    }
                }else{
                    dq_output_.data[i] = 0.0;
                }
            }
            else{
                dq_output_.data[i] = 0.0;
            }
        }
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
