#include "ros2_control_explorer/input_integrator.h"

namespace space_control
{
    InputIntegrator::InputIntegrator(rclcpp::Node::SharedPtr n)
    : n_(n)
    , x_init_()
    , x_desired_()
    , dx_desired_()
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        //init settings
        max_vel_ = 0.025; 
        max_vel_orientation_ = 0.5; 
        sampling_period_ = 0.01;

        error_ = false;
        end_init_ = false;
        call_service_attempt_ = 0;
        init_attempt_ = 0;
        success_init_ = false;

        //init variables
        x_desired_.position.x() = 0.0;
        x_desired_.position.y() = 0.0;
        x_desired_.position.z() = 0.0;
        x_desired_.orientation.x() = 0.0;
        x_desired_.orientation.y() = 0.0;
        x_desired_.orientation.z() = 0.0;
        x_desired_.orientation.w() = 1.0;

        x_init_.position.x() = 0.0;
        x_init_.position.y() = 0.0;
        x_init_.position.z() = 0.0;
        x_init_.orientation.x() = 0.0;
        x_init_.orientation.y() = 0.0;
        x_init_.orientation.z() = 0.0;
        x_init_.orientation.w() = 1.0;

        //init suscribers
        input_sub_ = n_->create_subscription<geometry_msgs::msg::TwistStamped>("/ros2_control_explorer/input_device_velocity", 10, std::bind(&InputIntegrator::callback_input, this, std::placeholders::_1));

        linear_speed_sub_ = n_->create_subscription<std_msgs::msg::Float64>("/ros2_control_explorer/max_linear_speed", 10, std::bind(&InputIntegrator::callback_linear_speed, this, std::placeholders::_1));
        angular_speed_sub_ = n_->create_subscription<std_msgs::msg::Float64>("/ros2_control_explorer/max_angular_speed", 10, std::bind(&InputIntegrator::callback_angular_speed, this, std::placeholders::_1));

        x_init_client_ = n_->create_client<custom_interfaces::srv::Pose>("/ros2_control_explorer/x_init");
        
        //init publishers
        dx_desired_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>("/ros2_control_explorer/dx_desired", 10);
        x_desired_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>("/ros2_control_explorer/x_desired", 10);

        auto request = std::make_shared<custom_interfaces::srv::Pose::Request>();

        while(init_attempt_< 10 && call_service_attempt_< 10 && success_init_ == false){
            request->ready = true;
        
            while (!x_init_client_->wait_for_service(1s) && error_ == false && call_service_attempt_< 10) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    error_ = true;

                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                call_service_attempt_ += 1;
            }

            if(call_service_attempt_ < 10){
                RCLCPP_INFO_ONCE(rclcpp::get_logger("rclcpp"), "service available");
                auto result = x_init_client_->async_send_request(request);
                if (rclcpp::spin_until_future_complete(n_, result) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto copy_result = result.get();
                    if(copy_result.get()->code_error == 0){   
                        x_init_pose_ =copy_result.get()->pose;
                        x_init_.position.x() = x_init_pose_.position.x;
                        x_init_.position.y() = x_init_pose_.position.y;
                        x_init_.position.z() = x_init_pose_.position.z;
                        x_init_.orientation.w() = x_init_pose_.orientation.w;
                        x_init_.orientation.x() = x_init_pose_.orientation.x;
                        x_init_.orientation.y() = x_init_pose_.orientation.y;
                        x_init_.orientation.z() = x_init_pose_.orientation.z;
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
        if( init_attempt_>= 10){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not initialise joints positions");
            exit(0);
        }
        
        //init x_desired with the simulation
        x_desired_ = x_init_;
        send_input();

        timer_ = n_->create_wall_timer(10ms, std::bind(&InputIntegrator::timer_callback, this));

    }

    void InputIntegrator::callback_input(const geometry_msgs::msg::TwistStamped & msg)
    {   
        dx_input_ = msg;   
    }

    void InputIntegrator::callback_linear_speed(const std_msgs::msg::Float64 & msg)
    {   
        max_vel_= msg.data;
    }

    void InputIntegrator::callback_angular_speed(const std_msgs::msg::Float64 & msg)
    {   
        max_vel_orientation_ = msg.data;
    }

    void InputIntegrator::timer_callback()
    {
       
        tf2::Quaternion q_orig, q_rot, q_new;

        dx_desired_.position.x() = (dx_input_.twist.linear.x * max_vel_);
        dx_desired_.position.y() = (dx_input_.twist.linear.y * max_vel_);
        dx_desired_.position.z() = (dx_input_.twist.linear.z * max_vel_);
        dx_desired_.orientation.w() = (0.0);
        dx_desired_.orientation.x() = (dx_input_.twist.angular.x * max_vel_orientation_);
        dx_desired_.orientation.y() = (dx_input_.twist.angular.y * max_vel_orientation_);
        dx_desired_.orientation.z() = (dx_input_.twist.angular.z * max_vel_orientation_);
            

        //Integrates cartesian velocities to get cartesian positions
        x_desired_.position.x() = (x_desired_.position.x() + dx_desired_.position.x() * sampling_period_ );
        x_desired_.position.y() = (x_desired_.position.y() + dx_desired_.position.y() * sampling_period_ );
        x_desired_.position.z() = (x_desired_.position.z() + dx_desired_.position.z() * sampling_period_);

        //converts in quaternion
        q_orig[0]=x_desired_.orientation.x();
        q_orig[1]=x_desired_.orientation.y();
        q_orig[2]=x_desired_.orientation.z();
        q_orig[3]=x_desired_.orientation.w();

        q_rot.setRPY(dx_desired_.orientation.x() * sampling_period_ , dx_desired_.orientation.y() * sampling_period_, dx_desired_.orientation.z() * sampling_period_ );
                    
        q_new = q_rot * q_orig;
        q_new.normalize();

        x_desired_.orientation.x() = (q_new[0]);
        x_desired_.orientation.y() = (q_new[1]);
        x_desired_.orientation.z() = (q_new[2]);
        x_desired_.orientation.w() = (q_new[3]);

        send_input();
    }

    void InputIntegrator::send_input()
    {
        geometry_msgs::msg::Pose x_desired_pose;
        x_desired_pose.position.x = x_desired_.position.x();
        x_desired_pose.position.y = x_desired_.position.y();
        x_desired_pose.position.z = x_desired_.position.z();
        x_desired_pose.orientation.w = x_desired_.orientation.w();
        x_desired_pose.orientation.x = x_desired_.orientation.x();
        x_desired_pose.orientation.y = x_desired_.orientation.y();
        x_desired_pose.orientation.z = x_desired_.orientation.z();

        geometry_msgs::msg::Pose dx_desired_pose;
        dx_desired_pose.position.x = dx_desired_.position.x();
        dx_desired_pose.position.y = dx_desired_.position.y();
        dx_desired_pose.position.z = dx_desired_.position.z();
        dx_desired_pose.orientation.w = dx_desired_.orientation.w();
        dx_desired_pose.orientation.x = dx_desired_.orientation.x();
        dx_desired_pose.orientation.y = dx_desired_.orientation.y();
        dx_desired_pose.orientation.z = dx_desired_.orientation.z();

        dx_desired_pub_->publish(dx_desired_pose);
        x_desired_pub_->publish(x_desired_pose);

    }
}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    
    auto n = rclcpp::Node::make_shared("input_integrator", node_options);

    InputIntegrator input_integrator(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
