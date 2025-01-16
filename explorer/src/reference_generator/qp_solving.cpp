
#include "ros2_control_explorer/qp_solving.h"

namespace space_control
{
    QPSolving::QPSolving(rclcpp::Node::SharedPtr n)
    : n_(n)
    , ik_(n, 6)
    , fk_(n, 20)
    , sampling_period_(0.0)
    , q_current_(20)
    , dq_desired_(6)
    , x_current_()
    , x_input_()
    , dx_input_()
    , x_desired_()
    , dx_desired_()
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        
        //init settings
        sampling_period_ = 0.01;
        init = false;
        wheelchair = false;
        first_use = true;
        go_home = false;
        //init inverse and forward kinematic 
        ik_.init("tool0", sampling_period_);
        fk_.init("tool0");

        //init variables
        dq_output_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        q_command_prec_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        q_init_={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        q_current_debug.data={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        current_pos_.name = {"left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint", "left_wheel_joint", "right_wheel_joint", "left_right_head_joint", "up_down_head_joint", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "left_external_rod_joint_mimic", "left_fingertip_joint_mimic", "left_finger_joint_mimic", "right_external_rod_joint_mimic", "right_fingertip_joint_mimic", "right_finger_joint"};
        current_pos_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_pos_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_pos_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //init subscribers
        current_pos_sub_ = n_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&QPSolving::callback_current_pos_, this, std::placeholders::_1));
        dx_input_sub_ = n_->create_subscription<geometry_msgs::msg::Pose>("/ros2_control_explorer/dx_desired", 10, std::bind(&QPSolving::callback_dx_input_, this, std::placeholders::_1));
        x_input_sub_ = n_->create_subscription<geometry_msgs::msg::Pose>("/ros2_control_explorer/x_desired", 10, std::bind(&QPSolving::callback_x_input_, this, std::placeholders::_1));
        q_command_sub_ = n_->create_subscription<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10, std::bind(&QPSolving::callback_q_command_prec_, this, std::placeholders::_1));
        home_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/home_pressed", 10, std::bind(&QPSolving::callback_home_pressed_, this, std::placeholders::_1));
        x_des_updated_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/ros2_control_explorer/x_des_updated", 10, std::bind(&QPSolving::callback_x_des_updated_, this, std::placeholders::_1));

        //init publishers
        dq_output_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/dq_output", 10);
        x_current_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>("/ros2_control_explorer/x_current", 10);
        q_current_debug_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/debug/q_current", 10);

        timer_ = n_->create_wall_timer(10ms, std::bind(&QPSolving::timer_callback, this));

        x_init_service_ = n_->create_service<custom_interfaces::srv::Pose>("/ros2_control_explorer/x_init", std::bind(&QPSolving::callback_x_init_, this, std::placeholders::_1, std::placeholders::_2));

        q_init_service_ = n_->create_service<custom_interfaces::srv::Float64>("/ros2_control_explorer/q_init", std::bind(&QPSolving::callback_q_init_, this, std::placeholders::_1, std::placeholders::_2));
    }

    void QPSolving::callback_current_pos_(const sensor_msgs::msg::JointState & msg)
    {   
        int j = 0;

        if(init ==false){

            //Identify if there is the wheelchair or not
            while (current_pos_.name[0]!=msg.name[j] && j<20 ){
                j++;
                if(current_pos_.name[0]== msg.name[j]){
                    wheelchair = true;  
                } 
            }

            //Get the order of the joint state for the simulation with the wheelchair
            if(wheelchair){
                for (int i=0; i< 20; i++){
                    j=0;
                    while (current_pos_.name[i]!=msg.name[j] && j<20 ){
                        j++;
                    }
                    if(current_pos_.name[i]== msg.name[j]){
                        RCLCPP_DEBUG_STREAM(n_->get_logger(), current_pos_.name[i] << ": " << j);
                        joint_order[i] = j;
                    }
                }
            }
            //Get the order of the joint state for the simulation of the Explorer only
            else{
                for (int i=0; i< 12; i++){
                    j=0;
                    while (current_pos_.name[i+8]!=msg.name[j] && j<20 ){
                        j++;
                    }
                    if(current_pos_.name[i+8]== msg.name[j]){
                        RCLCPP_DEBUG_STREAM(n_->get_logger(), current_pos_.name[i+8] << ": " << j);
                        joint_order[i] = j;
                    }
                }
            }

            current_pos_ = msg;
            init = true;
        }
        else{
            current_pos_ = msg;
        }
    }
       
    void QPSolving::callback_home_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_home = true;
        }
       
    }

    void QPSolving::callback_x_des_updated_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_home = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_q_command_prec_(const std_msgs::msg::Float64MultiArray & msg)
    {   
        q_command_prec_ = msg;
    }

    void QPSolving::callback_dx_input_(const geometry_msgs::msg::Pose & msg)
    {
        dx_input_.position.x() = msg.position.x;
        dx_input_.position.y() = msg.position.y;
        dx_input_.position.z() = msg.position.z;
        dx_input_.orientation.w() = msg.orientation.w;
        dx_input_.orientation.x() = msg.orientation.x;
        dx_input_.orientation.y() = msg.orientation.y;
        dx_input_.orientation.z() = msg.orientation.z;
    }

    void QPSolving::callback_x_input_(const geometry_msgs::msg::Pose & msg)
    {
        x_input_.position.x() = msg.position.x;
        x_input_.position.y() = msg.position.y;
        x_input_.position.z() = msg.position.z;
        x_input_.orientation.w() = msg.orientation.w;
        x_input_.orientation.x() = msg.orientation.x;
        x_input_.orientation.y() = msg.orientation.y;
        x_input_.orientation.z() = msg.orientation.z;
    }

    void QPSolving::timer_callback()
    {
        while(init==false);
        for(int i=0; i< 20; i++){
           q_current_[i] = current_pos_.position[joint_order[i]];
        }
        if(first_use == true){
            first_use = false;
        }
        else{
            if(wheelchair){  
                for(int i=0; i< 6; i++){
                    q_current_[i+8] = q_command_prec_.data[i];
                }
            }
            else{
                for(int i=0; i< 6; i++){
                    q_current_[i] = q_command_prec_.data[i];
                }
            }
        }
        
        x_desired_ = x_input_;
        dx_desired_ = dx_input_;

        // RCLCPP_INFO(n_->get_logger(), "=== Start FK computation...");
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Input joint position :");
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "q_current_           : " << q_current_);
        fk_.setQCurrent(q_current_);
        fk_.resolveForwardKinematic();
        fk_.getXCurrent(x_current_);
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Forward kinematic computes space position : ");
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "x_current_           : " << x_current_);
        
        if(go_home == false){
            
            // RCLCPP_INFO(n_->get_logger(), "=== Start IK computation...");
            ik_.setQCurrent(q_current_);
            ik_.setXCurrent(x_current_);
            // RCLCPP_DEBUG_STREAM(n_->get_logger(), "x_desired_           : " << x_desired_);
            // RCLCPP_DEBUG_STREAM(n_->get_logger(), "dx_desired_           : " << dx_desired_);
            // RCLCPP_DEBUG_STREAM(n_->get_logger(), "dq_desired_           : " << dq_desired_);
            ik_.resolveInverseKinematic(dq_desired_, dx_desired_, x_desired_, false, wheelchair);
            // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Inverse kinematic computes joint velocity :");
            // RCLCPP_DEBUG_STREAM(n_->get_logger(), "dq_desired_          : " << dq_desired_);
            send_output();
        }
       
        publishDebugTopic_();
        
    }

    void QPSolving::callback_x_init_(const std::shared_ptr<custom_interfaces::srv::Pose::Request> req,
                                           std::shared_ptr<custom_interfaces::srv::Pose::Response> res)
    {
        
        if(init == true){
            for(int i=0; i< 20; i++){
                q_current_[i] = current_pos_.position[joint_order[i]];
            }
            fk_.setQCurrent(q_current_);
            fk_.resolveForwardKinematic();
            fk_.getXCurrent(x_current_);

            x_init_.position.x = x_current_.position.x();
            x_init_.position.y = x_current_.position.y();
            x_init_.position.z = x_current_.position.z();
            x_init_.orientation.w = x_current_.orientation.w();
            x_init_.orientation.x = x_current_.orientation.x();
            x_init_.orientation.y = x_current_.orientation.y();
            x_init_.orientation.z = x_current_.orientation.z();
        
            res->code_error = 0;
        }
        else{
            res->code_error = 1;
        }
        res->pose = x_init_;
    }

    void QPSolving::callback_q_init_(const std::shared_ptr<custom_interfaces::srv::Float64::Request> req,
                                           std::shared_ptr<custom_interfaces::srv::Float64::Response> res)
    {
        if(init == true){ 
            if(wheelchair){  
                for(int i=0; i< 6; i++){
                    q_init_[i] = current_pos_.position[joint_order[i+8]];
                }
                q_init_[6] = current_pos_.position[joint_order[19]];
            }
            else{
                for(int i=0; i< 6; i++){
                    q_init_[i] = current_pos_.position[joint_order[i]];
                }
                q_init_[6] = current_pos_.position[joint_order[11]];
            }
            res->code_error = 0;
        }
        else{
            res->code_error = 1;
        }

        res->data = q_init_;
    }

    void QPSolving::send_output()
    {
        for(int i=0; i< 6; i++){
                dq_output_.data[i]=dq_desired_[i];
        }
        dq_output_pub_->publish(dq_output_);
    }

    void QPSolving::publishDebugTopic_()
    {
        // debug current space position (result of forward kinematic)
        geometry_msgs::msg::Pose x_current_pose;
        x_current_pose.position.x = x_current_.position.x();
        x_current_pose.position.y = x_current_.position.y();
        x_current_pose.position.z = x_current_.position.z();
        x_current_pose.orientation.w = x_current_.orientation.w();
        x_current_pose.orientation.x = x_current_.orientation.x();
        x_current_pose.orientation.y = x_current_.orientation.y();
        x_current_pose.orientation.z = x_current_.orientation.z();
        x_current_pub_->publish(x_current_pose);

        for(int i=0; i< 20; i++){
            q_current_debug.data[i] = q_current_[i];
        }
        q_current_debug_pub_->publish(q_current_debug);

    }
 
}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    
    auto n = rclcpp::Node::make_shared("qp_solving", node_options);

    QPSolving qp_solving(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}