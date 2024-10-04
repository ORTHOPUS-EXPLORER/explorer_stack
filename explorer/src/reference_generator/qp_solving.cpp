
#include "ros2_control_explorer/qp_solving.h"

namespace space_control
{
    QPSolving::QPSolving(rclcpp::Node::SharedPtr n)
    : n_(n)
    , ik_(n, 6)
    , fk_(n, 18)
    , sampling_period_(0.0)
    , q_current_(18)
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

        //init inverse and forward kinematic 
        ik_.init("tool0", sampling_period_);
        fk_.init("tool0");

        //init variables
        dq_output_.data={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        current_pos_.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "right_finger_joint", "right_external_rod_joint_mimic", "right_fingertip_joint_mimic", "left_finger_joint_mimic", "left_external_rod_joint_mimic", "left_fingertip_joint_mimic", "left_wheel_joint", "right_wheel_joint","left_front_wheel_joint","right_front_wheel_joint","left_rear_wheel_joint","right_rear_wheel_joint"};
        current_pos_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_pos_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_pos_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //init subscribers
        current_pos_sub_ = n_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&QPSolving::callback_current_pos_, this, std::placeholders::_1));
        dx_input_sub_ = n_->create_subscription<geometry_msgs::msg::Pose>("/ros2_control_explorer/dx_desired", 10, std::bind(&QPSolving::callback_dx_input_, this, std::placeholders::_1));
        x_input_sub_ = n_->create_subscription<geometry_msgs::msg::Pose>("/ros2_control_explorer/x_desired", 10, std::bind(&QPSolving::callback_x_input_, this, std::placeholders::_1));
        
        //init publishers
        dq_output_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/ros2_control_explorer/dq_output", 10);
        x_init_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>("/ros2_control_explorer/x_init", 10);

        timer_ = n_->create_wall_timer(10ms, std::bind(&QPSolving::timer_callback, this));

        for(int i=0; i< 7; i++){
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
        
        x_init_pub_->publish(x_init_);
    }

    void QPSolving::callback_current_pos_(const sensor_msgs::msg::JointState & msg)
    {   
        int j;
        if(init ==false){
            for (int i=0; i< 18; i++){
                j=0;
                while (current_pos_.name[i]!=msg.name[j] && j<18 ){
                    j++;
                }
                if(current_pos_.name[i]== msg.name[j]){
                    joint_order[i] = j;
                }
            }
            init = true;
        }
       
        current_pos_ = msg;
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

        for(int i=0; i< 7; i++){
            q_current_[i] = current_pos_.position[joint_order[i]];
        }

        x_desired_ = x_input_;
        dx_desired_ = dx_input_;

        //RCLCPP_INFO(n_->get_logger(), "=== Start FK computation...");
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Input joint position :");
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "q_current_           : " << q_current_);
        fk_.setQCurrent(q_current_);
        fk_.resolveForwardKinematic();
        fk_.getXCurrent(x_current_);
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Forward kinematic computes space position : ");
        //RCLCPP_DEBUG_STREAM(n_->get_logger(), "x_current_           : " << x_current_);

        //RCLCPP_INFO(n_->get_logger(), "=== Start IK computation...");
        ik_.setQCurrent(q_current_);
        ik_.setXCurrent(x_current_);
        ik_.resolveInverseKinematic(dq_desired_, dx_desired_, x_desired_, true);
        // RCLCPP_DEBUG_STREAM(n_->get_logger(), "Inverse kinematic computes joint velocity :");
        //RCLCPP_DEBUG_STREAM(n_->get_logger(), "dq_desired_          : " << dq_desired_);

        send_output();
    }

    void QPSolving::send_output()
    {
        for(int i=0; i< 6; i++){
                dq_output_.data[i]=dq_desired_[i];
        }
        dq_output_pub_->publish(dq_output_);
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