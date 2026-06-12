
#include "explorer_controllers/qp_cartesian/qp_solving.h"

namespace space_control
{
    QPSolving::QPSolving(rclcpp::Node::SharedPtr n)
    : n_(n)
    , ik_(n, 6)
    , fk_(n, 20)
    , q_current_(20), dq_desired_(6), x_current_(), x_input_(), dx_input_(), x_desired_(), dx_desired_(), sampling_period_(0.01), init_(false), wheelchair_(false), first_use_(true), go_home_(false)
    , go_zero_(false)
    , go_J1_zero_(false)
    , go_J2_zero_(false)
    , go_J3_zero_(false)
    , go_J4_zero_(false)
    , go_J5_zero_(false)
    , go_J6_zero_(false)
    {
        rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        
        // init node parameters
        controller_position_topic_name_ = n_->get_parameter("controller_position_topic_name").as_string();
        if (controller_position_topic_name_.empty()) {
            throw std::runtime_error(
                "Parameter 'controller_position_topic_name' is required");
        }

        // Load movement detection parameters for global drift prevention
        if (!n_->get_parameter("enable_movement_detection_global", enable_movement_detection_global_)) {
            enable_movement_detection_global_ = true;  // Default: enabled
            RCLCPP_INFO(n_->get_logger(), "[qp_solving] Using default enable_movement_detection_global: %s", enable_movement_detection_global_ ? "true" : "false");
        } else {
            RCLCPP_INFO(n_->get_logger(), "[qp_solving] Loaded enable_movement_detection_global: %s", enable_movement_detection_global_ ? "true" : "false");
        }

        if (!n_->get_parameter("movement_detection_threshold_global", movement_detection_threshold_global_)) {
            movement_detection_threshold_global_ = 1e-3;  // Default: small threshold
            RCLCPP_INFO(n_->get_logger(), "[qp_solving] Using default movement_detection_threshold_global: %.2e", movement_detection_threshold_global_);
        } else {
            RCLCPP_INFO(n_->get_logger(), "[qp_solving] Loaded movement_detection_threshold_global: %.2e", movement_detection_threshold_global_);
        }
        
        //init inverse and forward kinematic 
        ik_.init("tool0", sampling_period_);
        fk_.init("tool0");

        //init variables
        dq_output_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        q_command_prec_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        q_init_={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        q_current_debug_.data={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //current_pos_.name = {"left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint", "left_wheel_joint", "right_wheel_joint", "left_right_head_joint", "up_down_head_joint", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "left_external_rod_joint_mimic", "left_fingertip_joint_mimic", "left_finger_joint_mimic", "right_external_rod_joint_mimic", "right_fingertip_joint_mimic", "right_finger_joint"};
        current_pos_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_pos_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        current_pos_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //init subscribers
        current_pos_sub_ = n_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS(), std::bind(&QPSolving::callback_current_pos_, this, std::placeholders::_1));
        dx_input_sub_ = n_->create_subscription<geometry_msgs::msg::Pose>("/explorer_controllers/input_integrator/dx_desired", 10, std::bind(&QPSolving::callback_dx_input_, this, std::placeholders::_1));
        x_input_sub_ = n_->create_subscription<geometry_msgs::msg::Pose>("/explorer_controllers/input_integrator/x_desired", 10, std::bind(&QPSolving::callback_x_input_, this, std::placeholders::_1));
        q_command_sub_ = n_->create_subscription<std_msgs::msg::Float64MultiArray>(controller_position_topic_name_, 10, std::bind(&QPSolving::callback_q_command_prec_, this, std::placeholders::_1));
        home_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/home_pressed", 10, std::bind(&QPSolving::callback_home_pressed_, this, std::placeholders::_1));
        home_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/home_released", 10, std::bind(&QPSolving::callback_home_released_, this, std::placeholders::_1));
        zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/zero_pressed", 10, std::bind(&QPSolving::callback_zero_pressed_, this, std::placeholders::_1));
        zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/zero_released", 10, std::bind(&QPSolving::callback_zero_released_, this, std::placeholders::_1));
        J1_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J1_zero_pressed", 10, std::bind(&QPSolving::callback_J1_zero_pressed_, this, std::placeholders::_1));
        J1_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J1_zero_released", 10, std::bind(&QPSolving::callback_J1_zero_released_, this, std::placeholders::_1));
        J2_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J2_zero_pressed", 10, std::bind(&QPSolving::callback_J2_zero_pressed_, this, std::placeholders::_1));
        J2_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J2_zero_released", 10, std::bind(&QPSolving::callback_J2_zero_released_, this, std::placeholders::_1));
        J3_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J3_zero_pressed", 10, std::bind(&QPSolving::callback_J3_zero_pressed_, this, std::placeholders::_1));
        J3_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J3_zero_released", 10, std::bind(&QPSolving::callback_J3_zero_released_, this, std::placeholders::_1));
        J4_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J4_zero_pressed", 10, std::bind(&QPSolving::callback_J4_zero_pressed_, this, std::placeholders::_1));
        J4_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J4_zero_released", 10, std::bind(&QPSolving::callback_J4_zero_released_, this, std::placeholders::_1));
        J5_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J5_zero_pressed", 10, std::bind(&QPSolving::callback_J5_zero_pressed_, this, std::placeholders::_1));
        J5_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J5_zero_released", 10, std::bind(&QPSolving::callback_J5_zero_released_, this, std::placeholders::_1));
        J6_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J6_zero_pressed", 10, std::bind(&QPSolving::callback_J6_zero_pressed_, this, std::placeholders::_1));
        J6_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_user_interfaces/rqt_armcontrol/J6_zero_released", 10, std::bind(&QPSolving::callback_J6_zero_released_, this, std::placeholders::_1));
        x_des_updated_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/explorer_controllers/input_integrator/x_des_updated", 10, std::bind(&QPSolving::callback_x_des_updated_, this, std::placeholders::_1));
        control_frame_sub_ = n_->create_subscription<explorer_msgs::msg::ControlFrameSelection>("/explorer_controllers/command_node/control_frame_selection", 10, std::bind(&QPSolving::callback_control_frame_selection_, this, std::placeholders::_1));
        reset_sub_ = n_->create_subscription<std_msgs::msg::Bool>("/command_node/reset_qp_solving", 10, std::bind(&QPSolving::callback_reset_, this, std::placeholders::_1));

        //init publishers
        dq_output_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/explorer_controllers/qp_solving/dq_output", 10);
        x_current_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>("/explorer_controllers/qp_solving/x_current", 10);
        q_current_debug_pub_ = n_->create_publisher<std_msgs::msg::Float64MultiArray>("/explorer_controllers/qp_solving/debug/q_current", 10);

        timer_ = n_->create_wall_timer(10ms, std::bind(&QPSolving::timer_callback_, this));

        x_init_service_ = n_->create_service<explorer_msgs::srv::Pose>("/explorer_controllers/qp_solving/x_init", std::bind(&QPSolving::callback_x_init_, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(n_->get_logger(), "x_init [input integrator]: creating service ...");
        q_init_service_ = n_->create_service<explorer_msgs::srv::Float64>("/explorer_controllers/qp_solving/q_init", std::bind(&QPSolving::callback_q_init_, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(n_->get_logger(), "q_init [output integrator]: creating service ...");
    }

    
    void QPSolving::callback_current_pos_(const sensor_msgs::msg::JointState & msg) {
        if (!init_) {
            current_pos_ = msg;
            // Check for explorer
            bool valid_explorer = std::all_of(expected_names_explorer_.begin(), expected_names_explorer_.end(), [&](const std::string &name) {
                if (
                    name == "left_external_rod_joint_mimic" ||
                    name == "left_fingertip_joint_mimic" ||
                    name == "left_finger_joint_mimic" ||
                    name == "right_external_rod_joint_mimic" ||
                    name == "right_fingertip_joint_mimic"
                ) {
                    // allow them to be missing
                    return true;
                }
                return std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end();
            });

            // Check for wheelchair
            bool all_wheelchair =
                std::all_of(expected_names_wheelchair_.begin(), expected_names_wheelchair_.end(),
                            [&](const std::string &name) {
                                return std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end();
                            });

            // Check for any wheelchair joints present
            bool any_wheelchair =
                std::any_of(expected_names_wheelchair_.begin(), expected_names_wheelchair_.end(),
                            [&](const std::string &name) {
                                return std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end();
                            });

            // CASES:
            if (valid_explorer && all_wheelchair) {
                mode_ = Mode::FULL;
                RCLCPP_INFO(n_->get_logger(), "[qp_solving] Full robot (explorer + wheelchair) detected.");
            } else if (valid_explorer && !any_wheelchair) {
                mode_ = Mode::EXPLORER;
                RCLCPP_INFO(n_->get_logger(), "[qp_solving] Explorer-only configuration detected.");
            } else {
                mode_ = Mode::INVALID;
                RCLCPP_ERROR(n_->get_logger(), "[qp_solving] Invalid joint configuration detected! Initialization failed.");
                // Optionally, handle the error (throw, return, etc)
                // return;
            }

            if (valid_explorer && all_wheelchair) {
                mode_ = Mode::FULL;
                RCLCPP_INFO(n_->get_logger(), "[qp_solving] Full robot (explorer + wheelchair) detected.");
                
                // Build order: wheelchair first, then explorer
                joint_order_.clear();
                joint_order_.reserve(expected_names_wheelchair_.size() + expected_names_explorer_.size());
                for (const auto &name : expected_names_wheelchair_) {
                    auto it = std::find(msg.name.begin(), msg.name.end(), name);
                    joint_order_.push_back(std::distance(msg.name.begin(), it));
                }
                for (const auto &name : expected_names_explorer_) {
                    auto it = std::find(msg.name.begin(), msg.name.end(), name);
                    if (it != msg.name.end()) {
                        joint_order_.push_back(std::distance(msg.name.begin(), it));
                    } else if (
                        name == "left_external_rod_joint_mimic" ||
                        name == "left_fingertip_joint_mimic" ||
                        name == "left_finger_joint_mimic" ||
                        name == "right_external_rod_joint_mimic" ||
                        name == "right_fingertip_joint_mimic"
                    ) {
                        // fallback to "right_finger_joint"
                        auto fallback_it = std::find(msg.name.begin(), msg.name.end(), "right_finger_joint");
                        if (fallback_it != msg.name.end()) {
                            joint_order_.push_back(std::distance(msg.name.begin(), fallback_it));
                            RCLCPP_WARN(n_->get_logger(), "[qp_solving] Joint %s missing, using right_finger_joint as fallback", name.c_str());
                        } else {
                            RCLCPP_ERROR(n_->get_logger(), "[qp_solving] Neither %s nor right_finger_joint found! Cannot initialize properly", name.c_str());
                        }
                    } else {
                        RCLCPP_ERROR(n_->get_logger(), "[qp_solving] Joint %s not found!", name.c_str());
                    }
                }
                init_ = true;
                return;
            } else if (valid_explorer && !any_wheelchair) {
                mode_ = Mode::EXPLORER;
                RCLCPP_INFO(n_->get_logger(), "[qp_solving] Explorer-only configuration detected.");
                
                // Build order: just the explorer
                joint_order_.clear();
                joint_order_.reserve(expected_names_explorer_.size());
                for (const auto &name : expected_names_explorer_) {
                    auto it = std::find(msg.name.begin(), msg.name.end(), name);
                    if (it != msg.name.end()) {
                        joint_order_.push_back(std::distance(msg.name.begin(), it));
                    } else if (
                        name == "left_external_rod_joint_mimic" ||
                        name == "left_fingertip_joint_mimic" ||
                        name == "left_finger_joint_mimic" ||
                        name == "right_external_rod_joint_mimic" ||
                        name == "right_fingertip_joint_mimic"
                    ) {
                        auto fallback_it = std::find(msg.name.begin(), msg.name.end(), "right_finger_joint");
                        if (fallback_it != msg.name.end()) {
                            joint_order_.push_back(std::distance(msg.name.begin(), fallback_it));
                            RCLCPP_WARN(n_->get_logger(), "[qp_solving] Joint %s missing, using right_finger_joint as fallback", name.c_str());
                        } else {
                            RCLCPP_ERROR(n_->get_logger(), "[qp_solving] Neither %s nor right_finger_joint found! Cannot initialize properly", name.c_str());
                            return;
                        }
                    } else {
                        RCLCPP_ERROR(n_->get_logger(), "[qp_solving] Joint %s not found and no fallback defined", name.c_str());
                        return;
                    }
                }

                // Debug: Print out sizes to check bounds
                RCLCPP_INFO(n_->get_logger(), "joint_order.size() = %zu", joint_order_.size());
                RCLCPP_INFO(n_->get_logger(), "current_pos_.name.size() = %zu", current_pos_.name.size());

                // Decide safe upper bound
                int safe_limit = std::min<int>(joint_order_.size(), current_pos_.name.size());
                int n_to_print = std::min<int>(safe_limit, (wheelchair_ ? 20 : 12));

                for (int i = 0; i < n_to_print; i++) {
                    RCLCPP_INFO(n_->get_logger(), "Joint order[%d]: %d, Name: %s", 
                                i, 
                                joint_order_[i], 
                                current_pos_.name[joint_order_[i]].c_str());
                }
                // If there's a mismatch, warn
                if (joint_order_.size() < static_cast<size_t>(n_to_print) || current_pos_.name.size() < static_cast<size_t>(n_to_print)) {
                    RCLCPP_WARN(n_->get_logger(), "WARNING: joint_order or current_pos_.name was smaller than expected! Potential config problem.");
                }
                init_ = true;
                RCLCPP_INFO(n_->get_logger(), "[qp_solving] Init done.");
                return;
            }
        }

        current_pos_ = msg;
    }
       
    void QPSolving::callback_home_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_home_ = true;
        }
        else{
            go_home_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_home_released_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_home_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_zero_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_zero_ = true;
        }
        else{
            go_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_zero_released_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J1_zero_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J1_zero_ = true;
        }
        else{
            go_J1_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J1_zero_released_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J1_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J2_zero_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J2_zero_ = true;
        }
        else{
            go_J2_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J2_zero_released_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J2_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J3_zero_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J3_zero_ = true;
        }
        else{
            go_J3_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J3_zero_released_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J3_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J4_zero_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J4_zero_ = true;
        }
        else{
            go_J4_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J4_zero_released_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J4_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J5_zero_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J5_zero_ = true;
        }
        else{
            go_J5_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J5_zero_released_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J5_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J6_zero_pressed_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J6_zero_ = true;
        }
        else{
            go_J6_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_J6_zero_released_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_J6_zero_ = false;
            ik_.reset();
        }
       
    }

    void QPSolving::callback_x_des_updated_(const std_msgs::msg::Bool & msg)
    {   
        if(msg.data == true){
            go_home_ = false;
            go_zero_ = false;
            go_J1_zero_ = false;
            go_J2_zero_ = false;
            go_J3_zero_ = false;
            go_J4_zero_ = false;
            go_J5_zero_ = false;
            go_J6_zero_ = false;
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

    void QPSolving::timer_callback_()
    {
        if (!init_) {
            static int warn_cnt = 0;
            if (++warn_cnt % 50 == 0) {
                RCLCPP_WARN(n_->get_logger(), "Waiting for /joint_states init...");
            }
            return;
        }

        // Make sure q_current_ is the right size
        if (q_current_.size() != joint_order_.size()) {
            q_current_.resize(joint_order_.size(), 0.0);
        }

        // Fill q_current_ in correct order
        for (size_t i = 0; i < joint_order_.size(); ++i) {
            q_current_[i] = current_pos_.position[joint_order_[i]];
        }

        size_t wc_size = expected_names_wheelchair_.size();
        size_t explorer_size = expected_names_explorer_.size();

        if (first_use_) {
            first_use_ = false;
        } else {
            if (mode_ == Mode::FULL) {
                for (size_t i = 0; i < explorer_size; ++i) {
                    q_current_[wc_size + i] = q_command_prec_.data[i];
                }
            } else if (mode_ == Mode::EXPLORER) {
                for (size_t i = 0; i < explorer_size; ++i) {
                    q_current_[i] = q_command_prec_.data[i];
                }
            }
        }

        x_desired_ = x_input_;
        dx_desired_ = dx_input_;

        fk_.setQCurrent(q_current_);
        fk_.resolveForwardKinematic();
        fk_.getXCurrent(x_current_);

        if (!go_home_ && !go_zero_ && !go_J1_zero_ && !go_J2_zero_ && !go_J3_zero_ && !go_J4_zero_ && !go_J5_zero_ && !go_J6_zero_) {
            // Check if movement detection is enabled
            if (enable_movement_detection_global_) {
                // Check if there is actual user input (velocity command from joystick)
                // Calculate input velocity magnitude to detect intentional movement
                double input_velocity_magnitude = std::sqrt(
                    dx_input_.position.x() * dx_input_.position.x() +
                    dx_input_.position.y() * dx_input_.position.y() +
                    dx_input_.position.z() * dx_input_.position.z() +
                    dx_input_.orientation.x() * dx_input_.orientation.x() +
                    dx_input_.orientation.y() * dx_input_.orientation.y() +
                    dx_input_.orientation.z() * dx_input_.orientation.z()
                );

                bool user_input_detected = input_velocity_magnitude > movement_detection_threshold_global_;

                if (user_input_detected) {
                    // User is actively commanding movement - run IK solver
                    ik_.setQCurrent(q_current_);
                    ik_.setXCurrent(x_current_);
                    ik_.resolveInverseKinematic(dq_desired_, dx_desired_, x_desired_, false, mode_ == Mode::FULL);
                    send_output_();
                } else {
                    // No user input - send zero velocities to prevent drift
                    for (int i = 0; i < 6; i++) {
                        dq_output_.data[i] = 0.0;
                    }
                    dq_output_pub_->publish(dq_output_);
                }
            } else {
                // Movement detection disabled - always run IK solver (legacy behavior)
                ik_.setQCurrent(q_current_);
                ik_.setXCurrent(x_current_);
                ik_.resolveInverseKinematic(dq_desired_, dx_desired_, x_desired_, false, mode_ == Mode::FULL);
                send_output_();
            }
        }
        publishDebugTopic_();
    }

    void QPSolving::callback_x_init_(const std::shared_ptr<explorer_msgs::srv::Pose::Request> req,
                                           std::shared_ptr<explorer_msgs::srv::Pose::Response> res)
    {
        //RCLCPP_INFO(n_->get_logger(), "x_init [input integrator]: service called");
        (void)req;
        if(init_ == true){
            for(int i=0; i< joint_order_.size(); i++){
                q_current_[i] = current_pos_.position[joint_order_[i]];
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
            RCLCPP_INFO(n_->get_logger(), "service sent X ");
        }
        else{
            res->code_error = 1;
            //RCLCPP_INFO(n_->get_logger(), "service FAILED to send X ");
        }
        res->pose = x_init_;
    }

    void QPSolving::callback_q_init_(const std::shared_ptr<explorer_msgs::srv::Float64::Request> req,
                                           std::shared_ptr<explorer_msgs::srv::Float64::Response> res)
    {
        //RCLCPP_INFO(n_->get_logger(), "q_init [output integrator]: service called");
        if(init_ == true){ 
            if(wheelchair_){  
                for(int i=0; i< 6; i++){
                    q_init_[i] = current_pos_.position[joint_order_[i+8]];
                }
                q_init_[6] = current_pos_.position[joint_order_[19]];
            }
            else{
                for(int i=0; i< 6; i++){
                    q_init_[i] = current_pos_.position[joint_order_[i]];
                }
                q_init_[6] = current_pos_.position[joint_order_[11]];
            }
            res->code_error = 0;
            RCLCPP_INFO(n_->get_logger(), "service sent q ");
        }
        else{
            res->code_error = 1;
            //RCLCPP_INFO(n_->get_logger(), "service FAILED to send q ");
        }

        res->data = q_init_;
    }

    void QPSolving::send_output_()
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
            q_current_debug_.data[i] = q_current_[i];
        }
        q_current_debug_pub_->publish(q_current_debug_);

    }

    void QPSolving::callback_control_frame_selection_(const explorer_msgs::msg::ControlFrameSelection::SharedPtr msg)
    {
        // Convert message frame enum to internal enum
        auto convertFrameToInternal = [this](uint8_t msg_frame) -> space_control::InverseKinematic::ControlFrame {
            switch(msg_frame) {
                case explorer_msgs::msg::ControlFrameSelection::FRAME_WORLD: 
                    return space_control::InverseKinematic::ControlFrame::World;
                case explorer_msgs::msg::ControlFrameSelection::FRAME_TOOL: 
                    return space_control::InverseKinematic::ControlFrame::Tool;
                case explorer_msgs::msg::ControlFrameSelection::FRAME_DRINK_SMALL: 
                    return space_control::InverseKinematic::ControlFrame::DrinkSmall;
                case explorer_msgs::msg::ControlFrameSelection::FRAME_DRINK_BIG: 
                    return space_control::InverseKinematic::ControlFrame::DrinkBig;
                default: 
                    RCLCPP_WARN(n_->get_logger(), "Unknown frame type %d, defaulting to World", msg_frame);
                    return space_control::InverseKinematic::ControlFrame::World;
            }
        };

        auto convertFrameToString = [](uint8_t msg_frame) -> std::string {
            switch(msg_frame) {
                case explorer_msgs::msg::ControlFrameSelection::FRAME_WORLD: return "World";
                case explorer_msgs::msg::ControlFrameSelection::FRAME_TOOL: return "Tool";
                case explorer_msgs::msg::ControlFrameSelection::FRAME_DRINK_SMALL: return "DrinkSmall";
                case explorer_msgs::msg::ControlFrameSelection::FRAME_DRINK_BIG: return "DrinkBig";
                default: return "Unknown";
            }
        };
        
        // Convert requested frames to internal format
        auto requested_position_frame = convertFrameToInternal(msg->position_control_frame);
        auto requested_orientation_frame = convertFrameToInternal(msg->orientation_control_frame);
        
        // Get current frames
        auto current_position_frame = ik_.getPositionControlFrame();
        auto current_orientation_frame = ik_.getOrientationControlFrame();
        
        // Only update and log if frames are different
        bool position_changed = (current_position_frame != requested_position_frame);
        bool orientation_changed = (current_orientation_frame != requested_orientation_frame);
        
        if (position_changed || orientation_changed) {
            // Set frames directly in the inverse kinematics solver
            if (position_changed) {
                ik_.setPositionControlFrame(requested_position_frame);
            }
            if (orientation_changed) {
                ik_.setOrientationControlFrame(requested_orientation_frame);
            }
            
            // Log the change
            RCLCPP_INFO(n_->get_logger(), 
                       "Control frames updated via topic - Position: %s%s, Orientation: %s%s",
                       convertFrameToString(msg->position_control_frame).c_str(),
                       position_changed ? " (changed)" : "",
                       convertFrameToString(msg->orientation_control_frame).c_str(),
                       orientation_changed ? " (changed)" : "");
        }
        // If no frames changed, we don't log anything to avoid flooding
    }

    void QPSolving::callback_reset_(const std_msgs::msg::Bool & msg) {

        if(msg.data == true) {
            ik_.reset();
            fk_.reset();
            first_use_ = true;
            for (size_t i = 0; i < joint_order_.size(); ++i) {
                q_current_[i] = current_pos_.position[joint_order_[i]];
            }
        }
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
