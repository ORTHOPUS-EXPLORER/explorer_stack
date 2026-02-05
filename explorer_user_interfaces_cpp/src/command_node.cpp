#include "explorer_user_interfaces_cpp/command_node.h"

namespace space_control
{
    CommandNode::CommandNode(rclcpp::Node::SharedPtr n)
    : n_(n),
      button_handler(),
      trajectory_manager(),
      controller_switcher(n)
    {
        RCLCPP_INFO(n->get_logger(), "CommandNode constructor");

        n_->declare_parameter<int>("button_threshold_ms", 500);
        n_->declare_parameter<double>("speed_change_threshold", 0.95);
        n_->declare_parameter<double>("speed_level_multiplier", 0.25);
        n_->declare_parameter<double>("sampling_period", 0.01);

        button_threshold_ms = n_->get_parameter("button_threshold_ms").as_int();
        speed_change_threshold = n_->get_parameter("speed_change_threshold").as_double();
        speed_level_multiplier = n_->get_parameter("speed_level_multiplier").as_double();
        sampling_period_ = n_->get_parameter("sampling_period").as_double();

        button_handler.init(button_threshold_ms);

        // Map control behaviors to corresponding functions
        control_behaviors_ = {
            {"cartesian_X",       std::bind(&CommandNode::cartesian_linear, this, std::placeholders::_1)},
            {"cartesian_Y",       std::bind(&CommandNode::cartesian_linear, this, std::placeholders::_1)},
            {"cartesian_Z",       std::bind(&CommandNode::cartesian_linear, this, std::placeholders::_1)},
            {"rotation_X",        std::bind(&CommandNode::cartesian_rotation, this, std::placeholders::_1)},
            {"rotation_Y",        std::bind(&CommandNode::cartesian_rotation, this, std::placeholders::_1)},
            {"rotation_Z",        std::bind(&CommandNode::cartesian_rotation, this, std::placeholders::_1)},
            {"joint_1",      std::bind(&CommandNode::joint_direct, this, std::placeholders::_1)},
            {"joint_2",      std::bind(&CommandNode::joint_direct, this, std::placeholders::_1)},
            {"joint_3",      std::bind(&CommandNode::joint_direct, this, std::placeholders::_1)},
            {"joint_4",      std::bind(&CommandNode::joint_direct, this, std::placeholders::_1)},
            {"joint_5",      std::bind(&CommandNode::joint_direct, this, std::placeholders::_1)},
            {"joint_6",      std::bind(&CommandNode::joint_direct, this, std::placeholders::_1)},
            {"change_speed",      std::bind(&CommandNode::change_speed, this, std::placeholders::_1)},
            {"drink",      std::bind(&CommandNode::drink, this, std::placeholders::_1)},
            {"gripper",      std::bind(&CommandNode::gripper, this, std::placeholders::_1)},
            {"complex_X",      std::bind(&CommandNode::complex, this, std::placeholders::_1)},
            {"complex_Y",      std::bind(&CommandNode::complex, this, std::placeholders::_1)},
            {"trajectory_control",      std::bind(&CommandNode::trajectory_control, this, std::placeholders::_1)}
        };

        n_->declare_parameter<std::string>("mode_file", "");
        n_->declare_parameter<std::string>("trajectory_file", "");
        n_->declare_parameter<bool>("active_trajectory", true);

        // Get the value of the mode_file parameter
        std::string mode_file;
        n_->get_parameter("mode_file", mode_file);

        // Load mode configuration from YAML file
        data = loadModeData(mode_file);

        if (!validateModeData(data)) {
            RCLCPP_FATAL(n_->get_logger(),
                         "YAML configuration validation failed. Shutting down node.");
            rclcpp::shutdown();
            return;
        }

        n_->get_parameter("active_trajectory", active_trajectory_);

        if(active_trajectory_) {
            RCLCPP_INFO(n_->get_logger(), "Active trajectory control mode enabled.");
            bool exists = false;

            for (const auto& [mode_name, mode] : data.button_modes_map) {
                for (const auto& axis : mode.axes) {
                    if (axis.control_name == "trajectory_control") {
                        exists = true;
                        break;
                    }
                }
                if (exists)
                    break;
            }
            
            if (exists) {
                // Get the value of the trajectory_file parameter
                std::string trajectory_file;
                n_->get_parameter("trajectory_file", trajectory_file);
                
                if (!trajectory_manager.loadTrajectory(trajectory_file)) {
                    RCLCPP_FATAL(n_->get_logger(),
                                "YAML trajectory configuration failed. Shutting down node.");
                    rclcpp::shutdown();
                    return;
                }

                if(!trajectory_manager.validateTrajectory()){
                    RCLCPP_FATAL(n_->get_logger(),
                                "YAML trajectory validation failed. Shutting down node.");
                    rclcpp::shutdown();
                    return;
                }

                lock_ = true;
            }
            else{
                lock_ = false;
            }
        } 
        else {
            lock_ = false;
        }


        // Initialize subscribers and publishers
        joy_sub_ = n->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&CommandNode::callback_joystick, this, std::placeholders::_1));
        x_current_sub_ = n_->create_subscription<geometry_msgs::msg::Pose>("/explorer_controllers/qp_solving/x_current", 10, std::bind(&CommandNode::callback_x_current, this, std::placeholders::_1));
        q_current_sub_ = n_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&CommandNode::callback_q_current_, this, std::placeholders::_1));
        q_forward_controller_sub = n_->create_subscription<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10, std::bind(&CommandNode::callback_q_forward_controller, this, std::placeholders::_1));

        joint_vel_pub_ = n->create_publisher<std_msgs::msg::Float64MultiArray>("command_node/joint_velocity_command", 10);
        cartesian_vel_pub_ = n->create_publisher<geometry_msgs::msg::TwistStamped>("command_node/cartesian_velocity_command", 10);
        mode_name_pub_ = n->create_publisher<std_msgs::msg::String>("command_node/mode_name", 10);
        speed_level_pub_ = n->create_publisher<std_msgs::msg::Int32>("command_node/speed_level", 10);
        frame_id_pub_ = n->create_publisher<explorer_msgs::msg::ControlFrameSelection>("/explorer_controllers/command_node/control_frame_selection", 10);
        gripper_pub_ = n->create_publisher<std_msgs::msg::Float64>("command_node/gripper_velocity_command", 10);
        trajectory_pub_ = n_->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
        reset_qp_solving_pub_ = n_->create_publisher<std_msgs::msg::Bool>("/command_node/reset_qp_solving", 10);
        retract_status_pub_ = n_->create_publisher<std_msgs::msg::String>("command_node/retract_status", 10);

        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(n_, "qp_solving");

        // Initialize speed control variables
        speed_factor = 1.0;
        speed_level = 2;
        joy_prec = 0.0;

        complex_mode_ = false;
        rotation_speed_scale = 1.0;

        // Initialize Cartesian and joint velocities to zero
        resetVelocities();

        frame_id_.position_control_frame = 0;
        frame_id_.orientation_control_frame = 0;

        retract_status_pub_->publish(std_msgs::msg::String().set__data("retracted"));

        status_prec = "retracted";

        // Timer callback
        timer_ = n_->create_wall_timer(std::chrono::duration<double>(sampling_period_), std::bind(&CommandNode::timer, this));
    }

    // Load mode configuration from YAML file
    ModeData CommandNode::loadModeData(const std::string& filename) {

        auto getDefaultModeData = []() -> ModeData {
            ModeData default_data;
            default_data.mode_info.name = "default";
            default_data.mode_info.display_name = "Default Mode";
            default_data.mode_info.description = "Fallback mode";
            return default_data;
        };

        YAML::Node root;
        try {
            root = YAML::LoadFile(filename);
        } catch (const YAML::BadFile& e) {
            RCLCPP_ERROR(n_->get_logger(), "Cannot open mode_file: %s", filename.c_str());
            return getDefaultModeData();
        } catch (const YAML::ParserException& e) {
            RCLCPP_ERROR(n_->get_logger(), "YAML parsing error in file %s: %s", filename.c_str(), e.what());
            return getDefaultModeData();
        }
        
        // Parse mode information
        if (root["mode_info"]) {
            auto info = root["mode_info"];
            data.mode_info.name = info["name"].as<std::string>("");
            data.mode_info.display_name = info["display_name"].as<std::string>("");
            data.mode_info.description = info["description"].as<std::string>("");
        }

        // Parse button modes and their configurations
        if (root["button_mappings"]) {
            auto mappings = root["button_mappings"];
            bool first = true;

            for (auto it = mappings.begin(); it != mappings.end(); ++it) {
                ButtonMode mode;
                mode.name = it->first.as<std::string>();
                auto button_mode = it->second;

                if (first) {
                    current_mode_name = mode.name;
                    first = false;
                }

                // axes
                if (button_mode["axes"]) {
                    for (auto axis_node : button_mode["axes"]) {
                        AxisInfo axis;
                        axis.control_name = axis_node["control_name"].as<std::string>("");
                        axis.joystick_axis = axis_node["joystick_axis"].as<std::string>("");
                        axis.direction = axis_node["direction"].as<int>(1);
                        axis.scale = axis_node["scale"].as<double>(1.0);
                        
                        // Parse smoothing_alpha (default 1.0 = no smoothing)
                        axis.smoothing_alpha = axis_node["smoothing_alpha"].as<double>(1.0);
                        // Clamp alpha to valid range [0.0, 1.0]
                        axis.smoothing_alpha = std::max(0.0, std::min(1.0, axis.smoothing_alpha));

                        if (axis_node["params"]) {
                            for (auto p : axis_node["params"]) {
                                axis.params[p.first.as<std::string>()] = p.second.as<double>();
                            }
                        }
                        mode.axes.push_back(axis);
                    }
                }

                // buttons
                if (button_mode["button"]) {
                    ButtonAction action;  
                    for (auto button_action_node : button_mode["button"]) {
                        if (button_action_node["long_click"] && !button_action_node["long_click"].as<std::string>().empty())
                            action.long_click = button_action_node["long_click"].as<std::string>();
                        if (button_action_node["short_click"] && !button_action_node["short_click"].as<std::string>().empty())
                            action.short_click = button_action_node["short_click"].as<std::string>();
                    }
                    mode.buttons = action;
                } 

                data.button_modes_map[mode.name] = mode;
            }
        }

        return data;
    }

    bool CommandNode::validateModeData(const ModeData& data)
    {
        // --- mode_info verification ---
        if (data.mode_info.name.empty() || data.mode_info.display_name.empty()) {
            RCLCPP_ERROR(n_->get_logger(), 
                "Invalid YAML: mode_info.name or display_name missing");
            return false;
        }
        // --- button_modes_map verification ---
        if (data.button_modes_map.empty()) {
            RCLCPP_ERROR(n_->get_logger(),
                "Invalid YAML: button_mappings must contain at least one mode");
            return false;
        }

        // Valid control names
        std::unordered_set<std::string> valid_control_names;
        for(const auto& kv : control_behaviors_) valid_control_names.insert(kv.first);

        // Valid joystick axes
        std::unordered_set<std::string> valid_axes = {"ax1", "ax2"};

        // --- Validate each button mode ---
        for (auto& [name, mode] : data.button_modes_map) {

            // Axes check
            for (const auto& axis : mode.axes) {

                // Special case: empty control_name means inactive axis
                if (axis.control_name.empty()) {
                    if (axis.direction != 0) {
                        RCLCPP_ERROR(n_->get_logger(),
                            "Invalid axis in mode '%s': direction must be 0 when control_name is empty",
                            name.c_str());
                        return false;
                    }
                    if (axis.scale != 0.0) {
                        RCLCPP_ERROR(n_->get_logger(),
                            "Invalid axis in mode '%s': scale must be 0 when control_name is empty",
                            name.c_str());
                        return false;
                    }
                    if (!axis.joystick_axis.empty()) {
                        RCLCPP_ERROR(n_->get_logger(),
                            "Invalid axis in mode '%s': joystick_axis must be empty when control_name is empty",
                            name.c_str());
                        return false;
                    }
                    // skip the rest of validation for this axis
                    continue;
                }
            
                // Normal validations
                // control_name verification
                if (!valid_control_names.count(axis.control_name)) {
                    RCLCPP_ERROR(n_->get_logger(),
                        "Invalid control_name '%s' in mode '%s'",
                        axis.control_name.c_str(), name.c_str());
                    return false;
                }
                // joystick_axis verification
                if (!valid_axes.count(axis.joystick_axis)) {
                    RCLCPP_ERROR(n_->get_logger(),
                        "Invalid joystick_axis '%s' in mode '%s' (must be ax1 or ax2)",
                        axis.joystick_axis.c_str(), name.c_str());
                    return false;
                }
                // direction verification
                if (axis.direction != 1 && axis.direction != -1) {
                    RCLCPP_ERROR(n_->get_logger(),
                        "direction must be 1 or -1 in mode '%s' axis '%s'",
                        name.c_str(), axis.control_name.c_str());
                    return false;
                }
                // scale verification
                if (axis.scale <= 0) {
                    RCLCPP_ERROR(n_->get_logger(),
                        "scale must be > 0 in mode '%s' axis '%s'",
                        name.c_str(), axis.control_name.c_str());
                    return false;
                }
            }

            // Buttons validity
            if (!mode.buttons.short_click.empty()) {
                if (!data.button_modes_map.count(mode.buttons.short_click)) {
                    RCLCPP_ERROR(n_->get_logger(),
                        "Invalid short_click reference '%s' from mode '%s'",
                        mode.buttons.short_click.c_str(), name.c_str());
                    return false;
                }
            }
            
            if (!mode.buttons.long_click.empty()) {
                if (!data.button_modes_map.count(mode.buttons.long_click)) {
                    RCLCPP_ERROR(n_->get_logger(),
                        "Invalid long_click reference '%s' from mode '%s'",
                        mode.buttons.long_click.c_str(), name.c_str());
                    return false;
                }
            }        
        }

        RCLCPP_INFO(n_->get_logger(), "YAML mode configuration validated successfully");
        return true;
    }

    void CommandNode::callback_joystick(const sensor_msgs::msg::Joy & msg) {
        bool button_actual;
        std::lock_guard<std::mutex> lock_axis(mutex_axis_); 
        if (msg.axes.size() >= 2) {
            // Store raw joystick values (smoothing is applied per-axis in readAxisValue)
            axis_1_raw_ = msg.axes[0];
            axis_2_raw_ = msg.axes[1];
        } else {
            RCLCPP_WARN_THROTTLE(n_->get_logger(), *n_->get_clock(), 1000, 
                                 "Joystick has insufficient axes");
            return;
        }

        if (msg.buttons.size() >= 1) {
            button_actual = msg.buttons[0];
        } else {
            RCLCPP_WARN_THROTTLE(n_->get_logger(), *n_->get_clock(), 1000, 
                                 "Joystick has insufficient buttons");
            return;
        }

        button_handler.update(button_actual);
    }

    void CommandNode::callback_x_current(const geometry_msgs::msg::Pose & msg) {
        x_current_ = msg;
    }

    void CommandNode::callback_q_current_(const sensor_msgs::msg::JointState & msg) {
        if (!init) {
            current_pos_ = msg;
            // Check for explorer
            bool valid_explorer = std::all_of(expected_names_explorer.begin(), expected_names_explorer.end(), [&](const std::string &name) {
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
                std::all_of(expected_names_wheelchair.begin(), expected_names_wheelchair.end(),
                            [&](const std::string &name) {
                                return std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end();
                            });
    
            // Check for any wheelchair joints present
            bool any_wheelchair =
                std::any_of(expected_names_wheelchair.begin(), expected_names_wheelchair.end(),
                            [&](const std::string &name) {
                                return std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end();
                            });
    
            // CASES:
            if (valid_explorer && all_wheelchair) {
                mode = Mode::FULL;
                RCLCPP_INFO(n_->get_logger(), "[command_node] Full robot (explorer + wheelchair) detected.");
            } else if (valid_explorer && !any_wheelchair) {
                mode = Mode::EXPLORER;
                RCLCPP_INFO(n_->get_logger(), "[command_node] Explorer-only configuration detected.");
            } else {
                mode = Mode::INVALID;
                RCLCPP_ERROR(n_->get_logger(), "[command_node] Invalid joint configuration detected! Initialization failed.");
                // Optionally, handle the error (throw, return, etc)
                // return;
            }
    
            if (valid_explorer && all_wheelchair) {
                mode = Mode::FULL;
                RCLCPP_INFO(n_->get_logger(), "[command_node] Full robot (explorer + wheelchair) detected.");
                
                // Build order: wheelchair first, then explorer
                joint_order.clear();
                joint_order.reserve(expected_names_wheelchair.size() + expected_names_explorer.size());
                for (const auto &name : expected_names_wheelchair) {
                    auto it = std::find(msg.name.begin(), msg.name.end(), name);
                    joint_order.push_back(std::distance(msg.name.begin(), it));
                }
                for (const auto &name : expected_names_explorer) {
                    auto it = std::find(msg.name.begin(), msg.name.end(), name);
                    if (it != msg.name.end()) {
                        joint_order.push_back(std::distance(msg.name.begin(), it));
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
                            joint_order.push_back(std::distance(msg.name.begin(), fallback_it));
                            RCLCPP_WARN(n_->get_logger(), "[command_node] Joint %s missing, using right_finger_joint as fallback", name.c_str());
                        } else {
                            RCLCPP_ERROR(n_->get_logger(), "[command_node] Neither %s nor right_finger_joint found! Cannot initialize properly", name.c_str());
                        }
                    } else {
                        RCLCPP_ERROR(n_->get_logger(), "[command_node] Joint %s not found!", name.c_str());
                    }
                }
                init = true;
                return;
            } else if (valid_explorer && !any_wheelchair) {
                mode = Mode::EXPLORER;
                RCLCPP_INFO(n_->get_logger(), "[command_node] Explorer-only configuration detected.");
                
                // Build order: just the explorer
                joint_order.clear();
                joint_order.reserve(expected_names_explorer.size());
                for (const auto &name : expected_names_explorer) {
                    auto it = std::find(msg.name.begin(), msg.name.end(), name);
                    if (it != msg.name.end()) {
                        joint_order.push_back(std::distance(msg.name.begin(), it));
                    } else if (
                        name == "left_external_rod_joint_mimic" ||
                        name == "left_fingertip_joint_mimic" ||
                        name == "left_finger_joint_mimic" ||
                        name == "right_external_rod_joint_mimic" ||
                        name == "right_fingertip_joint_mimic"
                    ) {
                        auto fallback_it = std::find(msg.name.begin(), msg.name.end(), "right_finger_joint");
                        if (fallback_it != msg.name.end()) {
                            joint_order.push_back(std::distance(msg.name.begin(), fallback_it));
                            RCLCPP_WARN(n_->get_logger(), "[command_node] Joint %s missing, using right_finger_joint as fallback", name.c_str());
                        } else {
                            RCLCPP_ERROR(n_->get_logger(), "[command_node] Neither %s nor right_finger_joint found! Cannot initialize properly", name.c_str());
                            return;
                        }
                    } else {
                        RCLCPP_ERROR(n_->get_logger(), "[command_node] Joint %s not found and no fallback defined", name.c_str());
                        return;
                    }
                }
    
                // Debug: Print out sizes to check bounds
                RCLCPP_INFO(n_->get_logger(), "joint_order.size() = %zu", joint_order.size());
                RCLCPP_INFO(n_->get_logger(), "current_pos_.name.size() = %zu", current_pos_.name.size());
    
                // Decide safe upper bound
                int safe_limit = std::min<int>(joint_order.size(), current_pos_.name.size());
                int n_to_print = std::min<int>(safe_limit, (wheelchair ? 20 : 12));
    
                for (int i = 0; i < n_to_print; i++) {
                    RCLCPP_INFO(n_->get_logger(), "Joint order[%d]: %ld, Name: %s", 
                                i, 
                                joint_order[i], 
                                current_pos_.name[joint_order[i]].c_str());
                }
                // If there's a mismatch, warn
                if (joint_order.size() < static_cast<size_t>(n_to_print) || current_pos_.name.size() < static_cast<size_t>(n_to_print)) {
                    RCLCPP_WARN(n_->get_logger(), "WARNING: joint_order or current_pos_.name was smaller than expected! Potential config problem.");
                }
                init = true;
                RCLCPP_INFO(n_->get_logger(), "[command_node] Init done.");
                return;
            }
        }
    
        current_pos_ = msg;
    }

    void CommandNode::callback_q_forward_controller(const std_msgs::msg::Float64MultiArray & msg) {
        q_gripper_ = msg.data[6];
    }

    void CommandNode::handle_controller_state()
{
    switch (control_state_) {

        case ControlState::FORWARD:
            reset_qp_solving_pub_->publish(std_msgs::msg::Bool().set__data(false));
            if (trajectory_requested_) {
                RCLCPP_INFO(n_->get_logger(), "→ SWITCHING TO TRAJECTORY MODE");
                control_state_ = ControlState::SWITCHING_TO_TRAJ;
            }
            break;

        case ControlState::SWITCHING_TO_TRAJ:
            if (!switch_in_progress_) {
                switch_in_progress_ = true;

                auto future = controller_switcher.switch_controller_async(
                    {"forward_position_controller"},
                    {"joint_trajectory_controller"}
                );

                // Callback when the switch completes
                std::thread([this, future = std::move(future)]() mutable {
                    try {
                        bool success = future.get(); // Bloque ici, mais dans un thread séparé
                        if (success) {
                            RCLCPP_INFO(n_->get_logger(), "Switched to joint_trajectory_controller");
                            control_state_ = ControlState::TRAJECTORY;
                        } else {
                            RCLCPP_ERROR(n_->get_logger(), "Failed to switch to joint_trajectory_controller");
                            control_state_ = ControlState::FORWARD;
                        }
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(n_->get_logger(), "Exception during controller switch: %s", e.what());
                        control_state_ = ControlState::FORWARD;
                    }
                    switch_in_progress_ = false;
                }).detach(); // détache le thread pour ne pas bloquer le main executor
            }
            break;

        case ControlState::TRAJECTORY:
            if (!trajectory_requested_) {
                RCLCPP_INFO(n_->get_logger(), "→ SWITCHING TO FORWARD MODE");
                control_state_ = ControlState::SWITCHING_TO_FORWARD;
            }
            break;

        case ControlState::SWITCHING_TO_FORWARD:
            if (!switch_in_progress_) {
                switch_in_progress_ = true;

                trajectory_manager.reset();
                reset_qp_solving_pub_->publish(std_msgs::msg::Bool().set__data(true));

                auto future = controller_switcher.switch_controller_async(
                    {"joint_trajectory_controller"},
                    {"forward_position_controller"}
                );

                std::thread([this, future = std::move(future)]() mutable {
                    try {
                        bool success = future.get();
                        if (success) {
                            RCLCPP_INFO(n_->get_logger(), "Switched to forward_position_controller");
                            control_state_ = ControlState::FORWARD;
                            RCLCPP_INFO(n_->get_logger(), "→ FORWARD MODE");
                        } else {
                            RCLCPP_WARN(n_->get_logger(), "Failed to switch to forward_position_controller");
                            control_state_ = ControlState::TRAJECTORY;
                        }
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(n_->get_logger(), "Exception during controller switch: %s", e.what());
                        control_state_ = ControlState::TRAJECTORY;
                    }
                    switch_in_progress_ = false;
                }).detach();
            }
            break;
    }
}

    void CommandNode::modifyTargetNodeParameter(const std::string & param_name, const rclcpp::ParameterValue & value) {
        if (!param_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(n_->get_logger(), "Parameter service of qp_solving not available");
            return;
        }

        rclcpp::Parameter param(param_name, value);

        param_client_->set_parameters({param}, [this, param_name](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
        {
            const auto results = future.get();
        
            if (!results.empty() && results[0].successful)
            {
                RCLCPP_INFO(n_->get_logger(), "Parameter %s modified successfully", param_name.c_str());
            }
            else
            {
                RCLCPP_ERROR(n_->get_logger(), "Failed to modify parameter %s", param_name.c_str());
            }
        });
    }
    
    void CommandNode::timer() {

        // Step 1: Get the current mode to know which smoothing alphas to use
        ButtonMode mode = data.button_modes_map[current_mode_name];

        // Step 2: Read raw values and apply smoothing for each axis ONCE
        // Find the smoothing alpha for each joystick axis from the current mode config
        float alpha_ax1 = 1.0f;  // default: no smoothing
        float alpha_ax2 = 1.0f;
        for (const auto& axis : mode.axes) {
            if (axis.joystick_axis == "ax1") {
                alpha_ax1 = axis.smoothing_alpha;
            } else if (axis.joystick_axis == "ax2") {
                alpha_ax2 = axis.smoothing_alpha;
            }
        }

        // Step 3: Atomically read raw values and apply smoothing
        {
            std::lock_guard<std::mutex> lock_axis(mutex_axis_);
            axis_1_smoothed_ = alpha_ax1 * axis_1_raw_ + (1.0f - alpha_ax1) * axis_1_smoothed_;
            axis_2_smoothed_ = alpha_ax2 * axis_2_raw_ + (1.0f - alpha_ax2) * axis_2_smoothed_;
        }

        // Debug: log both smoothed values periodically
        RCLCPP_DEBUG_THROTTLE(n_->get_logger(), *n_->get_clock(), 500,
            "Smoothed axes: ax1=%.3f, ax2=%.3f (alphas: %.2f, %.2f)",
            axis_1_smoothed_, axis_2_smoothed_, alpha_ax1, alpha_ax2);

        // Step 4: Reset velocities
        resetVelocities();

        complex_mode_ = false;
        
        trajectory_requested_ = false;

        // Step 5: Execute control behaviors for each axis (uses pre-smoothed values)
        for (const auto& axis : mode.axes) {
            executeBehavior(axis);
        }

        if(complex_mode_ && !lock_) {
            complex_calculation(rotation_speed_scale);
        }

        handle_controller_state();

        // Step 6: Publish the computed velocities
        RCLCPP_DEBUG_THROTTLE(n_->get_logger(), *n_->get_clock(), 500,
            "Publishing velocities: linear(%.3f, %.3f, %.3f)",
            cartesian_vel_.twist.linear.x, cartesian_vel_.twist.linear.y, cartesian_vel_.twist.linear.z);
        cartesian_vel_pub_->publish(cartesian_vel_);
        joint_vel_pub_->publish(joint_vel_);
        frame_id_pub_->publish(frame_id_);

        // Handle mode switching based on button clicks
        if(button_handler.isShortClick() && mode.buttons.short_click != "") {
            current_mode_name = mode.buttons.short_click;
            // Reset smoothed values when switching modes to avoid artifacts
            axis_1_smoothed_ = 0.0f;
            axis_2_smoothed_ = 0.0f;
        }
        else if(button_handler.isLongClick() && mode.buttons.long_click != "") {
            current_mode_name = mode.buttons.long_click;
            // Reset smoothed values when switching modes to avoid artifacts
            axis_1_smoothed_ = 0.0f;
            axis_2_smoothed_ = 0.0f;
        }

        mode_name_pub_->publish(std_msgs::msg::String().set__data(current_mode_name));
        speed_level_pub_->publish(std_msgs::msg::Int32().set__data(speed_level));
        gripper_pub_->publish(gripper_vel_);
        retract_status_pub_->publish(std_msgs::msg::String().set__data(trajectory_manager.getStatusString()));

        if(trajectory_manager.getStatusString() != "ready" && status_prec == "ready") {
            modifyTargetNodeParameter("j2.max", rclcpp::ParameterValue(2.112));
        }
        else if(trajectory_manager.getStatusString() == "ready" && status_prec == "in progress") {
            modifyTargetNodeParameter("j2.max", rclcpp::ParameterValue(0.524));
        }

        status_prec = trajectory_manager.getStatusString();
        
    }

    void CommandNode::executeBehavior(const AxisInfo& axis) {
        if (control_behaviors_.count(axis.control_name)) {
            control_behaviors_[axis.control_name](axis);
        }
    }

    // Read joystick axis value (smoothing already applied at start of timer)
    float CommandNode::readAxisValue(const AxisInfo& axis_info) {
        float smoothed_value = 0.0f;
        
        // Simply return the pre-smoothed value for the requested axis
        if(axis_info.joystick_axis == "ax1") {
            smoothed_value = axis_1_smoothed_;
        }  
        else if(axis_info.joystick_axis == "ax2") {
            smoothed_value = axis_2_smoothed_;
        }

        double deadzone = 0.0;
        if (axis_info.params.count("deadzone")) {
            deadzone = axis_info.params.at("deadzone");
        }

        if (std::abs(smoothed_value) < deadzone) {
            smoothed_value = 0.0f;  // Zero out values within the deadzone
        }
        else {
            // Rescale values outside the deadzone to the range [0, 1]
            if (smoothed_value > 0) {
                smoothed_value = (smoothed_value - deadzone) / (1.0 - deadzone);
            } else {
                smoothed_value = (smoothed_value + deadzone) / (1.0 - deadzone);
            }
        }

        // Apply direction, scale and speed factor
        float value = smoothed_value * axis_info.direction * axis_info.scale * speed_factor;

        return value;
    }

    // Reset Cartesian and joint velocities to zero
    void CommandNode::resetVelocities() {
        cartesian_vel_.twist.linear = geometry_msgs::msg::Vector3();
        cartesian_vel_.twist.angular = geometry_msgs::msg::Vector3();
        joint_vel_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gripper_vel_.data = 0.0;
    }

    void CommandNode::complex_calculation(const double rotation_speed_scale) {
        double omega_z = 0.0;
        double x_E = x_current_.position.x;
        double y_E = x_current_.position.y;
        double denom = x_E * x_E + y_E * y_E;
        if(denom > 1e-6) {
            omega_z = (x_E * v_y - y_E * v_x) / denom;
        }
        cartesian_vel_.twist.angular.z = omega_z * rotation_speed_scale;
    }

    // Behavior implementations
    void CommandNode::cartesian_linear(const AxisInfo& axis_info) {
        if(!lock_) {

            // Determine joystick axis value
            float value = 0.0;
            
            value = readAxisValue(axis_info);
            
            // Assign to the appropriate Cartesian linear velocity component
            if (axis_info.control_name == "cartesian_X") {
                cartesian_vel_.twist.linear.x = value;
            } else if (axis_info.control_name == "cartesian_Y") {
                cartesian_vel_.twist.linear.y = value;
            } else if (axis_info.control_name == "cartesian_Z") {
                cartesian_vel_.twist.linear.z = value;
            }
        }
    }

    void CommandNode::cartesian_rotation(const AxisInfo& axis_info) {
        if(!lock_) {

            // Determine joystick axis value
            float value = 0.0;
            
            value = readAxisValue(axis_info);
            
            // Assign to the appropriate Cartesian angular velocity component
            if (axis_info.control_name == "rotation_X") {
                cartesian_vel_.twist.angular.x = value;
            } else if (axis_info.control_name == "rotation_Y") {
                cartesian_vel_.twist.angular.y = value;
            } else if (axis_info.control_name == "rotation_Z") {
                cartesian_vel_.twist.angular.z = value;
            }

            frame_id_.orientation_control_frame = 1;
        }
    }

    void CommandNode::joint_direct(const AxisInfo& axis_info) {
        if(!lock_) {
            
            // Determine joystick axis value
            float value = 0.0;
            
            value = readAxisValue(axis_info);

            // Assign to the appropriate joint velocity component
            if (axis_info.control_name == "joint_1") {
                joint_vel_.data[0] = value;
            }
            else if (axis_info.control_name == "joint_2") {
                joint_vel_.data[1] = value;
            }
            else if (axis_info.control_name == "joint_3") {
                joint_vel_.data[2] = value;
            }
            else if (axis_info.control_name == "joint_4") {
                joint_vel_.data[3] = value;
            }
            else if (axis_info.control_name == "joint_5") {
                joint_vel_.data[4] = value;
            }
            else if (axis_info.control_name == "joint_6") {
                cartesian_vel_.twist.angular.x = value;
                // Set control frame to end-effector for joint 6 control
                frame_id_.orientation_control_frame = 1;
                
            } 
        }
        
    }

    void CommandNode::change_speed(const AxisInfo& axis_info) {
        // Get min/max speed levels from params (default: 1 to 4)
        int min_level = 1;
        int max_level = 4;
        if (axis_info.params.count("min_speed_level")) {
            min_level = static_cast<int>(axis_info.params.at("min_speed_level"));
        }
        if (axis_info.params.count("max_speed_level")) {
            max_level = static_cast<int>(axis_info.params.at("max_speed_level"));
        }

        // Determine joystick axis value (use raw values for instant threshold detection)
        float value = 0.0;
        std::lock_guard<std::mutex> lock_axis(mutex_axis_);
        if(axis_info.joystick_axis == "ax1" ) {
            value = axis_1_raw_;
        }  
        else if(axis_info.joystick_axis == "ax2" ) {
            value = axis_2_raw_;
        }

        value *= axis_info.direction * axis_info.scale;

        // Change speed level based on joystick movement
        if(value > speed_change_threshold && joy_prec <= speed_change_threshold) {
            speed_level += 1;
            if(speed_level > max_level){
                speed_level = max_level;
            }
        }
        else if(value < -speed_change_threshold && joy_prec >= -speed_change_threshold) {
            speed_level -= 1;
            if(speed_level < min_level){
                speed_level = min_level;
            }
        }

        joy_prec = value;
        speed_factor = speed_level_multiplier * speed_level;
    }

    void CommandNode::drink(const AxisInfo& axis_info) {
        if(!lock_) {
            
            // Determine joystick axis value
            float value = 0.0;
            
            value = readAxisValue(axis_info);

            // Assign to the appropriate joint velocity component for drinking action
            cartesian_vel_.twist.angular.x = value;

            // Set control frame to end-effector for drinking action
            frame_id_.orientation_control_frame = 3;
        }
        
    }

    void CommandNode::gripper(const AxisInfo& axis_info) {
        if(!lock_) { 
            
            // Determine joystick axis value
            float value = 0.0;
            
            value = readAxisValue(axis_info);

            // Assign to gripper velocity
            gripper_vel_.data = value;
        }
       
    }


    void CommandNode::complex(const AxisInfo& axis_info) {
        if(!lock_) {
            // Placeholder for complex behavior implementation
            float value = 0.0;

            complex_mode_ = true;
    
            value = readAxisValue(axis_info);

            // Assign to appropriate complex mode variables
            if (axis_info.control_name == "complex_X") {
                v_x = value;
                cartesian_vel_.twist.linear.x = value;
            }
            if (axis_info.control_name == "complex_Y") {
                v_y = value;
                cartesian_vel_.twist.linear.y = value;
            }

            if (axis_info.params.count("rotation_speed_scale")) {
                rotation_speed_scale = static_cast<double>(axis_info.params.at("rotation_speed_scale"));
            }

            frame_id_.orientation_control_frame = 0;
        }

    }

    void CommandNode::trajectory_control(const AxisInfo& axis_info)
    {
        if(!active_trajectory_) {
            return;
        }

        trajectory_requested_ = true;
        
        trajectory_msgs::msg::JointTrajectory traj_msg;
        
        if (control_state_ != ControlState::TRAJECTORY) {
            return;  // pas encore actif → on attend le switch
        }

        for (size_t i = 0; i < 7; ++i) {
            q_current_[i] = current_pos_.position[joint_order[i]];
        }
        
        float value = readAxisValue(axis_info);
        trajectory_manager.update(q_current_, q_gripper_, value);
        
        lock_ = trajectory_manager.getLock();

        traj_msg = trajectory_manager.getTrajectory();

        trajectory_pub_->publish(traj_msg);
    }
    
}

using namespace space_control;
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    
    auto n = rclcpp::Node::make_shared("command_node", node_options);

    CommandNode command_node(n);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}


