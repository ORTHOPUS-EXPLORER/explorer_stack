#include "explorer_user_interfaces_cpp/command_node.h"

namespace space_control
{
    CommandNode::CommandNode(rclcpp::Node::SharedPtr n)
    : n_(n),
      button_handler()
    {
        RCLCPP_INFO(n->get_logger(), "CommandNode constructor");

        n_->declare_parameter<int>("button_threshold_ms", 500);
        n_->declare_parameter<double>("speed_change_threshold", 0.95);
        n_->declare_parameter<double>("speed_level_multiplier", 0.25);

        button_threshold_ms = n_->get_parameter("button_threshold_ms").as_int();
        speed_change_threshold = n_->get_parameter("speed_change_threshold").as_double();
        speed_level_multiplier = n_->get_parameter("speed_level_multiplier").as_double();

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
            {"gripper",      std::bind(&CommandNode::gripper, this, std::placeholders::_1)}
        };

        n_->declare_parameter<std::string>("mode_file", "");

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
        
        // Initialize subscribers and publishers
        joy_sub_ = n->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&CommandNode::callback_joystick, this, std::placeholders::_1));

        joint_vel_pub_ = n->create_publisher<std_msgs::msg::Float64MultiArray>("command_node/joint_velocity_command", 10);
        cartesian_vel_pub_ = n->create_publisher<geometry_msgs::msg::TwistStamped>("command_node/cartesian_velocity_command", 10);
        mode_name_pub_ = n->create_publisher<std_msgs::msg::String>("command_node/mode_name", 10);
        speed_level_pub_ = n->create_publisher<std_msgs::msg::Int32>("command_node/speed_level", 10);
        frame_id_pub_ = n->create_publisher<explorer_msgs::msg::ControlFrameSelection>("/explorer_controllers/qp_solving/control_frame_selection", 10);
        gripper_pub_ = n->create_publisher<std_msgs::msg::Float64>("command_node/gripper_velocity_command", 10);

        // Timer callback 
        timer_ = n->create_timer(100ms, std::bind(&CommandNode::timer, this));

        // Initialize joystick axes and button states
        axis_1 = 0.0;
        axis_2 = 0.0;

        // Initialize speed control variables
        speed_factor = 1.0;
        speed_level = 2;
        joy_prec = 0.0;

        // Initialize Cartesian and joint velocities to zero
        resetVelocities();

        frame_id_.position_control_frame = 0;
        frame_id_.orientation_control_frame = 0;
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
            axis_1 = msg.axes[0];
            axis_2 = msg.axes[1];
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

    void CommandNode::timer() {
        // Reset velocities
        resetVelocities();

        ButtonMode mode = data.button_modes_map[current_mode_name];

        // Execute control behaviors for each axis in the current mode
        for (const auto& axis : mode.axes) {
            executeBehavior(axis);
        }

        // Publish the computed velocities
        cartesian_vel_pub_->publish(cartesian_vel_);
        joint_vel_pub_->publish(joint_vel_);
        frame_id_pub_->publish(frame_id_);

        // Handle mode switching based on button clicks
        if(button_handler.isShortClick() && mode.buttons.short_click != "") {
            current_mode_name = mode.buttons.short_click; 
        }
        else if(button_handler.isLongClick() && mode.buttons.long_click != "") {
            current_mode_name = mode.buttons.long_click;
        }
        mode_name_pub_->publish(std_msgs::msg::String().set__data(current_mode_name));
        speed_level_pub_->publish(std_msgs::msg::Int32().set__data(speed_level));
        gripper_pub_->publish(gripper_vel_);
    }

    void CommandNode::executeBehavior(const AxisInfo& axis) {
        if (control_behaviors_.count(axis.control_name)) {
            control_behaviors_[axis.control_name](axis);
        }
    }

    // Read joystick axis value
    float CommandNode::readAxisValue(const AxisInfo& axis_info) {
        float value = 0.0;
        std::lock_guard<std::mutex> lock_axis(mutex_axis_);
        if(axis_info.joystick_axis == "ax1" ) {
            value = axis_1;
        }  
        else if(axis_info.joystick_axis == "ax2" ) {
            value = axis_2;
        }

        value *= axis_info.direction * axis_info.scale * speed_factor;

        return value;
    }

    // Reset Cartesian and joint velocities to zero
    void CommandNode::resetVelocities() {
        cartesian_vel_.twist.linear = geometry_msgs::msg::Vector3();
        cartesian_vel_.twist.angular = geometry_msgs::msg::Vector3();
        joint_vel_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        gripper_vel_.data = 0.0;
    }

    // Behavior implementations
    void CommandNode::cartesian_linear(const AxisInfo& axis_info) {
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

    void CommandNode::cartesian_rotation(const AxisInfo& axis_info) {
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

        frame_id_.orientation_control_frame = 0;
    }

    void CommandNode::joint_direct(const AxisInfo& axis_info) {
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

    void CommandNode::change_speed(const AxisInfo& axis_info) {
        // Determine joystick axis value
        float value = 0.0;
        if(axis_info.joystick_axis == "ax1" ) {
            value = axis_1;
        }  
        else if(axis_info.joystick_axis == "ax2" ) {
            value = axis_2;
        }

        value *= axis_info.direction * axis_info.scale;

        // Change speed level based on joystick movement
        if(value > speed_change_threshold && joy_prec <= speed_change_threshold) {
            speed_level += 1;
            if(speed_level > 4){
                speed_level = 4;
            }
        }
        else if(value < -speed_change_threshold && joy_prec >= -speed_change_threshold) {
            speed_level -= 1;
            if(speed_level < 1){
                speed_level = 1;
            }
        }

        joy_prec = value;
        speed_factor = speed_level_multiplier * speed_level;
    }

    void CommandNode::drink(const AxisInfo& axis_info) {
        // Determine joystick axis value
        float value = 0.0;
        
        value = readAxisValue(axis_info);

        // Assign to the appropriate joint velocity component for drinking action
        cartesian_vel_.twist.angular.x = value;

        // Set control frame to end-effector for drinking action
        frame_id_.orientation_control_frame = 3;
    }

    void CommandNode::gripper(const AxisInfo& axis_info) {
        // Determine joystick axis value
        float value = 0.0;
        
        value = readAxisValue(axis_info);

        // Assign to gripper velocity
        gripper_vel_.data = value;
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


