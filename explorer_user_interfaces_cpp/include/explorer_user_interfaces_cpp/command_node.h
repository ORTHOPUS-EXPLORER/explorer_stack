#include <unordered_map>
#include <string>
#include <functional>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "explorer_user_interfaces_cpp/button_handler.h"
#include "explorer_user_interfaces_cpp/trajectory_manager.h"
#include "explorer_msgs/msg/control_frame_selection.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "atomic"
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include "explorer_user_interfaces_cpp/controller_switcher.h"

using namespace std::chrono;

namespace space_control
{  
    // Information about each axis control
    struct AxisInfo {
        std::string control_name;
        std::string joystick_axis;
        int direction;
        double scale;
        double smoothing_alpha = 1.0;  // Smoothing factor (1.0 = no smoothing, 0.1 = heavy smoothing)
        std::map<std::string, double> params; 
    };
    
    // Actions associated with button clicks
    struct ButtonAction {
        std::string long_click;
        std::string short_click;
    };
    
    // Information about each button mode
    struct ButtonMode {
        std::string name; 
        std::vector<AxisInfo> axes;
        ButtonAction buttons;
    };
    
    // Information about the overall mode
    struct ModeInfo {
        std::string name;
        std::string display_name;
        std::string description;
    };
    
    // Complete mode data structure
    struct ModeData {
        ModeInfo mode_info;
        std::unordered_map<std::string, ButtonMode> button_modes_map;
    };

    class CommandNode
    {
    public:
        CommandNode(rclcpp::Node::SharedPtr n);
    protected:
    private:
        rclcpp::Node::SharedPtr n_;
        
        ButtonHandler button_handler_;
        TrajectoryManager trajectory_manager_;
        ControllerSwitcher controller_switcher_;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr x_current_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr q_current_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr default_controller_sub_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_name_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_level_pub_;
        rclcpp::Publisher<explorer_msgs::msg::ControlFrameSelection>::SharedPtr frame_id_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_qp_solving_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr retract_status_pub_;

        rclcpp::AsyncParametersClient::SharedPtr param_client_;

        rclcpp::TimerBase::SharedPtr timer_;
        
        std::string current_mode_name_;
        std::unordered_map<std::string, std::function<void(AxisInfo)>> control_behaviors_;

        // Joystick state variables
        mutable std::mutex mutex_axis_;
        
        // Raw joystick values (before smoothing)
        float axis_1_raw_ RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = 0.0f;
        float axis_2_raw_ RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = 0.0f;
        
        // Smoothed joystick values (computed once per timer cycle)
        float axis_1_smoothed_ = 0.0f;
        float axis_2_smoothed_ = 0.0f;

        int button_threshold_ms_;
        double sampling_period_;

        ModeData data_;

        // Speed control variables
        float speed_factor_;
        int speed_level_;
        float joy_prec_;
        float speed_change_threshold_;
        float speed_level_multiplier_;

        bool complex_mode_;
        double v_x_ = 0.0;
        double v_y_ = 0.0;
        double rotation_speed_scale_;

        bool active_trajectory_;

        std::atomic<bool> lock_{false};

        enum class ControlState
        {
            DEFAULT_CONTROLLER,        // default controller used (forward_position_controller, explorer_custom_controller ... ?)
            SWITCHING_TO_TRAJ,
            TRAJECTORY,    // joint_trajectory_controller
            RESTORING_DEFAULT_CONTROLLER
        };

        ControlState control_state_ = ControlState::DEFAULT_CONTROLLER;
        
        bool trajectory_requested_ = false;
        bool switch_in_progress_ = false;

        sensor_msgs::msg::JointState current_state_;

        // Velocity messages
        geometry_msgs::msg::TwistStamped cartesian_vel_;
        std_msgs::msg::Float64MultiArray joint_vel_;
        std_msgs::msg::Float64 gripper_vel_;

        explorer_msgs::msg::ControlFrameSelection frame_id_;

        geometry_msgs::msg::Pose x_current_;
        std::array<double,7> q_current_;
        double q_gripper_;

        sensor_msgs::msg::JointState current_pos_;

        bool init_;
        bool wheelchair_;
        bool first_use_;

        enum class Mode { INVALID, EXPLORER, FULL };


        std::vector<size_t> joint_order_;
        Mode mode_;

        std::optional<double> j2_max_cached_;
        std::optional<double> j2_operational_max_cached_;
        std::optional<double> j3_max_cached_;
        std::optional<double> j3_operational_max_cached_;

        double j2_max_;
        double j2_operational_max_;
        double j3_max_;
        double j3_operational_max_;

        double actual_j2_limit_;
        double actual_j3_limit_;

        bool limits_initialized_ = false;

        std::string default_controller_name_;
        std::string default_controller_position_topic_name_;

        ModeData loadModeData_(const std::string& filename);
        bool validateModeData_(const ModeData& data);

        void callback_joystick_(const sensor_msgs::msg::Joy & msg);

        void callback_x_current_(const geometry_msgs::msg::Pose & msg);

        void callback_q_current_(const sensor_msgs::msg::JointState & msg);

        void callback_defaut_controller_(const std_msgs::msg::Float64MultiArray & msg);

        void handle_controller_state_();

        void modifyTargetNodeParameter_(const std::string& param_name, const rclcpp::ParameterValue& value);

        void getDoubleParameter_(const std::string & param_name, std::optional<double> & value);

        void timer_callback_();
        
        // Execute behavior based on axis information
        void executeBehavior_(const AxisInfo& axis);

        // Read joystick axis value
        float readAxisValue_(const AxisInfo& axis_info);

        void resetVelocities_();

        void complex_calculation_(const double rotation_speed_scale);

        // Behavior functions
        void cartesian_linear_(const AxisInfo& axis_info);
        void cartesian_rotation_(const AxisInfo& axis_info); 
        void joint_direct_(const AxisInfo& axis_info);
        void change_speed_(const AxisInfo& axis_info);
        void drink_(const AxisInfo& axis_info);
        void gripper_(const AxisInfo& axis_info);
        void complex_(const AxisInfo& axis_info);
        void trajectory_control_(const AxisInfo& axis_info);
    
    };


}