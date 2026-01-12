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

    // Parameter for each axis
    struct AxisParam {
        std::string name;
        double value;
    };
    
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
        
        ButtonHandler button_handler;
        TrajectoryManager trajectory_manager;
        ControllerSwitcher controller_switcher;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr x_current_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr q_current_sub_;

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

        rclcpp::TimerBase::SharedPtr timer_;
        
        std::string current_mode_name;
        std::unordered_map<std::string, std::function<void(AxisInfo)>> control_behaviors_;

        // Joystick state variables
        mutable std::mutex mutex_axis_;
        
        // Raw joystick values (before smoothing)
        float axis_1_raw_ RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = 0.0f;
        float axis_2_raw_ RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = 0.0f;
        
        // Smoothed joystick values per axis (exponential moving average)
        std::map<std::string, float> axis_smoothed_;  // key: "ax1" or "ax2"

        int button_threshold_ms;
        double sampling_period_;

        ModeData data;

        // Speed control variables
        float speed_factor;
        int speed_level;
        float joy_prec;
        float speed_change_threshold;
        float speed_level_multiplier;

        bool complex_mode_;
        double v_x = 0.0;
        double v_y = 0.0;

        bool active_trajectory_;

        std::atomic<bool> lock_{false};

        enum class ControlState
        {
            FORWARD,        // forward_position_controller
            SWITCHING_TO_TRAJ,
            TRAJECTORY,    // joint_trajectory_controller
            SWITCHING_TO_FORWARD
        };

        ControlState control_state_ = ControlState::FORWARD;
        
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

        sensor_msgs::msg::JointState current_pos_;

        bool init;
        bool wheelchair;
        bool first_use;

        enum class Mode { INVALID, EXPLORER, FULL };

        std::vector<std::string> expected_names_explorer = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "left_external_rod_joint_mimic", "left_fingertip_joint_mimic", "left_finger_joint_mimic", "right_external_rod_joint_mimic", "right_fingertip_joint_mimic", "right_finger_joint"};
        std::vector<std::string> expected_names_wheelchair = {"left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint", "left_wheel_joint", "right_wheel_joint", "left_right_head_joint", "up_down_head_joint"};

        std::vector<size_t> joint_order;
        Mode mode;

        ModeData loadModeData(const std::string& filename);
        bool validateModeData(const ModeData& data);

        void callback_joystick(const sensor_msgs::msg::Joy & msg);

        void callback_x_current(const geometry_msgs::msg::Pose & msg);

        void callback_q_current_(const sensor_msgs::msg::JointState & msg);

        void handle_controller_state();

        void timer();
        
        // Execute behavior based on axis information
        void executeBehavior(const AxisInfo& axis);

        // Read joystick axis value
        float readAxisValue(const AxisInfo& axis_info);

        void resetVelocities();

        void complex_calculation();

        // Behavior functions
        void cartesian_linear(const AxisInfo& axis_info);
        void cartesian_rotation(const AxisInfo& axis_info); 
        void joint_direct(const AxisInfo& axis_info);
        void change_speed(const AxisInfo& axis_info);
        void drink(const AxisInfo& axis_info);
        void gripper(const AxisInfo& axis_info);
        void complex(const AxisInfo& axis_info);
        void trajectory_control(const AxisInfo& axis_info);
    
    };


}