#include <unordered_map>
#include <string>
#include <functional>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "explorer_user_interfaces_cpp/button_handler.h"


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
        std::string image;
    };
    
    // Information about the overall mode
    struct ModeInfo {
        std::string name;
        std::string display_name;
        std::string description;
        std::string default_image;
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

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_name_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_level_pub_;

        rclcpp::TimerBase::SharedPtr timer_;
        
        std::string current_mode_name;
        std::unordered_map<std::string, std::function<void(AxisInfo)>> control_behaviors_;

        // Joystick state variables
        mutable std::mutex mutex_axis_;
        float axis_1 RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = 0;
        float axis_2 RCPPUTILS_TSA_GUARDED_BY(mutex_axis_) = 0;

        int button_threshold_ms;

        ModeData data;

        // Speed control variables
        float speed_factor;
        int speed_level;
        float joy_prec;
        float speed_change_threshold;
        float speed_level_multiplier;

        // Velocity messages
        geometry_msgs::msg::TwistStamped cartesian_vel_;
        std_msgs::msg::Float64MultiArray joint_vel_;

        ModeData loadModeData(const std::string& filename);
        bool validateModeData(const ModeData& data);

        void callback_joystick(const sensor_msgs::msg::Joy & msg);

        void timer();
        
        // Execute behavior based on axis information
        void executeBehavior(const AxisInfo& axis);

        // Read joystick axis value
        float readAxisValue(const AxisInfo& axis_info);

        void resetVelocities();

        // Behavior functions
        void cartesian_linear(const AxisInfo& axis_info);
        void cartesian_rotation(const AxisInfo& axis_info); 
        void joint_direct(const AxisInfo& axis_info);
        void change_speed(const AxisInfo& axis_info);

    };


}