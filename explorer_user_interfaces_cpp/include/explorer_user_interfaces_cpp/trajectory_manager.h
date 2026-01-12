#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>
#include <functional>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono;

namespace space_control
{

    struct JointLimit {
        bool enabled;
        double min;
        double max;
    };
    
    class TrajectoryManager {
        public:
            TrajectoryManager();

            bool loadTrajectory(const std::string& filename);

            bool validateTrajectory() const;

            void update(std::array<double,7> q_current, float axe_value);

            bool getLock();

            trajectory_msgs::msg::JointTrajectory getTrajectory();

            void reset();

            std::string getStatusString();

        private:

        std::vector<JointLimit> joint_limits_;

        YAML::Node traj;

        std::vector<std::array<double, 6>> init_points_;
        int current_point_index_;

        std::array<double,6> q_current_;
        double gripper;
        float axe_value_;
        float axe_value_prev_;
        bool newDirection_;
        bool lock_;
        
        double traj_end_time_;

        bool return_sequence_active_ = false;
        bool needs_return_to_ready_ = false;
        bool ready_just_reached_ = false;
        bool trajectory_completed_ = false;
        std::string status;
        
        static bool pointAlmostEqual(const std::array<double, 6>& p1, const std::array<double, 6>& p2, double epsilon = 3.5e-2); //2° tolerance

        void updateStatusString();
    };

}