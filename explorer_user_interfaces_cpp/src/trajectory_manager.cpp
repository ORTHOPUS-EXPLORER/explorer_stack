#include "explorer_user_interfaces_cpp/trajectory_manager.h"

namespace space_control
{
    TrajectoryManager::TrajectoryManager(){
        lock_ = true;
        current_point_index_ = 0;
        axe_value_ = 0.0;
        axe_value_prev_ = 0.0;
        newDirection_ = false;
        traj_end_time_ = 0.0;
        q_hold_.fill(0.0);
        status = "not ready";  // Initial status before trajectory is loaded
    }

    bool TrajectoryManager::loadTrajectory(const std::string& filename)
    {
        YAML::Node root;
        try {
            root = YAML::LoadFile(filename);
        } catch (const YAML::BadFile& e) {
            RCLCPP_ERROR(rclcpp::get_logger("trajectory_manager"), "Cannot open trajectory file: %s", filename.c_str());
            return false; 
        } catch (const YAML::ParserException& e) {
            RCLCPP_ERROR(rclcpp::get_logger("trajectory_manager"), "YAML parsing error in %s: %s", filename.c_str(), e.what());
            return false;
        }

        init_points_.clear();

        traj = root["initialisation_trajectory"];

        if (traj && traj.IsSequence()) {
            for (const auto& p : traj) {
                std::array<double, 6> point{};
                for (size_t i = 0; i < 6; ++i) {
                    point[i] = p[i].as<double>(0.0); 
                }
                init_points_.push_back(point);
            }
        }
        return true;
    }

    bool TrajectoryManager::validateTrajectory() const
    {
        if (!traj || !traj.IsSequence()) {
            RCLCPP_ERROR(rclcpp::get_logger("trajectory_manager"),
                "Invalid trajectory: initialisation_trajectory missing or not a sequence");
            return false;
        }

        if (traj.size() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("trajectory_manager"), "Trajectory is empty");
            return false;
        }

        return true;
    }

    void TrajectoryManager::update(std::array<double,7> q_current, double q_gripper, float axe_value) {

        for (size_t i = 0; i < 6; ++i)
        {
            q_current_[i] = q_current[i];
        }
        gripper_ = q_gripper;

        // Initialize q_hold_ with actual position on first update
        if (!q_hold_initialized_) {
            q_hold_ = q_current_;
            q_hold_initialized_ = true;
        }

        axe_value_ = axe_value;
    }

    bool TrajectoryManager::getLock() {
        return lock_;
    }

    trajectory_msgs::msg::JointTrajectory TrajectoryManager::getTrajectory() {
        
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msgs::msg::JointTrajectoryPoint traj_point_start;
        trajectory_msgs::msg::JointTrajectoryPoint traj_point_target;

        double dq = 0.0;

        newDirection_ = (axe_value_ > 0 && axe_value_prev_ <= 0) || (axe_value_ < 0 && axe_value_prev_ >= 0);

        // Capture hold position when joystick is released (transition from moving to zero)
        bool joystick_just_released = (axe_value_ == 0.0 && axe_value_prev_ != 0.0);
        if (joystick_just_released) {
            q_hold_ = q_current_;
        }
        
        if(axe_value_> 0.0 && !trajectory_completed_){
            if(current_point_index_ < init_points_.size() - 1){
                if(newDirection_ || pointAlmostEqual(q_current_, init_points_[current_point_index_])){
                    current_point_index_++;
                    RCLCPP_INFO(rclcpp::get_logger("trajectory_manager"), "Current Point Index: %ld", current_point_index_);
                    RCLCPP_INFO(rclcpp::get_logger("trajectory_manager"), "axe_value_: %f", axe_value_);
                }

            } 
        } 
        else if (axe_value_ < 0.0)
        {
            if (trajectory_completed_ && !return_sequence_active_)
            {
                return_sequence_active_ = true;
                needs_return_to_ready_ = true;
                ready_just_reached_ = false;
                current_point_index_ = init_points_.size() - 1;

                RCLCPP_INFO(rclcpp::get_logger("trajectory_manager"), "Entering return sequence: forcing READY");
            }
            if (needs_return_to_ready_)
            {
                if (pointAlmostEqual(q_current_, init_points_.back()))
                {
                    needs_return_to_ready_ = false;
                    ready_just_reached_ = true;
                    RCLCPP_INFO(rclcpp::get_logger("trajectory_manager"), "READY reached, continuing to retract");
                }
            }
            else
            {
                if (ready_just_reached_)
                {
                    if (current_point_index_ > 0)
                        current_point_index_--;

                    ready_just_reached_ = false;
                }
                else if (current_point_index_ > 0)
                {
                    if (newDirection_ ||
                        pointAlmostEqual(q_current_, init_points_[current_point_index_]))
                    {
                        current_point_index_--;
                    }
                }
            }
        }
        

        // RCLCPP_INFO(rclcpp::get_logger("trajectory_manager"), "Current Point Index: %d", current_point_index_);
        // RCLCPP_INFO(rclcpp::get_logger("trajectory_manager"), "Target Q: [%f, %f, %f, %f, %f, %f]", 
        //                 init_points_[current_point_index_][0], init_points_[current_point_index_][1], init_points_[current_point_index_][2], 
        //                 init_points_[current_point_index_][3], init_points_[current_point_index_][4], init_points_[current_point_index_][5]);

        if((current_point_index_ == init_points_.size() - 1) && pointAlmostEqual(q_current_, init_points_[current_point_index_])){
            lock_ = false;
            trajectory_completed_ = true;
        } else {
            lock_ = true;
            trajectory_completed_ = false;
        }

        if (axe_value_ != 0.0) {
            for (int i = 0; i < 6; ++i) {
                double delta = std::abs(init_points_[current_point_index_][i] - q_current_[i]);
                dq = std::max(dq, delta);
            }
        
            traj_end_time_ = dq / std::abs(axe_value_);
        }

        traj_point_start.time_from_start = rclcpp::Duration::from_seconds(0.0);
        traj_point_target.time_from_start = rclcpp::Duration::from_seconds(traj_end_time_);

        // Use hold position when joystick is released to prevent drift
        const std::array<double, 6>& start_pos = (axe_value_ == 0.0) ? q_hold_ : q_current_;

        for (int i = 0; i < 6; i++)
        {
            traj_point_start.positions.push_back(start_pos[i]);
            traj_point_target.positions.push_back(init_points_[current_point_index_][i]);
        }
        traj_point_start.positions.push_back(gripper_);
        traj_point_target.positions.push_back(gripper_); // keep gripper position unchanged

        trajectory_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "right_finger_joint"};

        trajectory_msg.points.push_back(traj_point_start);
        if(axe_value_ != 0.0 && std::abs(dq) > 1e-6){
            trajectory_msg.points.push_back(traj_point_target);
        }
        
        axe_value_prev_ = axe_value_;

        updateStatusString();

        return trajectory_msg;
    }

    bool TrajectoryManager::pointAlmostEqual(const std::array<double, 6>& p1, const std::array<double, 6>& p2, double epsilon) {
        for (size_t i = 0; i < 6; ++i) {
            if (std::fabs(p1[i] - p2[i]) > epsilon)
                return false; 
        }
        return true;

    }

    void TrajectoryManager::reset()
    {
        axe_value_prev_ = 0.0;
        newDirection_ = false;
        q_hold_ = q_current_;
        q_hold_initialized_ = false;  // Force reinitialization when re-entering trajectory mode
    }

    std::string TrajectoryManager::getStatusString()
    {
        return status;
    }

    void TrajectoryManager::updateStatusString()
    {
        // State machine based on trajectory completion and active movement
        if (trajectory_completed_) {
            status = "ready";  // GREEN: deployment completed, robot is ready
        }
        else if (axe_value_ != 0.0) {
            status = "in progress";  // ORANGE: actively deploying/retracting
        }
        else {
            status = "not ready";  // RED: not deployed yet (retracted or unknown position)
        }
    }

}