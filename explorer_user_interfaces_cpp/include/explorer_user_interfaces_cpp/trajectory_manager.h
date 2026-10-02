#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono;

namespace space_control
{

struct JointLimit
{
  bool enabled;
  double min;
  double max;
};

// Indicates arm position according to deploy trajectory
enum class RetractStatus
{
  RETRACTED,
  IN_PROGRESS,
  READY
};

// Convert RetractStatus to string
std::string to_string(RetractStatus status);

class TrajectoryManager
{
public:
  TrajectoryManager() = default;

  // Load and validate the trajectory file.
  // Trajectory control is enabled only when this succeeds
  bool load_trajectory(const std::string& filename);

  // Report trajectory control "readyness".
  bool is_enabled() const;

  void update(std::array<double, 7> q_current, float axe_value);

  std::optional<trajectory_msgs::msg::JointTrajectory> get_trajectory();

  void reset();

  // Get current RetractStatus
  RetractStatus get_status() const;

private:
  bool enabled_ = false;

  std::vector<JointLimit> joint_limits_;

  YAML::Node traj;

  std::vector<std::array<double, 6>> init_points_;
  std::size_t current_point_index_ = 0;

  std::array<double, 6> q_current_;
  // Position to hold when joystick is released
  std::array<double, 6> q_hold_ = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  };
  // Track if q_hold_ has been initialized with actual position
  bool q_hold_initialized_ = false;
  double gripper_;
  float axe_value_ = 0.0;
  float axe_value_prev_ = axe_value_;
  bool new_direction_ = false;

  double traj_end_time_ = 0.0;

  bool return_sequence_active_ = false;
  bool needs_return_to_ready_ = false;
  bool ready_just_reached_ = false;
  bool trajectory_completed_ = false;
  RetractStatus status_ = RetractStatus::RETRACTED;

  static bool are_point_almost_equal_(
    const std::array<double, 6>& p1, const std::array<double, 6>& p2,
    double epsilon = 3.5e-2);  //2° tolerance

  bool validate_trajectory_() const;

  void update_status_();
};

}  // namespace space_control