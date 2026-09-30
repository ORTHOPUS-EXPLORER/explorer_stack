#ifndef EXPLORER_JOINT_UTILS_JOINT_MODE_RESOLVER_H
#define EXPLORER_JOINT_UTILS_JOINT_MODE_RESOLVER_H

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace space_control
{
enum class JointMode
{
  INVALID,
  EXPLORER,
  FULL
};

class JointModeResolver
{
public:
  JointModeResolver(
    std::vector<std::string> expected_names_explorer =
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
       "left_external_rod_joint_mimic", "left_fingertip_joint_mimic", "left_finger_joint_mimic",
       "right_external_rod_joint_mimic", "right_fingertip_joint_mimic", "right_finger_joint"},
    std::vector<std::string> expected_names_wheelchair =
      {"left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint",
       "right_rear_wheel_joint", "left_wheel_joint", "right_wheel_joint", "left_right_head_joint",
       "up_down_head_joint"},
    std::vector<std::string> mimic_joint_names =
      {"left_external_rod_joint_mimic", "left_fingertip_joint_mimic", "left_finger_joint_mimic",
       "right_external_rod_joint_mimic", "right_fingertip_joint_mimic"},
    std::string fallback_joint_name = "right_finger_joint")
  : expected_names_explorer_(std::move(expected_names_explorer)),
    expected_names_wheelchair_(std::move(expected_names_wheelchair)),
    mimic_joint_names_(std::move(mimic_joint_names)),
    fallback_joint_name_(std::move(fallback_joint_name))
  {
  }

  JointMode detect_mode(
    const std::vector<std::string>& msg_names, const rclcpp::Logger& logger,
    const std::string& log_tag) const
  {
    // Check for explorer
    bool valid_explorer = std::all_of(
      expected_names_explorer_.begin(), expected_names_explorer_.end(),
      [&](const std::string& name)
      {
        if (is_mimic_joint_(name))
        {
          // allow them to be missing
          return true;
        }
        return std::find(msg_names.begin(), msg_names.end(), name) != msg_names.end();
      });

    // Check for wheelchair
    bool all_wheelchair = std::all_of(
      expected_names_wheelchair_.begin(), expected_names_wheelchair_.end(),
      [&](const std::string& name)
      { return std::find(msg_names.begin(), msg_names.end(), name) != msg_names.end(); });

    // Check for any wheelchair joints present
    bool any_wheelchair = std::any_of(
      expected_names_wheelchair_.begin(), expected_names_wheelchair_.end(),
      [&](const std::string& name)
      { return std::find(msg_names.begin(), msg_names.end(), name) != msg_names.end(); });

    if (valid_explorer && all_wheelchair)
    {
      RCLCPP_INFO(logger, "%s Full robot (explorer + wheelchair) detected.", log_tag.c_str());
      return JointMode::FULL;
    }
    else if (valid_explorer && !any_wheelchair)
    {
      RCLCPP_INFO(logger, "%s Explorer-only configuration detected.", log_tag.c_str());
      return JointMode::EXPLORER;
    }
    else
    {
      RCLCPP_ERROR(
        logger, "%s Invalid joint configuration detected! Initialization failed.", log_tag.c_str());
      return JointMode::INVALID;
    }
  }

  bool build_joint_order(
    JointMode mode, const std::vector<std::string>& msg_names, std::vector<size_t>& joint_order,
    const rclcpp::Logger& logger, const std::string& log_tag) const
  {
    joint_order.clear();
    if (mode != JointMode::FULL && mode != JointMode::EXPLORER)
    {
      return false;
    }

    if (mode == JointMode::FULL)
    {
      joint_order.reserve(expected_names_wheelchair_.size() + expected_names_explorer_.size());
      for (const auto& name : expected_names_wheelchair_)
      {
        auto it = std::find(msg_names.begin(), msg_names.end(), name);
        joint_order.push_back(std::distance(msg_names.begin(), it));
      }
    }
    else
    {
      joint_order.reserve(expected_names_explorer_.size());
    }

    for (const auto& name : expected_names_explorer_)
    {
      auto it = std::find(msg_names.begin(), msg_names.end(), name);
      if (it != msg_names.end())
      {
        joint_order.push_back(std::distance(msg_names.begin(), it));
      }
      // Fallback to fixed joint for mimic joint(s)
      else if (is_mimic_joint_(name))
      {
        auto fallback_it = std::find(msg_names.begin(), msg_names.end(), fallback_joint_name_);
        if (fallback_it != msg_names.end())
        {
          joint_order.push_back(std::distance(msg_names.begin(), fallback_it));
          RCLCPP_WARN(
            logger, "%s Joint %s missing, using %s as fallback", log_tag.c_str(), name.c_str(),
            fallback_joint_name_.c_str());
        }
        else
        {
          RCLCPP_ERROR(
            logger, "%s Neither %s nor %s found! Cannot initialize properly", log_tag.c_str(),
            name.c_str(), fallback_joint_name_.c_str());
          return false;
        }
      }
      else
      {
        RCLCPP_ERROR(
          logger, "%s Joint %s not found and no fallback defined", log_tag.c_str(), name.c_str());
        return false;
      }
    }

    log_joint_order_debug_(mode, msg_names, joint_order, logger);
    return true;
  }

  const std::vector<std::string>& expected_names_explorer() const
  {
    return expected_names_explorer_;
  }

  const std::vector<std::string>& expected_names_wheelchair() const
  {
    return expected_names_wheelchair_;
  }

private:
  bool is_mimic_joint_(const std::string& name) const
  {
    return std::find(mimic_joint_names_.begin(), mimic_joint_names_.end(), name) !=
           mimic_joint_names_.end();
  }

  void log_joint_order_debug_(
    JointMode mode, const std::vector<std::string>& msg_names,
    const std::vector<size_t>& joint_order, const rclcpp::Logger& logger) const
  {
    // Debug: Print out sizes to check bounds
    RCLCPP_INFO(logger, "joint_order.size() = %zu", joint_order.size());
    RCLCPP_INFO(logger, "current_pos_.name.size() = %zu", msg_names.size());

    int safe_limit = std::min<int>(joint_order.size(), msg_names.size());
    int n_to_print = std::min<int>(safe_limit, (mode == JointMode::FULL ? 20 : 12));

    for (int i = 0; i < n_to_print; i++)
    {
      RCLCPP_INFO(
        logger, "Joint order[%d]: %zu, Name: %s", i, joint_order[i],
        msg_names[joint_order[i]].c_str());
    }

    if (
      joint_order.size() < static_cast<size_t>(n_to_print) ||
      msg_names.size() < static_cast<size_t>(n_to_print))
    {
      RCLCPP_WARN(
        logger,
        "WARNING: joint_order or current_pos_.name was smaller than expected! Potential config "
        "problem.");
    }
  }

  std::vector<std::string> expected_names_explorer_;
  std::vector<std::string> expected_names_wheelchair_;
  std::vector<std::string> mimic_joint_names_;
  std::string fallback_joint_name_;
};

}  // namespace space_control

#endif  // EXPLORER_JOINT_UTILS_JOINT_MODE_RESOLVER_H
