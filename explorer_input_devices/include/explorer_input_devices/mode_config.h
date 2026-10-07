/*
 *  mode_config.h
 *  Copyright (C) 2025 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_INPUT_DEVICES_MODE_CONFIG_H
#define EXPLORER_INPUT_DEVICES_MODE_CONFIG_H

#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/*
 * Data model and parser for the joystick mode configuration YAML (config_mode_*.yaml).
 */
namespace input_device
{
// Indicates a physical stick axis
// (NONE means an axis entry is declared but inactive)
enum class JoystickAxis
{
  NONE,
  AX1,
  AX2
};

// What a stick axis does in a given mode
// (NONE is an inactive axis)
enum class ControlName
{
  NONE,
  CARTESIAN_X,
  CARTESIAN_Y,
  CARTESIAN_Z,
  ROTATION_X,
  ROTATION_Y,
  ROTATION_Z,
  JOINT_6,
  CHANGE_SPEED,
  DRINK,
  GRIPPER,
  COMPLEX_X,
  COMPLEX_Y,
  GRIPPER_X,
  GRIPPER_Y,
  GRIPPER_Z,
  TRAJECTORY_CONTROL
};

// YAML spellings ("cartesian_X", "ax1", ...). NONE maps to the empty string.
std::string to_string(ControlName control);
std::string to_string(JoystickAxis axis);
std::optional<ControlName> control_name_from_string(const std::string& text);
std::optional<JoystickAxis> joystick_axis_from_string(const std::string& text);

// Information about each axis control
struct AxisInfo
{
  ControlName control_name = ControlName::NONE;
  JoystickAxis joystick_axis = JoystickAxis::NONE;
  int direction;
  double scale;
  double smoothing_alpha = 1.0;  // Smoothing factor (1.0 = no smoothing, 0.1 = heavy smoothing)
  std::map<std::string, double> params;
};

// Actions associated with button clicks
struct ButtonAction
{
  std::string long_click;
  std::string short_click;
};

// Information about each button mode
struct ButtonMode
{
  std::string name;
  std::vector<AxisInfo> axes;
  ButtonAction buttons;
};

// Information about the overall mode
struct ModeInfo
{
  std::string name;
  std::string display_name;
  std::string description;
};

// Complete mode data structure
struct ModeData
{
  ModeInfo mode_info;
  std::string initial_mode;
  std::unordered_map<std::string, ButtonMode> button_modes_map;
};

/**
  * \brief Parse a mode configuration file
  *
  * \param filename Path to the YAML file
  * \param error    Human readable error string on failure, empty otherwise
  * \return The parsed configuration, or std::nu llopt when the file cannot be opened or invalid config
  */
std::optional<ModeData> load_mode_data(const std::string& filename, std::string& error);

/**
  * \brief Check a parsed configuration for consistency
  *
  * \param data                 Configuration to check
  * \param valid_control_names  The control_name values the consuming device knows how to
  *                             execute. The config module owns the structural rules; the
  *                             device owns the vocabulary
  * \param error                Human readable error string on failure, empty otherwise
  * \return true when the configuration is correct
  */
bool validate_mode_data(
  const ModeData& data, const std::unordered_set<ControlName>& valid_control_names,
  std::string& error);
}  // namespace input_device

#endif
