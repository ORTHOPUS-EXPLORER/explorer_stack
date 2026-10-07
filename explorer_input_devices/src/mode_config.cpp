/*
 *  mode_config.cpp
 *  Copyright (C) 2025 Orthopus
 *  All rights reserved.
 */
#include "explorer_input_devices/mode_config.h"

#include <algorithm>
#include <cstddef>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace input_device
{
namespace
{
// One table per enum, used in both directions
constexpr std::pair<const char*, ControlName> control_names[] = {
  {"", ControlName::NONE},
  {"cartesian_X", ControlName::CARTESIAN_X},
  {"cartesian_Y", ControlName::CARTESIAN_Y},
  {"cartesian_Z", ControlName::CARTESIAN_Z},
  {"rotation_X", ControlName::ROTATION_X},
  {"rotation_Y", ControlName::ROTATION_Y},
  {"rotation_Z", ControlName::ROTATION_Z},
  {"joint_6", ControlName::JOINT_6},
  {"change_speed", ControlName::CHANGE_SPEED},
  {"drink", ControlName::DRINK},
  {"gripper", ControlName::GRIPPER},
  {"complex_X", ControlName::COMPLEX_X},
  {"complex_Y", ControlName::COMPLEX_Y},
  {"gripper_X", ControlName::GRIPPER_X},
  {"gripper_Y", ControlName::GRIPPER_Y},
  {"gripper_Z", ControlName::GRIPPER_Z},
  {"trajectory_control", ControlName::TRAJECTORY_CONTROL},
};

constexpr std::pair<const char*, JoystickAxis> joystick_axes[] = {
  {"", JoystickAxis::NONE},
  {"ax1", JoystickAxis::AX1},
  {"ax2", JoystickAxis::AX2},
};

template <typename Enum, std::size_t N>
std::string enum_to_string(const std::pair<const char*, Enum> (&enum_pair_array)[N], Enum value)
{
  for (const auto& [text, existing_value] : enum_pair_array)
  {
    if (existing_value == value) return text;
  }
  return "unknown";
}

template <typename Enum, std::size_t N>
std::optional<Enum> enum_from_string(
  const std::pair<const char*, Enum> (&enum_pair_array)[N], const std::string& text)
{
  for (const auto& [name, existing_value] : enum_pair_array)
  {
    if (text == name) return existing_value;
  }
  return std::nullopt;
}
}  // namespace

std::string to_string(ControlName control) { return enum_to_string(control_names, control); }

std::string to_string(JoystickAxis axis) { return enum_to_string(joystick_axes, axis); }

std::optional<ControlName> control_name_from_string(const std::string& text)
{
  return enum_from_string(control_names, text);
}

std::optional<JoystickAxis> joystick_axis_from_string(const std::string& text)
{
  return enum_from_string(joystick_axes, text);
}

std::optional<ModeData> load_mode_data(const std::string& filename, std::string& error)
{
  error.clear();

  YAML::Node root;
  try
  {
    root = YAML::LoadFile(filename);
  }
  catch (const YAML::BadFile&)
  {
    error = "Cannot open mode_file: " + filename;
    return std::nullopt;
  }
  catch (const YAML::ParserException& e)
  {
    error = "YAML parsing error in file " + filename + ": " + e.what();
    return std::nullopt;
  }

  ModeData data;

  // Parse mode information
  if (root["mode_info"])
  {
    auto info = root["mode_info"];
    data.mode_info.name = info["name"].as<std::string>("");
    data.mode_info.display_name = info["display_name"].as<std::string>("");
    data.mode_info.description = info["description"].as<std::string>("");
  }

  // Parse button modes and their configurations
  if (root["button_mappings"])
  {
    auto mappings = root["button_mappings"];

    for (auto it = mappings.begin(); it != mappings.end(); ++it)
    {
      ButtonMode mode;
      mode.name = it->first.as<std::string>();
      auto button_mode = it->second;

      if (data.initial_mode.empty())
      {
        data.initial_mode = mode.name;
      }

      // axes
      if (button_mode["axes"])
      {
        for (auto axis_node : button_mode["axes"])
        {
          AxisInfo axis;

          const auto control_text = axis_node["control_name"].as<std::string>("");
          const auto control = control_name_from_string(control_text);
          if (!control)
          {
            error = "Unknown control_name '" + control_text + "' in mode '" + mode.name + "'";
            return std::nullopt;
          }
          axis.control_name = *control;

          const auto axis_text = axis_node["joystick_axis"].as<std::string>("");
          const auto joystick_axis = joystick_axis_from_string(axis_text);
          if (!joystick_axis)
          {
            error = "Unknown joystick_axis '" + axis_text + "' in mode '" + mode.name +
                    "' (must be ax1 or ax2)";
            return std::nullopt;
          }
          axis.joystick_axis = *joystick_axis;

          axis.direction = axis_node["direction"].as<int>(1);
          axis.scale = axis_node["scale"].as<double>(1.0);

          // Parse smoothing_alpha (default 1.0 = no smoothing)
          axis.smoothing_alpha = axis_node["smoothing_alpha"].as<double>(1.0);
          // Clamp alpha to valid range [0.0, 1.0]
          axis.smoothing_alpha = std::max(0.0, std::min(1.0, axis.smoothing_alpha));

          if (axis_node["params"])
          {
            for (auto p : axis_node["params"])
            {
              axis.params[p.first.as<std::string>()] = p.second.as<double>();
            }
          }
          mode.axes.push_back(axis);
        }
      }

      // buttons
      if (button_mode["button"])
      {
        ButtonAction action;
        for (auto button_action_node : button_mode["button"])
        {
          if (
            button_action_node["long_click"] &&
            !button_action_node["long_click"].as<std::string>().empty())
            action.long_click = button_action_node["long_click"].as<std::string>();
          if (
            button_action_node["short_click"] &&
            !button_action_node["short_click"].as<std::string>().empty())
            action.short_click = button_action_node["short_click"].as<std::string>();
        }
        mode.buttons = action;
      }

      data.button_modes_map[mode.name] = mode;
    }
  }

  return data;
}

bool validate_mode_data(
  const ModeData& data, const std::unordered_set<ControlName>& valid_control_names,
  std::string& error)
{
  error.clear();

  // --- mode_info verification ---
  if (data.mode_info.name.empty() || data.mode_info.display_name.empty())
  {
    error = "Invalid YAML: mode_info.name or display_name missing";
    return false;
  }
  // --- button_modes_map verification ---
  if (data.button_modes_map.empty())
  {
    error = "Invalid YAML: button_mappings must contain at least one mode";
    return false;
  }
  if (!data.button_modes_map.count(data.initial_mode))
  {
    error = "Invalid YAML: initial mode '" + data.initial_mode + "' is not a declared mode";
    return false;
  }

  // --- Validate each button mode ---
  for (const auto& [name, mode] : data.button_modes_map)
  {
    // Axes check
    for (const auto& axis : mode.axes)
    {
      // Special case: inactive axis
      if (axis.control_name == ControlName::NONE)
      {
        if (axis.direction != 0)
        {
          error =
            "Invalid axis in mode '" + name + "': direction must be 0 when control_name is empty";
          return false;
        }
        if (axis.scale != 0.0)
        {
          error = "Invalid axis in mode '" + name + "': scale must be 0 when control_name is empty";
          return false;
        }
        if (axis.joystick_axis != JoystickAxis::NONE)
        {
          error = "Invalid axis in mode '" + name +
                  "': joystick_axis must be empty when control_name is empty";
          return false;
        }
        // skip the rest of validation for this axis
        continue;
      }

      // Normal validations
      // control_name verification
      if (!valid_control_names.count(axis.control_name))
      {
        error =
          "Unsupported control_name '" + to_string(axis.control_name) + "' in mode '" + name + "'";
        return false;
      }
      // joystick_axis verification
      if (axis.joystick_axis == JoystickAxis::NONE)
      {
        error = "Missing joystick_axis for '" + to_string(axis.control_name) + "' in mode '" +
                name + "' (must be ax1 or ax2)";
        return false;
      }
      // direction verification
      if (axis.direction != 1 && axis.direction != -1)
      {
        error = "direction must be 1 or -1 in mode '" + name + "' axis '" +
                to_string(axis.control_name) + "'";
        return false;
      }
      // scale verification
      if (axis.scale <= 0)
      {
        error =
          "scale must be > 0 in mode '" + name + "' axis '" + to_string(axis.control_name) + "'";
        return false;
      }
    }

    // Buttons validity
    if (!mode.buttons.short_click.empty())
    {
      if (!data.button_modes_map.count(mode.buttons.short_click))
      {
        error = "Invalid short_click reference '" + mode.buttons.short_click + "' from mode '" +
                name + "'";
        return false;
      }
    }

    if (!mode.buttons.long_click.empty())
    {
      if (!data.button_modes_map.count(mode.buttons.long_click))
      {
        error =
          "Invalid long_click reference '" + mode.buttons.long_click + "' from mode '" + name + "'";
        return false;
      }
    }
  }

  return true;
}
}  // namespace input_device
