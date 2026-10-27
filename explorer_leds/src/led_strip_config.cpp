/*
 *  led_strip_config.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include "explorer_leds/led_strip_config.h"

#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace explorer_leds
{
namespace
{
/// Value of \p key in \p root, throws if missing or not convertible to T
template <typename T>
T read_value(const YAML::Node& root, const std::string& key, const std::string& path)
{
  const YAML::Node node = root[key];
  if (!node)
  {
    throw std::runtime_error("LED strip config " + path + ": missing '" + key + "'");
  }
  try
  {
    return node.as<T>();
  }
  catch (const YAML::Exception&)
  {
    throw std::runtime_error("LED strip config " + path + ": invalid '" + key + "'");
  }
}

/// Value of \p key in \p root, \p default_value if missing, throws if not convertible to T
template <typename T>
T read_optional_value(
  const YAML::Node& root, const std::string& key, const T& default_value, const std::string& path)
{
  if (!root[key])
  {
    return default_value;
  }
  return read_value<T>(root, key, path);
}

/**
  * \brief Pattern of \p key in \p root, throws if missing or invalid.
  * flash_once is refused: a pattern replacing the effects lasts as long as the robot state.
  */
LedPattern read_pattern(const YAML::Node& root, const std::string& key, const std::string& path)
{
  const YAML::Node node = root[key];
  if (!node)
  {
    throw std::runtime_error("LED strip config " + path + ": missing '" + key + "'");
  }
  if (!node.IsMap())
  {
    throw std::runtime_error("LED strip config " + path + ": '" + key + "' must be a pattern");
  }

  const std::string context = path + " '" + key + "'";
  const auto mode_name = read_value<std::string>(node, "mode", context);
  const auto mode = mode_from_string(mode_name);
  if (!mode || *mode == LedMode::FLASH_ONCE)
  {
    throw std::runtime_error("LED strip config " + context + ": invalid mode '" + mode_name + "'");
  }

  LedPattern pattern;
  pattern.mode = *mode;
  pattern.speed = read_optional_value<double>(node, "speed", 0.0, context);
  pattern.intensities =
    read_optional_value<std::vector<double>>(node, "intensities", std::vector<double>{}, context);
  using Components = std::vector<std::vector<double>>;
  for (const auto& components :
       read_optional_value<Components>(node, "colors", Components{}, context))
  {
    if (components.size() != 3 && components.size() != 4)
    {
      throw std::runtime_error(
        "LED strip config " + context + ": a colour is [r, g, b] or [r, g, b, w]");
    }
    pattern.colors.push_back(
      {components[0], components[1], components[2], components.size() == 4 ? components[3] : 0.0});
  }

  // Same checks as a commanded effect
  const auto error = LedAnimation().set_pattern(pattern, 0.0);
  if (error)
  {
    throw std::runtime_error("LED strip config " + context + ": " + *error);
  }
  return pattern;
}
}  // namespace

LedStripConfig load_led_strip_config(const std::string& path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& e)
  {
    throw std::runtime_error("Cannot read LED strip config " + path + ": " + e.what());
  }

  if (!root.IsMap())
  {
    throw std::runtime_error("LED strip config " + path + ": expected key: value pairs");
  }

  LedStripConfig config;
  config.spi_device = read_value<std::string>(root, "spi_device", path);

  const auto led_count = read_value<int>(root, "led_count", path);
  if (led_count <= 0)
  {
    throw std::runtime_error("LED strip config " + path + ": 'led_count' must be positive");
  }
  config.led_count = static_cast<std::size_t>(led_count);

  config.rgbw = read_optional_value<bool>(root, "rgbw", config.rgbw, path);

  config.max_brightness =
    read_optional_value<double>(root, "max_brightness", config.max_brightness, path);
  if (!(config.max_brightness >= 0.0 && config.max_brightness <= 1.0))
  {
    throw std::runtime_error("LED strip config " + path + ": 'max_brightness' must be in [0, 1]");
  }

  config.lost_communication_pattern = read_pattern(root, "lost_communication", path);
  config.not_ready_pattern = read_pattern(root, "not_ready", path);

  return config;
}
}  // namespace explorer_leds
