#include "explorer_user_interfaces_cpp/led_status_config.h"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace space_control
{
namespace
{
using explorer_msgs::msg::LedColor;
using explorer_msgs::msg::LedCommand;
using explorer_msgs::msg::LedEffect;

/// Value of \p key in \p root, throws if missing or not convertible to T
template <typename T>
T read_value(const YAML::Node& root, const std::string& key, const std::string& context)
{
  const YAML::Node node = root[key];
  if (!node)
  {
    throw std::runtime_error("LED status config " + context + ": missing '" + key + "'");
  }
  try
  {
    return node.as<T>();
  }
  catch (const YAML::Exception&)
  {
    throw std::runtime_error("LED status config " + context + ": invalid '" + key + "'");
  }
}

/// Value of \p key in \p root, \p default_value if missing, throws if not convertible to T
template <typename T>
T read_optional_value(
  const YAML::Node& root, const std::string& key, const T& default_value,
  const std::string& context)
{
  if (!root[key])
  {
    return default_value;
  }
  return read_value<T>(root, key, context);
}

/// Mode of a status effect, throws if unknown. flash_once is refused: it's only accepted on the
/// flash topic, a status is sent on the command topic
std::string read_mode(const YAML::Node& node, const std::string& context)
{
  static const std::vector<std::string> MODES = {
    LedEffect::MODE_OFF,      LedEffect::MODE_SOLID, LedEffect::MODE_FADE_IN,
    LedEffect::MODE_FADE_OUT, LedEffect::MODE_BLINK, LedEffect::MODE_ALTERNATE,
    LedEffect::MODE_BREATHE,
  };

  const auto mode = read_value<std::string>(node, "mode", context);
  if (std::find(MODES.begin(), MODES.end(), mode) == MODES.end())
  {
    throw std::runtime_error("LED status config " + context + ": invalid mode '" + mode + "'");
  }
  return mode;
}

/// LED indices of \p key in \p node, empty if missing, throws if one doesn't fit in a LedEffect
std::vector<std::uint16_t> read_leds(
  const YAML::Node& node, const std::string& key, const std::string& context)
{
  std::vector<std::uint16_t> leds;
  for (const int led : read_optional_value<std::vector<int>>(node, key, {}, context))
  {
    if (led < 0 || led > std::numeric_limits<std::uint16_t>::max())
    {
      throw std::runtime_error(
        "LED status config " + context + ": invalid LED index " + std::to_string(led));
    }
    leds.push_back(static_cast<std::uint16_t>(led));
  }
  return leds;
}

LedEffect read_effect(const YAML::Node& node, const std::string& context)
{
  if (!node.IsMap())
  {
    throw std::runtime_error(
      "LED status config " + context + ": an effect must be key: value pairs");
  }

  LedEffect effect;
  effect.leds = read_leds(node, "leds", context);
  effect.mode = read_mode(node, context);
  effect.speed = read_optional_value<double>(node, "speed", 0.0, context);
  effect.intensities = read_optional_value<std::vector<double>>(node, "intensities", {}, context);
  effect.min_level = read_optional_value<double>(node, "min_level", 0.0, context);

  using Components = std::vector<std::vector<float>>;
  for (const auto& components : read_optional_value<Components>(node, "colors", {}, context))
  {
    if (components.size() != 3 && components.size() != 4)
    {
      throw std::runtime_error(
        "LED status config " + context + ": a colour is [r, g, b] or [r, g, b, w]");
    }
    effect.colors.push_back(
      LedColor()
        .set__r(components[0])
        .set__g(components[1])
        .set__b(components[2])
        .set__w(components.size() == 4 ? components[3] : 0.0f));
  }
  return effect;
}

/// Command of a status: its list of effects, at least one (explorer_leds ignores an empty command)
LedCommand read_command(const YAML::Node& node, const std::string& context)
{
  if (!node.IsSequence() || node.size() == 0)
  {
    throw std::runtime_error("LED status config " + context + ": expected a list of effects");
  }

  LedCommand command;
  for (const auto& effect : node)
  {
    command.effects.push_back(read_effect(effect, context));
  }
  return command;
}

/// Section \p key of \p root, as status name: list of effects. Empty if missing
std::map<std::string, LedCommand> read_section(
  const YAML::Node& root, const std::string& key, const std::string& path)
{
  std::map<std::string, LedCommand> commands;
  const YAML::Node section = root[key];
  if (!section)
  {
    return commands;
  }
  if (!section.IsMap())
  {
    throw std::runtime_error(
      "LED status config " + path + ": '" + key + "' must be status: list of effects pairs");
  }

  for (const auto& entry : section)
  {
    const auto name = entry.first.as<std::string>();
    commands[name] = read_command(entry.second, path + " '" + key + "." + name + "'");
  }
  return commands;
}

/// \p effect restricted to \p leds: its LEDs that are in \p leds (every one of \p leds if the
/// effect targets every LED)
LedEffect restrict_to(LedEffect effect, const std::vector<std::uint16_t>& leds)
{
  if (effect.leds.empty())
  {
    effect.leds = leds;
    return effect;
  }

  std::vector<std::uint16_t> kept;
  std::copy_if(
    effect.leds.begin(), effect.leds.end(), std::back_inserter(kept), [&leds](std::uint16_t led)
    { return std::find(leds.begin(), leds.end(), led) != leds.end(); });
  effect.leds = std::move(kept);
  return effect;
}

/// \p effect as a solid colour: its first colour, an off effect left off
LedEffect as_solid(LedEffect effect)
{
  if (effect.mode == LedEffect::MODE_OFF)
  {
    return effect;
  }

  effect.mode = LedEffect::MODE_SOLID;
  effect.speed = 0.0;
  effect.min_level = 0.0;
  // Solid takes a single colour, an alternate effect shows its first one
  effect.colors.resize(std::min<std::size_t>(effect.colors.size(), 1));
  effect.intensities.resize(std::min<std::size_t>(effect.intensities.size(), 1));
  return effect;
}
}  // namespace

LedStatusConfig load_led_status_config(const std::string& path)
{
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& e)
  {
    throw std::runtime_error("Cannot read LED status config " + path + ": " + e.what());
  }

  if (!root.IsMap())
  {
    throw std::runtime_error("LED status config " + path + ": expected key: value pairs");
  }

  LedStatusConfig config;

  for (auto& [name, command] : read_section(root, "retract_status", path))
  {
    // Named like the enum values. READY is left out: the mode is shown then
    if (name == "RETRACTED")
    {
      config.retract_status_commands[RetractStatus::RETRACTED] = std::move(command);
    }
    else if (name == "IN_PROGRESS")
    {
      config.retract_status_commands[RetractStatus::IN_PROGRESS] = std::move(command);
    }
    else
    {
      throw std::runtime_error(
        "LED status config " + path + ": invalid retract status '" + name +
        "', expected RETRACTED or IN_PROGRESS");
    }
  }

  for (auto& [name, command] : read_section(root, "modes", path))
  {
    config.mode_commands[name] = std::move(command);
  }

  config.centered_leds = read_leds(root, "centered_leds", path);

  return config;
}

std::optional<explorer_msgs::msg::LedCommand> select_led_command(
  const LedStatusConfig& config, RetractStatus retract_status,
  const std::optional<std::string>& mode_name)
{
  const auto mode_it = mode_name ? config.mode_commands.find(*mode_name) : config.mode_commands.end();
  const bool has_mode_command = mode_it != config.mode_commands.end();

  if (retract_status == RetractStatus::READY)
  {
    if (!has_mode_command)
    {
      return std::nullopt;
    }
    return mode_it->second;
  }

  LedCommand command;
  const auto retract_it = config.retract_status_commands.find(retract_status);
  if (retract_it != config.retract_status_commands.end())
  {
    command = retract_it->second;
  }

  // The mode colour over the retract status on the centered LEDs, applied after it so it wins
  // there. Only while RETRACTED: IN_PROGRESS shows on every LED, so they all switch to the mode
  // together once READY (LEDs already playing the mode would otherwise be out of phase with the
  // others)
  if (
    retract_status == RetractStatus::RETRACTED && has_mode_command && !config.centered_leds.empty())
  {
    for (const auto& effect : mode_it->second.effects)
    {
      auto restricted = restrict_to(effect, config.centered_leds);
      if (!restricted.leds.empty())
      {
        command.effects.push_back(as_solid(std::move(restricted)));
      }
    }
  }

  if (command.effects.empty())
  {
    return std::nullopt;
  }
  return command;
}

}  // namespace space_control
