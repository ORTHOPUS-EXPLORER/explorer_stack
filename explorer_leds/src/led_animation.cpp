/*
 *  led_animation.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include "explorer_leds/led_animation.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace explorer_leds
{
namespace
{
bool is_unit_value(double value) { return std::isfinite(value) && value >= 0.0 && value <= 1.0; }

bool is_unit_color(const Rgbw& color)
{
  return is_unit_value(color.r) && is_unit_value(color.g) && is_unit_value(color.b) &&
         is_unit_value(color.w);
}

Rgbw scale(const Rgbw& color, double factor)
{
  return {color.r * factor, color.g * factor, color.b * factor, color.w * factor};
}

/// Colour at \p index of \p pattern scaled by its intensity
Rgbw scaled_color(const LedPattern& pattern, std::size_t index)
{
  return scale(pattern.colors[index], pattern.intensities[index]);
}
}  // namespace

std::optional<LedMode> mode_from_string(const std::string& name)
{
  static const std::unordered_map<std::string, LedMode> MODES{
    {"off", LedMode::OFF},         {"solid", LedMode::SOLID},
    {"fade_in", LedMode::FADE_IN}, {"fade_out", LedMode::FADE_OUT},
    {"blink", LedMode::BLINK},     {"alternate", LedMode::ALTERNATE},
    {"breathe", LedMode::BREATHE}, {"flash_once", LedMode::FLASH_ONCE},
  };

  const auto mode = MODES.find(name);
  if (mode == MODES.end())
  {
    return std::nullopt;
  }
  return mode->second;
}

double capped_speed(LedMode mode, double speed)
{
  if (mode == LedMode::BLINK || mode == LedMode::ALTERNATE || mode == LedMode::BREATHE)
  {
    return std::min(speed, MAX_REPEATING_SPEED_HZ);
  }
  return speed;
}

std::optional<std::string> LedAnimation::set_pattern(const LedPattern& pattern, double now_seconds)
{
  auto error = validate_(pattern);
  if (error)
  {
    return error;
  }

  LedPattern capped = pattern;
  capped.speed = capped_speed(pattern.mode, pattern.speed);

  if (capped.mode == LedMode::FLASH_ONCE)
  {
    flash_ = PlayingPattern{capped, now_seconds};
  }
  else if (capped != base_.pattern)
  {
    base_ = PlayingPattern{capped, now_seconds};
    flash_.reset();
  }
  return std::nullopt;
}

std::optional<std::string> LedAnimation::validate_(const LedPattern& pattern)
{
  // Nothing to display, colours are ignored
  if (pattern.mode == LedMode::OFF)
  {
    return std::nullopt;
  }

  if (pattern.colors.size() != pattern.intensities.size())
  {
    return "colors and intensities must have the same size (" +
           std::to_string(pattern.colors.size()) + " colors, " +
           std::to_string(pattern.intensities.size()) + " intensities)";
  }

  if (pattern.mode == LedMode::ALTERNATE)
  {
    if (pattern.colors.size() < 2)
    {
      return "alternate mode needs at least 2 colors";
    }
  }
  else if (pattern.colors.size() != 1)
  {
    return "this mode needs exactly 1 color, got " + std::to_string(pattern.colors.size());
  }

  if (!std::all_of(pattern.colors.begin(), pattern.colors.end(), is_unit_color))
  {
    return "color components must be in [0, 1]";
  }

  if (!std::all_of(pattern.intensities.begin(), pattern.intensities.end(), is_unit_value))
  {
    return "intensities must be in [0, 1]";
  }

  // A solid colour doesn't move, any other mode needs a speed
  if (pattern.mode != LedMode::SOLID && !(std::isfinite(pattern.speed) && pattern.speed > 0.0))
  {
    return "speed must be strictly positive for this mode";
  }

  return std::nullopt;
}

Rgbw LedAnimation::compute(double now_seconds) const
{
  if (flash_)
  {
    const double flash_cycles = (now_seconds - flash_->start_seconds) * flash_->pattern.speed;
    if (flash_cycles < 1.0)
    {
      return compute_(*flash_, now_seconds);
    }
  }
  return compute_(base_, now_seconds);
}

Rgbw LedAnimation::compute_(const PlayingPattern& playing, double now_seconds)
{
  const LedPattern& pattern = playing.pattern;
  // Cycles done since the pattern started, the unit every mode is timed with
  const double cycles = std::max(now_seconds - playing.start_seconds, 0.0) * pattern.speed;

  switch (pattern.mode)
  {
    case LedMode::OFF:
      return {};
    case LedMode::SOLID:
    case LedMode::FLASH_ONCE:
      return scaled_color(pattern, 0);
    case LedMode::FADE_IN:
      return scale(scaled_color(pattern, 0), std::min(cycles, 1.0));
    case LedMode::FADE_OUT:
      return scale(scaled_color(pattern, 0), 1.0 - std::min(cycles, 1.0));
    case LedMode::BLINK:
      // On during the first half of each cycle
      return (cycles - std::floor(cycles)) < 0.5 ? scaled_color(pattern, 0) : Rgbw{};
    case LedMode::ALTERNATE:
      return scaled_color(pattern, static_cast<std::size_t>(cycles) % pattern.colors.size());
    case LedMode::BREATHE:
      // Smooth rise and fall: off at the start of each cycle, full colour halfway
      return scale(scaled_color(pattern, 0), (1.0 - std::cos(2.0 * M_PI * cycles)) / 2.0);
  }
  return {};
}
}  // namespace explorer_leds
