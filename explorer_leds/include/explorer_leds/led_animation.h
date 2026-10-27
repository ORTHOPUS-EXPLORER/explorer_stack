/*
 *  led_animation.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_LED_ANIMATION_H
#define EXPLORER_LEDS_LED_ANIMATION_H

#include <optional>
#include <string>
#include <vector>

namespace explorer_leds
{
/// Colour sent to the LEDs, each component in [0, 1], w driving the white LED of RGBW strips
struct Rgbw
{
  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
  double w = 0.0;

  bool operator==(const Rgbw& other) const
  {
    return r == other.r && g == other.g && b == other.b && w == other.w;
  }
  bool operator!=(const Rgbw& other) const { return !(*this == other); }
};

enum class LedMode
{
  OFF,
  SOLID,
  FADE_IN,
  FADE_OUT,
  BLINK,
  ALTERNATE,
  BREATHE,
  FLASH_ONCE
};

/// Parse a mode name ("off", "solid", "fade_in", "fade_out", "blink", "alternate", "breathe",
/// "flash_once")
[[nodiscard]] std::optional<LedMode> mode_from_string(const std::string& name);

/// Highest speed of the repeating modes (blink, alternate, breathe).
/// Flashing faster than 3 times per second can trigger photosensitive seizures.
constexpr double MAX_REPEATING_SPEED_HZ = 3.0;

/// Speed \p mode is played at: \p speed capped to MAX_REPEATING_SPEED_HZ for repeating modes
[[nodiscard]] double capped_speed(LedMode mode, double speed);

/// Animation requested for the LEDs, see explorer_msgs/msg/LedCommand for each field meaning
struct LedPattern
{
  LedMode mode = LedMode::OFF;
  double speed = 0.0;
  std::vector<Rgbw> colors;
  std::vector<double> intensities;

  bool operator==(const LedPattern& other) const
  {
    return mode == other.mode && speed == other.speed && colors == other.colors &&
           intensities == other.intensities;
  }
  bool operator!=(const LedPattern& other) const { return !(*this == other); }
};

/**
  * \brief Computes the colour the LEDs should display at a given time.
  *
  * Holds the pattern playing, and a flash played over it, and turns them into a single colour.
  * Times are in seconds on any monotonic clock, as long as the same one is used for every call.
  * Independent of ROS and of the LED hardware.
  */
class LedAnimation
{
public:
  /**
    * \brief Play \p pattern from \p now_seconds if it is valid.
    *
    * - A flash_once plays over the current pattern, which resumes once the flash is over.
    * - The same pattern as the one playing keeps going instead of restarting, so publishers can
    *   send their command again as often as they want.
    * - Any other pattern replaces the current one and cancels a flash in progress.
    *
    * Repeating modes are played at capped_speed().
    *
    * \return the reason the pattern was rejected, empty if accepted.
    *         A rejected pattern leaves the current one playing.
    */
  [[nodiscard]] std::optional<std::string> set_pattern(
    const LedPattern& pattern, double now_seconds);

  /// Colour to display at \p now_seconds
  [[nodiscard]] Rgbw compute(double now_seconds) const;

private:
  struct PlayingPattern
  {
    LedPattern pattern;
    double start_seconds = 0.0;
  };

  /// Reason a pattern can't be played, empty if valid
  [[nodiscard]] static std::optional<std::string> validate_(const LedPattern& pattern);

  /// Colour of \p playing at \p now_seconds
  [[nodiscard]] static Rgbw compute_(const PlayingPattern& playing, double now_seconds);

  PlayingPattern base_;
  std::optional<PlayingPattern> flash_;
};
}  // namespace explorer_leds

#endif
