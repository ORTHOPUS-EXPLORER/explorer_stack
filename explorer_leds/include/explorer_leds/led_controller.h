/*
 *  led_controller.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_LED_CONTROLLER_H
#define EXPLORER_LEDS_LED_CONTROLLER_H

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "explorer_leds/led_animation.h"
#include "explorer_leds/led_driver.h"
#include "explorer_leds/led_strip_config.h"

namespace explorer_leds
{
/// Effect requested on some LEDs, see explorer_msgs/msg/LedEffect
struct LedEffect
{
  // Indices of the LEDs it applies to, empty for every LED
  std::vector<std::size_t> leds;
  LedPattern pattern;
};

enum class LedState
{
  // Hardware not opened yet, until the first update
  INIT,
  // Hardware open, LEDs show the commanded effects
  READY,
  // Hardware that can't be opened or written, opening is retried at every update
  ERROR
};

/// State of the robot the LEDs report, overriding the commanded effects when not ready
enum class RobotState
{
  READY,
  NOT_READY,
  COMMUNICATION_LOST
};

struct LedControllerOptions
{
  // Consecutive write failures turning READY into ERROR
  int max_write_failures = 3;
};

/**
  * \brief Plays an effect on each LED of the strip and manages the hardware state.
  *
  * Each LED plays its own LedAnimation. Commands are accepted in any state, so the ones received
  * before the hardware is open play once it is. Times are in seconds on any monotonic clock, as
  * long as the same one is used for every call. Independent of ROS.
  */
class LedController
{
public:
  /**
    * \param strip strip the driver controls, with the patterns shown when the robot isn't ready
    * \param driver hardware the frames are sent to, opened by update()
    * \param options state machine settings
    */
  LedController(
    LedStripConfig strip, std::unique_ptr<LedDriver> driver, LedControllerOptions options);
  ~LedController();

  LedController(const LedController&) = delete;
  LedController& operator=(const LedController&) = delete;
  LedController(LedController&&) = delete;
  LedController& operator=(LedController&&) = delete;

  /**
    * \brief Play \p effects from \p now_seconds, in order, a later one overriding an earlier one on
    * the LEDs they share. LEDs no effect targets keep their current effect.
    * \return the reason the effects were rejected, empty if accepted.
    *         Rejected as a whole if any effect is invalid.
    */
  [[nodiscard]] std::optional<std::string> apply(
    const std::vector<LedEffect>& effects, double now_seconds);

  /**
    * \brief Every LED plays the strip pattern of \p robot_state instead of its effect, unless READY.
    * A pattern restarts from \p now_seconds each time the robot state changes.
    */
  void set_robot_state(RobotState robot_state, double now_seconds);

  /// Open the hardware when needed, then send the frame to display at \p now_seconds when READY
  void update(double now_seconds);

  [[nodiscard]] LedState state() const;

  /// Why the controller is in ERROR, empty otherwise
  [[nodiscard]] const std::string& error() const;

private:
  /// Try to open the hardware: READY on success, ERROR otherwise
  void open_hardware_();

  /// Colour of each LED at \p now_seconds
  [[nodiscard]] std::vector<Rgbw> frame_(double now_seconds) const;

  /// Send \p frame to the driver when it changed, ERROR after too many failures
  void write_frame_(const std::vector<Rgbw>& frame);

  LedStripConfig strip_;
  std::unique_ptr<LedDriver> driver_;
  LedControllerOptions options_;

  // One animation per LED
  std::vector<LedAnimation> animations_;
  RobotState robot_state_ = RobotState::READY;
  // Played on every LED while the robot isn't READY
  LedAnimation robot_state_animation_;

  LedState state_ = LedState::INIT;
  std::string error_;
  int write_failures_ = 0;
  // Last frame written, empty before the first write and after a reopening
  std::optional<std::vector<Rgbw>> last_frame_;
};
}  // namespace explorer_leds

#endif
