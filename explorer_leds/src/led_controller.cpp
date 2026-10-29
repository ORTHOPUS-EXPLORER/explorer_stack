/*
 *  led_controller.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include "explorer_leds/led_controller.h"

#include <utility>

namespace explorer_leds
{
LedController::LedController(
  LedStripConfig strip, std::unique_ptr<LedDriver> driver, LedControllerOptions options)
: strip_(std::move(strip)),
  driver_(std::move(driver)),
  options_(options),
  animations_(strip_.led_count)
{
}

LedController::~LedController()
{
  // Don't leave the LEDs showing the last frame
  if (state_ == LedState::READY)
  {
    (void)driver_->write(std::vector<Rgbw>(strip_.led_count));
  }
  driver_->close();
}

std::optional<std::string> LedController::apply(
  const std::vector<LedEffect>& effects, double now_seconds)
{
  if (effects.empty())
  {
    return "no effect";
  }

  // Applied on a copy, kept only if every effect is valid
  std::vector<LedAnimation> animations = animations_;
  for (std::size_t index = 0; index < effects.size(); ++index)
  {
    const LedEffect& effect = effects[index];
    const std::string prefix = "effect " + std::to_string(index) + ": ";

    std::vector<std::size_t> leds = effect.leds;
    if (leds.empty())
    {
      for (std::size_t led = 0; led < strip_.led_count; ++led)
      {
        leds.push_back(led);
      }
    }

    for (const std::size_t led : leds)
    {
      if (led >= strip_.led_count)
      {
        return prefix + "LED " + std::to_string(led) + " out of range 0-" +
               std::to_string(strip_.led_count - 1);
      }
      const auto error = animations[led].set_pattern(effect.pattern, now_seconds);
      if (error)
      {
        return prefix + *error;
      }
    }
  }

  animations_ = std::move(animations);
  return std::nullopt;
}

void LedController::set_robot_state(RobotState robot_state, double now_seconds)
{
  if (robot_state == robot_state_)
  {
    return;
  }
  robot_state_ = robot_state;

  // A fresh animation, so a pattern played again starts from its beginning
  robot_state_animation_ = LedAnimation();
  const LedPattern* pattern = nullptr;
  switch (robot_state)
  {
    case RobotState::COMMUNICATION_LOST:
      pattern = &strip_.lost_communication_pattern;
      break;
    case RobotState::NOT_READY:
      pattern = &strip_.not_ready_pattern;
      break;
    case RobotState::READY:
      return;
  }
  // Validated with the strip config
  (void)robot_state_animation_.set_pattern(*pattern, now_seconds);
}

void LedController::update(double now_seconds)
{
  if (state_ != LedState::READY)
  {
    open_hardware_();
  }

  if (state_ == LedState::READY)
  {
    write_frame_(frame_(now_seconds));
  }
}

LedState LedController::state() const { return state_; }

const std::string& LedController::error() const { return error_; }

void LedController::open_hardware_()
{
  const auto error = driver_->open();
  if (!error)
  {
    state_ = LedState::READY;
    error_.clear();
    write_failures_ = 0;
    last_frame_.reset();
    return;
  }

  state_ = LedState::ERROR;
  error_ = *error;
}

std::vector<Rgbw> LedController::frame_(double now_seconds) const
{
  if (robot_state_ != RobotState::READY)
  {
    return std::vector<Rgbw>(strip_.led_count, robot_state_animation_.compute(now_seconds));
  }

  std::vector<Rgbw> frame;
  frame.reserve(animations_.size());
  for (const auto& animation : animations_)
  {
    frame.push_back(animation.compute(now_seconds));
  }
  return frame;
}

void LedController::write_frame_(const std::vector<Rgbw>& frame)
{
  if (last_frame_ == frame)
  {
    return;
  }

  if (driver_->write(frame))
  {
    last_frame_ = frame;
    write_failures_ = 0;
    return;
  }

  // Not remembered, so the next update tries again
  write_failures_ += 1;
  if (write_failures_ >= options_.max_write_failures)
  {
    driver_->close();
    state_ = LedState::ERROR;
    error_ = "LED strip write failed " + std::to_string(write_failures_) + " times in a row";
  }
}
}  // namespace explorer_leds
