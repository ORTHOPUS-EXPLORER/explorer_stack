/*
 *  led_strip_config.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_LED_STRIP_CONFIG_H
#define EXPLORER_LEDS_LED_STRIP_CONFIG_H

#include <cstddef>
#include <string>

#include "explorer_leds/led_animation.h"

namespace explorer_leds
{
/// LED strip wiring and hardware, one config file per strip (see config/led_strip.yaml)
struct LedStripConfig
{
  // SPI device the strip data line is wired to
  std::string spi_device;
  std::size_t led_count = 0;
  // RGBW strip, with a white LED. Without it, white is made with the red, green and blue LEDs
  bool rgbw = false;
  // Brightness cap in [0, 1] applied to every colour, to stay within the power supply budget
  double max_brightness = 1.0;
  // Played on every LED instead of the commanded effects while communication with the robot is lost
  LedPattern lost_communication_pattern;
  // Played on every LED instead of the commanded effects while the robot isn't ready
  LedPattern not_ready_pattern;
};

/**
  * \brief Read a LED strip config file.
  *
  * spi_device, led_count and the lost_communication and not_ready patterns are required,
  * rgbw (false) and max_brightness (1.0) are optional.
  * \throw std::runtime_error if the file can't be read, or a value is missing or invalid
  */
[[nodiscard]] LedStripConfig load_led_strip_config(const std::string& path);
}  // namespace explorer_leds

#endif
