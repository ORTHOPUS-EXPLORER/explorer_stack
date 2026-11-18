/*
 *  led_driver.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_LED_DRIVER_H
#define EXPLORER_LEDS_LED_DRIVER_H

#include <optional>
#include <string>
#include <vector>

#include "explorer_leds/led_animation.h"

namespace explorer_leds
{
/// Sends colours to the LED hardware. Abstract so the controller can be tested with a fake one.
class LedDriver
{
public:
  virtual ~LedDriver() = default;

  /**
    * \brief Open the hardware, can be called again after a failure or a close().
    * \return why it could not be opened, empty on success
    */
  [[nodiscard]] virtual std::optional<std::string> open() = 0;

  /// Release the hardware opened by open()
  virtual void close() = 0;

  /**
    * \brief Display \p frame on the LEDs, one colour per LED, first LED first.
    * \return false if the hardware could not be written
    */
  [[nodiscard]] virtual bool write(const std::vector<Rgbw>& frame) = 0;
};
}  // namespace explorer_leds

#endif
