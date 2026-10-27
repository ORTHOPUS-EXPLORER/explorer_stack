/*
 *  simulated_led_driver.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_SIMULATED_LED_DRIVER_H
#define EXPLORER_LEDS_SIMULATED_LED_DRIVER_H

#include "explorer_leds/led_driver.h"

namespace explorer_leds
{
/// Driver without hardware, for simulation: always opens, accepts every frame and sends it nowhere
class SimulatedLedDriver : public LedDriver
{
public:
  [[nodiscard]] std::optional<std::string> open() override { return std::nullopt; }

  void close() override {}

  [[nodiscard]] bool write(const std::vector<Rgbw>& /*frame*/) override { return true; }
};
}  // namespace explorer_leds

#endif
