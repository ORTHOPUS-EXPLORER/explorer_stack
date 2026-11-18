/*
 *  radxa_led_driver.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_RADXA_LED_DRIVER_H
#define EXPLORER_LEDS_RADXA_LED_DRIVER_H

#include <cstdint>
#include <vector>

#include "explorer_leds/led_driver.h"
#include "explorer_leds/led_strip_config.h"

namespace explorer_leds
{
/// Drives the SK6812 LED strip wired to the Radxa Zero 3W SPI
class RadxaLedDriver : public LedDriver
{
public:
  explicit RadxaLedDriver(LedStripConfig config);
  ~RadxaLedDriver() override;

  RadxaLedDriver(const RadxaLedDriver&) = delete;
  RadxaLedDriver& operator=(const RadxaLedDriver&) = delete;
  RadxaLedDriver(RadxaLedDriver&&) = delete;
  RadxaLedDriver& operator=(RadxaLedDriver&&) = delete;

  /// Open the SPI device and configure it for the strip
  [[nodiscard]] std::optional<std::string> open() override;

  void close() override;

  /// \p frame is cut or padded with off LEDs to the strip LED count
  [[nodiscard]] bool write(const std::vector<Rgbw>& frame) override;

private:
  /// Send \p bytes on the SPI, false if closed or not fully written
  [[nodiscard]] bool write_bytes_(const std::vector<std::uint8_t>& bytes) const;

  LedStripConfig config_;
  // SPI device file descriptor, -1 when closed
  int fd_ = -1;
};
}  // namespace explorer_leds

#endif
