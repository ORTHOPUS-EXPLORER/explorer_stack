/*
 *  sk6812_encoder.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_SK6812_ENCODER_H
#define EXPLORER_LEDS_SK6812_ENCODER_H

#include <cstdint>
#include <vector>

#include "explorer_leds/led_animation.h"

namespace explorer_leds
{
/// SPI clock driving a SK6812 strip: 4 SPI bits per LED bit at the SK6812 800 kHz rate
constexpr std::uint32_t SK6812_SPI_SPEED_HZ = 3'200'000;

/// Zero bytes sent before the pixels: 300 us low (120 bytes at 3.2 MHz), a reset (>= 80 us) so the
/// first LED sees a clean frame start whatever the data line did before the transfer
constexpr std::size_t SK6812_RESET_BYTES = 120;

/// Zero bytes sent after the pixels: 150 us low (60 bytes at 3.2 MHz), a reset (>= 80 us) ending
/// the frame, so every LED displays the colour it just received
constexpr std::size_t SK6812_LATCH_BYTES = 60;

/**
  * \brief SPI bytes making a SK6812 strip show \p pixels, first pixel first.
  *
  * Each colour component is clamped to [0, 1], scaled by \p brightness, gamma corrected (2.2)
  * then sent as 8 LED bits, each one encoded as 4 SPI bits (1 -> 1100, 0 -> 1000).
  * Components are sent in the strip order: green, red, blue, then white when \p rgbw.
  * Without white LED (not \p rgbw), white is added to green, red and blue.
  */
[[nodiscard]] std::vector<std::uint8_t> encode_sk6812_frame(
  const std::vector<Rgbw>& pixels, bool rgbw, double brightness);
}  // namespace explorer_leds

#endif
