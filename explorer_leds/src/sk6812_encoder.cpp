/*
 *  sk6812_encoder.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include "explorer_leds/sk6812_encoder.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace explorer_leds
{
namespace
{
constexpr double GAMMA = 2.2;
// SPI bits encoding one LED bit
constexpr std::uint32_t SPI_BITS_ONE = 0b1100;
constexpr std::uint32_t SPI_BITS_ZERO = 0b1000;

/// Perceived brightness correction: LED output level for each linear 8 bit level
const std::array<std::uint8_t, 256>& gamma_table()
{
  static const std::array<std::uint8_t, 256> TABLE = []()
  {
    std::array<std::uint8_t, 256> table{};
    for (std::size_t level = 0; level < table.size(); ++level)
    {
      table[level] = static_cast<std::uint8_t>(
        std::lround(255.0 * std::pow(static_cast<double>(level) / 255.0, GAMMA)));
    }
    return table;
  }();
  return TABLE;
}

/// Append the 4 SPI bytes encoding the 8 LED bits of \p level, most significant bit first
void append_level(std::uint8_t level, std::vector<std::uint8_t>& out)
{
  std::uint32_t bits = 0;
  for (int bit = 7; bit >= 0; --bit)
  {
    const bool is_one = ((level >> bit) & 1U) != 0;
    bits = (bits << 4) | (is_one ? SPI_BITS_ONE : SPI_BITS_ZERO);
  }
  for (int byte = 3; byte >= 0; --byte)
  {
    out.push_back(static_cast<std::uint8_t>(bits >> (8 * byte)));
  }
}

/// Append one colour component, \p value in [0, 1] (clamped)
void append_component(double value, double brightness, std::vector<std::uint8_t>& out)
{
  const double scaled = std::clamp(value, 0.0, 1.0) * brightness * 255.0;
  // Truncated, as the LED level is the integer part of the scaled value
  const auto level = static_cast<std::size_t>(std::clamp(scaled, 0.0, 255.0));
  append_level(gamma_table()[level], out);
}
}  // namespace

std::vector<std::uint8_t> encode_sk6812_frame(
  const std::vector<Rgbw>& pixels, bool rgbw, double brightness)
{
  const std::size_t components = rgbw ? 4 : 3;
  std::vector<std::uint8_t> out(SK6812_RESET_BYTES, 0);
  out.reserve(SK6812_RESET_BYTES + pixels.size() * components * 4 + SK6812_LATCH_BYTES);

  for (const auto& pixel : pixels)
  {
    if (rgbw)
    {
      append_component(pixel.g, brightness, out);
      append_component(pixel.r, brightness, out);
      append_component(pixel.b, brightness, out);
      append_component(pixel.w, brightness, out);
    }
    else
    {
      // No white LED: white is made with the colour LEDs, clamped once added
      append_component(pixel.g + pixel.w, brightness, out);
      append_component(pixel.r + pixel.w, brightness, out);
      append_component(pixel.b + pixel.w, brightness, out);
    }
  }

  out.insert(out.end(), SK6812_LATCH_BYTES, 0);
  return out;
}
}  // namespace explorer_leds
