/*
 *  test_sk6812_encoder.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "explorer_leds/sk6812_encoder.h"

namespace explorer_leds
{
namespace
{
using Bytes = std::vector<std::uint8_t>;

// SPI bytes of one LED component: 4 SPI bits per LED bit, 1 -> 1100, 0 -> 1000
const Bytes LEVEL_0{0x88, 0x88, 0x88, 0x88};
const Bytes LEVEL_255{0xCC, 0xCC, 0xCC, 0xCC};

constexpr std::size_t RGB_PIXEL_BYTES = 3 * 4;

/// The 4 bytes of component \p index (0 = green, 1 = red, 2 = blue) of the first pixel
Bytes first_pixel_component(const Bytes& frame, std::size_t index)
{
  const auto begin = frame.begin() + static_cast<std::ptrdiff_t>(SK6812_RESET_BYTES + 4 * index);
  return {begin, begin + 4};
}
}  // namespace

// encode_sk6812_frame : the frame starts with the reset bytes, all zero
TEST(EncodeSk6812Frame, StartsWithResetBytes)
{
  // Act
  const auto frame = encode_sk6812_frame({{1.0, 1.0, 1.0}}, false, 1.0);

  // Assert
  ASSERT_GE(frame.size(), SK6812_RESET_BYTES);
  EXPECT_EQ(Bytes(frame.begin(), frame.begin() + SK6812_RESET_BYTES), Bytes(SK6812_RESET_BYTES, 0));
}

// encode_sk6812_frame : components are sent green first, then red, then blue
TEST(EncodeSk6812Frame, ComponentsAreInGrbOrder)
{
  // Act
  const auto frame = encode_sk6812_frame({{1.0, 0.0, 0.0}}, false, 1.0);

  // Assert
  EXPECT_EQ(first_pixel_component(frame, 0), LEVEL_0);
  EXPECT_EQ(first_pixel_component(frame, 1), LEVEL_255);
  EXPECT_EQ(first_pixel_component(frame, 2), LEVEL_0);
}

// encode_sk6812_frame : pixels are sent in order, one after the other
TEST(EncodeSk6812Frame, PixelsAreInOrder)
{
  // Act
  const auto frame = encode_sk6812_frame({{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}, false, 1.0);

  // Assert: green of the second pixel, right after the 12 bytes of the first one
  const auto second_green = frame.begin() + SK6812_RESET_BYTES + RGB_PIXEL_BYTES;
  EXPECT_EQ(Bytes(second_green, second_green + 4), LEVEL_255);
}

// encode_sk6812_frame : component levels are gamma corrected
TEST(EncodeSk6812Frame, LevelIsGammaCorrected)
{
  // Act
  const auto frame = encode_sk6812_frame({{0.0, 0.5, 0.0}}, false, 1.0);

  // Assert: 0.5 * 255 = 127, 255 * (127 / 255)^2.2 = 55 = 0b00110111.
  // {0x8C, 0xCC, 0xCC, 0xCC} (127) if the gamma were lost
  EXPECT_EQ(first_pixel_component(frame, 0), (Bytes{0x88, 0xCC, 0x8C, 0xCC}));
}

// encode_sk6812_frame : component levels are scaled by the brightness
TEST(EncodeSk6812Frame, LevelIsScaledByBrightness)
{
  // Act
  const auto frame = encode_sk6812_frame({{1.0, 0.0, 0.0}}, false, 0.6);

  // Assert: 1.0 * 0.6 * 255 = 153, 255 * (153 / 255)^2.2 = 83 = 0b01010011
  EXPECT_EQ(first_pixel_component(frame, 1), (Bytes{0x8C, 0x8C, 0x88, 0xCC}));
}

// encode_sk6812_frame : a component above 1 is sent as 1
TEST(EncodeSk6812Frame, ComponentIsClamped)
{
  // Act
  const auto frame = encode_sk6812_frame({{1.5, 0.0, 0.0}}, false, 1.0);

  // Assert
  EXPECT_EQ(first_pixel_component(frame, 1), LEVEL_255);
}

// encode_sk6812_frame : an RGBW strip gets the white component after green, red and blue
TEST(EncodeSk6812Frame, RgbwSendsWhiteLast)
{
  // Act
  const auto frame = encode_sk6812_frame({{0.0, 0.0, 0.0, 1.0}}, true, 1.0);

  // Assert: reset + 4 components * 4 bytes + latch
  ASSERT_EQ(frame.size(), SK6812_RESET_BYTES + 16 + SK6812_LATCH_BYTES);
  EXPECT_EQ(first_pixel_component(frame, 2), LEVEL_0);
  EXPECT_EQ(first_pixel_component(frame, 3), LEVEL_255);
}

// encode_sk6812_frame : an RGB strip, without white LED, adds white to green, red and blue
TEST(EncodeSk6812Frame, RgbStripMakesWhiteWithColors)
{
  // Act
  const auto frame = encode_sk6812_frame({{0.0, 0.0, 0.0, 1.0}}, false, 1.0);

  // Assert: 3 components only, each 0 + 1.0 white. All LEVEL_0 if white were dropped
  ASSERT_EQ(frame.size(), SK6812_RESET_BYTES + RGB_PIXEL_BYTES + SK6812_LATCH_BYTES);
  EXPECT_EQ(first_pixel_component(frame, 0), LEVEL_255);
  EXPECT_EQ(first_pixel_component(frame, 1), LEVEL_255);
  EXPECT_EQ(first_pixel_component(frame, 2), LEVEL_255);
}

// encode_sk6812_frame : the frame ends with the latch bytes, all zero
TEST(EncodeSk6812Frame, EndsWithLatchBytes)
{
  // Act
  const auto frame = encode_sk6812_frame({{1.0, 1.0, 1.0}}, false, 1.0);

  // Assert: reset + 3 components * 4 bytes + latch
  ASSERT_EQ(frame.size(), SK6812_RESET_BYTES + RGB_PIXEL_BYTES + SK6812_LATCH_BYTES);
  EXPECT_EQ(Bytes(frame.end() - SK6812_LATCH_BYTES, frame.end()), Bytes(SK6812_LATCH_BYTES, 0));
}
}  // namespace explorer_leds
