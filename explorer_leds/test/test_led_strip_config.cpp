/*
 *  test_led_strip_config.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "explorer_leds/led_strip_config.h"

namespace explorer_leds
{
namespace
{
/// Writes the config file each test reads, removed after the test
class LedStripConfigTest : public ::testing::Test
{
protected:
  void TearDown() override { std::filesystem::remove(path_); }

  [[nodiscard]] std::string write_config(const std::string& content) const
  {
    std::ofstream(path_) << content;
    return path_.string();
  }

  std::filesystem::path path_ =
    std::filesystem::temp_directory_path() / ("led_strip_" + std::to_string(::getpid()) + ".yaml");
};

// Valid required values, each rejection test breaks only one of them
const std::string SPI_DEVICE = "spi_device: /dev/spidev3.0\n";
const std::string LED_COUNT = "led_count: 16\n";
const std::string LOST_COMMUNICATION =
  "lost_communication: {mode: solid, colors: [[1, 0, 0]], intensities: [1]}\n";
const std::string NOT_READY = "not_ready: {mode: solid, colors: [[1, 0.5, 0]], intensities: [1]}\n";
const std::string REQUIRED_CONFIG = SPI_DEVICE + LED_COUNT + LOST_COMMUNICATION + NOT_READY;
}  // namespace

// load_led_strip_config : a file that doesn't exist is rejected
TEST_F(LedStripConfigTest, MissingFileIsRejected)
{
  EXPECT_THROW((void)load_led_strip_config("/nonexistent/led_strip.yaml"), std::runtime_error);
}

// load_led_strip_config : a file that isn't valid YAML is rejected
TEST_F(LedStripConfigTest, InvalidYamlIsRejected)
{
  // Arrange
  const auto path = write_config("spi_device: [unclosed");

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a file that isn't key: value pairs is rejected
TEST_F(LedStripConfigTest, NonMapIsRejected)
{
  // Arrange
  const auto path = write_config("- /dev/spidev3.0\n- 16\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a missing spi_device is rejected
TEST_F(LedStripConfigTest, MissingSpiDeviceIsRejected)
{
  // Arrange
  const auto path = write_config(LED_COUNT + LOST_COMMUNICATION + NOT_READY);

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a missing led_count is rejected
TEST_F(LedStripConfigTest, MissingLedCountIsRejected)
{
  // Arrange
  const auto path = write_config(SPI_DEVICE + LOST_COMMUNICATION + NOT_READY);

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a led_count that isn't an integer is rejected
TEST_F(LedStripConfigTest, NonIntegerLedCountIsRejected)
{
  // Arrange
  const auto path =
    write_config(SPI_DEVICE + "led_count: sixteen\n" + LOST_COMMUNICATION + NOT_READY);

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a led_count of zero or less is rejected
TEST_F(LedStripConfigTest, NonPositiveLedCountIsRejected)
{
  // Arrange
  const auto path = write_config(SPI_DEVICE + "led_count: 0\n" + LOST_COMMUNICATION + NOT_READY);

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : optional values take their defaults when absent
TEST_F(LedStripConfigTest, OptionalValuesDefault)
{
  // Arrange
  const auto path = write_config(REQUIRED_CONFIG);

  // Act
  const auto config = load_led_strip_config(path);

  // Assert
  EXPECT_FALSE(config.rgbw);
  EXPECT_EQ(config.max_brightness, 1.0);
}

// load_led_strip_config : a max_brightness outside [0, 1] is rejected
TEST_F(LedStripConfigTest, BrightnessOutOfRangeIsRejected)
{
  // Arrange
  const auto path = write_config(REQUIRED_CONFIG + "max_brightness: 1.5\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a missing robot state pattern is rejected
TEST_F(LedStripConfigTest, MissingPatternIsRejected)
{
  // Arrange
  const auto path = write_config(SPI_DEVICE + LED_COUNT + LOST_COMMUNICATION);

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a robot state pattern that isn't a map of its fields is rejected
TEST_F(LedStripConfigTest, PatternNotMapIsRejected)
{
  // Arrange
  const auto path =
    write_config(SPI_DEVICE + LED_COUNT + LOST_COMMUNICATION + "not_ready: solid\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a robot state pattern with an unknown mode is rejected
TEST_F(LedStripConfigTest, PatternUnknownModeIsRejected)
{
  // Arrange
  const auto path = write_config(
    SPI_DEVICE + LED_COUNT + LOST_COMMUNICATION +
    "not_ready: {mode: rainbow, colors: [[1, 0, 0]], intensities: [1]}\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : flash_once is rejected as a robot state pattern, it wouldn't last
TEST_F(LedStripConfigTest, PatternFlashOnceIsRejected)
{
  // Arrange
  const auto path = write_config(
    SPI_DEVICE + LED_COUNT + LOST_COMMUNICATION +
    "not_ready: {mode: flash_once, speed: 1, colors: [[1, 0, 0]], intensities: [1]}\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a pattern colour without 3 or 4 components is rejected
TEST_F(LedStripConfigTest, PatternColorSizeIsRejected)
{
  // Arrange
  const auto path = write_config(
    SPI_DEVICE + LED_COUNT + LOST_COMMUNICATION +
    "not_ready: {mode: solid, colors: [[1, 0]], intensities: [1]}\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : a pattern a command couldn't play either is rejected
TEST_F(LedStripConfigTest, InvalidPatternIsRejected)
{
  // Arrange: intensity missing for the colour
  const auto path = write_config(
    SPI_DEVICE + LED_COUNT + "lost_communication: {mode: solid, colors: [[1, 0, 0]]}\n" +
    NOT_READY);

  // Act & Assert
  EXPECT_THROW((void)load_led_strip_config(path), std::runtime_error);
}

// load_led_strip_config : robot state patterns are read, with white off when not given
TEST_F(LedStripConfigTest, PatternIsRead)
{
  // Arrange
  const auto path = write_config(
    SPI_DEVICE + LED_COUNT +
    "lost_communication: {mode: blink, speed: 2, colors: [[1, 0, 0]], intensities: [0.8]}\n"
    "not_ready: {mode: solid, colors: [[0, 0, 0, 1]], intensities: [1]}\n");

  // Act
  const auto config = load_led_strip_config(path);

  // Assert
  EXPECT_EQ(config.lost_communication_pattern.mode, LedMode::BLINK);
  EXPECT_EQ(config.lost_communication_pattern.speed, 2.0);
  EXPECT_EQ(config.lost_communication_pattern.colors, (std::vector<Rgbw>{{1.0, 0.0, 0.0, 0.0}}));
  EXPECT_EQ(config.lost_communication_pattern.intensities, std::vector<double>{0.8});
  EXPECT_EQ(config.not_ready_pattern.colors, (std::vector<Rgbw>{{0.0, 0.0, 0.0, 1.0}}));
}

// load_led_strip_config : a pattern min_level is read, 0 (fully off) when not given
TEST_F(LedStripConfigTest, PatternMinLevelIsRead)
{
  // Arrange
  const auto path = write_config(
    SPI_DEVICE + LED_COUNT + LOST_COMMUNICATION +
    "not_ready: {mode: breathe, speed: 1, colors: [[1, 0, 0]], intensities: [1], min_level: 0.3}\n");

  // Act
  const auto config = load_led_strip_config(path);

  // Assert
  EXPECT_EQ(config.not_ready_pattern.min_level, 0.3);
  EXPECT_EQ(config.lost_communication_pattern.min_level, 0.0);
}

// load_led_strip_config : every value of a complete file is read
TEST_F(LedStripConfigTest, CompleteFileIsRead)
{
  // Arrange
  const auto path = write_config(
    "spi_device: /dev/spidev1.0\nled_count: 30\nrgbw: true\nmax_brightness: 0.4\n" +
    LOST_COMMUNICATION + NOT_READY);

  // Act
  const auto config = load_led_strip_config(path);

  // Assert
  EXPECT_EQ(config.spi_device, "/dev/spidev1.0");
  EXPECT_EQ(config.led_count, 30U);
  EXPECT_TRUE(config.rgbw);
  EXPECT_EQ(config.max_brightness, 0.4);
}
}  // namespace explorer_leds
