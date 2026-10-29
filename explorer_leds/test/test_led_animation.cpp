/*
 *  test_led_animation.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include <gtest/gtest.h>

#include <limits>
#include <ostream>

#include "explorer_leds/led_animation.h"

namespace explorer_leds
{
// Lets gtest print colours in failure messages
void PrintTo(const Rgbw& color, std::ostream* os)
{
  *os << "{" << color.r << ", " << color.g << ", " << color.b << ", " << color.w << "}";
}

namespace
{
const Rgbw RED{1.0, 0.0, 0.0};
const Rgbw GREEN{0.0, 1.0, 0.0};
const Rgbw BLUE{0.0, 0.0, 1.0};

constexpr double EPSILON = 1e-12;

LedPattern make_pattern(
  LedMode mode, double speed, std::vector<Rgbw> colors, std::vector<double> intensities)
{
  LedPattern pattern;
  pattern.mode = mode;
  pattern.speed = speed;
  pattern.colors = std::move(colors);
  pattern.intensities = std::move(intensities);
  return pattern;
}

void expect_rgb_near(const Rgbw& actual, const Rgbw& expected)
{
  EXPECT_NEAR(actual.r, expected.r, EPSILON);
  EXPECT_NEAR(actual.g, expected.g, EPSILON);
  EXPECT_NEAR(actual.b, expected.b, EPSILON);
}
}  // namespace

// mode_from_string : every documented mode name is recognised
TEST(ModeFromString, KnownNamesAreParsed)
{
  EXPECT_EQ(mode_from_string("off"), LedMode::OFF);
  EXPECT_EQ(mode_from_string("solid"), LedMode::SOLID);
  EXPECT_EQ(mode_from_string("fade_in"), LedMode::FADE_IN);
  EXPECT_EQ(mode_from_string("fade_out"), LedMode::FADE_OUT);
  EXPECT_EQ(mode_from_string("blink"), LedMode::BLINK);
  EXPECT_EQ(mode_from_string("alternate"), LedMode::ALTERNATE);
  EXPECT_EQ(mode_from_string("breathe"), LedMode::BREATHE);
  EXPECT_EQ(mode_from_string("flash_once"), LedMode::FLASH_ONCE);
}

// mode_from_string : an unknown name is rejected
TEST(ModeFromString, UnknownNameIsRejected)
{
  EXPECT_FALSE(mode_from_string("rainbow").has_value());
  EXPECT_FALSE(mode_from_string("Solid").has_value());
}

// capped_speed : repeating modes faster than 3 Hz are slowed down to 3 Hz
TEST(CappedSpeed, RepeatingModeAboveCapIsCapped)
{
  EXPECT_EQ(capped_speed(LedMode::BLINK, 10.0), 3.0);
  EXPECT_EQ(capped_speed(LedMode::ALTERNATE, 10.0), 3.0);
  EXPECT_EQ(capped_speed(LedMode::BREATHE, 10.0), 3.0);
}

// capped_speed : repeating modes at or below 3 Hz keep their speed
TEST(CappedSpeed, RepeatingModeBelowCapIsKept)
{
  EXPECT_EQ(capped_speed(LedMode::BLINK, 2.0), 2.0);
  EXPECT_EQ(capped_speed(LedMode::BLINK, 3.0), 3.0);
}

// capped_speed : modes playing once are not capped
TEST(CappedSpeed, OneShotModeIsNotCapped)
{
  EXPECT_EQ(capped_speed(LedMode::FADE_IN, 10.0), 10.0);
  EXPECT_EQ(capped_speed(LedMode::FADE_OUT, 10.0), 10.0);
  EXPECT_EQ(capped_speed(LedMode::FLASH_ONCE, 10.0), 10.0);
}

// set_pattern : off mode is accepted without any colour
TEST(SetPattern, OffIgnoresColors)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto error = animation.set_pattern(make_pattern(LedMode::OFF, 0.0, {RED}, {}), 0.0);

  // Assert
  EXPECT_FALSE(error.has_value());
}

// set_pattern : colors and intensities of different sizes are rejected
TEST(SetPattern, SizeMismatchIsRejected)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto error =
    animation.set_pattern(make_pattern(LedMode::ALTERNATE, 1.0, {RED, GREEN}, {1.0}), 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
}

// set_pattern : alternate with a single colour is rejected
TEST(SetPattern, AlternateWithOneColorIsRejected)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto error =
    animation.set_pattern(make_pattern(LedMode::ALTERNATE, 1.0, {RED}, {1.0}), 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
}

// set_pattern : a single colour mode given two colours is rejected
TEST(SetPattern, SingleColorModeWithTwoColorsIsRejected)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto error =
    animation.set_pattern(make_pattern(LedMode::BLINK, 1.0, {RED, GREEN}, {1.0, 1.0}), 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
}

// set_pattern : a single colour mode given no colour is rejected
TEST(SetPattern, SingleColorModeWithoutColorIsRejected)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto error = animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {}, {}), 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
}

// set_pattern : a colour component outside [0, 1] is rejected, white included
TEST(SetPattern, ColorOutOfRangeIsRejected)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto red_error =
    animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {{1.5, 0.0, 0.0}}, {1.0}), 0.0);
  const auto white_error =
    animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {{0.0, 0.0, 0.0, 1.5}}, {1.0}), 0.0);

  // Assert
  EXPECT_TRUE(red_error.has_value());
  EXPECT_TRUE(white_error.has_value());
}

// set_pattern : an intensity outside [0, 1] is rejected
TEST(SetPattern, IntensityOutOfRangeIsRejected)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto error = animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {RED}, {-0.1}), 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
}

// set_pattern : a min_level outside [0, 1] is rejected
TEST(SetPattern, MinLevelOutOfRangeIsRejected)
{
  // Arrange
  LedAnimation animation;
  auto pattern = make_pattern(LedMode::BREATHE, 1.0, {RED}, {1.0});
  pattern.min_level = 1.5;

  // Act
  const auto error = animation.set_pattern(pattern, 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
}

// set_pattern : an animated mode without a positive speed is rejected
TEST(SetPattern, AnimatedModeWithoutSpeedIsRejected)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto zero_error =
    animation.set_pattern(make_pattern(LedMode::BLINK, 0.0, {RED}, {1.0}), 0.0);
  const auto nan_error = animation.set_pattern(
    make_pattern(LedMode::BLINK, std::numeric_limits<double>::quiet_NaN(), {RED}, {1.0}), 0.0);

  // Assert
  EXPECT_TRUE(zero_error.has_value());
  EXPECT_TRUE(nan_error.has_value());
}

// set_pattern : solid mode is accepted without speed
TEST(SetPattern, SolidAcceptsZeroSpeed)
{
  // Arrange
  LedAnimation animation;

  // Act
  const auto error = animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {RED}, {1.0}), 0.0);

  // Assert
  EXPECT_FALSE(error.has_value());
}

// set_pattern : a rejected pattern keeps the previous one playing
TEST(SetPattern, RejectedPatternKeepsPrevious)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {RED}, {1.0}), 0.0));

  // Act
  const auto error = animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {GREEN}, {}), 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
  EXPECT_EQ(animation.compute(0.0), RED);
}

// set_pattern : a repeating mode faster than the cap plays at the capped speed
TEST(SetPattern, RepeatingSpeedIsCapped)
{
  // Arrange
  LedAnimation animation;

  // Act
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::BLINK, 10.0, {RED}, {1.0}), 0.0));

  // Assert: 0.2 s * 3 Hz = 0.6 cycle, second half so off. On if not capped: 0.2 s * 10 Hz = 2 cycles
  EXPECT_EQ(animation.compute(0.2), Rgbw{});
}

// set_pattern : the pattern already playing keeps going instead of restarting
TEST(SetPattern, SamePatternKeepsPlaying)
{
  // Arrange
  LedAnimation animation;
  const auto blink = make_pattern(LedMode::BLINK, 1.0, {RED}, {1.0});
  ASSERT_FALSE(animation.set_pattern(blink, 0.0));

  // Act
  ASSERT_FALSE(animation.set_pattern(blink, 0.75));

  // Assert: 0.75 s * 1 Hz = 0.75 cycle, second half so off. On if restarted at 0.75 s
  EXPECT_EQ(animation.compute(0.75), Rgbw{});
}

// set_pattern : flash_once shows its colour over the current pattern
TEST(SetPattern, FlashOnceShowsOverCurrentPattern)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {GREEN}, {1.0}), 0.0));

  // Act
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FLASH_ONCE, 2.0, {RED}, {1.0}), 1.0));

  // Assert: the flash lasts 1 / 2 Hz = 0.5 s, from 1.0 to 1.5 s
  EXPECT_EQ(animation.compute(1.25), RED);
}

// set_pattern : after a flash the current pattern resumes with its own timing
TEST(SetPattern, CurrentPatternResumesAfterFlash)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::BLINK, 1.0, {GREEN}, {1.0}), 0.0));

  // Act
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FLASH_ONCE, 2.0, {RED}, {1.0}), 1.0));

  // Assert: flash over at 1.5 s, blink at 1.75 s * 1 Hz = 1.75 cycles, second half so off.
  // On if the blink restarted when the flash ended
  EXPECT_EQ(animation.compute(1.75), Rgbw{});
}

// set_pattern : sending the pattern already playing during a flash doesn't cut the flash
TEST(SetPattern, SamePatternDuringFlashKeepsFlash)
{
  // Arrange
  LedAnimation animation;
  const auto solid_green = make_pattern(LedMode::SOLID, 0.0, {GREEN}, {1.0});
  ASSERT_FALSE(animation.set_pattern(solid_green, 0.0));
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FLASH_ONCE, 2.0, {RED}, {1.0}), 1.0));

  // Act
  ASSERT_FALSE(animation.set_pattern(solid_green, 1.1));

  // Assert
  EXPECT_EQ(animation.compute(1.25), RED);
}

// set_pattern : a new pattern during a flash cancels the flash
TEST(SetPattern, NewPatternCancelsFlash)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {GREEN}, {1.0}), 0.0));
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FLASH_ONCE, 2.0, {RED}, {1.0}), 1.0));

  // Act
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {BLUE}, {1.0}), 1.1));

  // Assert
  EXPECT_EQ(animation.compute(1.25), BLUE);
}

// compute : without any pattern the LEDs are off
TEST(Compute, DefaultIsOff)
{
  // Arrange
  const LedAnimation animation;

  // Act
  const auto color = animation.compute(1.0);

  // Assert
  EXPECT_EQ(color, Rgbw{});
}

// compute : off mode switches the LEDs off
TEST(Compute, OffIsBlack)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {RED}, {1.0}), 0.0));
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::OFF, 0.0, {}, {}), 0.0));

  // Act
  const auto color = animation.compute(1.0);

  // Assert
  EXPECT_EQ(color, Rgbw{});
}

// compute : solid mode shows its colour scaled by its intensity, whatever the time
TEST(Compute, SolidIsColorTimesIntensity)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(
    animation.set_pattern(make_pattern(LedMode::SOLID, 0.0, {{1.0, 0.5, 0.0, 1.0}}, {0.5}), 0.0));

  // Act
  const auto color = animation.compute(10.0);

  // Assert: {1.0, 0.5, 0.0, 1.0} * 0.5, white scaled like the other components
  EXPECT_EQ(color, (Rgbw{0.5, 0.25, 0.0, 0.5}));
}

// compute : fade in ramps linearly up to the colour over 1 / speed seconds
TEST(Compute, FadeInRampsUp)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FADE_IN, 2.0, {RED}, {1.0}), 0.0));

  // Act
  const auto color = animation.compute(0.125);

  // Assert: 0.125 s * 2 Hz = 0.25 of the fade done
  EXPECT_EQ(color, (Rgbw{0.25, 0.0, 0.0}));
}

// compute : fade in holds the colour once the fade is over
TEST(Compute, FadeInHoldsColorAfterFade)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FADE_IN, 2.0, {RED}, {0.5}), 0.0));

  // Act
  const auto color = animation.compute(3.0);

  // Assert: full colour times intensity, 3.0 if the progress weren't capped to 1
  EXPECT_EQ(color, (Rgbw{0.5, 0.0, 0.0}));
}

// compute : fade out ramps linearly down from the colour over 1 / speed seconds
TEST(Compute, FadeOutRampsDown)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FADE_OUT, 2.0, {RED}, {1.0}), 0.0));

  // Act
  const auto color = animation.compute(0.125);

  // Assert: 0.125 s * 2 Hz = 0.25 of the fade done, 1 - 0.25 left
  EXPECT_EQ(color, (Rgbw{0.75, 0.0, 0.0}));
}

// compute : fade out stays off once the fade is over
TEST(Compute, FadeOutStaysOffAfterFade)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FADE_OUT, 2.0, {RED}, {1.0}), 0.0));

  // Act
  const auto color = animation.compute(3.0);

  // Assert
  EXPECT_EQ(color, Rgbw{});
}

// compute : blink is on during the first half of each cycle
TEST(Compute, BlinkIsOnFirstHalfOfCycle)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::BLINK, 2.0, {GREEN}, {1.0}), 0.0));

  // Act: 1.1 s * 2 Hz = 2.2 cycles
  const auto color = animation.compute(1.1);

  // Assert
  EXPECT_EQ(color, GREEN);
}

// compute : blink is off during the second half of each cycle
TEST(Compute, BlinkIsOffSecondHalfOfCycle)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::BLINK, 2.0, {GREEN}, {1.0}), 0.0));

  // Act: 1.375 s * 2 Hz = 2.75 cycles
  const auto color = animation.compute(1.375);

  // Assert
  EXPECT_EQ(color, Rgbw{});
}

// compute : alternate shows each colour for 1 / speed seconds, in order, then loops
TEST(Compute, AlternateCyclesThroughColors)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(
    make_pattern(LedMode::ALTERNATE, 2.0, {RED, GREEN, BLUE}, {1.0, 0.5, 1.0}), 0.0));

  // Act & Assert: colour index is floor(t * 2 Hz) modulo 3
  EXPECT_EQ(animation.compute(0.25), RED);
  EXPECT_EQ(animation.compute(0.75), (Rgbw{0.0, 0.5, 0.0}));
  EXPECT_EQ(animation.compute(1.25), BLUE);
  EXPECT_EQ(animation.compute(1.75), RED);
}

// compute : breathe rises smoothly from off to the colour at half cycle, then falls back
TEST(Compute, BreatheRisesAndFalls)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::BREATHE, 1.0, {RED}, {1.0}), 0.0));

  // Act & Assert: brightness is (1 - cos(2 pi t * 1 Hz)) / 2
  expect_rgb_near(animation.compute(0.0), Rgbw{});
  expect_rgb_near(animation.compute(0.25), (Rgbw{0.5, 0.0, 0.0}));
  expect_rgb_near(animation.compute(0.5), RED);
  expect_rgb_near(animation.compute(0.75), (Rgbw{0.5, 0.0, 0.0}));
  expect_rgb_near(animation.compute(1.0), Rgbw{});
}

// compute : breathe rises from min_level instead of off, still reaching the full colour
TEST(Compute, BreatheRisesFromMinLevel)
{
  // Arrange
  LedAnimation animation;
  auto pattern = make_pattern(LedMode::BREATHE, 1.0, {RED}, {1.0});
  pattern.min_level = 0.2;
  ASSERT_FALSE(animation.set_pattern(pattern, 0.0));

  // Act & Assert: brightness is 0.2 + (1 - 0.2) * (1 - cos(2 pi t * 1 Hz)) / 2
  expect_rgb_near(animation.compute(0.0), (Rgbw{0.2, 0.0, 0.0}));
  expect_rgb_near(animation.compute(0.25), (Rgbw{0.6, 0.0, 0.0}));
  expect_rgb_near(animation.compute(0.5), RED);
}

// compute : breathe min_level is a fraction of the colour intensity
TEST(Compute, BreatheMinLevelFollowsIntensity)
{
  // Arrange
  LedAnimation animation;
  auto pattern = make_pattern(LedMode::BREATHE, 1.0, {RED}, {0.5});
  pattern.min_level = 0.2;
  ASSERT_FALSE(animation.set_pattern(pattern, 0.0));

  // Act
  const auto color = animation.compute(0.0);

  // Assert: 0.2 min_level * 0.5 intensity, 0.2 if min_level were absolute
  expect_rgb_near(color, (Rgbw{0.1, 0.0, 0.0}));
}

// compute : a time before the pattern started is the start of the animation
TEST(Compute, TimeBeforeStartIsStart)
{
  // Arrange
  LedAnimation animation;
  ASSERT_FALSE(animation.set_pattern(make_pattern(LedMode::FADE_OUT, 1.0, {RED}, {1.0}), 1.0));

  // Act
  const auto color = animation.compute(0.0);

  // Assert: start of the fade, 2.0 if the time weren't clamped
  EXPECT_EQ(color, RED);
}
}  // namespace explorer_leds
