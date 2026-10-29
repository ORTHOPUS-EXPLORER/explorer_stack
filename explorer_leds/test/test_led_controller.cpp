/*
 *  test_led_controller.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include <gtest/gtest.h>

#include <deque>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "explorer_leds/led_controller.h"

namespace explorer_leds
{
// Lets gtest print colours in failure messages
void PrintTo(const Rgbw& color, std::ostream* os)
{
  *os << "{" << color.r << ", " << color.g << ", " << color.b << ", " << color.w << "}";
}

namespace
{
const Rgbw OFF{};
const Rgbw RED{1.0, 0.0, 0.0};
const Rgbw GREEN{0.0, 1.0, 0.0};
const Rgbw BLUE{0.0, 0.0, 1.0};
// Robot state patterns of the test strip
const Rgbw LOST_COLOR{1.0, 0.0, 1.0};
const Rgbw NOT_READY_COLOR{0.0, 1.0, 1.0};

constexpr std::size_t LED_COUNT = 4;

/// What the fake driver does and what it received, shared with the test since the controller
/// owns the driver
struct FakeHardware
{
  // Result of the next open() calls, success once empty
  std::deque<std::optional<std::string>> open_results;
  bool write_succeeds = true;

  int open_calls = 0;
  bool closed = false;
  std::vector<std::vector<Rgbw>> frames;
};

class FakeLedDriver : public LedDriver
{
public:
  explicit FakeLedDriver(std::shared_ptr<FakeHardware> hardware) : hardware_(std::move(hardware)) {}

  [[nodiscard]] std::optional<std::string> open() override
  {
    ++hardware_->open_calls;
    hardware_->closed = false;
    if (hardware_->open_results.empty())
    {
      return std::nullopt;
    }
    auto result = hardware_->open_results.front();
    hardware_->open_results.pop_front();
    return result;
  }

  void close() override { hardware_->closed = true; }

  [[nodiscard]] bool write(const std::vector<Rgbw>& frame) override
  {
    if (!hardware_->write_succeeds)
    {
      return false;
    }
    hardware_->frames.push_back(frame);
    return true;
  }

private:
  std::shared_ptr<FakeHardware> hardware_;
};

LedPattern solid(const Rgbw& color)
{
  LedPattern pattern;
  pattern.mode = LedMode::SOLID;
  pattern.colors = {color};
  pattern.intensities = {1.0};
  return pattern;
}

LedEffect effect(std::vector<std::size_t> leds, const LedPattern& pattern)
{
  return LedEffect{std::move(leds), pattern};
}

class LedControllerTest : public ::testing::Test
{
protected:
  LedControllerTest()
  {
    strip_.spi_device = "/dev/fake";
    strip_.led_count = LED_COUNT;
    strip_.lost_communication_pattern = solid(LOST_COLOR);
    strip_.not_ready_pattern = solid(NOT_READY_COLOR);
  }

  [[nodiscard]] std::unique_ptr<LedController> make_controller() const
  {
    LedControllerOptions options;
    options.max_write_failures = 3;
    return std::make_unique<LedController>(
      strip_, std::make_unique<FakeLedDriver>(hardware_), options);
  }

  /// Controller already READY at t = 0, with its first frame written
  [[nodiscard]] std::unique_ptr<LedController> make_ready_controller() const
  {
    auto controller = make_controller();
    controller->update(0.0);
    return controller;
  }

  [[nodiscard]] const std::vector<Rgbw>& last_frame() const { return hardware_->frames.back(); }

  LedStripConfig strip_;
  std::shared_ptr<FakeHardware> hardware_ = std::make_shared<FakeHardware>();
};
}  // namespace

// constructor : the controller starts in INIT, before trying to open the hardware
TEST_F(LedControllerTest, StartsInInit)
{
  // Act
  const auto controller = make_controller();

  // Assert
  EXPECT_EQ(controller->state(), LedState::INIT);
  EXPECT_EQ(hardware_->open_calls, 0);
}

// destructor : a READY controller switches every LED off and closes the hardware
TEST_F(LedControllerTest, DestructorSwitchesLedsOff)
{
  // Arrange
  auto controller = make_ready_controller();
  ASSERT_FALSE(controller->apply({effect({}, solid(RED))}, 0.0));
  controller->update(0.0);

  // Act
  controller.reset();

  // Assert
  EXPECT_EQ(last_frame(), std::vector<Rgbw>(LED_COUNT, OFF));
  EXPECT_TRUE(hardware_->closed);
}

// apply : a command without effect is rejected
TEST_F(LedControllerTest, NoEffectIsRejected)
{
  // Arrange
  auto controller = make_ready_controller();

  // Act
  const auto error = controller->apply({}, 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
}

// apply : a LED index outside the strip is rejected
TEST_F(LedControllerTest, LedOutOfRangeIsRejected)
{
  // Arrange
  auto controller = make_ready_controller();

  // Act: indices go from 0 to 3
  const auto error = controller->apply({effect({LED_COUNT}, solid(RED))}, 0.0);

  // Assert
  EXPECT_TRUE(error.has_value());
}

// apply : an invalid effect rejects the whole command, valid effects before it included
TEST_F(LedControllerTest, InvalidEffectRejectsWholeCommand)
{
  // Arrange
  auto controller = make_ready_controller();
  LedPattern invalid = solid(GREEN);
  invalid.intensities = {};

  // Act
  const auto error = controller->apply({effect({0}, solid(RED)), effect({1}, invalid)}, 0.0);
  controller->update(0.0);

  // Assert: LED 0 would be red if the valid effect had been applied
  EXPECT_TRUE(error.has_value());
  EXPECT_EQ(last_frame(), std::vector<Rgbw>(LED_COUNT, OFF));
}

// apply : an effect without LED indices applies to every LED
TEST_F(LedControllerTest, NoLedIndexMeansEveryLed)
{
  // Arrange
  auto controller = make_ready_controller();

  // Act
  ASSERT_FALSE(controller->apply({effect({}, solid(RED))}, 0.0));
  controller->update(0.0);

  // Assert
  EXPECT_EQ(last_frame(), std::vector<Rgbw>(LED_COUNT, RED));
}

// apply : each LED plays the effect targeting it
TEST_F(LedControllerTest, EachLedPlaysItsOwnEffect)
{
  // Arrange
  auto controller = make_ready_controller();

  // Act
  ASSERT_FALSE(controller->apply({effect({0, 1}, solid(RED)), effect({3}, solid(BLUE))}, 0.0));
  controller->update(0.0);

  // Assert
  EXPECT_EQ(last_frame(), (std::vector<Rgbw>{RED, RED, OFF, BLUE}));
}

// apply : a later effect overrides an earlier one on the LEDs they share
TEST_F(LedControllerTest, LaterEffectOverridesSharedLeds)
{
  // Arrange
  auto controller = make_ready_controller();

  // Act
  ASSERT_FALSE(
    controller->apply({effect({0, 1, 2}, solid(RED)), effect({2, 3}, solid(BLUE))}, 0.0));
  controller->update(0.0);

  // Assert: LED 2 is in both, the second effect wins
  EXPECT_EQ(last_frame(), (std::vector<Rgbw>{RED, RED, BLUE, BLUE}));
}

// apply : LEDs a command doesn't target keep their current effect
TEST_F(LedControllerTest, UntargetedLedsKeepTheirEffect)
{
  // Arrange
  auto controller = make_ready_controller();
  ASSERT_FALSE(controller->apply({effect({}, solid(RED))}, 0.0));

  // Act
  ASSERT_FALSE(controller->apply({effect({0}, solid(GREEN))}, 0.0));
  controller->update(0.0);

  // Assert
  EXPECT_EQ(last_frame(), (std::vector<Rgbw>{GREEN, RED, RED, RED}));
}

// apply : effects received before the hardware is open play once it is
TEST_F(LedControllerTest, EffectsBeforeReadyPlayOnceReady)
{
  // Arrange
  auto controller = make_controller();
  hardware_->open_results = {std::string("not plugged")};
  controller->update(0.0);
  ASSERT_EQ(controller->state(), LedState::ERROR);

  // Act: the next update opens and writes
  ASSERT_FALSE(controller->apply({effect({}, solid(BLUE))}, 0.01));
  controller->update(0.02);
  controller->update(0.04);

  // Assert
  ASSERT_EQ(controller->state(), LedState::READY);
  EXPECT_EQ(last_frame(), std::vector<Rgbw>(LED_COUNT, BLUE));
}

// set_robot_state : lost communication plays the strip lost communication pattern on every LED
TEST_F(LedControllerTest, CommunicationLostOverridesEveryLed)
{
  // Arrange
  auto controller = make_ready_controller();
  ASSERT_FALSE(controller->apply({effect({0}, solid(BLUE))}, 0.0));

  // Act
  controller->set_robot_state(RobotState::COMMUNICATION_LOST, 0.0);
  controller->update(0.0);

  // Assert
  EXPECT_EQ(last_frame(), std::vector<Rgbw>(LED_COUNT, LOST_COLOR));
}

// set_robot_state : a not ready robot plays the strip not ready pattern on every LED
TEST_F(LedControllerTest, NotReadyOverridesEveryLed)
{
  // Arrange
  auto controller = make_ready_controller();
  ASSERT_FALSE(controller->apply({effect({0}, solid(BLUE))}, 0.0));

  // Act
  controller->set_robot_state(RobotState::NOT_READY, 0.0);
  controller->update(0.0);

  // Assert
  EXPECT_EQ(last_frame(), std::vector<Rgbw>(LED_COUNT, NOT_READY_COLOR));
}

// set_robot_state : a robot state pattern starts when the robot state changes
TEST_F(LedControllerTest, RobotStatePatternStartsOnChange)
{
  // Arrange: lost communication blinks at 1 Hz
  LedPattern blink = solid(LOST_COLOR);
  blink.mode = LedMode::BLINK;
  blink.speed = 1.0;
  strip_.lost_communication_pattern = blink;
  auto controller = make_ready_controller();

  // Act
  controller->set_robot_state(RobotState::COMMUNICATION_LOST, 10.5);
  controller->update(10.75);

  // Assert: 0.25 cycle since the change, first half so on. Off if timed from 0: 10.75 cycles
  EXPECT_EQ(last_frame(), std::vector<Rgbw>(LED_COUNT, LOST_COLOR));
}

// set_robot_state : back to READY, every LED plays its effect again
TEST_F(LedControllerTest, ReadyAgainPlaysEffects)
{
  // Arrange
  auto controller = make_ready_controller();
  ASSERT_FALSE(controller->apply({effect({}, solid(BLUE))}, 0.0));
  controller->set_robot_state(RobotState::NOT_READY, 0.0);
  controller->update(0.0);

  // Act
  controller->set_robot_state(RobotState::READY, 0.02);
  controller->update(0.02);

  // Assert
  EXPECT_EQ(last_frame(), std::vector<Rgbw>(LED_COUNT, BLUE));
}

// update : the hardware opening turns INIT into READY
TEST_F(LedControllerTest, OpeningMakesReady)
{
  // Arrange
  auto controller = make_controller();

  // Act
  controller->update(0.0);

  // Assert
  EXPECT_EQ(controller->state(), LedState::READY);
  EXPECT_TRUE(controller->error().empty());
}

// update : a failed opening turns INIT into ERROR and reports the driver error
TEST_F(LedControllerTest, FailedOpeningMakesError)
{
  // Arrange
  auto controller = make_controller();
  hardware_->open_results = {std::string("not plugged")};

  // Act
  controller->update(0.0);

  // Assert
  EXPECT_EQ(controller->state(), LedState::ERROR);
  EXPECT_EQ(controller->error(), "not plugged");
}

// update : an ERROR controller retries opening at the next update and becomes READY on success
TEST_F(LedControllerTest, ErrorRetriesOpeningAtNextUpdate)
{
  // Arrange
  auto controller = make_controller();
  hardware_->open_results = {std::string("not plugged")};
  controller->update(0.0);
  ASSERT_EQ(controller->state(), LedState::ERROR);

  // Act: opening succeeds once the scripted failure is used
  controller->update(0.02);

  // Assert
  EXPECT_EQ(hardware_->open_calls, 2);
  EXPECT_EQ(controller->state(), LedState::READY);
  EXPECT_TRUE(controller->error().empty());
}

// update : a frame identical to the last one written isn't written again
TEST_F(LedControllerTest, UnchangedFrameNotRewritten)
{
  // Arrange
  auto controller = make_ready_controller();
  controller->update(0.0);
  const auto writes = hardware_->frames.size();

  // Act
  controller->update(0.5);

  // Assert
  EXPECT_EQ(hardware_->frames.size(), writes);
}

// update : a single write failure keeps the controller READY
TEST_F(LedControllerTest, SingleWriteFailureKeepsReady)
{
  // Arrange
  auto controller = make_controller();
  hardware_->write_succeeds = false;

  // Act: opens, then fails to write the first frame
  controller->update(0.0);

  // Assert
  EXPECT_EQ(controller->state(), LedState::READY);
}

// update : consecutive write failures turn READY into ERROR and close the hardware
TEST_F(LedControllerTest, RepeatedWriteFailuresMakeError)
{
  // Arrange
  auto controller = make_controller();
  hardware_->write_succeeds = false;

  // Act: 3 failed writes, the first frame is retried at each update since never written
  controller->update(0.0);
  controller->update(0.02);
  controller->update(0.04);

  // Assert
  EXPECT_EQ(controller->state(), LedState::ERROR);
  EXPECT_FALSE(controller->error().empty());
  EXPECT_TRUE(hardware_->closed);
}
}  // namespace explorer_leds
