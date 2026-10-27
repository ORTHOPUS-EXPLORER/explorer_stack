/*
 *  test_led_node.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "explorer_leds/led_node.h"
#include "explorer_leds/led_qos.h"
#include "explorer_msgs/msg/led_color.hpp"
#include "explorer_msgs/msg/led_command.hpp"
#include "explorer_msgs/msg/led_effect.hpp"
#include "explorer_msgs/msg/led_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace explorer_leds
{
namespace
{
using explorer_msgs::msg::LedCommand;
using explorer_msgs::msg::LedEffect;
using LedStateMsg = explorer_msgs::msg::LedState;
using Frame = std::vector<Rgbw>;
using namespace std::chrono_literals;

constexpr auto TIMEOUT = 5s;

const Rgbw OFF{};
const Rgbw RED{1.0, 0.0, 0.0};
const Rgbw GREEN{0.0, 1.0, 0.0};
const Rgbw BLUE{0.0, 0.0, 1.0};
// Robot state patterns of the test strip config
const Rgbw LOST_COLOR{1.0, 0.0, 1.0};
const Rgbw NOT_READY_COLOR{0.0, 1.0, 1.0};

constexpr std::size_t LED_COUNT = 4;

/// Records every frame written, shared with the test since the node owns the driver
class FakeLedDriver : public LedDriver
{
public:
  explicit FakeLedDriver(std::shared_ptr<std::vector<Frame>> frames) : frames_(std::move(frames)) {}

  [[nodiscard]] std::optional<std::string> open() override { return std::nullopt; }

  void close() override {}

  [[nodiscard]] bool write(const Frame& frame) override
  {
    frames_->push_back(frame);
    return true;
  }

private:
  std::shared_ptr<std::vector<Frame>> frames_;
};

LedEffect make_effect(
  const std::string& mode, double speed, const Rgbw& color, std::vector<uint16_t> leds = {})
{
  LedEffect effect;
  effect.leds = std::move(leds);
  effect.mode = mode;
  effect.speed = speed;
  explorer_msgs::msg::LedColor ros_color;
  ros_color.r = static_cast<float>(color.r);
  ros_color.g = static_cast<float>(color.g);
  ros_color.b = static_cast<float>(color.b);
  ros_color.w = static_cast<float>(color.w);
  effect.colors = {ros_color};
  effect.intensities = {1.0};
  return effect;
}

LedCommand make_command(const LedEffect& effect)
{
  LedCommand command;
  command.effects = {effect};
  return command;
}

/// Command playing \p color on \p leds (every LED if empty)
LedCommand make_solid(const Rgbw& color, std::vector<uint16_t> leds = {})
{
  return make_command(make_effect(LedEffect::MODE_SOLID, 0.0, color, std::move(leds)));
}

LedCommand make_flash(const Rgbw& color, double speed)
{
  return make_command(make_effect(LedEffect::MODE_FLASH_ONCE, speed, color));
}

Frame all_leds(const Rgbw& color) { return Frame(LED_COUNT, color); }

std_msgs::msg::Bool make_heartbeat(bool ready) { return std_msgs::msg::Bool().set__data(ready); }

class LedNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    write_strip_config(
      "spi_device: /dev/fake\nled_count: " + std::to_string(LED_COUNT) +
      "\nlost_communication: {mode: solid, colors: [[1, 0, 1]], intensities: [1]}"
      "\nnot_ready: {mode: solid, colors: [[0, 1, 1]], intensities: [1]}\n");

    publisher_node_ = std::make_shared<rclcpp::Node>("led_test_publisher");
    command_pub_ =
      publisher_node_->create_publisher<LedCommand>("/explorer_leds/command", led_command_qos());
    flash_pub_ = publisher_node_->create_publisher<LedCommand>("/explorer_leds/flash", 10);
    heartbeat_pub_ =
      publisher_node_->create_publisher<std_msgs::msg::Bool>("/command_node/ready", 10);
    state_sub_ = publisher_node_->create_subscription<LedStateMsg>(
      "/explorer_leds/state", rclcpp::QoS(1).transient_local(),
      [this](const LedStateMsg& state) { last_state_ = state; });
    executor_.add_node(publisher_node_);

    start_led_node();
  }

  void TearDown() override
  {
    stop_led_node();
    std::filesystem::remove(strip_config_path_);
  }

  void write_strip_config(const std::string& content) const
  {
    std::ofstream(strip_config_path_) << content;
  }

  void start_led_node()
  {
    node_ = std::make_shared<rclcpp::Node>(
      "led_node", rclcpp::NodeOptions().parameter_overrides(
                    {{"strip_config_file", strip_config_path_.string()},
                     {"watchdog_timeout_seconds", watchdog_timeout_seconds_}}));
    led_node_ = std::make_unique<LedNode>(
      node_, [this](const LedStripConfig&) { return std::make_unique<FakeLedDriver>(frames_); });
    executor_.add_node(node_);

    ASSERT_TRUE(spin_until(
      [this]()
      {
        return command_pub_->get_subscription_count() > 0 &&
               flash_pub_->get_subscription_count() > 0 &&
               heartbeat_pub_->get_subscription_count() > 0;
      }));
  }

  void stop_led_node()
  {
    if (node_)
    {
      executor_.remove_node(node_);
    }
    led_node_.reset();
    node_.reset();
  }

  /// Spin all nodes until \p condition holds, false on timeout
  bool spin_until(const std::function<bool()>& condition)
  {
    const auto deadline = std::chrono::steady_clock::now() + TIMEOUT;
    while (!condition())
    {
      if (std::chrono::steady_clock::now() > deadline)
      {
        return false;
      }
      executor_.spin_some(10ms);
    }
    return true;
  }

  bool spin_until_written(const Frame& frame)
  {
    return spin_until([this, frame]() { return !frames_->empty() && frames_->back() == frame; });
  }

  bool spin_until_state(uint8_t state)
  {
    return spin_until([this, state]() { return last_state_ && last_state_->state == state; });
  }

  /// Whether \p color was ever written on any LED
  [[nodiscard]] bool was_written(const Rgbw& color) const
  {
    return std::any_of(
      frames_->begin(), frames_->end(), [&color](const Frame& frame)
      { return std::find(frame.begin(), frame.end(), color) != frame.end(); });
  }

  // Disabled unless a test is about it, so it can't interfere
  double watchdog_timeout_seconds_ = 0.0;
  std::filesystem::path strip_config_path_ =
    std::filesystem::temp_directory_path() /
    ("led_node_strip_" + std::to_string(::getpid()) + ".yaml");
  std::shared_ptr<std::vector<Frame>> frames_ = std::make_shared<std::vector<Frame>>();
  std::optional<LedStateMsg> last_state_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<LedCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<LedCommand>::SharedPtr flash_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<LedStateMsg>::SharedPtr state_sub_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<LedNode> led_node_;
};

class LedNodeWatchdogTest : public LedNodeTest
{
protected:
  LedNodeWatchdogTest() { watchdog_timeout_seconds_ = 0.2; }
};
}  // namespace

// constructor : with a valid strip config and hardware opening, the state topic reports READY
TEST_F(LedNodeTest, ReadyStateIsPublished) { EXPECT_TRUE(spin_until_state(LedStateMsg::READY)); }

// constructor : an invalid strip config is reported as ERROR on the state topic
TEST_F(LedNodeTest, InvalidConfigPublishesError)
{
  // Arrange
  stop_led_node();
  write_strip_config("spi_device: /dev/fake\n");

  // Act
  start_led_node();

  // Assert
  ASSERT_TRUE(spin_until_state(LedStateMsg::ERROR));
  EXPECT_FALSE(last_state_->message.empty());
}

// on_command : an effect lights only the LEDs it targets
TEST_F(LedNodeTest, EffectLightsOnlyTargetedLeds)
{
  // Act
  command_pub_->publish(make_solid(RED, {0, 1}));

  // Assert
  EXPECT_TRUE(spin_until_written(Frame{RED, RED, OFF, OFF}));
}

// on_command : the white component of a colour reaches the LEDs
TEST_F(LedNodeTest, WhiteReachesLeds)
{
  // Arrange
  const Rgbw white{0.0, 0.0, 0.0, 1.0};

  // Act
  command_pub_->publish(make_solid(white));

  // Assert
  EXPECT_TRUE(spin_until_written(all_leds(white)));
}

// on_command : a command published before the LED node started is still displayed (latched)
TEST_F(LedNodeTest, CommandPublishedBeforeStartIsDisplayed)
{
  // Arrange
  stop_led_node();
  command_pub_->publish(make_solid(BLUE));
  frames_->clear();

  // Act
  start_led_node();

  // Assert
  EXPECT_TRUE(spin_until_written(all_leds(BLUE)));
}

// on_flash : a flash shows over the current command, which then comes back
TEST_F(LedNodeTest, FlashShowsOverCommand)
{
  // Arrange
  command_pub_->publish(make_solid(GREEN));
  ASSERT_TRUE(spin_until_written(all_leds(GREEN)));

  // Act: 1 / 2 Hz = 0.5 s flash
  flash_pub_->publish(make_flash(RED, 2.0));

  // Assert
  ASSERT_TRUE(spin_until_written(all_leds(RED)));
  EXPECT_TRUE(spin_until_written(all_leds(GREEN)));
}

// on_heartbeat : a not ready robot plays the strip not ready pattern instead of the command
TEST_F(LedNodeTest, NotReadyShowsNotReadyColor)
{
  // Arrange
  command_pub_->publish(make_solid(BLUE));
  ASSERT_TRUE(spin_until_written(all_leds(BLUE)));

  // Act
  heartbeat_pub_->publish(make_heartbeat(false));

  // Assert
  EXPECT_TRUE(spin_until_written(all_leds(NOT_READY_COLOR)));
}

// on_heartbeat : once the robot is ready again the command is displayed again
TEST_F(LedNodeTest, ReadyAgainShowsCommand)
{
  // Arrange
  command_pub_->publish(make_solid(BLUE));
  heartbeat_pub_->publish(make_heartbeat(false));
  ASSERT_TRUE(spin_until_written(all_leds(NOT_READY_COLOR)));

  // Act
  heartbeat_pub_->publish(make_heartbeat(true));

  // Assert
  EXPECT_TRUE(spin_until_written(all_leds(BLUE)));
}

// on_heartbeat : a heartbeat after communication was lost shows the command again
TEST_F(LedNodeWatchdogTest, HeartbeatAfterLostCommunicationShowsCommand)
{
  // Arrange
  command_pub_->publish(make_solid(BLUE));
  ASSERT_TRUE(spin_until_written(all_leds(LOST_COLOR)));

  // Act
  heartbeat_pub_->publish(make_heartbeat(true));

  // Assert
  EXPECT_TRUE(spin_until_written(all_leds(BLUE)));
}

// to_effects : an unknown mode is ignored and never reaches the LEDs
TEST_F(LedNodeTest, UnknownModeIsIgnored)
{
  // Arrange
  auto unknown = make_solid(GREEN);
  unknown.effects[0].mode = "rainbow";

  // Act: blue is sent after it to know the unknown mode was processed
  command_pub_->publish(unknown);
  command_pub_->publish(make_solid(BLUE));

  // Assert
  ASSERT_TRUE(spin_until_written(all_leds(BLUE)));
  EXPECT_FALSE(was_written(GREEN));
}

// to_effects : a mode other than flash_once on the flash topic is ignored
TEST_F(LedNodeTest, NonFlashOnFlashTopicIsIgnored)
{
  // Act: a blue flash is sent after it to know the solid green was processed
  flash_pub_->publish(make_solid(GREEN));
  flash_pub_->publish(make_flash(BLUE, 1.0));

  // Assert
  ASSERT_TRUE(spin_until_written(all_leds(BLUE)));
  EXPECT_FALSE(was_written(GREEN));
}

// to_effects : flash_once on the command topic is ignored, it belongs to the flash topic
TEST_F(LedNodeTest, FlashOnCommandTopicIsIgnored)
{
  // Act: blue is sent after it to know the flash was processed
  command_pub_->publish(make_flash(GREEN, 1.0));
  command_pub_->publish(make_solid(BLUE));

  // Assert
  ASSERT_TRUE(spin_until_written(all_leds(BLUE)));
  EXPECT_FALSE(was_written(GREEN));
}

// play : a command rejected by the controller never reaches the LEDs
TEST_F(LedNodeTest, RejectedCommandIsIgnored)
{
  // Act: LED 4 doesn't exist (0 to 3), blue is sent after it to know green was processed
  command_pub_->publish(make_solid(GREEN, {0, 4}));
  command_pub_->publish(make_solid(BLUE));

  // Assert
  ASSERT_TRUE(spin_until_written(all_leds(BLUE)));
  EXPECT_FALSE(was_written(GREEN));
}

// update : without heartbeat for the watchdog timeout, the LEDs play the lost communication pattern
TEST_F(LedNodeWatchdogTest, NoHeartbeatShowsLostCommunication)
{
  // Arrange
  command_pub_->publish(make_solid(BLUE));
  heartbeat_pub_->publish(make_heartbeat(true));
  ASSERT_TRUE(spin_until_written(all_leds(BLUE)));

  // Act & Assert: no heartbeat for longer than the 0.2 s timeout
  EXPECT_TRUE(spin_until_written(all_leds(LOST_COLOR)));
}

// destructor : the LEDs are switched off when the node stops
TEST_F(LedNodeTest, StoppingSwitchesLedsOff)
{
  // Arrange
  command_pub_->publish(make_solid(BLUE));
  ASSERT_TRUE(spin_until_written(all_leds(BLUE)));

  // Act
  stop_led_node();

  // Assert
  EXPECT_EQ(frames_->back(), all_leds(OFF));
}
}  // namespace explorer_leds
