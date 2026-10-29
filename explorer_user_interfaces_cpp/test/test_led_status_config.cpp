#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "explorer_user_interfaces_cpp/led_status_config.h"

namespace space_control
{
namespace
{
using explorer_msgs::msg::LedColor;
using explorer_msgs::msg::LedCommand;
using explorer_msgs::msg::LedEffect;

/// Writes the config file each test reads, removed after the test
class LedStatusConfigTest : public ::testing::Test
{
protected:
  void TearDown() override { std::filesystem::remove(path_); }

  [[nodiscard]] std::string write_config(const std::string& content) const
  {
    std::ofstream(path_) << content;
    return path_.string();
  }

  std::filesystem::path path_ =
    std::filesystem::temp_directory_path() / ("led_status_" + std::to_string(::getpid()) + ".yaml");
};

/// Config with \p effect as the only effect of mode B1
std::string mode_b1_config(const std::string& effect)
{
  return "modes:\n  B1:\n    - " + effect + "\n";
}

/// Command of a single solid effect of colour (r, 0, 0), on every LED
LedCommand solid_red(float r)
{
  LedEffect effect;
  effect.mode = LedEffect::MODE_SOLID;
  effect.colors = {LedColor().set__r(r)};
  effect.intensities = {1.0};
  LedCommand command;
  command.effects = {effect};
  return command;
}

/// Config with a different command for each retract status and for modes B1 and B2
LedStatusConfig status_config()
{
  LedStatusConfig config;
  config.retract_status_commands[RetractStatus::RETRACTED] = solid_red(0.1f);
  config.retract_status_commands[RetractStatus::IN_PROGRESS] = solid_red(0.2f);
  config.mode_commands["B1"] = solid_red(0.3f);
  config.mode_commands["B2"] = solid_red(0.4f);
  return config;
}

/// \p command with its effects restricted to \p leds
LedCommand on_leds(LedCommand command, const std::vector<std::uint16_t>& leds)
{
  for (auto& effect : command.effects)
  {
    effect.leds = leds;
  }
  return command;
}

/// \p first effects followed by \p second ones
LedCommand concat(LedCommand first, const LedCommand& second)
{
  first.effects.insert(first.effects.end(), second.effects.begin(), second.effects.end());
  return first;
}
}  // namespace

// load_led_status_config : a file that doesn't exist is rejected
TEST_F(LedStatusConfigTest, MissingFileIsRejected)
{
  EXPECT_THROW((void)load_led_status_config("/nonexistent/led_status.yaml"), std::runtime_error);
}

// load_led_status_config : a file that isn't valid YAML is rejected
TEST_F(LedStatusConfigTest, InvalidYamlIsRejected)
{
  // Arrange
  const auto path = write_config("modes: [unclosed");

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : a file that isn't key: value pairs is rejected
TEST_F(LedStatusConfigTest, NonMapIsRejected)
{
  // Arrange
  const auto path = write_config("- B1\n- B2\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : a section that isn't status: effects pairs is rejected
TEST_F(LedStatusConfigTest, NonMapSectionIsRejected)
{
  // Arrange
  const auto path = write_config("modes: [B1, B2]\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : a status that isn't a list of effects is rejected
TEST_F(LedStatusConfigTest, NonListStatusIsRejected)
{
  // Arrange
  const auto path = write_config("modes:\n  B1: {mode: solid, colors: [[1, 0, 0]]}\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : a status without effect is rejected, explorer_leds would ignore it
TEST_F(LedStatusConfigTest, EmptyStatusIsRejected)
{
  // Arrange
  const auto path = write_config("modes:\n  B1: []\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : an effect that isn't key: value pairs is rejected
TEST_F(LedStatusConfigTest, NonMapEffectIsRejected)
{
  // Arrange
  const auto path = write_config(mode_b1_config("solid"));

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : a negative LED index is rejected
TEST_F(LedStatusConfigTest, NegativeLedIndexIsRejected)
{
  // Arrange
  const auto path = write_config(mode_b1_config("{leds: [-1], mode: off}"));

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : a LED index beyond what LedEffect can hold is rejected
TEST_F(LedStatusConfigTest, TooLargeLedIndexIsRejected)
{
  // Arrange: uint16 max + 1
  const auto path = write_config(mode_b1_config("{leds: [65536], mode: off}"));

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : an effect without mode is rejected
TEST_F(LedStatusConfigTest, MissingModeIsRejected)
{
  // Arrange
  const auto path = write_config(mode_b1_config("{colors: [[1, 0, 0]], intensities: [1]}"));

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : an unknown mode is rejected
TEST_F(LedStatusConfigTest, UnknownModeIsRejected)
{
  // Arrange
  const auto path = write_config(mode_b1_config("{mode: sparkle}"));

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : flash_once is rejected, explorer_leds only accepts it on the flash topic
TEST_F(LedStatusConfigTest, FlashOnceIsRejected)
{
  // Arrange
  const auto path =
    write_config(mode_b1_config("{mode: flash_once, speed: 2, colors: [[1, 0, 0]]}"));

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : a colour that isn't [r, g, b] or [r, g, b, w] is rejected
TEST_F(LedStatusConfigTest, InvalidColourSizeIsRejected)
{
  // Arrange
  const auto path = write_config(mode_b1_config("{mode: solid, colors: [[1, 0]]}"));

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : an unknown retract status is rejected
TEST_F(LedStatusConfigTest, UnknownRetractStatusIsRejected)
{
  // Arrange: typo of RETRACTED
  const auto path = write_config("retract_status:\n  RETRACED:\n    - {mode: off}\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : READY is rejected as a retract status, the mode is shown then
TEST_F(LedStatusConfigTest, ReadyRetractStatusIsRejected)
{
  // Arrange
  const auto path = write_config("retract_status:\n  READY:\n    - {mode: off}\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : retract status commands are read by status
TEST_F(LedStatusConfigTest, RetractStatusCommandsAreRead)
{
  // Arrange
  const auto path = write_config(
    "retract_status:\n"
    "  RETRACTED:\n    - {mode: solid, colors: [[0.1, 0, 0]], intensities: [1]}\n"
    "  IN_PROGRESS:\n    - {mode: solid, colors: [[0.2, 0, 0]], intensities: [1]}\n");

  // Act
  const auto config = load_led_status_config(path);

  // Assert
  ASSERT_EQ(config.retract_status_commands.size(), 2u);
  EXPECT_EQ(config.retract_status_commands.at(RetractStatus::RETRACTED), solid_red(0.1f));
  EXPECT_EQ(config.retract_status_commands.at(RetractStatus::IN_PROGRESS), solid_red(0.2f));
}

// load_led_status_config : mode commands are read by mode name
TEST_F(LedStatusConfigTest, ModeCommandsAreRead)
{
  // Arrange
  const auto path = write_config(
    "modes:\n"
    "  B1:\n    - {mode: solid, colors: [[0.3, 0, 0]], intensities: [1]}\n"
    "  B2:\n    - {mode: solid, colors: [[0.4, 0, 0]], intensities: [1]}\n");

  // Act
  const auto config = load_led_status_config(path);

  // Assert
  ASSERT_EQ(config.mode_commands.size(), 2u);
  EXPECT_EQ(config.mode_commands.at("B1"), solid_red(0.3f));
  EXPECT_EQ(config.mode_commands.at("B2"), solid_red(0.4f));
}

// load_led_status_config : every field of an effect is read, effects kept in order
TEST_F(LedStatusConfigTest, EffectFieldsAreRead)
{
  // Arrange
  const auto path = write_config(
    "modes:\n"
    "  B1:\n"
    "    - {mode: off}\n"
    "    - {leds: [0, 3], mode: alternate, speed: 2, colors: [[1, 0, 0], [0, 1, 0, 0.5]], "
    "intensities: [0.5, 1], min_level: 0.2}\n");

  // Act
  const auto config = load_led_status_config(path);

  // Assert
  const auto& effects = config.mode_commands.at("B1").effects;
  ASSERT_EQ(effects.size(), 2u);
  EXPECT_EQ(effects[0].mode, LedEffect::MODE_OFF);
  EXPECT_EQ(effects[1].leds, (std::vector<std::uint16_t>{0, 3}));
  EXPECT_EQ(effects[1].mode, LedEffect::MODE_ALTERNATE);
  EXPECT_DOUBLE_EQ(effects[1].speed, 2.0);
  EXPECT_EQ(
    effects[1].colors,
    (std::vector<LedColor>{LedColor().set__r(1.0f), LedColor().set__g(1.0f).set__w(0.5f)}));
  EXPECT_EQ(effects[1].intensities, (std::vector<double>{0.5, 1.0}));
  EXPECT_DOUBLE_EQ(effects[1].min_level, 0.2);
}

// load_led_status_config : optional effect fields left out are empty, an effect without leds
// applying to every LED
TEST_F(LedStatusConfigTest, MissingEffectFieldsAreEmpty)
{
  // Arrange
  const auto path = write_config(mode_b1_config("{mode: off}"));

  // Act
  const auto config = load_led_status_config(path);

  // Assert
  const auto& effect = config.mode_commands.at("B1").effects.at(0);
  EXPECT_TRUE(effect.leds.empty());
  EXPECT_DOUBLE_EQ(effect.speed, 0.0);
  EXPECT_TRUE(effect.colors.empty());
  EXPECT_TRUE(effect.intensities.empty());
  EXPECT_DOUBLE_EQ(effect.min_level, 0.0);
}

// load_led_status_config : missing sections give no command, nothing is sent for those statuses
TEST_F(LedStatusConfigTest, MissingSectionsAreEmpty)
{
  // Arrange
  const auto path = write_config("{}\n");

  // Act
  const auto config = load_led_status_config(path);

  // Assert
  EXPECT_TRUE(config.retract_status_commands.empty());
  EXPECT_TRUE(config.mode_commands.empty());
  EXPECT_TRUE(config.centered_leds.empty());
}

// load_led_status_config : a negative centered LED index is rejected
TEST_F(LedStatusConfigTest, NegativeCenteredLedIsRejected)
{
  // Arrange
  const auto path = write_config("centered_leds: [1, -2]\n");

  // Act & Assert
  EXPECT_THROW((void)load_led_status_config(path), std::runtime_error);
}

// load_led_status_config : the config file shipped with the package is valid
TEST_F(LedStatusConfigTest, ShippedConfigIsValid)
{
  EXPECT_NO_THROW((void)load_led_status_config(SHIPPED_LED_STATUS_CONFIG));
}

// select_led_command : while READY, nothing is selected until a mode is received
TEST(SelectLedCommand, ReadyWithoutModeSelectsNothing)
{
  EXPECT_EQ(select_led_command(status_config(), RetractStatus::READY, std::nullopt), std::nullopt);
}

// select_led_command : while READY, a mode without command selects nothing
TEST(SelectLedCommand, ReadyWithUnknownModeSelectsNothing)
{
  EXPECT_EQ(select_led_command(status_config(), RetractStatus::READY, "BOIRE"), std::nullopt);
}

// select_led_command : while READY, the command of the current mode is selected on every LED,
// centered LEDs included
TEST(SelectLedCommand, ReadySelectsModeCommand)
{
  // Arrange
  auto config = status_config();
  config.centered_leds = {1, 2};

  // Act & Assert
  EXPECT_EQ(select_led_command(config, RetractStatus::READY, "B2"), solid_red(0.4f));
}

// select_led_command : while RETRACTED without centered LEDs, the RETRACTED command is selected
// whatever the mode
TEST(SelectLedCommand, RetractedSelectsItsCommand)
{
  EXPECT_EQ(select_led_command(status_config(), RetractStatus::RETRACTED, "B1"), solid_red(0.1f));
}

// select_led_command : while IN_PROGRESS without centered LEDs, the IN_PROGRESS command is selected
// whatever the mode
TEST(SelectLedCommand, InProgressSelectsItsCommand)
{
  EXPECT_EQ(select_led_command(status_config(), RetractStatus::IN_PROGRESS, "B1"), solid_red(0.2f));
}

// select_led_command : a retract status without command and without centered LEDs selects nothing,
// not the mode command
TEST(SelectLedCommand, RetractStatusWithoutCommandSelectsNothing)
{
  // Arrange
  auto config = status_config();
  config.retract_status_commands.erase(RetractStatus::RETRACTED);

  // Act & Assert
  EXPECT_EQ(select_led_command(config, RetractStatus::RETRACTED, "B1"), std::nullopt);
}

// select_led_command : while RETRACTED, the mode colour shows solid on the centered LEDs, after the
// retract status so it wins there
TEST(SelectLedCommand, ModeShowsSolidOnCenteredLedsWhileRetracted)
{
  // Arrange: B1 breathes red 0.3
  auto config = status_config();
  config.centered_leds = {1, 2};
  auto breathe = solid_red(0.3f);
  breathe.effects[0].mode = LedEffect::MODE_BREATHE;
  breathe.effects[0].speed = 0.5;
  breathe.effects[0].min_level = 0.4;
  config.mode_commands["B1"] = breathe;

  // Act
  const auto command = select_led_command(config, RetractStatus::RETRACTED, "B1");

  // Assert: solid red 0.3, without the breathe speed and min_level
  EXPECT_EQ(command, concat(solid_red(0.1f), on_leds(solid_red(0.3f), {1, 2})));
}

// select_led_command : while RETRACTED, a mode effect with several colours shows its first one
TEST(SelectLedCommand, AlternateModeShowsFirstColour)
{
  // Arrange
  auto config = status_config();
  config.centered_leds = {1};
  auto alternate = solid_red(0.3f);
  alternate.effects[0].mode = LedEffect::MODE_ALTERNATE;
  alternate.effects[0].colors.push_back(LedColor().set__g(1.0f));
  alternate.effects[0].intensities.push_back(1.0);
  config.mode_commands["B1"] = alternate;

  // Act
  const auto command = select_led_command(config, RetractStatus::RETRACTED, "B1");

  // Assert: solid takes a single colour, explorer_leds would reject the command otherwise
  EXPECT_EQ(command, concat(solid_red(0.1f), on_leds(solid_red(0.3f), {1})));
}

// select_led_command : while RETRACTED, an off mode effect stays off on the centered LEDs
TEST(SelectLedCommand, OffModeStaysOffOnCenteredLeds)
{
  // Arrange
  auto config = status_config();
  config.centered_leds = {1};
  LedCommand off;
  off.effects.resize(1);
  off.effects[0].mode = LedEffect::MODE_OFF;
  config.mode_commands["B1"] = off;

  // Act
  const auto command = select_led_command(config, RetractStatus::RETRACTED, "B1");

  // Assert
  EXPECT_EQ(command, concat(solid_red(0.1f), on_leds(off, {1})));
}

// select_led_command : while IN_PROGRESS, the centered LEDs show the retract status like the
// others, so every LED switches to the mode at the same time once READY
TEST(SelectLedCommand, InProgressShowsRetractStatusOnCenteredLeds)
{
  // Arrange
  auto config = status_config();
  config.centered_leds = {1, 2};

  // Act & Assert
  EXPECT_EQ(select_led_command(config, RetractStatus::IN_PROGRESS, "B1"), solid_red(0.2f));
}

// select_led_command : while RETRACTED, a mode effect on some LEDs only keeps those that are
// centered LEDs
TEST(SelectLedCommand, ModeEffectKeepsItsCenteredLeds)
{
  // Arrange: LED 3 isn't a centered LED
  auto config = status_config();
  config.centered_leds = {1, 2};
  config.mode_commands["B1"] = on_leds(solid_red(0.3f), {2, 3});

  // Act
  const auto command = select_led_command(config, RetractStatus::RETRACTED, "B1");

  // Assert
  EXPECT_EQ(command, concat(solid_red(0.1f), on_leds(solid_red(0.3f), {2})));
}

// select_led_command : while RETRACTED, a mode effect on no centered LED is left out
TEST(SelectLedCommand, ModeEffectOutsideCenteredLedsIsLeftOut)
{
  // Arrange
  auto config = status_config();
  config.centered_leds = {1, 2};
  config.mode_commands["B1"] = on_leds(solid_red(0.3f), {5});

  // Act & Assert: an effect with no LED would be sent for every LED
  EXPECT_EQ(select_led_command(config, RetractStatus::RETRACTED, "B1"), solid_red(0.1f));
}

// select_led_command : while RETRACTED, the centered LEDs show the mode even when the retract
// status has no command
TEST(SelectLedCommand, ModeShowsOnCenteredLedsWithoutRetractCommand)
{
  // Arrange
  auto config = status_config();
  config.centered_leds = {1, 2};
  config.retract_status_commands.erase(RetractStatus::RETRACTED);

  // Act & Assert
  EXPECT_EQ(
    select_led_command(config, RetractStatus::RETRACTED, "B1"), on_leds(solid_red(0.3f), {1, 2}));
}

// select_led_command : while RETRACTED, without a mode received the retract status shows on every
// LED
TEST(SelectLedCommand, NoModeShowsRetractStatusOnCenteredLeds)
{
  // Arrange
  auto config = status_config();
  config.centered_leds = {1, 2};

  // Act & Assert
  EXPECT_EQ(select_led_command(config, RetractStatus::RETRACTED, std::nullopt), solid_red(0.1f));
}

}  // namespace space_control
