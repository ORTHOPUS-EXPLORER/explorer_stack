#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include "explorer_command_controllers/position_saturation.hpp"

using explorer_command_controllers::PositionSaturation;
using explorer_command_controllers::PositionSaturationConfig;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kEpsilon = 1e-12;

PositionSaturationConfig make_config(bool enable, double threshold, bool persistent)
{
  PositionSaturationConfig config;
  config.enable = enable;
  config.threshold = threshold;
  config.persistent = persistent;
  return config;
}

// Run one cycle when only its effect on the following cycles matters, not the command
void run_cycle(
  PositionSaturation& saturation, std::size_t joint, double reference,
  std::optional<double> measured)
{
  [[maybe_unused]] const double command = saturation.compute(joint, reference, measured);
}
}  // namespace

// constructor : a negative threshold is refused when creating the saturation
TEST(PositionSaturation, ConstructorRejectsNegativeThreshold)
{
  // Arrange
  const auto config = make_config(true, -0.1, false);

  // Act & Assert
  EXPECT_THROW(PositionSaturation(1, config), std::invalid_argument);
}

// set_config : a negative threshold is refused when changing the configuration
TEST(PositionSaturation, SetConfigRejectsNegativeThreshold)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));

  // Act & Assert
  EXPECT_THROW(saturation.set_config(make_config(true, -1.0, false)), std::invalid_argument);
}

// set_config : after a refused configuration, the previous threshold is still applied
TEST(PositionSaturation, RejectedConfigKeepsPreviousThreshold)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));
  EXPECT_THROW(saturation.set_config(make_config(true, -1.0, false)), std::invalid_argument);

  // Act
  const double command = saturation.compute(0, 0.5, 0.0);

  // Assert: 0.0 measured + 0.2 previous threshold
  EXPECT_NEAR(command, 0.2, kEpsilon);
}

// reset : after a reset, the reference is sent without the earlier persistent shift
TEST(PositionSaturation, ResetRemovesPersistentShift)
{
  // Arrange: reference 0.5 clamped to 0.2 (shift of -0.3)
  PositionSaturation saturation(1, make_config(true, 0.2, true));
  run_cycle(saturation, 0, 0.5, 0.0);

  // Act
  saturation.reset();
  const double command = saturation.compute(0, 0.1, 0.0);

  // Assert: 0.1 is within 0.0 +/- 0.2 (0.1 - 0.3 = -0.2 if the shift were still applied)
  EXPECT_DOUBLE_EQ(command, 0.1);
}

// compute : without a valid reference, no command is sent (NaN)
TEST(PositionSaturation, NaNReferenceGivesNaNCommand)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));

  // Act
  const double command = saturation.compute(0, kNaN, 0.0);

  // Assert
  EXPECT_TRUE(std::isnan(command));
}

// compute : an infinite reference is treated as missing (NaN command)
TEST(PositionSaturation, InfiniteReferenceGivesNaNCommand)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));

  // Act
  const double command = saturation.compute(0, std::numeric_limits<double>::infinity(), 0.0);

  // Assert
  EXPECT_TRUE(std::isnan(command));
}

// compute : a missing reference for one cycle does not lose the persistent shift
TEST(PositionSaturation, NaNReferenceKeepsPersistentShift)
{
  // Arrange: reference 0.5 clamped to 0.2, then one cycle without reference
  PositionSaturation saturation(1, make_config(true, 0.2, true));
  run_cycle(saturation, 0, 0.5, 0.0);
  run_cycle(saturation, 0, kNaN, 0.0);

  // Act: same reference, robot measured at the previous command
  const double command = saturation.compute(0, 0.5, 0.2);

  // Assert: 0.5 - 0.3 shift (0.4 if the shift were lost: 0.5 clamped to 0.2 + 0.2)
  EXPECT_NEAR(command, 0.2, kEpsilon);
}

// compute : when saturation is disabled, a far command is sent unchanged
TEST(PositionSaturation, DisabledDoesNotClamp)
{
  // Arrange
  PositionSaturation saturation(1, make_config(false, 0.2, false));

  // Act
  const double command = saturation.compute(0, 0.5, 0.0);

  // Assert
  EXPECT_DOUBLE_EQ(command, 0.5);
}

// compute : disabling saturation keeps the shift caused by an earlier persistent clamping
TEST(PositionSaturation, PersistentShiftIsKeptWhenDisabled)
{
  // Arrange: reference 0.5 clamped to 0.2, then saturation disabled
  PositionSaturation saturation(1, make_config(true, 0.2, true));
  run_cycle(saturation, 0, 0.5, 0.0);
  saturation.set_config(make_config(false, 0.2, true));

  // Act
  const double command = saturation.compute(0, 0.5, 0.0);

  // Assert: 0.5 - 0.3 shift, not clamped since disabled
  EXPECT_NEAR(command, 0.2, kEpsilon);
}

// compute : without a measured position, the command is sent unchanged
TEST(PositionSaturation, MissingMeasureDoesNotClamp)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));

  // Act
  const double command = saturation.compute(0, 0.5, std::nullopt);

  // Assert
  EXPECT_DOUBLE_EQ(command, 0.5);
}

// compute : a cycle without measured position does not create a persistent shift
TEST(PositionSaturation, MissingMeasureDoesNotShiftLaterCommands)
{
  // Arrange: far reference while the measure is missing
  PositionSaturation saturation(1, make_config(true, 0.2, true));
  run_cycle(saturation, 0, 0.5, std::nullopt);

  // Act
  const double command = saturation.compute(0, 0.1, 0.0);

  // Assert: 0.1 is within 0.0 +/- 0.2 and no shift was stored
  EXPECT_DOUBLE_EQ(command, 0.1);
}

// compute : a NaN measured position is treated as missing: the command is sent unchanged
TEST(PositionSaturation, NaNMeasureDoesNotClamp)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));

  // Act
  const double command = saturation.compute(0, 0.5, kNaN);

  // Assert
  EXPECT_DOUBLE_EQ(command, 0.5);
}

// compute : a command close enough to the measured position is sent unchanged
TEST(PositionSaturation, PassesThroughCommandWithinThreshold)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));

  // Act
  const double command = saturation.compute(0, 0.15, 0.0);

  // Assert: 0.15 is within 0.0 +/- 0.2
  EXPECT_DOUBLE_EQ(command, 0.15);
}

// compute : a command too far above the measured position is clamped to measured + threshold
TEST(PositionSaturation, ClampsCommandAboveUpperBound)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));

  // Act
  const double command = saturation.compute(0, 0.5, 0.1);

  // Assert: 0.1 measured + 0.2 threshold
  EXPECT_NEAR(command, 0.3, kEpsilon);
}

// compute : a command too far below the measured position is clamped to measured - threshold
TEST(PositionSaturation, ClampsCommandBelowLowerBound)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.2, false));

  // Act
  const double command = saturation.compute(0, -0.5, 0.1);

  // Assert: 0.1 measured - 0.2 threshold
  EXPECT_NEAR(command, -0.1, kEpsilon);
}

// compute : with a zero threshold, the robot is held at its measured position
TEST(PositionSaturation, ZeroThresholdHoldsMeasuredPosition)
{
  // Arrange
  PositionSaturation saturation(1, make_config(true, 0.0, false));

  // Act
  const double command = saturation.compute(0, 1.0, 0.4);

  // Assert: 0.4 measured +/- 0.0 threshold
  EXPECT_DOUBLE_EQ(command, 0.4);
}

// compute : with persistence, later commands keep the shift caused by a clamping
TEST(PositionSaturation, PersistentShiftFollowsReferenceMotion)
{
  // Arrange: reference 0.5 clamped to 0.2
  PositionSaturation saturation(1, make_config(true, 0.2, true));
  run_cycle(saturation, 0, 0.5, 0.0);

  // Act: upstream integrator moves by +0.01, robot measured at the previous command
  const double command = saturation.compute(0, 0.51, 0.2);

  // Assert: 0.51 - 0.3 shift, previous command + 0.01 (0.4 if the shift were lost)
  EXPECT_NEAR(command, 0.21, kEpsilon);
}

// compute : with persistence, the robot stays where it was held even when the threshold allows more
TEST(PositionSaturation, PersistentDoesNotSpringBack)
{
  // Arrange: robot held at 0.0, then threshold widened
  PositionSaturation saturation(1, make_config(true, 0.0, true));
  run_cycle(saturation, 0, 0.5, 0.0);
  saturation.set_config(make_config(true, 1.0, true));

  // Act
  const double command = saturation.compute(0, 0.5, 0.0);

  // Assert: 0.5 - 0.5 shift (0.5 if it sprang back to the reference)
  EXPECT_NEAR(command, 0.0, kEpsilon);
}

// compute : without persistence, the original reference is sent again once the robot is close to it
TEST(PositionSaturation, NonPersistentSpringsBackToReference)
{
  // Arrange: reference clamped while the robot was held away from it
  PositionSaturation saturation(1, make_config(true, 0.2, false));
  run_cycle(saturation, 0, 0.5, 0.0);

  // Act: robot measured close to the reference again
  const double command = saturation.compute(0, 0.5, 0.45);

  // Assert: 0.5 is within 0.45 +/- 0.2 (0.5 - 0.3 = 0.2 if the shift were kept)
  EXPECT_DOUBLE_EQ(command, 0.5);
}

// compute : a clamping on one joint does not shift the commands of another joint
TEST(PositionSaturation, JointsAreIndependent)
{
  // Arrange: joint 0 clamped with persistence
  PositionSaturation saturation(2, make_config(true, 0.2, true));
  run_cycle(saturation, 0, 0.5, 0.0);

  // Act
  const double command = saturation.compute(1, 0.1, 0.0);

  // Assert: 0.1 is within 0.0 +/- 0.2 (0.1 - 0.3 = -0.2 if joint 0 shift applied)
  EXPECT_DOUBLE_EQ(command, 0.1);
}
