#include <gtest/gtest.h>

#include "explorer_user_interfaces_cpp/timed_input.h"

namespace space_control
{
namespace
{
constexpr double TIMEOUT_SECONDS = 0.3;
// Small step to land just past the timeout
constexpr double EPSILON_SECONDS = 0.01;

rclcpp::Time at(double seconds) { return rclcpp::Time(static_cast<int64_t>(seconds * 1e9)); }

TEST(TimedInput, IsZeroWhenNeverSet)
{
  const double now = 10.0;

  const TimedInput input(TIMEOUT_SECONDS);
  EXPECT_DOUBLE_EQ(input.get(at(now)), 0.0);
}

TEST(TimedInput, ReturnsValueWithinTimeout)
{
  const double set_time = 10.0;
  const double value = 0.5;

  TimedInput input(TIMEOUT_SECONDS);
  input.set(value, at(set_time));
  EXPECT_DOUBLE_EQ(input.get(at(set_time)), value);
  EXPECT_DOUBLE_EQ(input.get(at(set_time + TIMEOUT_SECONDS)), value);
}

TEST(TimedInput, IsZeroOnceStale)
{
  const double set_time = 10.0;

  TimedInput input(TIMEOUT_SECONDS);
  input.set(-0.5, at(set_time));
  EXPECT_DOUBLE_EQ(input.get(at(set_time + TIMEOUT_SECONDS + EPSILON_SECONDS)), 0.0);
}

TEST(TimedInput, RefreshExtendsValidity)
{
  const double initial_time = 10.0;
  const double second_time = initial_time + TIMEOUT_SECONDS / 2.0;
  const double second_value = 0.8;

  TimedInput input(TIMEOUT_SECONDS);
  input.set(0.5, at(initial_time));
  input.set(second_value, at(second_time));
  // Past the initial value timeout, still valid thanks to the refresh
  EXPECT_DOUBLE_EQ(input.get(at(second_time + TIMEOUT_SECONDS)), second_value);
}

TEST(TimedInput, ResetForcesZeroUntilNextSet)
{
  const double initial_time = 10.0;
  const double second_time = initial_time + TIMEOUT_SECONDS / 2.0;
  const double second_value = 0.7;

  TimedInput input(TIMEOUT_SECONDS);
  input.set(0.5, at(initial_time));
  input.reset();
  EXPECT_DOUBLE_EQ(input.get(at(initial_time)), 0.0);

  input.set(second_value, at(second_time));
  EXPECT_DOUBLE_EQ(input.get(at(second_time)), second_value);
}
}  // namespace
}  // namespace space_control
