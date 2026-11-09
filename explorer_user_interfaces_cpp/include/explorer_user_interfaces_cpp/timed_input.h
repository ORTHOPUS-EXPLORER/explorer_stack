#ifndef EXPLORER_USER_INTERFACES_CPP_TIMED_INPUT_H
#define EXPLORER_USER_INTERFACES_CPP_TIMED_INPUT_H

#include <optional>

#include "rclcpp/time.hpp"

namespace space_control
{
/**
 * \brief Last value received from an input topic, which falls back to 0 when not refreshed.
 *
 * A source that vanishes while sending a non-zero command (crash, network loss) must not keep
 * driving the robot: the value is only valid for timeout_seconds after it was set.
 */
class TimedInput
{
public:
  explicit TimedInput(double timeout_seconds);

  /// Store a new value received at now.
  void set(double value, const rclcpp::Time& now);

  /// Force the input back to 0, until the next set().
  void reset();

  /// Last value set, or 0 if never set, reset, or older than the timeout.
  [[nodiscard]] double get(const rclcpp::Time& now) const;

private:
  double timeout_seconds_;
  double value_ = 0.0;
  std::optional<rclcpp::Time> last_time_;
};
}  // namespace space_control

#endif
