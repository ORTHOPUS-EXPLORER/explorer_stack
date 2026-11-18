#include "explorer_user_interfaces_cpp/timed_input.h"

namespace space_control
{
TimedInput::TimedInput(double timeout_seconds) : timeout_seconds_(timeout_seconds) {}

void TimedInput::set(double value, const rclcpp::Time& now)
{
  value_ = value;
  last_time_ = now;
}

void TimedInput::reset()
{
  value_ = 0.0;
  last_time_.reset();
}

double TimedInput::get(const rclcpp::Time& now) const
{
  if (!last_time_ || (now - *last_time_).seconds() > timeout_seconds_)
  {
    return 0.0;
  }
  return value_;
}
}  // namespace space_control
