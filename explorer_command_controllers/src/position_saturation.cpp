#include "explorer_command_controllers/position_saturation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace explorer_command_controllers
{
PositionSaturation::PositionSaturation(
  std::size_t joint_count, const PositionSaturationConfig& config)
: offsets_(joint_count, 0.0)
{
  set_config(config);
}

void PositionSaturation::set_config(const PositionSaturationConfig& config)
{
  if (config.threshold < 0.0)
  {
    throw std::invalid_argument("PositionSaturation: threshold must be >= 0");
  }
  config_ = config;
}

void PositionSaturation::reset() { std::fill(offsets_.begin(), offsets_.end(), 0.0); }

double PositionSaturation::compute(
  std::size_t joint, double reference, std::optional<double> measured)
{
  if (!std::isfinite(reference))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double command = reference + offsets_[joint];
  if (!config_.enable || !measured || !std::isfinite(*measured))
  {
    return command;
  }

  const double clamped =
    std::clamp(command, *measured - config_.threshold, *measured + config_.threshold);
  if (config_.persistent)
  {
    offsets_[joint] = clamped - reference;
  }
  return clamped;
}

}  // namespace explorer_command_controllers
