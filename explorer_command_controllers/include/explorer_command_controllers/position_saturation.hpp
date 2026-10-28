#ifndef EXPLORER_COMMAND_CONTROLLERS__POSITION_SATURATION_HPP_
#define EXPLORER_COMMAND_CONTROLLERS__POSITION_SATURATION_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace explorer_command_controllers
{
struct PositionSaturationConfig
{
  // Clamp the command to [measured - threshold, measured + threshold]
  bool enable = true;
  // Maximum allowed distance between command and measured position (rad), must be >= 0
  double threshold = 0.0;
  // Keep the clamping as an offset on the reference instead of springing back to it
  bool persistent = false;
};

/**
 * ROS independent position error saturation, used by PositionSaturationController.
 *
 * The reference comes from an upstream integrator (qontrol_controller, output_integrator).
 * In persistent mode, the clamping is therefore stored as an offset added to
 * every following reference, which is equivalent to overwriting the integrated value.
 */
class PositionSaturation
{
public:
  /// @throws std::invalid_argument if config.threshold < 0
  PositionSaturation(std::size_t joint_count, const PositionSaturationConfig& config);

  /// @throws std::invalid_argument if config.threshold < 0
  void set_config(const PositionSaturationConfig& config);

  /// Clear all offsets
  void reset();

  /**
   * Compute the command of one joint (real-time safe, no allocation).
   *
   * @param joint joint index, must be < the joint_count given to the constructor
   * @param reference position from the upstream integrator; NaN gives a NaN command
   * @param measured measured position; when missing or NaN the command is not saturated
   * @returns reference + offset, clamped around measured when enabled
   */
  [[nodiscard]] double compute(std::size_t joint, double reference, std::optional<double> measured);

private:
  PositionSaturationConfig config_;
  std::vector<double> offsets_;
};

}  // namespace explorer_command_controllers

#endif  // EXPLORER_COMMAND_CONTROLLERS__POSITION_SATURATION_HPP_
