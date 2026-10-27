#ifndef EXPLORER_USER_INTERFACES_CPP_LED_STATUS_CONFIG_H
#define EXPLORER_USER_INTERFACES_CPP_LED_STATUS_CONFIG_H

#include <map>
#include <optional>
#include <string>
#include <unordered_map>

#include "explorer_msgs/msg/led_command.hpp"
#include "explorer_user_interfaces_cpp/trajectory_manager.h"

namespace space_control
{
/**
 * \brief LED commands shown for each robot status, read from the LED status config file
 *
 * Until the arm is deployed, the LEDs show its retract status; once deployed (READY), the input
 * device mode.
 */
struct LedStatusConfig
{
  // Commands by retract status, READY excluded (the mode is shown then)
  std::map<RetractStatus, explorer_msgs::msg::LedCommand> retract_status_commands;
  // Commands by input device mode name (e.g. B1), shown while READY
  std::unordered_map<std::string, explorer_msgs::msg::LedCommand> mode_commands;
};

/**
 * \brief Load the LED status config file, throws std::runtime_error if missing or invalid
 *
 * Only the structure and the mode names are checked here, explorer_leds checks the rest
 * (speed, colour and intensity values) and ignores an invalid command.
 */
[[nodiscard]] LedStatusConfig load_led_status_config(const std::string& path);

/**
 * \brief LED command for the robot status: the retract status one until the arm is READY, then
 * the one of \p mode_name.
 *
 * Empty when the status has no command (or READY with no mode received yet): the LEDs then keep
 * their current effect.
 */
[[nodiscard]] std::optional<explorer_msgs::msg::LedCommand> select_led_command(
  const LedStatusConfig& config, RetractStatus retract_status,
  const std::optional<std::string>& mode_name);

}  // namespace space_control

#endif
