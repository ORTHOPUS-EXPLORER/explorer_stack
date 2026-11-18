#ifndef EXPLORER_USER_INTERFACES_CPP_LED_STATUS_CONFIG_H
#define EXPLORER_USER_INTERFACES_CPP_LED_STATUS_CONFIG_H

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "explorer_msgs/msg/led_command.hpp"
#include "explorer_user_interfaces_cpp/trajectory_manager.h"

namespace space_control
{
/**
 * \brief LED commands shown for each robot status, read from the LED status config file
 *
 * Until the arm is deployed, the LEDs show its retract status, except centered_leds which show
 * the input device mode colour while RETRACTED; once deployed (READY), every LED shows the mode.
 */
struct LedStatusConfig
{
  // Commands by retract status, READY excluded (the mode is shown then)
  std::map<RetractStatus, explorer_msgs::msg::LedCommand> retract_status_commands;
  // Commands by input device mode name (e.g. B1), shown while READY
  std::unordered_map<std::string, explorer_msgs::msg::LedCommand> mode_commands;
  // List the middle leds of each side
  std::vector<std::uint16_t> centered_leds;
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
 * While RETRACTED, the effects of \p mode_name follow the retract status ones, restricted to
 * centered_leds and turned into their first colour, solid (so they show over it there).
 * Empty when there's nothing to show (no command for the status, or no mode received yet): the
 * LEDs then keep their current effect.
 */
[[nodiscard]] std::optional<explorer_msgs::msg::LedCommand> select_led_command(
  const LedStatusConfig& config, RetractStatus retract_status,
  const std::optional<std::string>& mode_name);

}  // namespace space_control

#endif
