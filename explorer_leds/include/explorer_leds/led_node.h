/*
 *  led_node.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_LED_NODE_H
#define EXPLORER_LEDS_LED_NODE_H

#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "explorer_leds/led_controller.h"
#include "explorer_leds/led_driver.h"
#include "explorer_leds/led_strip_config.h"
#include "explorer_msgs/msg/led_command.hpp"
#include "explorer_msgs/msg/led_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace explorer_leds
{
/// Creates the driver of the strip described by its config
using LedDriverFactory = std::function<std::unique_ptr<LedDriver>(const LedStripConfig&)>;

/**
  * \brief ROS interface of the LedController.
  *
  * - command_topic: latched status, publishers send it once when it changes (see
  *   led_command_qos()). flash_once is refused there, it would replace the latched status.
  * - flash_topic: flash_once only, played over the current status.
  * - heartbeat_topic: periodic readiness of the robot. Without heartbeat for
  *   watchdog_timeout_seconds, communication is considered lost.
  * - state_topic: latched LedState, published when it changes.
  *
  * The strip config is read from strip_config_file. If it is invalid, the node stays in ERROR
  * (until restarted) and ignores commands.
  */
class LedNode
{
public:
  /**
    * \param node ROS node
    * \param make_driver creates the driver of the configured strip
    */
  LedNode(rclcpp::Node::SharedPtr node, const LedDriverFactory& make_driver);

private:
  void on_command_(const explorer_msgs::msg::LedCommand& command);

  void on_flash_(const explorer_msgs::msg::LedCommand& command);

  void on_heartbeat_(const std_msgs::msg::Bool& ready);

  /**
    * \brief Convert \p command to effects, empty and logged if it can't be.
    * \param flash whether it comes from the flash topic, which only accepts flash_once
    */
  [[nodiscard]] std::optional<std::vector<LedEffect>> to_effects_(
    const explorer_msgs::msg::LedCommand& command, bool flash) const;

  /// Play \p effects, logs why if they are rejected or their speed capped
  void play_(const std::vector<LedEffect>& effects);

  /// Timer callback, updates the robot state and the controller
  void update_();

  /// Indicates whether no heartbeat was received for longer than the watchdog timeout
  [[nodiscard]] bool is_communication_lost_(const rclcpp::Time& now) const;

  /// Publish and log \p state with \p message if either changed
  void publish_state_(LedState state, const std::string& message);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<explorer_msgs::msg::LedCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<explorer_msgs::msg::LedCommand>::SharedPtr flash_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<explorer_msgs::msg::LedState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Disabled when <= 0
  double watchdog_timeout_seconds_;

  // Empty when the strip config is invalid
  std::unique_ptr<LedController> controller_;
  rclcpp::Time last_heartbeat_time_;
  bool robot_ready_ = true;
  // Whether the last update found communication lost, to log only changes
  bool communication_lost_ = false;
  // Last state published, empty before the first publication
  std::optional<explorer_msgs::msg::LedState> last_state_;
};
}  // namespace explorer_leds

#endif
