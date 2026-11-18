/*
 *  led_node.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include "explorer_leds/led_node.h"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace explorer_leds
{
namespace
{
std::uint8_t to_msg_state(LedState state)
{
  switch (state)
  {
    case LedState::INIT:
      return explorer_msgs::msg::LedState::INIT;
    case LedState::READY:
      return explorer_msgs::msg::LedState::READY;
    case LedState::ERROR:
      return explorer_msgs::msg::LedState::ERROR;
  }
  return explorer_msgs::msg::LedState::ERROR;
}
}  // namespace

LedNode::LedNode(rclcpp::Node::SharedPtr node, const LedDriverFactory& make_driver)
: node_(std::move(node))
{
  // ROS Parameters
  node_->declare_parameter<std::string>(
    "strip_config_file",
    ament_index_cpp::get_package_share_directory("explorer_leds") + "/config/led_strip.yaml");
  node_->declare_parameter<std::string>("command_topic", "/explorer_leds/command");
  node_->declare_parameter<std::string>("flash_topic", "/explorer_leds/flash");
  node_->declare_parameter<std::string>("heartbeat_topic", "/command_node/ready");
  node_->declare_parameter<std::string>("state_topic", "/explorer_leds/state");
  node_->declare_parameter<double>("update_rate_hz", 50.0);
  node_->declare_parameter<double>("watchdog_timeout_seconds", 0.5);

  const auto strip_config_file = node_->get_parameter("strip_config_file").as_string();
  const auto command_topic = node_->get_parameter("command_topic").as_string();
  const auto flash_topic = node_->get_parameter("flash_topic").as_string();
  const auto heartbeat_topic = node_->get_parameter("heartbeat_topic").as_string();
  const auto state_topic = node_->get_parameter("state_topic").as_string();
  const auto update_rate_hz = node_->get_parameter("update_rate_hz").as_double();
  watchdog_timeout_seconds_ = node_->get_parameter("watchdog_timeout_seconds").as_double();

  // No heartbeat yet counts as lost communication once the timeout is over
  last_heartbeat_time_ = node_->now();

  // Publisher(s)
  // Latched: published on change only, late subscribers still get the current state
  state_pub_ = node_->create_publisher<explorer_msgs::msg::LedState>(
    state_topic, rclcpp::QoS(1).transient_local());

  try
  {
    const auto strip = load_led_strip_config(strip_config_file);
    RCLCPP_INFO(
      node_->get_logger(), "LED strip: %zu %s LEDs on %s, brightness capped to %.2f",
      strip.led_count, strip.rgbw ? "RGBW" : "RGB", strip.spi_device.c_str(), strip.max_brightness);
    auto driver = make_driver(strip);
    controller_ = std::make_unique<LedController>(strip, std::move(driver), LedControllerOptions{});
    publish_state_(controller_->state(), controller_->error());
  }
  catch (const std::runtime_error& e)
  {
    publish_state_(LedState::ERROR, e.what());
  }

  // Subscriber(s)
  // Latched, matching led_command_qos(): on (re)start, every publisher re-sends its last status
  command_sub_ = node_->create_subscription<explorer_msgs::msg::LedCommand>(
    command_topic, rclcpp::QoS(10).transient_local(),
    [this](const explorer_msgs::msg::LedCommand& command) { on_command_(command); });

  flash_sub_ = node_->create_subscription<explorer_msgs::msg::LedCommand>(
    flash_topic, 10, [this](const explorer_msgs::msg::LedCommand& command) { on_flash_(command); });

  heartbeat_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    heartbeat_topic, 10, [this](const std_msgs::msg::Bool& ready) { on_heartbeat_(ready); });

  // Timer(s)
  update_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / update_rate_hz), [this]() { update_(); });
}

void LedNode::on_command_(const explorer_msgs::msg::LedCommand& command)
{
  const auto effects = to_effects_(command, false);
  if (effects)
  {
    play_(*effects);
  }
}

void LedNode::on_flash_(const explorer_msgs::msg::LedCommand& command)
{
  const auto effects = to_effects_(command, true);
  if (effects)
  {
    play_(*effects);
  }
}

void LedNode::on_heartbeat_(const std_msgs::msg::Bool& ready)
{
  last_heartbeat_time_ = node_->now();

  if (ready.data != robot_ready_)
  {
    RCLCPP_INFO(node_->get_logger(), "Robot %s", ready.data ? "ready" : "not ready");
    robot_ready_ = ready.data;
  }

  update_();
}

std::optional<std::vector<LedEffect>> LedNode::to_effects_(
  const explorer_msgs::msg::LedCommand& command, bool flash) const
{
  if (!controller_)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Ignoring LED command: invalid LED strip config");
    return std::nullopt;
  }

  std::vector<LedEffect> effects;
  for (const auto& effect_msg : command.effects)
  {
    const auto mode = mode_from_string(effect_msg.mode);
    if (!mode)
    {
      RCLCPP_WARN(
        node_->get_logger(), "Ignoring LED command: unknown mode '%s'", effect_msg.mode.c_str());
      return std::nullopt;
    }

    if (flash && *mode != LedMode::FLASH_ONCE)
    {
      RCLCPP_WARN(
        node_->get_logger(), "Ignoring LED flash: mode '%s' must be published on the command topic",
        effect_msg.mode.c_str());
      return std::nullopt;
    }
    if (!flash && *mode == LedMode::FLASH_ONCE)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Ignoring LED command: flash_once must be published on the flash topic");
      return std::nullopt;
    }

    LedEffect effect;
    effect.leds.assign(effect_msg.leds.begin(), effect_msg.leds.end());
    effect.pattern.mode = *mode;
    effect.pattern.speed = effect_msg.speed;
    effect.pattern.intensities = effect_msg.intensities;
    effect.pattern.min_level = effect_msg.min_level;
    for (const auto& color : effect_msg.colors)
    {
      effect.pattern.colors.push_back({color.r, color.g, color.b, color.w});
    }
    effects.push_back(effect);
  }
  return effects;
}

void LedNode::play_(const std::vector<LedEffect>& effects)
{
  const auto error = controller_->apply(effects, node_->now().seconds());
  if (error)
  {
    RCLCPP_WARN(node_->get_logger(), "Ignoring LED command: %s", error->c_str());
    return;
  }

  for (const auto& effect : effects)
  {
    if (capped_speed(effect.pattern.mode, effect.pattern.speed) < effect.pattern.speed)
    {
      RCLCPP_WARN(
        node_->get_logger(), "LED effect speed capped from %.1f to %.1f Hz", effect.pattern.speed,
        MAX_REPEATING_SPEED_HZ);
    }
  }

  update_();
}

void LedNode::update_()
{
  if (!controller_)
  {
    return;
  }

  const rclcpp::Time now = node_->now();
  const bool lost = is_communication_lost_(now);

  if (lost != communication_lost_)
  {
    if (lost)
    {
      RCLCPP_WARN(
        node_->get_logger(), "No heartbeat for %.1f s, communication lost",
        watchdog_timeout_seconds_);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Heartbeat received again");
    }
    communication_lost_ = lost;
  }

  if (lost)
  {
    controller_->set_robot_state(RobotState::COMMUNICATION_LOST, now.seconds());
  }
  else if (!robot_ready_)
  {
    controller_->set_robot_state(RobotState::NOT_READY, now.seconds());
  }
  else
  {
    controller_->set_robot_state(RobotState::READY, now.seconds());
  }

  controller_->update(now.seconds());
  publish_state_(controller_->state(), controller_->error());
}

bool LedNode::is_communication_lost_(const rclcpp::Time& now) const
{
  return watchdog_timeout_seconds_ > 0.0 &&
         (now - last_heartbeat_time_).seconds() > watchdog_timeout_seconds_;
}

void LedNode::publish_state_(LedState state, const std::string& message)
{
  explorer_msgs::msg::LedState msg;
  msg.state = to_msg_state(state);
  msg.message = message;
  if (last_state_ == msg)
  {
    return;
  }

  switch (state)
  {
    case LedState::INIT:
      if (message.empty())
      {
        RCLCPP_INFO(node_->get_logger(), "LED state: init");
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "LED state: init, %s", message.c_str());
      }
      break;
    case LedState::READY:
      RCLCPP_INFO(node_->get_logger(), "LED state: ready");
      break;
    case LedState::ERROR:
      RCLCPP_ERROR(node_->get_logger(), "LED state: error, %s", message.c_str());
      break;
  }

  state_pub_->publish(msg);
  last_state_ = msg;
}
}  // namespace explorer_leds
