/*
 *  led_qos.h
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#ifndef EXPLORER_LEDS_LED_QOS_H
#define EXPLORER_LEDS_LED_QOS_H

#include <rclcpp/qos.hpp>

namespace explorer_leds
{
/**
  * \brief QoS to publish on the LED command topic with.
  *
  * Latched: publishers send a status once, when it changes, and a LED node started later still
  * gets the last status of each publisher. The LED node subscribes with TRANSIENT_LOCAL durability,
  * so a publisher with the default (volatile) durability is incompatible and never received.
  */
[[nodiscard]] inline rclcpp::QoS led_command_qos() { return rclcpp::QoS(1).transient_local(); }
}  // namespace explorer_leds

#endif
