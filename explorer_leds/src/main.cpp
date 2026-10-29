/*
 *  main.cpp
 *  Copyright (C) 2026 Orthopus
 *  All rights reserved.
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "explorer_leds/led_node.h"
#include "explorer_leds/radxa_led_driver.h"
#include "explorer_leds/simulated_led_driver.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("led_node");

  // Without hardware, commands are still received and played, but sent nowhere
  const bool simulation = node->declare_parameter<bool>("simulation", false);

  explorer_leds::LedDriverFactory make_driver;
  if (simulation)
  {
    RCLCPP_INFO(node->get_logger(), "Simulation: LED colours are not sent to any hardware");
    make_driver = [](const explorer_leds::LedStripConfig&)
    { return std::make_unique<explorer_leds::SimulatedLedDriver>(); };
  }
  else
  {
    make_driver = [](const explorer_leds::LedStripConfig& strip)
    { return std::make_unique<explorer_leds::RadxaLedDriver>(strip); };
  }

  {
    explorer_leds::LedNode led_node(node, make_driver);
    rclcpp::spin(node);
  }
  rclcpp::shutdown();

  return 0;
}
