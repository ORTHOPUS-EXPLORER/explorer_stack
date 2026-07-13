#!/bin/bash

ros2 service call  /explorer_joint_1/command orthopus_vesc_interfaces/srv/Cmd "{cmd: 'o_param set ctrl_bypass_torque_control 1', wait_for_ms: 50}"
ros2 service call  /explorer_joint_2/command orthopus_vesc_interfaces/srv/Cmd "{cmd: 'o_param set ctrl_bypass_torque_control 1', wait_for_ms: 50}"
ros2 service call  /explorer_joint_3/command orthopus_vesc_interfaces/srv/Cmd "{cmd: 'o_param set ctrl_bypass_torque_control 1', wait_for_ms: 50}"
ros2 service call  /explorer_joint_4/command orthopus_vesc_interfaces/srv/Cmd "{cmd: 'o_param set ctrl_bypass_torque_control 1', wait_for_ms: 50}"
ros2 service call  /explorer_joint_5/command orthopus_vesc_interfaces/srv/Cmd "{cmd: 'o_param set ctrl_bypass_torque_control 1', wait_for_ms: 50}"
ros2 service call  /explorer_joint_6/command orthopus_vesc_interfaces/srv/Cmd "{cmd: 'o_param set ctrl_bypass_torque_control 1', wait_for_ms: 50}"
