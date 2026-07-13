#!/bin/bash

ros2 service call /explorer_joint_6/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_6', mode: 'impedance'}"
ros2 service call /explorer_joint_5/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_5', mode: 'impedance'}"
ros2 service call /explorer_joint_4/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_4', mode: 'impedance'}"
ros2 service call /explorer_joint_3/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_3', mode: 'impedance'}"
ros2 service call /explorer_joint_2/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_2', mode: 'impedance'}"
ros2 service call /explorer_joint_1/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_1', mode: 'impedance'}"
