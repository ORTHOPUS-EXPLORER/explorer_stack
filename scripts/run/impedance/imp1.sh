#!/bin/bash

ros2 service call /explorer_joint_1/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_1', mode: 'impedance'}"
