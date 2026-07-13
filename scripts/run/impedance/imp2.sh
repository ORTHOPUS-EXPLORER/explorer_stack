#!/bin/bash

ros2 service call /explorer_joint_2/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_2', mode: 'impedance'}"
