#!/bin/bash

ros2 service call /explorer_joint_5/mode orthopus_vesc_interfaces/srv/SetMode "{joint_name: 'joint_5', mode: 'impedance'}"
