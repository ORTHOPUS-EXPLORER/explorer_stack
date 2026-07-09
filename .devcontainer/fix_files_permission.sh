#! /bin/bash

if [ -z "${USER_UID}" ] || [ -z "${GROUP_INPUT_UID}" ]; then
    echo "USER_UID or GROUP_INPUT_UID env var not set, file permissions will occurs, please provides proper variables."
else
    chown -R ${ROS_USER}:${ROS_USER} ${PWD}
    chown -R ${ROS_USER}:${ROS_USER} ${HOME}/.ccache
fi