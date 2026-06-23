#!/usr/bin/env bash

# Source this file; do not execute it:
#   source ./ros2-local.sh
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this script with: source ${BASH_SOURCE[0]}" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

echo "Groundstation ROS 2 environment configured for the local ROS2DDS bridge."
