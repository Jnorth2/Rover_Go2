#!/usr/bin/env bash

# Source this file; do not execute it:
#   source ./ros2-local.sh
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this script with: source ${BASH_SOURCE[0]}" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI="${SCRIPT_DIR}/client.json5"

echo "Groundstation ROS 2 environment: rmw_zenoh_cpp -> localhost:7447"
