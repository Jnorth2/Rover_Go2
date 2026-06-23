#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  echo "Set ROS_SETUP to the setup.bash for your ROS 2 installation." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u
export ZENOH_ROUTER_CONFIG_URI="${SCRIPT_DIR}/router.json5"

echo "Starting the groundstation Zenoh router"
echo "  Jetson endpoint: tcp/192.168.123.99:7447"
echo "  Config: ${SCRIPT_DIR}/router.json5"
exec ros2 run rmw_zenoh_cpp rmw_zenohd
