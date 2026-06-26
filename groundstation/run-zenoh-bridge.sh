#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
BRIDGE_BIN="${BRIDGE_BIN:-/usr/bin/zenoh-bridge-ros2dds}"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -x "${BRIDGE_BIN}" ]]; then
  echo "Zenoh ROS2DDS bridge not found: ${BRIDGE_BIN}" >&2
  echo "Install zenoh-bridge-ros2dds v1.9.0 as described in README.md." >&2
  exit 1
fi

# ROS setup scripts expect some variables to be unset.
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

echo "Starting the groundstation ROS2DDS bridge"
echo "  Local router: tcp/127.0.0.1:7447"
echo "  Config: ${SCRIPT_DIR}/bridge.json5"
exec "${BRIDGE_BIN}" --config "${SCRIPT_DIR}/bridge.json5"
