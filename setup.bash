#!/usr/bin/env bash
set -e

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONNOUSERSITE=1
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif command -v ros2 >/dev/null 2>&1; then
  :
else
  echo "warning: ros2 is not installed on this Pi; LeRobot direct tools still work." >&2
fi

WORKSPACE_SETUP="${INDORY_ROBOT_RUNTIME_SETUP:-$ROOT_DIR/install/setup.bash}"
if [ -f "$WORKSPACE_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$WORKSPACE_SETUP"
fi
