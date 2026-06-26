#!/usr/bin/env zsh
set -e

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONNOUSERSITE=1
ROOT_DIR="$(cd "$(dirname "${(%):-%N}")" && pwd)"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.zsh" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.zsh"
elif [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif command -v ros2 >/dev/null 2>&1; then
  :
else
  echo "warning: ros2 is not installed on this Pi; LeRobot direct tools still work." >&2
fi

WORKSPACE_SETUP="${INDORY_ROBOT_RUNTIME_SETUP:-$ROOT_DIR/install/setup.zsh}"
if [ -f "$WORKSPACE_SETUP" ]; then
  source "$WORKSPACE_SETUP"
elif [ -f "$ROOT_DIR/install/setup.bash" ]; then
  source "$ROOT_DIR/install/setup.bash"
fi
