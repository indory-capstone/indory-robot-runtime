#!/usr/bin/env zsh
set -e

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONNOUSERSITE=1

if [ -f "/opt/ros/${ROS_DISTRO}/setup.zsh" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.zsh"
elif [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif command -v ros2 >/dev/null 2>&1; then
  :
else
  echo "warning: ros2 is not installed on this Pi; LeRobot direct tools still work." >&2
fi

if [ -f /home/pi/indoory_ros/install/setup.zsh ]; then
  source /home/pi/indoory_ros/install/setup.zsh
elif [ -f /home/pi/indoory_ros/install/setup.bash ]; then
  source /home/pi/indoory_ros/install/setup.bash
fi
