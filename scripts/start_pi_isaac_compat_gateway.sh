#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${INDOORY_ROS_ROOT:-/home/pi/indoory_ros}"
if [ ! -d "${ROOT_DIR}" ]; then
  ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

exec "${ROOT_DIR}/ros_bridge/start_pi_isaac_compat_gateway.sh" "$@"
