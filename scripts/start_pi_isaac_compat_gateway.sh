#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${INDORY_ROBOT_RUNTIME_ROOT:-${INDOORY_ROS_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}}"

exec "${ROOT_DIR}/ros_bridge/start_pi_isaac_compat_gateway.sh" "$@"
