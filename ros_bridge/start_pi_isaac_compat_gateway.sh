#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${INDOORY_ROS_ROOT:-/home/pi/indoory_ros}"
if [ ! -d "${ROOT_DIR}" ]; then
  ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi
ENV_FILE="${XLEROBOT_IO_ENV:-${ROOT_DIR}/robot/xlerobot_robot_io.env}"

if [ -f "${ENV_FILE}" ]; then
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
fi

if [ "${USE_VENV:-true}" = "true" ] && [ -f "${XLE_ROBOT_VENV:-/home/pi/xlerobot-io-venv}/bin/activate" ]; then
  # shellcheck disable=SC1090
  source "${XLE_ROBOT_VENV:-/home/pi/xlerobot-io-venv}/bin/activate"
elif [ -f "${XLE_ROBOT_CONDA_SH:-/home/pi/.miniforge3/etc/profile.d/conda.sh}" ]; then
  # shellcheck disable=SC1090
  source "${XLE_ROBOT_CONDA_SH:-/home/pi/.miniforge3/etc/profile.d/conda.sh}"
  conda activate "${ISAAC_COMPAT_CONDA_ENV:-${XLE_ROBOT_CONDA_ENV:-lerobot}}"
fi

if [ -z "${ROSBRIDGE_URL:-}" ] && [ -n "${ROSBRIDGE_URI:-}" ]; then
  ROSBRIDGE_URL="${ROSBRIDGE_URI}"
fi

if [ -z "${ROSBRIDGE_URL:-}" ]; then
  if [ -n "${COMPUTE_PC_HOST:-}" ]; then
    ROSBRIDGE_URL="ws://${COMPUTE_PC_HOST}:9090"
  elif [ -n "${ROSBRIDGE_HOST:-}" ]; then
    ROSBRIDGE_URL="ws://${ROSBRIDGE_HOST}:${ROSBRIDGE_PORT:-9090}"
  else
    ROSBRIDGE_URL="ws://127.0.0.1:9090"
  fi
fi

: "${ISAAC_COMPAT_PYTHON:=python3}"

EXTRA_ARGS=()
case "${ISAAC_COMPAT_ALLOW_RAW_JOINT_TARGETS:-false}" in
  1|true|yes|on)
    EXTRA_ARGS+=(--allow-raw-joint-targets)
    ;;
esac

exec "${ISAAC_COMPAT_PYTHON}" "${ROOT_DIR}/ros_bridge/isaac_compat_gateway.py" \
  --rosbridge-url "${ROSBRIDGE_URL}" \
  --bind-host "${ISAAC_COMPAT_BIND_HOST:-127.0.0.1}" \
  --pub-port "${ISAAC_COMPAT_PUB_PORT:-8855}" \
  --pull-port "${ISAAC_COMPAT_PULL_PORT:-8856}" \
  --rep-port "${ISAAC_COMPAT_REP_PORT:-8857}" \
  --robot-id "${ISAAC_COMPAT_ROBOT_ID:-0}" \
  --num-robots "${ISAAC_COMPAT_NUM_ROBOTS:-1}" \
  --scan-topic "${ISAAC_COMPAT_SCAN_TOPIC:-${SCAN_TOPIC:-/xlerobot/scan}}" \
  --odom-topic "${ISAAC_COMPAT_ODOM_TOPIC:-${ODOM_TOPIC:-/xlerobot/odom}}" \
  --cmd-vel-topic "${ISAAC_COMPAT_CMD_VEL_TOPIC:-${CMD_TOPIC:-/xlerobot/cmd_vel}}" \
  --joint-target-topic "${ISAAC_COMPAT_JOINT_TARGET_TOPIC:-${JOINT_TARGET_TOPIC:-/xlerobot/teleop/joint_targets}}" \
  --scan-frame "${ISAAC_COMPAT_SCAN_FRAME:-${LIDAR_FRAME:-laser}}" \
  --base-frame "${ISAAC_COMPAT_BASE_FRAME:-${BASE_FRAME:-base_link}}" \
  --log-level "${ISAAC_COMPAT_LOG_LEVEL:-${LOG_LEVEL:-INFO}}" \
  "${EXTRA_ARGS[@]}"
