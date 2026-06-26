#!/usr/bin/env bash
# Systemd supervisor for the Pi-side Indoory live runtime.
# Owns the Pi runtime only. Local ROS/web scripts must never start or stop this.

set -euo pipefail

ROOT_DIR="${INDOORY_PI_ROOT:-/home/pi/indoory_ros}"
LIVE_STACK="${INDOORY_LIVE_STACK:-$ROOT_DIR/scripts/indoory_live_stack.sh}"
TELEOP_ROOT="${INDOORY_TELEOP_ROOT:-/home/pi/teleoperation}"
TELEOP_START="$TELEOP_ROOT/start_indoory_fast_teleop.sh"
TELEOP_STOP="$TELEOP_ROOT/stop_indoory_fast_teleop.sh"
LOG_DIR="$ROOT_DIR/run"
mkdir -p "$LOG_DIR"

log() {
  printf '[indoory-pi-live] %s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

env_file_value() {
  local key="$1"
  local file="$ROOT_DIR/robot/xlerobot_robot_io.env"
  [[ -f "$file" ]] || return 0
  awk -F= -v key="$key" '$1 == key {print substr($0, index($0, "=") + 1); exit}' "$file"
}

port_open() {
  local host="$1"
  local port="$2"
  timeout 1 bash -lc "</dev/tcp/${host}/${port}" >/dev/null 2>&1
}

require_file() {
  local path="$1"
  if [[ ! -x "$path" ]]; then
    log "missing executable: $path"
    return 1
  fi
}

export COMPUTE_PC_HOST="${COMPUTE_PC_HOST:-127.0.0.1}"
export ROSBRIDGE_HOST="${ROSBRIDGE_HOST:-$COMPUTE_PC_HOST}"
export ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
export FAST_ZMQ_PUB_PORT="${FAST_ZMQ_PUB_PORT:-8855}"
export FAST_ZMQ_PULL_PORT="${FAST_ZMQ_PULL_PORT:-8856}"
export FAST_ZMQ_REP_PORT="${FAST_ZMQ_REP_PORT:-8857}"
export FAST_ZMQ_ROBOT_ID="${FAST_ZMQ_ROBOT_ID:-0}"
export CAMERA_OPT_PORT="${CAMERA_OPT_PORT:-8866}"
export RGBD_ZMQ_ENABLE="${RGBD_ZMQ_ENABLE:-true}"
export RGBD_ZMQ_PORT="${RGBD_ZMQ_PORT:-8867}"
export RGBD_ZMQ_TOPIC="${RGBD_ZMQ_TOPIC:-/xlerobot/head/rgbd}"
export RGBD_ZMQ_FPS="${RGBD_ZMQ_FPS:-15}"
export RGBD_ZMQ_COLOR_MODE="${RGBD_ZMQ_COLOR_MODE:-jpeg}"
export RGBD_BINARY_ENABLE="${RGBD_BINARY_ENABLE:-false}"
export RGBD_BINARY_HOST="${RGBD_BINARY_HOST:-$COMPUTE_PC_HOST}"
export RGBD_BINARY_PORT="${RGBD_BINARY_PORT:-9102}"
export RGBD_BINARY_FPS="${RGBD_BINARY_FPS:-0}"
export VIDEO_RTSP_ENABLE="${VIDEO_RTSP_ENABLE:-true}"
export VIDEO_RTSP_BASE_URL="${VIDEO_RTSP_BASE_URL:-rtsp://$COMPUTE_PC_HOST:8554}"
export VIDEO_RTSP_TRANSPORT="${VIDEO_RTSP_TRANSPORT:-tcp}"

floor_from_env="$(env_file_value FLOOR_CAMERA_DEVICE || true)"
export FLOOR_CAMERA_DEVICE="${floor_from_env:-${FLOOR_CAMERA_DEVICE:-none}}"
export TELEOP_SKIP_ROBOT_STOP=1
export TELEOP_RESTART=1
export TELEOP_CAMERA_ZMQ_START="${TELEOP_CAMERA_ZMQ_START:-off}"
export TELEOP_CAMERA_ZMQ_ENDPOINTS="${TELEOP_CAMERA_ZMQ_ENDPOINTS:-tcp://127.0.0.1:$CAMERA_OPT_PORT}"
export TELEOP_CAMERA_FEEDS="${TELEOP_CAMERA_FEEDS:-head,left,right,floor}"
export TELEOP_CAMERA_FORMAT="${TELEOP_CAMERA_FORMAT:-h264}"
export TELEOP_FORWARD_MODE="${TELEOP_FORWARD_MODE:-robot}"
export ROBOT_FAST_HOST="${ROBOT_FAST_HOST:-127.0.0.1}"
export ROBOT_FAST_PUB_PORT="$FAST_ZMQ_PUB_PORT"
export ROBOT_FAST_PULL_PORT="$FAST_ZMQ_PULL_PORT"
export ROBOT_FAST_REP_PORT="$FAST_ZMQ_REP_PORT"

start_live_stack() {
  require_file "$LIVE_STACK" || return 1
  log "starting live stack: state=$FAST_ZMQ_PUB_PORT/$FAST_ZMQ_PULL_PORT/$FAST_ZMQ_REP_PORT video=$CAMERA_OPT_PORT rgbd=$RGBD_ZMQ_PORT floor=$FLOOR_CAMERA_DEVICE"
  bash "$LIVE_STACK" start >>"$LOG_DIR/indoory_pi_live_service_live_stack.log" 2>&1
}

start_teleop() {
  require_file "$TELEOP_START" || return 1
  log "starting VR teleop: web=8443 camera=$TELEOP_CAMERA_ZMQ_ENDPOINTS feeds=$TELEOP_CAMERA_FEEDS"
  TELEOP_SKIP_ROBOT_STOP=1 TELEOP_RESTART=1 bash "$TELEOP_START" >>"$LOG_DIR/indoory_pi_live_service_teleop.log" 2>&1
}

stop_teleop() {
  if [[ -x "$TELEOP_STOP" ]]; then
    log "stopping VR teleop"
    TELEOP_SKIP_ROBOT_STOP=1 bash "$TELEOP_STOP" >>"$LOG_DIR/indoory_pi_live_service_teleop.log" 2>&1 || true
  fi
}

stop_live_stack() {
  if [[ -x "$LIVE_STACK" ]]; then
    log "stopping live stack"
    bash "$LIVE_STACK" stop >>"$LOG_DIR/indoory_pi_live_service_live_stack.log" 2>&1 || true
  fi
}

stop_all() {
  stop_teleop
  stop_live_stack
}

trap 'log "service stop requested"; stop_all; exit 0' INT TERM

start_live_stack || log "live stack start failed"
start_teleop || log "teleop start failed"

while true; do
  sleep "${INDOORY_PI_HEALTH_INTERVAL_SEC:-10}"

  live_missing=()
  for port in "$FAST_ZMQ_PUB_PORT" "$FAST_ZMQ_PULL_PORT" "$FAST_ZMQ_REP_PORT" "$CAMERA_OPT_PORT" "$RGBD_ZMQ_PORT"; do
    if ! port_open 127.0.0.1 "$port"; then
      live_missing+=("$port")
    fi
  done
  if ((${#live_missing[@]} > 0)); then
    log "live stack missing local ports: ${live_missing[*]}; restarting live stack"
    start_live_stack || log "live stack restart failed"
  fi

  if ! port_open 127.0.0.1 8443; then
    log "VR teleop web port 8443 missing; restarting teleop"
    start_teleop || log "teleop restart failed"
  fi
done
