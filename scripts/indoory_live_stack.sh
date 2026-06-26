#!/usr/bin/env bash
# One-touch live stack for the Indory robot runtime:
#   - robot fast state/control ZMQ: 8855/8856/8857
#   - rosbridge adapter is disabled by default; direct ZMQ is the live contract
#   - external optimized RGB H.264/fMP4 ZMQ: 0.0.0.0:8866 (for web fast path)
#   - optional legacy wrist RGB JPEG ZMQ: 127.0.0.1:8864
#   - optional external head RGB-D/depth ZMQ: 0.0.0.0:$RGBD_ZMQ_PORT
#   - optional binary head RGB-D TCP client: $COMPUTE_PC_HOST:$RGBD_BINARY_PORT

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="$ROOT_DIR/run"
mkdir -p "$RUN_DIR"

: "${COMPUTE_PC_HOST:=127.0.0.1}"
: "${ROSBRIDGE_HOST:=$COMPUTE_PC_HOST}"
: "${ROSBRIDGE_PORT:=9090}"
: "${RGBD_BINARY_HOST:=$COMPUTE_PC_HOST}"
: "${RGBD_BINARY_PORT:=9102}"

: "${ROBOT_STATE_LOG:=$RUN_DIR/live_robot_state.log}"
: "${WRIST_CAMERA_LOG:=$RUN_DIR/live_camera_zmq_wrist_raw.log}"
: "${RGB_RGBD_LOG:=$RUN_DIR/live_rgb_rgbd_combined.log}"

: "${ROBOT_STATE_PID:=$RUN_DIR/live_robot_state.pid}"
: "${WRIST_CAMERA_PID:=$RUN_DIR/live_camera_zmq_wrist_raw.pid}"
: "${RGB_RGBD_PID:=$RUN_DIR/live_rgb_rgbd_combined.pid}"

: "${TELEOP_ROOT:=$HOME/teleoperation}"
: "${TELEOP_PY:=$HOME/indory_isaac_sim/.venv-client/bin/python}"
: "${ROBOT_PY:=$HOME/.miniforge3/envs/lerobot/bin/python3}"
: "${RGB_RGBD_NICE:=12}"

: "${FAST_ZMQ_PUB_PORT:=8855}"
: "${FAST_ZMQ_PULL_PORT:=8856}"
: "${FAST_ZMQ_REP_PORT:=8857}"
: "${CAMERA_RAW_PORT:=8864}"
: "${CAMERA_OPT_PORT:=8866}"

ROBOT_IO_ENV_FILE="${ROBOT_IO_ENV_FILE:-$ROOT_DIR/robot/xlerobot_robot_io.env}"
env_file_value() {
  local key="$1"
  [[ -f "$ROBOT_IO_ENV_FILE" ]] || return 0
  awk -F= -v key="$key" '$1 == key {print substr($0, index($0, "=") + 1); exit}' "$ROBOT_IO_ENV_FILE"
}

# Hard-coded physical motor-driver role map for this Pi.
# Port names such as /dev/ttyACM0 and /dev/ttyACM1 can swap when USB order
# changes, so motor bus roles are bound by the stable CH9102 serial identity.
: "${XLEROBOT_LEFT_HEAD_SERIAL_SHORT:=5B14032190}"
: "${XLEROBOT_RIGHT_BASE_SERIAL_SHORT:=5B3D046415}"
motor_driver_by_serial() {
  local serial_short="$1"
  local fallback="$2"
  local path=""
  if [[ -d /dev/serial/by-id ]]; then
    path="$(find /dev/serial/by-id -maxdepth 1 -type l -name "*${serial_short}*" | sort | head -n 1)"
  fi
  if [[ -n "$path" ]]; then
    printf '%s
' "$path"
  else
    printf '%s
' "$fallback"
  fi
}
export XLEROBOT_LEFT_HEAD_SERIAL_SHORT XLEROBOT_RIGHT_BASE_SERIAL_SHORT

# Hard-coded physical camera role map for this Pi. Do not read role identity
# from robot/xlerobot_robot_io.env: /dev/videoN and USB port paths move after
# reconnects, so roles are bound only by stable device identity here.
WRIST_LEFT_CAMERA_MATCH='Innomaker_Innomaker-U20CAM-720P_SN0001'
WRIST_RIGHT_CAMERA_MATCH='Arducam_Technology_Co.__Ltd._Arducam_5MP_USB_Camera_AC20251017V1'
FLOOR_CAMERA_MATCH='USB_Camera_USB_Camera_202403071520007'
WRIST_LEFT_CAMERA_DEVICE=none
WRIST_RIGHT_CAMERA_DEVICE=none
FLOOR_CAMERA_DEVICE=none
export WRIST_LEFT_CAMERA_MATCH WRIST_RIGHT_CAMERA_MATCH FLOOR_CAMERA_MATCH
export WRIST_LEFT_CAMERA_DEVICE WRIST_RIGHT_CAMERA_DEVICE FLOOR_CAMERA_DEVICE
: "${WRIST_LEFT_INPUT_FORMAT:=$(env_file_value TELEOP_WRIST_LEFT_CAMERA_INPUT_FORMAT)}"
: "${WRIST_LEFT_INPUT_FORMAT:=MJPG}"
: "${WRIST_RIGHT_INPUT_FORMAT:=$(env_file_value TELEOP_WRIST_RIGHT_CAMERA_INPUT_FORMAT)}"
: "${WRIST_RIGHT_INPUT_FORMAT:=MJPG}"
: "${FLOOR_INPUT_FORMAT:=$(env_file_value TELEOP_FLOOR_CAMERA_INPUT_FORMAT)}"
: "${FLOOR_INPUT_FORMAT:=MJPG}"
: "${WRIST_LEFT_FLIP:=$(env_file_value TELEOP_WRIST_LEFT_CAMERA_FLIP)}"
: "${WRIST_LEFT_FLIP:=horizontal}"
: "${WRIST_RIGHT_FLIP:=$(env_file_value TELEOP_WRIST_RIGHT_CAMERA_FLIP)}"
: "${WRIST_RIGHT_FLIP:=both}"
: "${FLOOR_FLIP:=$(env_file_value TELEOP_FLOOR_CAMERA_FLIP)}"
: "${FLOOR_FLIP:=none}"

: "${FLOOR_CAMERA_DEVICE_FALLBACK:=none}"

camera_device_properties() {
  local device="$1"
  udevadm info --query=property --name="$device" 2>/dev/null || true
}

camera_props_match_any() {
  local props="$1"
  local pattern="$2"
  [[ -n "$pattern" ]] || return 1
  local old_ifs="$IFS"
  local term
  IFS='|'
  for term in $pattern; do
    IFS="$old_ifs"
    term="${term## }"
    term="${term%% }"
    [[ -n "$term" ]] || { IFS='|'; continue; }
    if grep -Fqi -- "$term" <<<"$props"; then
      IFS="$old_ifs"
      return 0
    fi
    IFS='|'
  done
  IFS="$old_ifs"
  return 1
}

camera_is_aux_capture() {
  local device="$1"
  [[ -n "$device" && "$device" != "none" ]] || return 1
  local resolved props
  resolved="$(readlink -f "$device" 2>/dev/null || printf '%s' "$device")"
  [[ -e "$resolved" ]] || return 1
  props="$(camera_device_properties "$resolved")"
  grep -q '^ID_V4L_CAPABILITIES=.*:capture:' <<<"$props" || return 1
  if grep -Eqi 'RealSense|Depth_Camera|pispbe|rpi-hevc|platform-1000' <<<"$props"; then
    return 1
  fi
  return 0
}

find_camera_by_identity() {
  local pattern="$1"
  [[ -n "$pattern" ]] || return 1
  local seen=" "
  local candidate resolved props
  shopt -s nullglob
  for candidate in /dev/v4l/by-id/*-video-index0 /dev/v4l/by-path/*-video-index0 /dev/video*; do
    resolved="$(readlink -f "$candidate" 2>/dev/null || printf '%s' "$candidate")"
    [[ -e "$resolved" ]] || continue
    case "$seen" in *" $resolved "*) continue ;; esac
    seen+="$resolved "
    camera_is_aux_capture "$resolved" || continue
    props="$(camera_device_properties "$resolved")"
    camera_props_match_any "$props" "$pattern" || continue
    if [[ "$candidate" == /dev/v4l/by-id/* ]]; then
      printf '%s
' "$candidate"
    else
      printf '%s
' "$resolved"
    fi
    return 0
  done
  return 1
}

resolve_camera_role() {
  local role="$1"
  local var_name="$2"
  local pattern="$3"
  local fallback="$4"
  local found=""
  if found="$(find_camera_by_identity "$pattern" 2>/dev/null)" && [[ -n "$found" ]]; then
    printf -v "$var_name" '%s' "$found"
    export "$var_name"
    echo "[camera-map] $role -> $found (match=$pattern)"
    return 0
  fi
  if camera_is_aux_capture "$fallback"; then
    found="$(readlink -f "$fallback" 2>/dev/null || printf '%s' "$fallback")"
    printf -v "$var_name" '%s' "$found"
    export "$var_name"
    echo "[camera-map] $role -> $found (fallback=$fallback)"
    return 0
  fi
  printf -v "$var_name" '%s' none
  export "$var_name"
  echo "[camera-map] $role -> none (no matching aux camera; match=$pattern)"
}

resolve_role_cameras() {
  resolve_camera_role wrist_left WRIST_LEFT_CAMERA_DEVICE "$WRIST_LEFT_CAMERA_MATCH" "$WRIST_LEFT_CAMERA_DEVICE"
  resolve_camera_role wrist_right WRIST_RIGHT_CAMERA_DEVICE "$WRIST_RIGHT_CAMERA_MATCH" "$WRIST_RIGHT_CAMERA_DEVICE"
  resolve_camera_role floor FLOOR_CAMERA_DEVICE "$FLOOR_CAMERA_MATCH" "$FLOOR_CAMERA_DEVICE"
}

resolve_camera_device() {
  local device="$1"
  [[ -n "$device" && "$device" != "none" ]] || return 1
  if command -v readlink >/dev/null 2>&1; then
    readlink -f "$device" 2>/dev/null || printf '%s
' "$device"
  else
    printf '%s
' "$device"
  fi
}

camera_device_exists() {
  local device="$1"
  local resolved
  resolved="$(resolve_camera_device "$device" 2>/dev/null || true)"
  [[ -n "$resolved" && -e "$resolved" ]]
}

ensure_distinct_floor_camera() {
  [[ -n "$FLOOR_CAMERA_DEVICE" && "$FLOOR_CAMERA_DEVICE" != "none" ]] || return 0
  local floor_resolved wl_resolved wr_resolved fallback_resolved
  floor_resolved="$(resolve_camera_device "$FLOOR_CAMERA_DEVICE" 2>/dev/null || true)"
  wl_resolved="$(resolve_camera_device "$WRIST_LEFT_CAMERA_DEVICE" 2>/dev/null || true)"
  wr_resolved="$(resolve_camera_device "$WRIST_RIGHT_CAMERA_DEVICE" 2>/dev/null || true)"
  if [[ -n "$floor_resolved" && ( "$floor_resolved" == "$wl_resolved" || "$floor_resolved" == "$wr_resolved" ) ]]; then
    fallback_resolved="$(resolve_camera_device "$FLOOR_CAMERA_DEVICE_FALLBACK" 2>/dev/null || true)"
    if [[ -n "$fallback_resolved" && -e "$fallback_resolved" && "$fallback_resolved" != "$wl_resolved" && "$fallback_resolved" != "$wr_resolved" ]]; then
      echo "[guard] floor camera resolved to an already-used wrist device ($floor_resolved); using stable floor by-path $FLOOR_CAMERA_DEVICE_FALLBACK"
      FLOOR_CAMERA_DEVICE="$FLOOR_CAMERA_DEVICE_FALLBACK"
      export FLOOR_CAMERA_DEVICE
    else
      echo "[guard] floor camera resolved to an already-used wrist device ($floor_resolved); disabling floor instead of stealing a wrist camera"
      FLOOR_CAMERA_DEVICE=none
      export FLOOR_CAMERA_DEVICE
    fi
  fi
}

: "${RGB_RAW_FPS:=15}"
: "${RGB_RAW_WIDTH:=640}"
: "${RGB_RAW_HEIGHT:=480}"
: "${RGB_RAW_JPEG_QUALITY:=70}"
: "${RGB_OPT_FPS:=15}"
: "${RGB_OPT_WIDTH:=640}"
: "${RGB_OPT_HEIGHT:=480}"
: "${RGB_OPT_JPEG_QUALITY:=70}"
: "${RGB_WIRE_FORMAT:=h264_fmp4}"
: "${WRIST_WIRE_FORMAT:=h264_fmp4}"
: "${VIDEO_RTSP_ENABLE:=true}"
: "${VIDEO_RTSP_BASE_URL:=rtsp://${COMPUTE_PC_HOST}:8554}"
: "${VIDEO_RTSP_TRANSPORT:=tcp}"
: "${VIDEO_RTSP_HEAD_PATH:=xlerobot_head}"
: "${VIDEO_RTSP_WRIST_LEFT_PATH:=xlerobot_wrist_left}"
: "${VIDEO_RTSP_WRIST_RIGHT_PATH:=xlerobot_wrist_right}"
: "${VIDEO_RTSP_FLOOR_PATH:=xlerobot_floor}"
: "${VIDEO_RTSP_H264_CRF:=28}"
: "${VIDEO_RTSP_H264_KEYINT_FRAMES:=15}"
: "${RGB_H264_CRF:=31}"
: "${RGB_H264_KEYINT_FRAMES:=15}"
: "${RGB_H264_INIT_INTERVAL_FRAMES:=30}"
: "${WRIST_H264_KEYINT_FRAMES:=15}"
: "${WRIST_H264_INIT_INTERVAL_FRAMES:=30}"
: "${WRIST_OPT_FPS:=12}"
: "${WRIST_LEFT_OPT_FPS:=12}"
: "${WRIST_RIGHT_OPT_FPS:=12}"
: "${FLOOR_OPT_FPS:=8}"
: "${WRIST_OPT_WIDTH:=640}"
: "${WRIST_OPT_HEIGHT:=480}"
: "${WRIST_OPT_JPEG_QUALITY:=55}"
: "${WRIST_H264_CRF:=31}"
: "${FAST_WRIST_CAPTURE_MODE:=direct}"
: "${WRIST_LEGACY_JPEG_ENABLE:=false}"
: "${FAST_CAMERA_FORCE_MJPEG_COPY:=0}"
if [[ "$FAST_CAMERA_FORCE_MJPEG_COPY" == "1" || "$FAST_CAMERA_FORCE_MJPEG_COPY" == "true" ]]; then
  WRIST_WIRE_FORMAT=jpeg
  WRIST_LEGACY_JPEG_ENABLE=true
fi
: "${FAST_CAMERA_TUNE_V4L2:=1}"
: "${FAST_CAMERA_POWER_LINE_FREQUENCY:=2}"
: "${WRIST_LEFT_EXPOSURE_ABSOLUTE:=167}"
: "${WRIST_LEFT_GAIN:=32}"
: "${WRIST_RIGHT_EXPOSURE_ABSOLUTE:=167}"
: "${WRIST_RIGHT_GAIN:=32}"
: "${FLOOR_EXPOSURE_ABSOLUTE:=167}"
: "${FLOOR_GAIN:=32}"
: "${RGB_H264_INPUT_FORMAT:=bgr24}"

: "${RGBD_CAPTURE_FPS:=15}"
: "${RGBD_BINARY_ENABLE:=false}"
: "${RGBD_BINARY_FPS:=0}"
: "${RGBD_BINARY_JPEG_QUALITY:=85}"
: "${RGBD_BINARY_WIDTH:=640}"
: "${RGBD_BINARY_HEIGHT:=480}"
: "${RGBD_ZMQ_ENABLE:=true}"
: "${RGBD_ZMQ_PORT:=8867}"
: "${RGBD_ZMQ_TOPIC:=/xlerobot/head/rgbd}"
: "${RGBD_ZMQ_FPS:=10}"
: "${RGBD_ZMQ_COLOR_MODE:=jpeg}"
: "${RGBD_DEPTH_FORMAT:=zstd16}"
: "${RGBD_COLOR_WIDTH:=640}"
: "${RGBD_COLOR_HEIGHT:=480}"
: "${RGBD_DEPTH_WIDTH:=640}"
: "${RGBD_DEPTH_HEIGHT:=480}"
: "${RGBD_DEPTH_FILTER_MODE:=accurate}"
: "${RGBD_DEPTH_MIN_M:=0.25}"
: "${RGBD_DEPTH_MAX_M:=5.0}"
: "${RGBD_DEPTH_VISUAL_PRESET:=3}"
: "${RGBD_DEPTH_LASER_POWER:=200}"
: "${RGBD_DEPTH_ENABLE_EMITTER:=1}"
: "${FAST_TF_HEAD_PAN_ZERO_TICK:=1950}"
: "${FAST_TF_HEAD_TILT_ZERO_TICK:=2055}"
export FAST_TF_HEAD_PAN_ZERO_TICK FAST_TF_HEAD_TILT_ZERO_TICK

is_alive() {
  local pid="$1"
  [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1
}

pid_from_file() {
  local file="$1"
  [[ -f "$file" ]] && tr -d '[:space:]' < "$file" || true
}

kill_pid_file() {
  local file="$1"
  local name="$2"
  local pid
  pid="$(pid_from_file "$file")"
  if is_alive "$pid"; then
    echo "[stop] $name pid=$pid"
    kill "$pid" >/dev/null 2>&1 || true
  fi
}

wait_gone() {
  local pid="$1"
  local deadline=$((SECONDS + 5))
  while is_alive "$pid" && (( SECONDS < deadline )); do
    sleep 0.2
  done
}

stop_stack() {
  local pids=()
  pids+=("$(pid_from_file "$RGB_RGBD_PID")")
  pids+=("$(pid_from_file "$WRIST_CAMERA_PID")")
  pids+=("$(pid_from_file "$ROBOT_STATE_PID")")

  kill_pid_file "$RGB_RGBD_PID" "rgb/rgbd side-channel"
  kill_pid_file "$WRIST_CAMERA_PID" "wrist camera raw"
  kill_pid_file "$ROBOT_STATE_PID" "robot state"

  for pid in "${pids[@]}"; do
    [[ -n "$pid" ]] && wait_gone "$pid"
  done

  for pattern in \
    "$ROOT_DIR/tools/rgb_rgbd_combined_sidechannel.py" \
    "teleoperation.camera_zmq_publisher" \
    "$ROOT_DIR/run_xlerobot_rosbridge_io.sh" \
    "python3 -m robot_io.xlerobot_fast_io" \
    "camera_mapping_debug_webview.py" \
    "arms_head_tf_debug_webview.py"
  do
    while IFS= read -r pid; do
      [[ -n "$pid" ]] || continue
      echo "[stop] stale match pid=$pid pattern=$pattern"
      kill "$pid" >/dev/null 2>&1 || true
    done < <(pgrep -f "$pattern" || true)
  done

  sleep 0.5

  pkill -f "$ROOT_DIR/tools/rgb_rgbd_combined_sidechannel.py" >/dev/null 2>&1 || true
  pkill -f "teleoperation.camera_zmq_publisher" >/dev/null 2>&1 || true
  pkill -f "$ROOT_DIR/run_xlerobot_rosbridge_io.sh" >/dev/null 2>&1 || true
  pkill -f "python3 -m robot_io.xlerobot_fast_io" >/dev/null 2>&1 || true
  pkill -f "camera_mapping_debug_webview.py" >/dev/null 2>&1 || true
  pkill -f "arms_head_tf_debug_webview.py" >/dev/null 2>&1 || true

  rm -f "$RGB_RGBD_PID" "$WRIST_CAMERA_PID" "$ROBOT_STATE_PID"
}

stop_rgb_rgbd() {
  local pid
  pid="$(pid_from_file "$RGB_RGBD_PID")"
  kill_pid_file "$RGB_RGBD_PID" "rgb/rgbd side-channel"
  [[ -n "$pid" ]] && wait_gone "$pid"
  while IFS= read -r pid; do
    [[ -n "$pid" ]] || continue
    echo "[stop] stale rgb/rgbd side-channel pid=$pid"
    kill "$pid" >/dev/null 2>&1 || true
  done < <(pgrep -f "$ROOT_DIR/tools/rgb_rgbd_combined_sidechannel.py" || true)
  sleep 0.3
  pkill -f "$ROOT_DIR/tools/rgb_rgbd_combined_sidechannel.py" >/dev/null 2>&1 || true
  rm -f "$RGB_RGBD_PID"
}

restart_rgb_rgbd() {
  stop_rgb_rgbd
  start_rgb_rgbd
  sleep 2
  status_stack
}

start_robot_state() {
  echo "[start] robot state/control ZMQ (fast ZMQ direct)"
  (
    cd "$ROOT_DIR"
    export COMPUTE_PC_HOST ROSBRIDGE_HOST ROSBRIDGE_PORT
    XLEROBOT_PORT1="$(motor_driver_by_serial "$XLEROBOT_LEFT_HEAD_SERIAL_SHORT" /dev/ttyACM1)"
    XLEROBOT_PORT2="$(motor_driver_by_serial "$XLEROBOT_RIGHT_BASE_SERIAL_SHORT" /dev/ttyACM0)"
    export XLEROBOT_PORT1 XLEROBOT_PORT2
    export PORT1="$XLEROBOT_PORT1"
    export PORT2="$XLEROBOT_PORT2"
    echo "[start] motor drivers: left+head=${XLEROBOT_PORT1} (${XLEROBOT_LEFT_HEAD_SERIAL_SHORT}), right+base=${XLEROBOT_PORT2} (${XLEROBOT_RIGHT_BASE_SERIAL_SHORT})"
    export ENABLE_FAST_ZMQ=true FAST_ZMQ_BIND_HOST=0.0.0.0
    export FAST_ZMQ_PUB_PORT FAST_ZMQ_PULL_PORT FAST_ZMQ_REP_PORT FAST_ZMQ_ROBOT_ID="${FAST_ZMQ_ROBOT_ID:-0}"
    export ENABLE_BASE="${ENABLE_BASE:-true}" ENABLE_LIDAR="${ENABLE_LIDAR:-true}" DRY_RUN="${DRY_RUN:-false}"
    export XLEROBOT_REALSENSE_RGB_TELEOP=false
    export ENABLE_DEPTH_SENSOR=false DEPTH_SENSOR_BINARY_ENABLE=false DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE=false
    export DEPTH_SENSOR_RTSP_ENABLE=false ENABLE_USB_CAMERA_RTSP=false USB_CAMERA_RTSP_ENABLE=false ENABLE_CAMERA=false
    exec setsid bash "$ROOT_DIR/run_xlerobot_rosbridge_io.sh"
  ) > "$ROBOT_STATE_LOG" 2>&1 &
  echo $! > "$ROBOT_STATE_PID"
}

start_wrist_camera() {
  if [[ "$FAST_WRIST_CAPTURE_MODE" == "direct" ]]; then
    echo "[start] wrist RGB raw ZMQ skipped; sidechannel owns legacy tcp://127.0.0.1:$CAMERA_RAW_PORT"
    return 0
  fi
  echo "[start] wrist RGB raw ZMQ on tcp://127.0.0.1:$CAMERA_RAW_PORT"
  (
    cd "$TELEOP_ROOT"
    exec setsid "$TELEOP_PY" -m teleoperation.camera_zmq_publisher \
      --bind-host 127.0.0.1 \
      --port "$CAMERA_RAW_PORT" \
      --robot-id 0 \
      --fps "$RGB_RAW_FPS" \
      --width "$RGB_RAW_WIDTH" \
      --height "$RGB_RAW_HEIGHT" \
      --jpeg-quality "$RGB_RAW_JPEG_QUALITY" \
      --front-device none \
      --wrist-left-device "$WRIST_LEFT_CAMERA_DEVICE" \
      --wrist-left-input-format "$WRIST_LEFT_INPUT_FORMAT" \
      --wrist-left-flip "$WRIST_LEFT_FLIP" \
      --wrist-right-device "$WRIST_RIGHT_CAMERA_DEVICE" \
      --wrist-right-input-format "$WRIST_RIGHT_INPUT_FORMAT" \
      --wrist-right-flip "$WRIST_RIGHT_FLIP"
  ) > "$WRIST_CAMERA_LOG" 2>&1 &
  echo $! > "$WRIST_CAMERA_PID"
}

tune_v4l2_device() {
  local device="$1"
  local controls="$2"
  [[ -n "$device" && -n "$controls" ]] || return 0
  command -v v4l2-ctl >/dev/null 2>&1 || return 0
  local resolved="$device"
  if command -v readlink >/dev/null 2>&1; then
    resolved="$(readlink -f "$device" 2>/dev/null || printf '%s' "$device")"
  fi
  [[ -e "$resolved" ]] || return 0
  v4l2-ctl -d "$resolved" --set-ctrl="$controls" >/dev/null 2>&1 || true
}

apply_camera_latency_tuning() {
  [[ "$FAST_CAMERA_TUNE_V4L2" == "1" || "$FAST_CAMERA_TUNE_V4L2" == "true" ]] || return 0
  tune_v4l2_device "$WRIST_LEFT_CAMERA_DEVICE" \
    "power_line_frequency=$FAST_CAMERA_POWER_LINE_FREQUENCY,auto_exposure=1,exposure_time_absolute=$WRIST_LEFT_EXPOSURE_ABSOLUTE,gain=$WRIST_LEFT_GAIN"
  tune_v4l2_device "$WRIST_RIGHT_CAMERA_DEVICE" \
    "power_line_frequency=$FAST_CAMERA_POWER_LINE_FREQUENCY,auto_exposure=1,exposure_time_absolute=$WRIST_RIGHT_EXPOSURE_ABSOLUTE,gain=$WRIST_RIGHT_GAIN"
  tune_v4l2_device "$FLOOR_CAMERA_DEVICE" \
    "power_line_frequency=$FAST_CAMERA_POWER_LINE_FREQUENCY,auto_exposure=1,exposure_time_absolute=$FLOOR_EXPOSURE_ABSOLUTE,gain=$FLOOR_GAIN"
}

start_rgb_rgbd() {
  echo "[start] optimized RGB ZMQ + RGB-D ZMQ=${RGBD_ZMQ_ENABLE} + binary RGB-D=${RGBD_BINARY_ENABLE}"
  (
    cd "$ROOT_DIR"
    resolve_role_cameras
    ensure_distinct_floor_camera
    apply_camera_latency_tuning
    wrist_jpeg_bind=""
    if [[ "$FAST_WRIST_CAPTURE_MODE" == "direct" && ( "$WRIST_LEGACY_JPEG_ENABLE" == "1" || "$WRIST_LEGACY_JPEG_ENABLE" == "true" ) ]]; then
      wrist_jpeg_bind="tcp://127.0.0.1:$CAMERA_RAW_PORT"
    fi
    binary_enable_arg=()
    if [[ "$RGBD_BINARY_ENABLE" == "1" || "$RGBD_BINARY_ENABLE" == "true" ]]; then
      binary_enable_arg=(--binary-enable)
    fi
    rgbd_zmq_enable_arg=()
    if [[ "$RGBD_ZMQ_ENABLE" == "1" || "$RGBD_ZMQ_ENABLE" == "true" ]]; then
      rgbd_zmq_enable_arg=(--rgbd-zmq-enable)
    fi
    rtsp_publish_args=()
    if [[ "$VIDEO_RTSP_ENABLE" == "1" || "$VIDEO_RTSP_ENABLE" == "true" ]]; then
      rtsp_publish_args=(
        --rtsp-publish-base "$VIDEO_RTSP_BASE_URL"
        --rtsp-publish-transport "$VIDEO_RTSP_TRANSPORT"
        --rtsp-head-path "$VIDEO_RTSP_HEAD_PATH"
        --rtsp-wrist-left-path "$VIDEO_RTSP_WRIST_LEFT_PATH"
        --rtsp-wrist-right-path "$VIDEO_RTSP_WRIST_RIGHT_PATH"
        --rtsp-floor-path "$VIDEO_RTSP_FLOOR_PATH"
        --rtsp-h264-crf "$VIDEO_RTSP_H264_CRF"
        --rtsp-h264-keyint-frames "$VIDEO_RTSP_H264_KEYINT_FRAMES"
      )
    fi
    exec setsid nice -n "$RGB_RGBD_NICE" "$ROBOT_PY" "$ROOT_DIR/tools/rgb_rgbd_combined_sidechannel.py" \
      --bind "tcp://0.0.0.0:$CAMERA_OPT_PORT" \
      --wrist-source "tcp://127.0.0.1:$CAMERA_RAW_PORT" \
      --wrist-capture-mode "$FAST_WRIST_CAPTURE_MODE" \
      --wrist-jpeg-bind "$wrist_jpeg_bind" \
      --wrist-topic /xlerobot/wrist_left/rgb/image_raw \
      --wrist-topic /xlerobot/wrist_right/rgb/image_raw \
      --wrist-left-device "$WRIST_LEFT_CAMERA_DEVICE" \
      --wrist-left-input-format "$WRIST_LEFT_INPUT_FORMAT" \
      --wrist-left-flip "$WRIST_LEFT_FLIP" \
      --wrist-right-device "$WRIST_RIGHT_CAMERA_DEVICE" \
      --wrist-right-input-format "$WRIST_RIGHT_INPUT_FORMAT" \
      --wrist-right-flip "$WRIST_RIGHT_FLIP" \
      --floor-device "$FLOOR_CAMERA_DEVICE" \
      --floor-input-format "$FLOOR_INPUT_FORMAT" \
      --floor-flip "$FLOOR_FLIP" \
      --floor-topic /xlerobot/floor/rgb/image_raw \
      --rgb-max-fps "$RGB_OPT_FPS" \
      --rgb-width "$RGB_OPT_WIDTH" \
      --rgb-height "$RGB_OPT_HEIGHT" \
      --rgb-jpeg-quality "$RGB_OPT_JPEG_QUALITY" \
      --rgb-wire-format "$RGB_WIRE_FORMAT" \
      --wrist-wire-format "$WRIST_WIRE_FORMAT" \
      "${rtsp_publish_args[@]}" \
      --h264-crf "$RGB_H264_CRF" \
      --h264-keyint-frames "$RGB_H264_KEYINT_FRAMES" \
      --h264-init-interval-frames "$RGB_H264_INIT_INTERVAL_FRAMES" \
      --h264-input-format "$RGB_H264_INPUT_FORMAT" \
      --wrist-h264-keyint-frames "$WRIST_H264_KEYINT_FRAMES" \
      --wrist-h264-init-interval-frames "$WRIST_H264_INIT_INTERVAL_FRAMES" \
      --wrist-max-fps "$WRIST_OPT_FPS" \
      --wrist-left-max-fps "$WRIST_LEFT_OPT_FPS" \
      --wrist-right-max-fps "$WRIST_RIGHT_OPT_FPS" \
      --floor-max-fps "$FLOOR_OPT_FPS" \
      --wrist-width "$WRIST_OPT_WIDTH" \
      --wrist-height "$WRIST_OPT_HEIGHT" \
      --wrist-jpeg-quality "$WRIST_OPT_JPEG_QUALITY" \
      --wrist-h264-crf "$WRIST_H264_CRF" \
      "${binary_enable_arg[@]}" \
      --binary-host "$RGBD_BINARY_HOST" \
      --binary-port "$RGBD_BINARY_PORT" \
      --binary-fps "$RGBD_BINARY_FPS" \
      --binary-jpeg-quality "$RGBD_BINARY_JPEG_QUALITY" \
      --binary-width "$RGBD_BINARY_WIDTH" \
      --binary-height "$RGBD_BINARY_HEIGHT" \
      "${rgbd_zmq_enable_arg[@]}" \
      --rgbd-zmq-bind "tcp://0.0.0.0:$RGBD_ZMQ_PORT" \
      --rgbd-zmq-topic "$RGBD_ZMQ_TOPIC" \
      --rgbd-zmq-fps "$RGBD_ZMQ_FPS" \
      --rgbd-zmq-color-mode "$RGBD_ZMQ_COLOR_MODE" \
      --depth-format "$RGBD_DEPTH_FORMAT" \
      --depth-filter-mode "$RGBD_DEPTH_FILTER_MODE" \
      --depth-min-m "$RGBD_DEPTH_MIN_M" \
      --depth-max-m "$RGBD_DEPTH_MAX_M" \
      --depth-visual-preset "$RGBD_DEPTH_VISUAL_PRESET" \
      --depth-laser-power "$RGBD_DEPTH_LASER_POWER" \
      --depth-enable-emitter "$RGBD_DEPTH_ENABLE_EMITTER" \
      --zstd-level "${RGBD_ZSTD_LEVEL:-1}" \
      --color-width "$RGBD_COLOR_WIDTH" \
      --color-height "$RGBD_COLOR_HEIGHT" \
      --depth-width "$RGBD_DEPTH_WIDTH" \
      --depth-height "$RGBD_DEPTH_HEIGHT" \
      --fps "$RGBD_CAPTURE_FPS"
  ) > "$RGB_RGBD_LOG" 2>&1 &
  echo $! > "$RGB_RGBD_PID"
}

start_stack() {
  stop_stack
  start_robot_state
  sleep 1
  start_wrist_camera
  sleep 1
  start_rgb_rgbd
  sleep 2
  status_stack
}

status_stack() {
  echo "== processes =="
  pgrep -af 'run_xlerobot_rosbridge_io|xlerobot_fast_io|camera_zmq_publisher|rgb_rgbd_combined_sidechannel' || true
  echo
  echo "== ports =="
  ss -ltnp | grep -E "(:$FAST_ZMQ_PUB_PORT|:$FAST_ZMQ_PULL_PORT|:$FAST_ZMQ_REP_PORT|:$CAMERA_RAW_PORT|:$CAMERA_OPT_PORT|:$RGBD_ZMQ_PORT)" || true
  echo
  echo "== state topics =="
  "$ROBOT_PY" - <<'PY' || true
import time, msgpack, zmq
ctx = zmq.Context.instance()
s = ctx.socket(zmq.SUB)
s.setsockopt(zmq.RCVTIMEO, 300)
s.connect("tcp://127.0.0.1:8855")
for t in (b"scan.0", b"odom.0", b"joint_states.0", b"tf.links.0", b"proprio.0"):
    s.setsockopt(zmq.SUBSCRIBE, t)
seen = {}
end = time.time() + 2.0
while time.time() < end and len(seen) < 5:
    try:
        parts = s.recv_multipart()
    except zmq.Again:
        continue
    topic = parts[0]
    seen[topic.decode("utf-8", "replace")] = seen.get(topic.decode("utf-8", "replace"), 0) + 1
print(seen)
PY
  echo
  echo "== rgb topics =="
  "$ROBOT_PY" - <<'PY' || true
import time, msgpack, zmq
ctx = zmq.Context.instance()
s = ctx.socket(zmq.SUB)
s.setsockopt(zmq.RCVTIMEO, 500)
s.connect("tcp://127.0.0.1:8866")
for t in (b"/xlerobot/head/rgb/image_raw", b"/xlerobot/floor/rgb/image_raw", b"/xlerobot/wrist_left/rgb/image_raw", b"/xlerobot/wrist_right/rgb/image_raw"):
    s.setsockopt(zmq.SUBSCRIBE, t)
seen = {}
end = time.time() + 3.0
while time.time() < end and len(seen) < 4:
    try:
        topic, payload = s.recv_multipart()
    except zmq.Again:
        continue
    msg = msgpack.unpackb(payload, raw=False)
    seen[topic.decode("utf-8", "replace")] = {
        "width": msg.get("width"),
        "height": msg.get("height"),
        "encoding": msg.get("encoding"),
        "bytes": len(msg.get("data", b"")),
        "init_bytes": len(msg.get("init", b"")),
        "chunk_seq": msg.get("chunk_seq"),
    }
print(seen)
PY
  echo
  echo "== rgbd zmq topic =="
  "$ROBOT_PY" - <<PY || true
import time, msgpack, zmq
ctx = zmq.Context.instance()
s = ctx.socket(zmq.SUB)
s.setsockopt(zmq.RCVTIMEO, 500)
s.connect("tcp://127.0.0.1:${RGBD_ZMQ_PORT}")
s.setsockopt(zmq.SUBSCRIBE, b"${RGBD_ZMQ_TOPIC}")
seen = None
end = time.time() + 3.0
while time.time() < end and seen is None:
    try:
        topic, payload = s.recv_multipart()
    except zmq.Again:
        continue
    msg = msgpack.unpackb(payload, raw=False)
    seen = {
        "topic": topic.decode("utf-8", "replace"),
        "encoding": msg.get("encoding"),
        "color_width": msg.get("color_width"),
        "color_height": msg.get("color_height"),
        "color_len": msg.get("color_len"),
        "depth_format": msg.get("depth_format"),
        "depth_width": msg.get("depth_width"),
        "depth_height": msg.get("depth_height"),
        "depth_len": msg.get("depth_len"),
        "depth_units": msg.get("depth_units"),
    }
print(seen or {})
PY
  echo
  if [[ "$RGBD_BINARY_ENABLE" == "1" || "$RGBD_BINARY_ENABLE" == "true" ]]; then
    echo
    echo "== rgbd tcp target =="
    timeout 2 bash -lc "</dev/tcp/$RGBD_BINARY_HOST/$RGBD_BINARY_PORT" \
      && echo "tcp://$RGBD_BINARY_HOST:$RGBD_BINARY_PORT open" \
      || echo "tcp://$RGBD_BINARY_HOST:$RGBD_BINARY_PORT closed_or_timeout"
    echo
  fi
  echo "== latest logs =="
  echo "-- robot --"
  tail -20 "$ROBOT_STATE_LOG" 2>/dev/null || true
  echo "-- rgb/rgbd --"
  tail -20 "$RGB_RGBD_LOG" 2>/dev/null || true
}

case "${1:-start}" in
  start) start_stack ;;
  restart) start_stack ;;
  restart-rgbd) restart_rgb_rgbd ;;
  restart-camera) restart_rgb_rgbd ;;
  stop) stop_stack ;;
  status) status_stack ;;
  *)
    echo "usage: $0 {start|stop|restart|restart-rgbd|restart-camera|status}" >&2
    exit 2
    ;;
esac
