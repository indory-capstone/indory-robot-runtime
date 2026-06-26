#!/usr/bin/env bash
# Lightweight checks for the Raspberry Pi / robot I/O computer.

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ENV_FILE="${ENV_FILE:-robot/xlerobot_robot_io.env}"
ROSBRIDGE_HOST_OVERRIDE="${ROSBRIDGE_HOST:-}"
ROSBRIDGE_PORT_OVERRIDE="${ROSBRIDGE_PORT:-}"
ROSBRIDGE_URI_OVERRIDE="${ROSBRIDGE_URI:-}"
if [[ -f "$ENV_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi
[[ -n "$ROSBRIDGE_HOST_OVERRIDE" ]] && ROSBRIDGE_HOST="$ROSBRIDGE_HOST_OVERRIDE"
[[ -n "$ROSBRIDGE_PORT_OVERRIDE" ]] && ROSBRIDGE_PORT="$ROSBRIDGE_PORT_OVERRIDE"
[[ -n "$ROSBRIDGE_URI_OVERRIDE" ]] && ROSBRIDGE_URI="$ROSBRIDGE_URI_OVERRIDE"

: "${XLE_ROBOT_VENV:=$HOME/xlerobot-io-venv}"
: "${ROSBRIDGE_HOST:=127.0.0.1}"
: "${ROSBRIDGE_PORT:=9090}"
: "${XLEROBOT_PORT1:=${PORT1:-/dev/ttyACM0}}"
: "${XLEROBOT_PORT2:=${PORT2:-/dev/ttyACM1}}"
: "${LIDAR_SERIAL:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_12703f59806eef11ba3ee8c2c169b110-if00-port0}"
: "${CAMERA_DEVICE:=/dev/video0}"
: "${CAMERA_TOPIC:=/xlerobot/base_camera/image/compressed}"
: "${CAMERA_INFO_TOPIC:=/xlerobot/base_camera/camera_info}"
: "${CAMERA_FRAME:=base_camera_optical_frame}"
: "${CAMERA_RATE_HZ:=${CAMERA_FPS:-8}}"
: "${CAMERA_JPEG_QUALITY:=60}"
: "${ENABLE_BASE:=true}"
: "${ENABLE_LIDAR:=true}"
: "${ENABLE_DEPTH_SENSOR:=true}"
: "${ENABLE_CAMERA:=false}"
: "${ENABLE_FAST_ZMQ:=true}"
: "${FAST_ZMQ_BIND_HOST:=0.0.0.0}"
: "${FAST_ZMQ_PUB_PORT:=8855}"
: "${FAST_ZMQ_PULL_PORT:=8856}"
: "${FAST_ZMQ_REP_PORT:=8857}"
: "${DEPTH_SENSOR_SERIAL:=}"
: "${DEPTH_SENSOR_DEPTH_TOPIC:=/xlerobot/head_camera/depth/image}"
: "${DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC:=/xlerobot/head_camera/depth/camera_info}"
: "${DEPTH_SENSOR_COLOR_TOPIC:=/xlerobot/head_camera/color/image}"
: "${DEPTH_SENSOR_COLOR_CAMERA_INFO_TOPIC:=/xlerobot/head_camera/color/camera_info}"
: "${DEPTH_SENSOR_IMU_TOPIC:=/xlerobot/head_camera/imu}"
: "${DEPTH_SENSOR_RTSP_ENABLE:=false}"
: "${DEPTH_SENSOR_RTSP_URL:=}"
export ENABLE_BASE ENABLE_LIDAR ENABLE_DEPTH_SENSOR ENABLE_CAMERA ENABLE_FAST_ZMQ CAMERA_DEVICE CAMERA_TOPIC CAMERA_INFO_TOPIC CAMERA_FRAME CAMERA_RATE_HZ CAMERA_JPEG_QUALITY
export DEPTH_SENSOR_SERIAL DEPTH_SENSOR_DEPTH_TOPIC DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC DEPTH_SENSOR_COLOR_TOPIC DEPTH_SENSOR_COLOR_CAMERA_INFO_TOPIC DEPTH_SENSOR_IMU_TOPIC

FAIL=0

ok() { printf '[ ok ] %s\n' "$*"; }
warn() { printf '[warn] %s\n' "$*"; }
bad() { printf '[err] %s\n' "$*"; FAIL=1; }

need_cmd() {
  if command -v "$1" >/dev/null 2>&1; then ok "command: $1"; else bad "missing command: $1"; fi
}

echo "============================================================"
echo "XLeRobot fast ZMQ robot I/O environment check"
echo "============================================================"
echo "ROOT=$ROOT"
echo "ENV_FILE=$ENV_FILE"
echo "XLE_ROBOT_VENV=$XLE_ROBOT_VENV"
echo "ENABLE_BASE=$ENABLE_BASE"
echo "ENABLE_LIDAR=$ENABLE_LIDAR"
echo "ENABLE_DEPTH_SENSOR=$ENABLE_DEPTH_SENSOR"
echo "ENABLE_FAST_ZMQ=$ENABLE_FAST_ZMQ (${FAST_ZMQ_BIND_HOST}:${FAST_ZMQ_PUB_PORT}/${FAST_ZMQ_PULL_PORT}/${FAST_ZMQ_REP_PORT})"
if [[ "$ENABLE_DEPTH_SENSOR" == "true" || "$ENABLE_DEPTH_SENSOR" == "1" ]]; then
  echo "DEPTH_SENSOR_SERIAL=${DEPTH_SENSOR_SERIAL:-<first-device>}"
  echo "DEPTH_SENSOR_DEPTH_TOPIC=$DEPTH_SENSOR_DEPTH_TOPIC"
  echo "DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC=$DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC"
  echo "DEPTH_SENSOR_IMU_TOPIC=$DEPTH_SENSOR_IMU_TOPIC"
  echo "DEPTH_SENSOR_RTSP_ENABLE=$DEPTH_SENSOR_RTSP_ENABLE"
  echo "DEPTH_SENSOR_RTSP_URL=${DEPTH_SENSOR_RTSP_URL:-<unset>}"
fi
echo "ENABLE_CAMERA=$ENABLE_CAMERA"
if [[ "$ENABLE_CAMERA" == "true" || "$ENABLE_CAMERA" == "1" ]]; then
  echo "CAMERA_DEVICE=$CAMERA_DEVICE"
  echo "CAMERA_TOPIC=$CAMERA_TOPIC"
  echo "CAMERA_INFO_TOPIC=$CAMERA_INFO_TOPIC"
  echo "CAMERA_RATE_HZ=$CAMERA_RATE_HZ"
  echo "CAMERA_JPEG_QUALITY=$CAMERA_JPEG_QUALITY"
fi
echo "============================================================"

need_cmd python3
if [[ "$DEPTH_SENSOR_RTSP_ENABLE" == "true" || "$DEPTH_SENSOR_RTSP_ENABLE" == "1" ]]; then
  need_cmd ffmpeg
fi

if [[ -f "$XLE_ROBOT_VENV/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "$XLE_ROBOT_VENV/bin/activate"
  ok "python venv activated"
else
  warn "python venv not found: $XLE_ROBOT_VENV"
fi

python3 - <<'PY'
import importlib.util
import os
import sys

required = ["numpy", "serial"]
if os.environ.get("ENABLE_BASE", "true").lower() == "true":
    required.append("lerobot")
if os.environ.get("ENABLE_DEPTH_SENSOR", "true").lower() == "true":
    required.append("pyrealsense2")
if os.environ.get("ENABLE_FAST_ZMQ", "true").lower() in ("1", "true", "yes", "on"):
    required.extend(["zmq", "msgpack"])
missing = [name for name in required if importlib.util.find_spec(name) is None]
if missing:
    print("[err] missing Python modules: " + ", ".join(missing))
    sys.exit(1)
print("[ ok ] Python modules: " + ", ".join(required))
if os.environ.get("ENABLE_DEPTH_SENSOR", "true").lower() == "true":
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        devices = list(ctx.query_devices())
        wanted = os.environ.get("DEPTH_SENSOR_SERIAL", "").strip()
        if wanted:
            matched = [dev for dev in devices if dev.get_info(rs.camera_info.serial_number) == wanted]
            if matched:
                print("[ ok ] depth sensor serial found: " + wanted)
            else:
                print("[warn] depth sensor serial not found now: " + wanted)
        elif devices:
            names = []
            for dev in devices:
                try:
                    names.append(
                        dev.get_info(rs.camera_info.name)
                        + ":"
                        + dev.get_info(rs.camera_info.serial_number)
                    )
                except Exception:
                    names.append("<unknown>")
            print("[ ok ] depth sensor SDK devices: " + ", ".join(names))
        else:
            print("[warn] no depth sensor SDK device found now")
    except Exception as exc:
        print("[warn] depth sensor device check failed: " + str(exc))
if os.environ.get("ENABLE_CAMERA", "false").lower() == "true":
    if importlib.util.find_spec("cv2") is None:
        print("[err] cv2 missing; install python3-opencv or keep ENABLE_CAMERA=false")
        sys.exit(1)
    print("[ ok ] Python module: cv2")
else:
    print("[ ok ] camera disabled")
PY
if [[ $? -ne 0 ]]; then FAIL=1; fi

if [[ "$ENABLE_BASE" == "true" ]]; then
  if [[ -e "$XLEROBOT_PORT1" ]]; then ok "xlerobot bus1 left/head exists: $XLEROBOT_PORT1"; else warn "xlerobot bus1 left/head not found yet: $XLEROBOT_PORT1"; fi
  if [[ -e "$XLEROBOT_PORT2" ]]; then ok "xlerobot bus2 right/base exists: $XLEROBOT_PORT2"; else warn "xlerobot bus2 right/base not found yet: $XLEROBOT_PORT2"; fi
else
  ok "base disabled"
fi
if [[ "$ENABLE_LIDAR" == "true" ]]; then
  if [[ -e "$LIDAR_SERIAL" ]]; then ok "lidar serial exists: $LIDAR_SERIAL"; else warn "lidar serial not found yet: $LIDAR_SERIAL"; fi
else
  ok "lidar disabled"
fi
if [[ "$ENABLE_DEPTH_SENSOR" == "true" ]]; then
  if command -v lsusb >/dev/null 2>&1 && lsusb | grep -Eiq 'Intel|RealSense|8086'; then
    ok "USB depth sensor device visible"
  else
    warn "USB depth sensor device not visible in lsusb yet"
  fi
else
  ok "depth sensor disabled"
fi
if [[ "$ENABLE_CAMERA" == "true" ]]; then
  if [[ -e "$CAMERA_DEVICE" ]]; then ok "camera device exists: $CAMERA_DEVICE"; else warn "camera device not found: $CAMERA_DEVICE"; fi
fi

if id -nG "$USER" | grep -qw dialout; then ok "user is in dialout group"; else warn "user is not in dialout group yet"; fi
if [[ "$ENABLE_CAMERA" == "true" || "$ENABLE_DEPTH_SENSOR" == "true" ]]; then
  if id -nG "$USER" | grep -qw video; then ok "user is in video group"; else warn "user is not in video group yet"; fi
fi

ok "state/command/RPC exposed on fast ZMQ ${FAST_ZMQ_BIND_HOST}:${FAST_ZMQ_PUB_PORT}/${FAST_ZMQ_PULL_PORT:-8856}/${FAST_ZMQ_REP_PORT:-8857} (compute-side indory_pi_bridge connects directly)"

echo "============================================================"
if [[ $FAIL -eq 0 ]]; then
  ok "robot I/O environment is ready enough"
else
  bad "robot I/O environment is incomplete"
fi

exit "$FAIL"
