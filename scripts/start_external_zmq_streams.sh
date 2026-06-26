#!/usr/bin/env bash
# Start/reset the external camera ZMQ contract:
# - motor-priority H.264 RGB on 8866
# - RGB-D on 8867 disabled unless explicitly overridden
# - wrist RGB uses the same H.264/fMP4 optimized path as front
# - legacy binary RGB-D TCP disabled

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-restart}"

exec env \
  RGB_RGBD_NICE="${RGB_RGBD_NICE:-12}" \
  RGB_OPT_FPS="${RGB_OPT_FPS:-15}" \
  RGB_OPT_JPEG_QUALITY="${RGB_OPT_JPEG_QUALITY:-70}" \
  RGB_WIRE_FORMAT="${RGB_WIRE_FORMAT:-h264_fmp4}" \
  RGB_H264_CRF="${RGB_H264_CRF:-31}" \
  RGB_H264_KEYINT_FRAMES="${RGB_H264_KEYINT_FRAMES:-15}" \
  RGB_H264_INIT_INTERVAL_FRAMES="${RGB_H264_INIT_INTERVAL_FRAMES:-30}" \
  VIDEO_RTSP_ENABLE=false \
  WRIST_OPT_FPS="${WRIST_OPT_FPS:-12}" \
  WRIST_LEFT_OPT_FPS="${WRIST_LEFT_OPT_FPS:-12}" \
  WRIST_RIGHT_OPT_FPS="${WRIST_RIGHT_OPT_FPS:-12}" \
  FLOOR_CAMERA_DEVICE=none \
  FLOOR_OPT_FPS="${FLOOR_OPT_FPS:-8}" \
  FAST_CAMERA_FORCE_MJPEG_COPY=0 \
  WRIST_LEGACY_JPEG_ENABLE=false \
  WRIST_OPT_JPEG_QUALITY="${WRIST_OPT_JPEG_QUALITY:-55}" \
  RGBD_CAPTURE_FPS="${RGBD_CAPTURE_FPS:-15}" \
  RGBD_BINARY_ENABLE=false \
  RGBD_BINARY_FPS="${RGBD_BINARY_FPS:-0}" \
  RGBD_ZMQ_ENABLE=false \
  RGBD_ZMQ_FPS="${RGBD_ZMQ_FPS:-5}" \
  RGBD_ZMQ_COLOR_MODE=jpeg \
  WRIST_H264_CRF="${WRIST_H264_CRF:-31}" \
  WRIST_H264_KEYINT_FRAMES="${WRIST_H264_KEYINT_FRAMES:-15}" \
  WRIST_H264_INIT_INTERVAL_FRAMES="${WRIST_H264_INIT_INTERVAL_FRAMES:-30}" \
  WRIST_WIRE_FORMAT="${WRIST_WIRE_FORMAT:-h264_fmp4}" \
  "$ROOT_DIR/scripts/indoory_live_stack.sh" "$action"
