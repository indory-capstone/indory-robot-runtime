#!/usr/bin/env python3
"""Combined optimized RGB ZMQ and binary RGB-D TCP side channel.

This process owns the RealSense once, then publishes both front RGB and RGB-D.
Wrist RGB can be proxied from the local camera publisher into the same external
RGB PUB socket.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import shutil
import socket
import struct
import subprocess
import threading
from pathlib import Path
import time
from typing import Any

import cv2
import msgpack
import numpy as np
import pyrealsense2 as rs
import zmq

try:
    import zstandard as zstd
except Exception:  # pragma: no cover - optional but required for zstd16 mode
    zstd = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="tcp://0.0.0.0:8866")
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--wrist-source", default="tcp://127.0.0.1:8864")
    parser.add_argument("--wrist-topic", action="append", default=[])
    parser.add_argument("--wrist-capture-mode", choices=("proxy", "direct", "disabled"), default="proxy")
    parser.add_argument("--wrist-jpeg-bind", default="")
    parser.add_argument("--wrist-left-device", default="")
    parser.add_argument("--wrist-left-input-format", default="MJPG")
    parser.add_argument("--wrist-left-flip", default="none")
    parser.add_argument("--wrist-right-device", default="")
    parser.add_argument("--wrist-right-input-format", default="MJPG")
    parser.add_argument("--wrist-right-flip", default="none")
    parser.add_argument("--floor-device", default="")
    parser.add_argument("--floor-input-format", default="MJPG")
    parser.add_argument("--floor-flip", default="none")
    parser.add_argument("--floor-topic", default="")
    parser.add_argument("--rgb-max-fps", type=float, default=15.0)
    parser.add_argument("--rgb-width", type=int, default=640)
    parser.add_argument("--rgb-height", type=int, default=480)
    parser.add_argument("--rgb-jpeg-quality", type=int, default=70)
    parser.add_argument("--rgb-wire-format", choices=("h264_fmp4", "jpeg"), default="h264_fmp4")
    parser.add_argument("--wrist-wire-format", choices=("h264_fmp4", "jpeg", "auto"), default="auto")
    parser.add_argument("--rtsp-publish-base", default="")
    parser.add_argument("--rtsp-publish-transport", choices=("tcp", "udp"), default="tcp")
    parser.add_argument("--rtsp-head-path", default="xlerobot_head")
    parser.add_argument("--rtsp-wrist-left-path", default="xlerobot_wrist_left")
    parser.add_argument("--rtsp-wrist-right-path", default="xlerobot_wrist_right")
    parser.add_argument("--rtsp-floor-path", default="xlerobot_floor")
    parser.add_argument("--rtsp-h264-crf", type=int, default=28)
    parser.add_argument("--rtsp-h264-keyint-frames", type=int, default=15)
    parser.add_argument("--h264-crf", type=int, default=30)
    parser.add_argument("--h264-keyint-frames", type=int, default=4)
    parser.add_argument("--h264-init-interval-frames", type=int, default=30)
    parser.add_argument("--h264-input-format", choices=("rgb24", "bgr24"), default="rgb24")
    parser.add_argument("--wrist-h264-keyint-frames", type=int, default=0)
    parser.add_argument("--wrist-h264-init-interval-frames", type=int, default=0)
    parser.add_argument("--wrist-max-fps", type=float, default=15.0)
    parser.add_argument("--wrist-left-max-fps", type=float, default=0.0)
    parser.add_argument("--wrist-right-max-fps", type=float, default=0.0)
    parser.add_argument("--floor-max-fps", type=float, default=0.0)
    parser.add_argument("--wrist-width", type=int, default=640)
    parser.add_argument("--wrist-height", type=int, default=480)
    parser.add_argument("--wrist-jpeg-quality", type=int, default=65)
    parser.add_argument("--wrist-h264-crf", type=int, default=30)
    parser.add_argument("--binary-host", default="127.0.0.1")
    parser.add_argument("--binary-port", type=int, default=9102)
    parser.add_argument("--binary-enable", action="store_true", default=False)
    parser.add_argument("--binary-fps", type=float, default=15.0)
    parser.add_argument("--binary-jpeg-quality", type=int, default=85)
    parser.add_argument("--binary-width", type=int, default=640)
    parser.add_argument("--binary-height", type=int, default=480)
    parser.add_argument("--rgbd-zmq-enable", action="store_true", default=False)
    parser.add_argument("--rgbd-zmq-bind", default="tcp://0.0.0.0:8867")
    parser.add_argument("--rgbd-zmq-topic", default="")
    parser.add_argument("--rgbd-zmq-fps", type=float, default=10.0)
    parser.add_argument("--rgbd-zmq-color-mode", choices=("reference", "jpeg"), default="jpeg")
    parser.add_argument("--depth-format", choices=("raw16", "png16", "zstd16", "rvl16"), default="zstd16")
    parser.add_argument("--png-compress", type=int, default=1)
    parser.add_argument("--zstd-level", type=int, default=1)
    parser.add_argument("--color-width", type=int, default=640)
    parser.add_argument("--color-height", type=int, default=480)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--depth-filter-mode", choices=("off", "balanced", "accurate"), default="accurate")
    parser.add_argument("--depth-min-m", type=float, default=0.25)
    parser.add_argument("--depth-max-m", type=float, default=5.0)
    parser.add_argument("--depth-visual-preset", type=int, default=3)
    parser.add_argument("--depth-laser-power", type=float, default=150.0)
    parser.add_argument("--depth-enable-emitter", type=int, choices=(-1, 0, 1), default=1)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--stats-interval-s", type=float, default=10.0)
    return parser.parse_args()


class H264Fmp4Encoder:
    """Low-latency H.264 fragmented MP4 encoder for one RGB topic."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        fps: float,
        crf: int,
        keyint_frames: int,
        init_interval_frames: int,
        input_format: str = "rgb24",
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1.0, float(fps))
        self.crf = int(crf)
        self.keyint_frames = max(1, int(keyint_frames))
        self.init_interval_frames = max(1, int(init_interval_frames))
        self.input_format = str(input_format or "rgb24").strip().lower()
        if self.input_format not in ("rgb24", "bgr24"):
            self.input_format = "rgb24"
        self.ffmpeg = os.environ.get("INDOORY_FFMPEG") or shutil.which("ffmpeg")
        self.seq = 0
        self.init_segment = b""
        self._proc: subprocess.Popen | None = None
        self._last_stderr = b""
        self._stdout_pending = b""

    def encode_bgr(self, frame_bgr: np.ndarray) -> dict[str, Any] | None:
        if self.ffmpeg is None:
            raise FileNotFoundError("ffmpeg")
        if frame_bgr.shape[:2] != (self.height, self.width):
            frame_bgr = cv2.resize(frame_bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
        if self.input_format == "bgr24":
            frame = np.ascontiguousarray(frame_bgr, dtype=np.uint8)
        else:
            frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            frame = np.ascontiguousarray(frame, dtype=np.uint8)
        proc = self._ensure_process()
        assert proc.stdin is not None
        try:
            proc.stdin.write(memoryview(frame).cast("B"))
            proc.stdin.flush()
        except (BrokenPipeError, OSError):
            self.close()
            proc = self._ensure_process()
            assert proc.stdin is not None
            proc.stdin.write(memoryview(frame).cast("B"))
            proc.stdin.flush()
        assert proc.stdout is not None
        assert proc.stderr is not None
        deadline = time.monotonic() + 0.040
        chunk = b""
        while True:
            self._last_stderr = (self._last_stderr + self._read_available(proc.stderr))[-4096:]
            chunk = self._read_complete_boxes(proc.stdout)
            if chunk:
                break
            if proc.poll() is not None:
                err = self._last_stderr.decode("utf-8", "replace").strip()
                raise RuntimeError(f"ffmpeg exited without H.264 chunk: {err}")
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.001)
        init, media = split_fmp4_init_media(chunk)
        if init:
            self.init_segment = init
        if not media:
            return None
        self.seq += 1
        init_out = (
            self.init_segment
            if self.init_segment
            and (
                init
                or self.seq == 1
                or self.init_interval_frames <= 1
                or self.seq % self.init_interval_frames == 0
            )
            else b""
        )
        return {"data": media, "init": init_out, "chunk_seq": self.seq}

    def close(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            return
        for stream in (proc.stdin, proc.stdout, proc.stderr):
            try:
                if stream is not None:
                    stream.close()
            except Exception:
                pass
        if proc.poll() is None:
            proc.terminate()
            deadline = time.monotonic() + 1.0
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.02)
            if proc.poll() is None:
                proc.kill()

    def _ensure_process(self) -> subprocess.Popen:
        if self._proc is not None and self._proc.poll() is None:
            return self._proc
        assert self.ffmpeg is not None
        fps_text = f"{self.fps:g}"
        gop = str(max(1, self.keyint_frames))
        x264_params = ":".join(
            [
                f"keyint={gop}",
                f"min-keyint={gop}",
                "scenecut=0",
                "sync-lookahead=0",
                "rc-lookahead=0",
                "sliced-threads=1",
            ]
        )
        cmd = [
            self.ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            self.input_format,
            "-video_size",
            f"{self.width}x{self.height}",
            "-framerate",
            fps_text,
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-profile:v",
            "baseline",
            "-threads",
            "1",
            "-x264-params",
            x264_params,
            "-pix_fmt",
            "yuv420p",
            "-g",
            gop,
            "-bf",
            "0",
            "-sc_threshold",
            "0",
            "-crf",
            str(int(self.crf)),
            "-movflags",
            "empty_moov+default_base_moof+frag_every_frame",
            "-flush_packets",
            "1",
            "-f",
            "mp4",
            "pipe:1",
        ]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        assert self._proc.stdout is not None
        assert self._proc.stderr is not None
        os.set_blocking(self._proc.stdout.fileno(), False)
        os.set_blocking(self._proc.stderr.fileno(), False)
        return self._proc

    def _read_available(self, stream: Any) -> bytes:
        chunks: list[bytes] = []
        while True:
            try:
                chunk = stream.read(65536)
            except (BlockingIOError, OSError):
                break
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)

    def _read_complete_boxes(self, stream: Any) -> bytes:
        incoming = self._read_available(stream)
        if incoming:
            self._stdout_pending += incoming
        data = self._stdout_pending
        offset = 0
        emit_end = 0

        def parse_box(pos: int) -> tuple[int, bytes, int] | None:
            if pos + 8 > len(data):
                return None
            size = int.from_bytes(data[pos:pos + 4], "big")
            box_type = data[pos + 4:pos + 8]
            header_size = 8
            if size == 1:
                if pos + 16 > len(data):
                    return None
                size = int.from_bytes(data[pos + 8:pos + 16], "big")
                header_size = 16
            elif size == 0:
                return None
            if size < header_size or pos + size > len(data):
                return None
            return size, box_type, pos + size

        while True:
            parsed = parse_box(offset)
            if parsed is None:
                break
            size, box_type, next_offset = parsed
            if box_type == b"moof":
                parsed_next = parse_box(next_offset)
                if parsed_next is None:
                    break
                next_size, next_type, after_mdat = parsed_next
                if next_type != b"mdat":
                    break
                offset = after_mdat
                emit_end = offset
                continue
            if box_type == b"mdat":
                # Never emit an orphan media-data box; keep it until the stream
                # can be resynchronized by the next complete moof+mdat pair.
                break
            offset = next_offset
            emit_end = offset
        complete = data[:emit_end]
        self._stdout_pending = data[emit_end:]
        return complete



class H264RtspPublisher(threading.Thread):
    """Queue-latest raw BGR frames into ffmpeg H.264 RTSP publishing."""

    def __init__(
        self,
        *,
        name: str,
        url: str,
        width: int,
        height: int,
        fps: float,
        crf: int,
        keyint_frames: int,
        transport: str,
    ) -> None:
        super().__init__(name=f"rtsp-publisher-{name}", daemon=True)
        self.camera_name = name
        self.url = str(url or "").strip()
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1.0, float(fps))
        self.crf = int(crf)
        self.keyint_frames = max(1, int(keyint_frames))
        self.transport = "udp" if str(transport).lower() == "udp" else "tcp"
        self.queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.ffmpeg = os.environ.get("INDOORY_FFMPEG") or shutil.which("ffmpeg")
        self.proc: subprocess.Popen | None = None
        self.last_error_log = 0.0
        self.frames_in = 0
        self.frames_written = 0
        self.frames_dropped = 0
        if self.url:
            self.start()

    def submit(self, frame_bgr: np.ndarray) -> None:
        if not self.url or self.stop_event.is_set():
            return
        if frame_bgr.shape[:2] != (self.height, self.width):
            frame_bgr = cv2.resize(frame_bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)
        frame = np.ascontiguousarray(frame_bgr, dtype=np.uint8)
        self.frames_in += 1
        try:
            self.queue.put_nowait(frame)
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.frames_dropped += 1
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(frame)
            except queue.Full:
                self.frames_dropped += 1

    def close(self) -> None:
        self.stop_event.set()
        self._close_proc()

    def run(self) -> None:
        if not self.url:
            return
        if self.ffmpeg is None:
            self._log_error("ffmpeg not found; RTSP disabled")
            return
        while not self.stop_event.is_set():
            try:
                frame = self.queue.get(timeout=0.25)
            except queue.Empty:
                continue
            proc = self._ensure_proc()
            if proc is None or proc.stdin is None:
                time.sleep(0.2)
                continue
            try:
                proc.stdin.write(memoryview(frame).cast("B"))
                proc.stdin.flush()
                self.frames_written += 1
                if self.frames_written % 120 == 0:
                    print(
                        f"[rgb_rgbd_sidechannel] rtsp {self.camera_name} frames={self.frames_written} dropped={self.frames_dropped} url={self.url}",
                        flush=True,
                    )
            except (BrokenPipeError, OSError) as exc:
                self._log_error(f"rtsp {self.camera_name} write failed: {exc}")
                self._close_proc()

    def _ensure_proc(self) -> subprocess.Popen | None:
        if self.proc is not None and self.proc.poll() is None:
            return self.proc
        assert self.ffmpeg is not None
        gop = str(max(1, self.keyint_frames))
        x264_params = ":".join([
            f"keyint={gop}",
            f"min-keyint={gop}",
            "scenecut=0",
            "sync-lookahead=0",
            "rc-lookahead=0",
            "sliced-threads=1",
        ])
        cmd = [
            self.ffmpeg,
            "-hide_banner",
            "-loglevel", "warning",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-video_size", f"{self.width}x{self.height}",
            "-framerate", f"{self.fps:g}",
            "-i", "pipe:0",
            "-an",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-profile:v", "baseline",
            "-threads", "1",
            "-x264-params", x264_params,
            "-pix_fmt", "yuv420p",
            "-g", gop,
            "-bf", "0",
            "-sc_threshold", "0",
            "-crf", str(self.crf),
            "-rtsp_transport", self.transport,
            "-f", "rtsp",
            self.url,
        ]
        try:
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
            print(f"[rgb_rgbd_sidechannel] rtsp {self.camera_name} -> {self.url}", flush=True)
            return self.proc
        except Exception as exc:
            self._log_error(f"rtsp {self.camera_name} start failed: {exc}")
            self.proc = None
            return None

    def _close_proc(self) -> None:
        proc = self.proc
        self.proc = None
        if proc is None:
            return
        try:
            if proc.stdin is not None:
                proc.stdin.close()
        except Exception:
            pass
        if proc.poll() is None:
            proc.terminate()
            deadline = time.monotonic() + 1.0
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.02)
            if proc.poll() is None:
                proc.kill()

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self.last_error_log >= 2.0:
            print(f"[rgb_rgbd_sidechannel] {msg}", flush=True)
            self.last_error_log = now


def _rtsp_publish_url(args: argparse.Namespace, name: str) -> str:
    base = str(getattr(args, "rtsp_publish_base", "") or "").strip().rstrip("/")
    if not base:
        return ""
    key = name.replace("-", "_")
    path = str(getattr(args, f"rtsp_{key}_path", "") or key).strip("/ ")
    if not path:
        return ""
    return f"{base}/{path}"

def split_fmp4_init_media(data: bytes) -> tuple[bytes, bytes]:
    if not data:
        return b"", b""
    offset = 0
    while offset + 8 <= len(data):
        size = int.from_bytes(data[offset:offset + 4], "big")
        box_type = data[offset + 4:offset + 8]
        if size == 1:
            if offset + 16 > len(data):
                break
            size = int.from_bytes(data[offset + 8:offset + 16], "big")
        if size <= 0 or offset + size > len(data):
            break
        if box_type == b"moof":
            return data[:offset], data[offset:]
        offset += size
    return data[:offset], b""


def stamp_from_ns(stamp_ns: int) -> dict[str, int]:
    return {"sec": int(stamp_ns // 1_000_000_000), "nanosec": int(stamp_ns % 1_000_000_000)}




class RealSenseFrameClock:
    """Convert RealSense frame timestamps into ROS-compatible wall-clock ns."""

    def __init__(self) -> None:
        self._offset_ns: int | None = None

    def stamp_ns(self, frame: Any) -> int:
        now_ns = time.time_ns()
        try:
            ts_ms = float(frame.get_timestamp())
        except Exception:
            return now_ns
        if not np.isfinite(ts_ms) or ts_ms <= 0.0:
            return now_ns
        raw_ns = int(ts_ms * 1_000_000.0)
        # Some RealSense timestamp domains are already system/global time.
        # If the raw timestamp is close to wall time, preserve it exactly.
        if abs(raw_ns - now_ns) <= 60_000_000_000:
            self._offset_ns = 0
            return raw_ns
        # Hardware-clock timestamps are camera-relative, so anchor the clock once
        # and then preserve RealSense frame timing instead of packet-build timing.
        if self._offset_ns is None:
            self._offset_ns = now_ns - raw_ns
        stamp_ns = int(self._offset_ns + raw_ns)
        if stamp_ns > now_ns + 1_000_000_000 or now_ns - stamp_ns > 5_000_000_000:
            self._offset_ns = now_ns - raw_ns
            stamp_ns = now_ns
        return stamp_ns
def camera_info_from_frame(frame: Any, st: dict[str, int], frame_id: str) -> dict[str, Any]:
    intr = frame.profile.as_video_stream_profile().get_intrinsics()
    return {
        "header": {"stamp": st, "frame_id": frame_id},
        "height": int(intr.height),
        "width": int(intr.width),
        "distortion_model": "plumb_bob",
        "d": [float(x) for x in intr.coeffs[:5]],
        "k": [
            float(intr.fx), 0.0, float(intr.ppx),
            0.0, float(intr.fy), float(intr.ppy),
            0.0, 0.0, 1.0,
        ],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [
            float(intr.fx), 0.0, float(intr.ppx), 0.0,
            0.0, float(intr.fy), float(intr.ppy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ],
    }


def encode_jpeg(image: np.ndarray, quality: int) -> bytes | None:
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    return encoded.tobytes()


def encode_rvl_depth(depth: np.ndarray) -> bytes:
    values = np.ascontiguousarray(depth.astype(np.uint16, copy=False)).reshape(-1)
    words: list[int] = []
    word = 0
    nibbles = 0

    def encode_nibble(nibble: int) -> None:
        nonlocal word, nibbles
        word = ((word << 4) | (int(nibble) & 0xF)) & 0xFFFFFFFF
        nibbles += 1
        if nibbles == 8:
            words.append(word)
            word = 0
            nibbles = 0

    def encode_vle(value: int) -> None:
        value = int(value)
        while True:
            nibble = value & 0x7
            value >>= 3
            if value:
                nibble |= 0x8
            encode_nibble(nibble)
            if not value:
                break

    total = int(values.size)
    i = 0
    previous = 0
    while i < total:
        zeros = 0
        while i < total and int(values[i]) == 0:
            zeros += 1
            i += 1
        encode_vle(zeros)

        start = i
        while i < total and int(values[i]) != 0:
            i += 1
        nonzeros = i - start
        encode_vle(nonzeros)

        for j in range(start, i):
            current = int(values[j])
            delta = current - previous
            positive = (delta << 1) ^ (delta >> 31)
            encode_vle(positive)
            previous = current

    if nibbles:
        words.append((word << (4 * (8 - nibbles))) & 0xFFFFFFFF)
    return b''.join(int(w).to_bytes(4, 'little', signed=False) for w in words)


def resize_color_depth(
    color: np.ndarray,
    depth: np.ndarray,
    *,
    width: int,
    height: int,
) -> tuple[np.ndarray, np.ndarray]:
    if width <= 0 or height <= 0:
        return color, depth
    color_h, color_w = color.shape[:2]
    depth_h, depth_w = depth.shape[:2]
    if color_w != width or color_h != height:
        color = cv2.resize(color, (width, height), interpolation=cv2.INTER_AREA)
    if depth_w != width or depth_h != height:
        depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
    return color, depth


def resize_bgr(
    image: np.ndarray,
    *,
    width: int,
    height: int,
) -> tuple[np.ndarray, int, int]:
    original_height, original_width = image.shape[:2]
    out_image = image
    if width > 0 and height > 0 and (original_width != width or original_height != height):
        out_image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    return out_image, int(original_width), int(original_height)


def jpeg_rgb_payload(
    *,
    topic: str,
    image: np.ndarray,
    robot_id: int,
    frame_id: str,
    width: int,
    height: int,
    jpeg_quality: int,
    seq: int,
    extra: dict[str, Any] | None = None,
) -> bytes | None:
    out_image, original_width, original_height = resize_bgr(image, width=width, height=height)
    data = encode_jpeg(out_image, jpeg_quality)
    if data is None:
        return None
    payload: dict[str, Any] = {
        "schema": "indoory_camera_zmq_v1",
        "topic": topic,
        "robot_id": int(robot_id),
        "stamp_ns": time.time_ns(),
        "encoding": "jpeg",
        "width": int(out_image.shape[1]),
        "height": int(out_image.shape[0]),
        "frame_id": frame_id,
        "seq": int(seq),
        "data": data,
        "optimized": True,
        "original_width": int(original_width),
        "original_height": int(original_height),
        "proxy_stamp_ns": time.time_ns(),
    }
    if extra:
        payload.update(extra)
    return msgpack.packb(payload, use_bin_type=True)


def jpeg_bytes_payload(
    *,
    topic: str,
    data: bytes,
    robot_id: int,
    frame_id: str,
    width: int,
    height: int,
    seq: int,
    extra: dict[str, Any] | None = None,
) -> bytes:
    payload: dict[str, Any] = {
        "schema": "indoory_camera_zmq_v1",
        "topic": topic,
        "robot_id": int(robot_id),
        "stamp_ns": time.time_ns(),
        "encoding": "jpeg",
        "width": int(width),
        "height": int(height),
        "frame_id": frame_id,
        "seq": int(seq),
        "data": bytes(data),
        "optimized": True,
        "original_width": int(width),
        "original_height": int(height),
        "proxy_stamp_ns": time.time_ns(),
    }
    if extra:
        payload.update(extra)
    return msgpack.packb(payload, use_bin_type=True)


def h264_rgb_payload(
    *,
    topic: str,
    image: np.ndarray,
    robot_id: int,
    frame_id: str,
    width: int,
    height: int,
    seq: int,
    encoder: H264Fmp4Encoder,
    extra: dict[str, Any] | None = None,
) -> bytes | None:
    out_image, original_width, original_height = resize_bgr(image, width=width, height=height)
    encoded = encoder.encode_bgr(out_image)
    if encoded is None:
        return None
    payload: dict[str, Any] = {
        "schema": "indoory_camera_zmq_v1",
        "topic": topic,
        "robot_id": int(robot_id),
        "stamp_ns": time.time_ns(),
        "encoding": "h264_fmp4",
        "codec": "h264",
        "container": "mp4",
        "source_encoding": "bgr8",
        "encoder_input_format": getattr(encoder, "input_format", "rgb24"),
        "video_pixel_format": "yuv420p",
        "width": int(out_image.shape[1]),
        "height": int(out_image.shape[0]),
        "frame_id": frame_id,
        "seq": int(seq),
        "chunk_seq": int(encoded["chunk_seq"]),
        "data": bytes(encoded["data"]),
        "optimized": True,
        "original_width": int(original_width),
        "original_height": int(original_height),
        "proxy_stamp_ns": time.time_ns(),
    }
    init = bytes(encoded.get("init") or b"")
    if init:
        payload["init"] = init
    if extra:
        payload.update(extra)
    return msgpack.packb(payload, use_bin_type=True)


def rgb_payload(
    *,
    topic: str,
    image: np.ndarray,
    robot_id: int,
    frame_id: str,
    width: int,
    height: int,
    jpeg_quality: int,
    seq: int,
    wire_format: str,
    encoder: H264Fmp4Encoder | None,
    extra: dict[str, Any] | None = None,
) -> bytes | None:
    if wire_format == "h264_fmp4":
        if encoder is None:
            return None
        return h264_rgb_payload(
            topic=topic,
            image=image,
            robot_id=robot_id,
            frame_id=frame_id,
            width=width,
            height=height,
            seq=seq,
            encoder=encoder,
            extra=extra,
        )
    return jpeg_rgb_payload(
        topic=topic,
        image=image,
        robot_id=robot_id,
        frame_id=frame_id,
        width=width,
        height=height,
        jpeg_quality=jpeg_quality,
        seq=seq,
        extra=extra,
    )


def decode_existing_payload_image(topic: str, payload_raw: bytes) -> tuple[np.ndarray, dict[str, Any]] | None:
    payload = msgpack.unpackb(payload_raw, raw=False)
    if not isinstance(payload, dict) or payload.get("encoding") != "jpeg":
        return None
    data = payload.get("data")
    if not isinstance(data, (bytes, bytearray)):
        return None
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        return None
    payload["topic"] = topic
    payload["original_bytes"] = len(data)
    return image, payload


class BinaryRgbdSender(threading.Thread):
    def __init__(self, host: str, port: int, fps: float) -> None:
        super().__init__(name="combined-rgbd-tcp", daemon=True)
        self.host = host.strip()
        self.port = int(port)
        self.fps = max(0.0, float(fps))
        self.queue: queue.Queue[tuple[dict[str, Any], bytes]] = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.sock: socket.socket | None = None
        self.last_submit = 0.0
        self.last_error_log = 0.0
        self.frames_sent = 0
        self.bytes_sent = 0

    def submit(self, header: dict[str, Any], payload: bytes) -> None:
        if not self.host or self.port <= 0:
            return
        now = time.monotonic()
        if self.fps > 0 and now - self.last_submit < 1.0 / self.fps:
            return
        self.last_submit = now
        try:
            self.queue.put_nowait((header, payload))
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait((header, payload))
            except queue.Full:
                pass

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                header, payload = self.queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if self.sock is None and not self._connect():
                continue
            try:
                header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
                packet = struct.pack("!I", len(header_bytes)) + header_bytes + payload
                assert self.sock is not None
                self.sock.sendall(packet)
                self.frames_sent += 1
                self.bytes_sent += len(packet)
                if self.frames_sent % 50 == 0:
                    mb = self.bytes_sent / 1_000_000.0
                    print(f"[rgb_rgbd_sidechannel] binary frames={self.frames_sent} bytes={mb:.1f}MB", flush=True)
            except Exception as exc:
                self._log_error(f"binary send failed: {exc}")
                self._close()

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=2.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(None)
            self.sock = sock
            print(f"[rgb_rgbd_sidechannel] binary connected tcp://{self.host}:{self.port}", flush=True)
            return True
        except Exception as exc:
            self._log_error(f"binary connect failed: {exc}")
            self._close()
            time.sleep(0.5)
            return False

    def _close(self) -> None:
        sock = self.sock
        self.sock = None
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self.last_error_log >= 2.0:
            print(f"[rgb_rgbd_sidechannel] {msg}", flush=True)
            self.last_error_log = now


def _is_enabled_device(device: str) -> bool:
    value = str(device or "").strip()
    return bool(value) and value.lower() not in {"none", "off", "false", "0", "disabled"}


def _resolve_device(device: str) -> str:
    try:
        return str(Path(device).resolve(strict=True))
    except OSError:
        return str(device)


def _configure_v4l2_capture(cap: Any, *, input_format: str, width: int, height: int, fps: float) -> None:
    fmt = str(input_format or "MJPG").strip().upper()
    if len(fmt) != 4:
        fmt = "MJPG"
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    cap.set(cv2.CAP_PROP_FPS, float(fps))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def _apply_flip(frame: np.ndarray, mode: str) -> np.ndarray:
    key = str(mode or "none").strip().lower().replace("-", "_")
    code = {
        "none": None,
        "off": None,
        "false": None,
        "0": None,
        "vertical": 0,
        "v": 0,
        "y": 0,
        "up_down": 0,
        "top_bottom": 0,
        "horizontal": 1,
        "h": 1,
        "x": 1,
        "left_right": 1,
        "both": -1,
        "xy": -1,
        "180": -1,
    }.get(key)
    return frame if code is None else cv2.flip(frame, code)


class FfmpegMjpegCamera(threading.Thread):
    def __init__(
        self,
        args: argparse.Namespace,
        *,
        name: str,
        topic: str,
        device: str,
        input_format: str,
        flip: str,
        fps: float,
        pub: zmq.Socket,
        pub_lock: threading.Lock,
        legacy_pub: zmq.Socket | None,
        legacy_lock: threading.Lock | None,
        outputs: tuple[tuple[str, str, str], ...] = (),
    ) -> None:
        super().__init__(name=f"ffmpeg-mjpeg-{name}", daemon=True)
        self.args = args
        self.camera_name = name
        self.topic = topic
        self.device = device
        self.input_format = input_format
        self.flip = flip
        self.fps = max(0.1, float(fps))
        self.outputs = outputs or ((name, topic, flip),)
        self.pub = pub
        self.pub_lock = pub_lock
        self.legacy_pub = legacy_pub
        self.legacy_lock = legacy_lock
        self.stop_event = threading.Event()
        self.ffmpeg = os.environ.get("INDOORY_FFMPEG") or shutil.which("ffmpeg")
        self.stats = {
            "frames": 0,
            "sent": 0,
            "legacy_jpeg_sent": 0,
            "rate_drop": 0,
            "open_fail": 0,
            "read_fail": 0,
            "encode_drop": 0,
            "hwm_drop": 0,
        }
        self._last_error_log = 0.0

    def run(self) -> None:
        if not _is_enabled_device(self.device) or not self.ffmpeg:
            return
        seq = 0
        while not self.stop_event.is_set():
            device = _resolve_device(self.device)
            if not Path(device).exists():
                self._log_error(f"{self.camera_name} device does not exist: {device}")
                self.stats["open_fail"] += 1
                self.stop_event.wait(1.0)
                continue
            cmd = [
                self.ffmpeg,
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "v4l2",
                "-input_format",
                "mjpeg",
                "-video_size",
                f"{int(self.args.wrist_width)}x{int(self.args.wrist_height)}",
                "-framerate",
                f"{self.fps:g}",
                "-i",
                device,
                "-an",
                "-c:v",
                "copy",
                "-f",
                "mjpeg",
                "pipe:1",
            ]
            proc: subprocess.Popen | None = None
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                )
                assert proc.stdout is not None
                print(
                    f"[rgb_rgbd_sidechannel] direct {self.camera_name} mjpeg-copy "
                    f"device={device} topics={','.join(t for _n, t, _f in self.outputs)} "
                    f"fmt={self.input_format} "
                    f"flip={self.flip}",
                    flush=True,
                )
                pending = b""
                while not self.stop_event.is_set():
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        if proc.poll() is not None:
                            self.stats["read_fail"] += 1
                            break
                        continue
                    pending += chunk
                    while True:
                        soi = pending.find(b"\xff\xd8")
                        if soi < 0:
                            pending = pending[-1:]
                            break
                        if soi > 0:
                            pending = pending[soi:]
                        eoi = pending.find(b"\xff\xd9", 2)
                        if eoi < 0:
                            if len(pending) > 2_000_000:
                                pending = pending[-1:]
                            break
                        frame = pending[:eoi + 2]
                        pending = pending[eoi + 2:]
                        self.stats["frames"] += 1
                        now = time.monotonic()
                        seq += 1
                        for output_name, output_topic, output_flip in self.outputs:
                            extra = {
                                "camera": output_name,
                                "device": device,
                                "source": "v4l2_mjpeg_copy",
                                "flip": output_flip,
                            }
                            packed = jpeg_bytes_payload(
                                topic=output_topic,
                                data=frame,
                                robot_id=int(self.args.robot_id),
                                frame_id=f"{output_name}_rgb",
                                width=int(self.args.wrist_width),
                                height=int(self.args.wrist_height),
                                seq=seq,
                                extra=extra,
                            )
                            topic_b = output_topic.encode("utf-8")
                            try:
                                with self.pub_lock:
                                    self.pub.send_multipart([topic_b, packed], flags=zmq.NOBLOCK)
                                self.stats["sent"] += 1
                            except zmq.Again:
                                self.stats["hwm_drop"] += 1
                            if self.legacy_pub is not None and self.legacy_lock is not None:
                                try:
                                    with self.legacy_lock:
                                        self.legacy_pub.send_multipart([topic_b, packed], flags=zmq.NOBLOCK)
                                    self.stats["legacy_jpeg_sent"] += 1
                                except zmq.Again:
                                    self.stats["hwm_drop"] += 1
            except Exception as exc:
                self.stats["read_fail"] += 1
                self._log_error(f"{self.camera_name} mjpeg-copy failed: {exc}")
            finally:
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=1.0)
                    except Exception:
                        proc.kill()
            self.stop_event.wait(0.2)

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log >= 2.0:
            print(f"[rgb_rgbd_sidechannel] {msg}", flush=True)
            self._last_error_log = now


class DirectWristCamera(threading.Thread):
    def __init__(
        self,
        args: argparse.Namespace,
        *,
        name: str,
        topic: str,
        device: str,
        input_format: str,
        flip: str,
        fps: float,
        pub: zmq.Socket,
        pub_lock: threading.Lock,
        legacy_pub: zmq.Socket | None,
        legacy_lock: threading.Lock | None,
    ) -> None:
        super().__init__(name=f"direct-wrist-{name}", daemon=True)
        self.args = args
        self.camera_name = name
        self.topic = topic
        self.device = device
        self.input_format = input_format
        self.flip = flip
        self.fps = max(0.1, float(fps))
        self.pub = pub
        self.pub_lock = pub_lock
        self.legacy_pub = legacy_pub
        self.legacy_lock = legacy_lock
        self.stop_event = threading.Event()
        self.encoder = (
            H264Fmp4Encoder(
                width=int(args.wrist_width),
                height=int(args.wrist_height),
                fps=self.fps,
                crf=int(args.wrist_h264_crf),
                keyint_frames=int(args.wrist_h264_keyint_frames or args.h264_keyint_frames),
                init_interval_frames=int(args.wrist_h264_init_interval_frames or args.h264_init_interval_frames),
                input_format=str(args.h264_input_format),
            )
            if str(args.wrist_wire_format) == "h264_fmp4"
            else None
        )
        rtsp_url = _rtsp_publish_url(args, self.camera_name)
        self.rtsp_publisher = (
            H264RtspPublisher(
                name=self.camera_name,
                url=rtsp_url,
                width=int(args.wrist_width),
                height=int(args.wrist_height),
                fps=self.fps,
                crf=int(args.rtsp_h264_crf),
                keyint_frames=int(args.rtsp_h264_keyint_frames),
                transport=str(args.rtsp_publish_transport),
            )
            if rtsp_url
            else None
        )
        self.stats = {
            "frames": 0,
            "sent": 0,
            "legacy_jpeg_sent": 0,
            "rate_drop": 0,
            "open_fail": 0,
            "read_fail": 0,
            "encode_drop": 0,
            "hwm_drop": 0,
        }
        self._last_error_log = 0.0

    def run(self) -> None:
        if not _is_enabled_device(self.device):
            return
        seq = 0
        min_period = 1.0 / self.fps
        last_sent = 0.0
        while not self.stop_event.is_set():
            device = _resolve_device(self.device)
            if not Path(device).exists():
                self._log_error(f"{self.camera_name} device does not exist: {device}")
                self.stats["open_fail"] += 1
                self.stop_event.wait(1.0)
                continue
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not cap.isOpened():
                self._log_error(f"{self.camera_name} camera failed to open: {device}")
                self.stats["open_fail"] += 1
                cap.release()
                self.stop_event.wait(1.0)
                continue
            try:
                _configure_v4l2_capture(
                    cap,
                    input_format=self.input_format,
                    width=int(self.args.wrist_width),
                    height=int(self.args.wrist_height),
                    fps=self.fps,
                )
                print(
                    f"[rgb_rgbd_sidechannel] direct {self.camera_name} camera "
                    f"device={device} topic={self.topic} fmt={self.input_format} flip={self.flip}",
                    flush=True,
                )
                while not self.stop_event.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        self.stats["read_fail"] += 1
                        self._log_error(f"{self.camera_name} camera read failed; reopening")
                        self.stop_event.wait(0.2)
                        break
                    self.stats["frames"] += 1
                    now = time.monotonic()
                    if now - last_sent < min_period:
                        self.stats["rate_drop"] += 1
                        continue
                    frame = _apply_flip(frame, self.flip)
                    if self.rtsp_publisher is not None:
                        self.rtsp_publisher.submit(frame)
                    seq += 1
                    extra = {"camera": self.camera_name, "device": device, "source": "v4l2_direct"}
                    packed = rgb_payload(
                        topic=self.topic,
                        image=frame,
                        robot_id=int(self.args.robot_id),
                        frame_id=f"{self.camera_name}_rgb",
                        width=int(self.args.wrist_width),
                        height=int(self.args.wrist_height),
                        jpeg_quality=int(self.args.wrist_jpeg_quality),
                        seq=seq,
                        wire_format=str(self.args.wrist_wire_format),
                        encoder=self.encoder,
                        extra=extra,
                    )
                    if packed is None:
                        self.stats["encode_drop"] += 1
                        continue
                    topic_b = self.topic.encode("utf-8")
                    try:
                        with self.pub_lock:
                            self.pub.send_multipart([topic_b, packed], flags=zmq.NOBLOCK)
                        self.stats["sent"] += 1
                        last_sent = now
                    except zmq.Again:
                        self.stats["hwm_drop"] += 1
                    if self.legacy_pub is not None and self.legacy_lock is not None:
                        if str(self.args.wrist_wire_format) == "jpeg":
                            legacy = packed
                        else:
                            legacy = jpeg_rgb_payload(
                                topic=self.topic,
                                image=frame,
                                robot_id=int(self.args.robot_id),
                                frame_id=f"{self.camera_name}_rgb",
                                width=int(self.args.wrist_width),
                                height=int(self.args.wrist_height),
                                jpeg_quality=int(self.args.wrist_jpeg_quality),
                                seq=seq,
                                extra=extra,
                            )
                        if legacy is not None:
                            try:
                                with self.legacy_lock:
                                    self.legacy_pub.send_multipart([topic_b, legacy], flags=zmq.NOBLOCK)
                                self.stats["legacy_jpeg_sent"] += 1
                            except zmq.Again:
                                self.stats["hwm_drop"] += 1
            finally:
                cap.release()
        if self.encoder is not None:
            self.encoder.close()
        if self.rtsp_publisher is not None:
            self.rtsp_publisher.close()

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log >= 2.0:
            print(f"[rgb_rgbd_sidechannel] {msg}", flush=True)
            self._last_error_log = now


class WristProxy(threading.Thread):
    def __init__(
        self,
        args: argparse.Namespace,
        pub: zmq.Socket,
        pub_lock: threading.Lock,
        topics: tuple[str, ...],
    ) -> None:
        super().__init__(name="combined-wrist-proxy", daemon=True)
        self.args = args
        self.pub = pub
        self.pub_lock = pub_lock
        self.topics = topics
        self.stop_event = threading.Event()
        self.last_sent: dict[str, float] = {}
        self.encoders: dict[str, H264Fmp4Encoder] = {}
        self.stats = {"recv": 0, "sent": 0, "rate_drop": 0, "decode_drop": 0, "hwm_drop": 0}

    def _encoder(self, topic: str) -> H264Fmp4Encoder:
        encoder = self.encoders.get(topic)
        if encoder is None:
            encoder = H264Fmp4Encoder(
                width=int(self.args.wrist_width),
                height=int(self.args.wrist_height),
                fps=float(self.args.wrist_max_fps),
                crf=int(self.args.wrist_h264_crf),
                keyint_frames=int(self.args.h264_keyint_frames),
                init_interval_frames=int(self.args.h264_init_interval_frames),
                input_format=str(self.args.h264_input_format),
            )
            self.encoders[topic] = encoder
        return encoder

    def run(self) -> None:
        ctx = zmq.Context.instance()
        sub = ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.LINGER, 0)
        sub.setsockopt(zmq.RCVHWM, 4)
        for topic in self.topics:
            sub.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
        sub.connect(str(self.args.wrist_source))
        min_period = 1.0 / max(0.1, float(self.args.wrist_max_fps))
        while not self.stop_event.is_set():
            try:
                topic_raw, payload_raw = sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.002)
                continue
            topic = topic_raw.decode("utf-8", "replace")
            self.stats["recv"] += 1
            now = time.monotonic()
            if now - self.last_sent.get(topic, 0.0) < min_period:
                self.stats["rate_drop"] += 1
                continue
            try:
                decoded = decode_existing_payload_image(topic, payload_raw)
                if decoded is None:
                    packed = None
                else:
                    image, source = decoded
                    packed = rgb_payload(
                        topic=topic,
                        image=image,
                        robot_id=int(source.get("robot_id") or 0),
                        frame_id=str(source.get("frame_id") or topic),
                        width=int(self.args.wrist_width),
                        height=int(self.args.wrist_height),
                        jpeg_quality=int(self.args.wrist_jpeg_quality),
                        seq=int(source.get("seq") or 0),
                        wire_format=str(self.args.wrist_wire_format),
                        encoder=self._encoder(topic) if self.args.wrist_wire_format == "h264_fmp4" else None,
                        extra={
                            "original_bytes": source.get("original_bytes"),
                            "camera": source.get("camera"),
                            "device": source.get("device"),
                        },
                    )
            except Exception as exc:
                if self.stats["decode_drop"] % 30 == 0:
                    print(f"[rgb_rgbd_sidechannel] wrist encode failed topic={topic}: {exc}", flush=True)
                packed = None
            if packed is None:
                self.stats["decode_drop"] += 1
                continue
            try:
                with self.pub_lock:
                    self.pub.send_multipart([topic_raw, packed], flags=zmq.NOBLOCK)
                self.last_sent[topic] = now
                self.stats["sent"] += 1
            except zmq.Again:
                self.stats["hwm_drop"] += 1
        sub.close(linger=0)
        for encoder in self.encoders.values():
            encoder.close()



def _set_rs_option(obj: Any, option: Any, value: float, label: str) -> None:
    try:
        obj.set_option(option, float(value))
    except Exception as exc:
        print(f"[rgb_rgbd_sidechannel] depth option {label} ignored: {exc}", flush=True)


def _configure_depth_sensor(sensor: Any, args: argparse.Namespace) -> None:
    preset = int(args.depth_visual_preset)
    if preset >= 0:
        _set_rs_option(sensor, rs.option.visual_preset, float(preset), "visual_preset")
    emitter = int(args.depth_enable_emitter)
    if emitter >= 0:
        _set_rs_option(sensor, rs.option.emitter_enabled, float(emitter), "emitter_enabled")
    laser = float(args.depth_laser_power)
    if laser >= 0.0:
        _set_rs_option(sensor, rs.option.laser_power, laser, "laser_power")


def build_depth_filters(args: argparse.Namespace) -> list[Any]:
    mode = str(args.depth_filter_mode).lower().strip()
    if mode == "off":
        print("[rgb_rgbd_sidechannel] depth filters disabled", flush=True)
        return []
    filters: list[Any] = []
    min_m = max(0.0, float(args.depth_min_m))
    max_m = max(min_m + 0.05, float(args.depth_max_m))
    try:
        threshold = rs.threshold_filter()
        _set_rs_option(threshold, rs.option.min_distance, min_m, "threshold.min_distance")
        _set_rs_option(threshold, rs.option.max_distance, max_m, "threshold.max_distance")
        filters.append(threshold)
    except Exception as exc:
        print(f"[rgb_rgbd_sidechannel] threshold filter unavailable: {exc}", flush=True)
    try:
        spatial = rs.spatial_filter()
        _set_rs_option(spatial, rs.option.filter_magnitude, 2.0 if mode == "accurate" else 1.0, "spatial.magnitude")
        _set_rs_option(spatial, rs.option.filter_smooth_alpha, 0.65 if mode == "accurate" else 0.5, "spatial.alpha")
        _set_rs_option(spatial, rs.option.filter_smooth_delta, 18.0 if mode == "accurate" else 30.0, "spatial.delta")
        _set_rs_option(spatial, rs.option.holes_fill, 1.0, "spatial.holes_fill")
        filters.append(spatial)
    except Exception as exc:
        print(f"[rgb_rgbd_sidechannel] spatial filter unavailable: {exc}", flush=True)
    try:
        temporal = rs.temporal_filter()
        _set_rs_option(temporal, rs.option.filter_smooth_alpha, 0.35 if mode == "accurate" else 0.45, "temporal.alpha")
        _set_rs_option(temporal, rs.option.filter_smooth_delta, 18.0 if mode == "accurate" else 35.0, "temporal.delta")
        filters.append(temporal)
    except Exception as exc:
        print(f"[rgb_rgbd_sidechannel] temporal filter unavailable: {exc}", flush=True)
    try:
        filters.append(rs.hole_filling_filter(1))
    except Exception as exc:
        print(f"[rgb_rgbd_sidechannel] hole filling filter unavailable: {exc}", flush=True)
    print(
        f"[rgb_rgbd_sidechannel] depth filters mode={mode} range={min_m:g}-{max_m:g}m count={len(filters)}",
        flush=True,
    )
    return filters


def apply_depth_filters(depth_frame: Any, filters: list[Any]) -> Any:
    frame = depth_frame
    for filt in filters:
        try:
            frame = filt.process(frame)
        except Exception as exc:
            print(f"[rgb_rgbd_sidechannel] depth filter failed; using unfiltered frame: {exc}", flush=True)
            return depth_frame
    try:
        out = frame.as_depth_frame()
        return out if out else depth_frame
    except Exception:
        return frame


def start_realsense(args: argparse.Namespace, *, enable_depth: bool) -> tuple[rs.pipeline, rs.align | None, float]:
    last_log = 0.0
    while True:
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, int(args.color_width), int(args.color_height), rs.format.bgr8, int(args.fps))
        if enable_depth:
            cfg.enable_stream(rs.stream.depth, int(args.depth_width), int(args.depth_height), rs.format.z16, int(args.fps))
        try:
            profile = pipeline.start(cfg)
        except Exception as exc:
            try:
                pipeline.stop()
            except Exception:
                pass
            now = time.monotonic()
            if now - last_log >= 2.0:
                print(f"[rgb_rgbd_sidechannel] waiting for RealSense: {exc}", flush=True)
                last_log = now
            time.sleep(1.0)
            continue
        align = rs.align(rs.stream.color) if enable_depth else None
        depth_units = 0.001
        if enable_depth:
            try:
                depth_sensor = profile.get_device().first_depth_sensor()
                _configure_depth_sensor(depth_sensor, args)
                depth_units = float(depth_sensor.get_depth_scale())
            except Exception:
                depth_units = 0.001
        usb_type = ""
        try:
            usb_type = str(profile.get_device().get_info(rs.camera_info.usb_type_descriptor))
        except Exception:
            pass
        depth_desc = (
            f"depth={args.depth_width}x{args.depth_height}@{args.fps} "
            if enable_depth else
            "depth=disabled "
        )
        print(
            "[rgb_rgbd_sidechannel] RealSense connected "
            f"color={args.color_width}x{args.color_height}@{args.fps} "
            f"{depth_desc}"
            f"usb={usb_type or 'unknown'} depth_units={depth_units}",
            flush=True,
        )
        return pipeline, align, depth_units


def main() -> int:
    args = parse_args()
    if str(args.wrist_wire_format) == "auto":
        args.wrist_wire_format = "jpeg"
    args.rgb_jpeg_quality = max(1, min(95, int(args.rgb_jpeg_quality)))
    args.wrist_jpeg_quality = max(1, min(95, int(args.wrist_jpeg_quality)))
    args.binary_jpeg_quality = max(1, min(95, int(args.binary_jpeg_quality)))
    args.png_compress = max(0, min(9, int(args.png_compress)))
    args.zstd_level = max(1, min(10, int(args.zstd_level)))
    binary_enabled = bool(args.binary_enable) and bool(str(args.binary_host).strip()) and int(args.binary_port) > 0
    rgbd_zmq_enabled = bool(args.rgbd_zmq_enable) and bool(str(args.rgbd_zmq_bind).strip())
    depth_enabled = binary_enabled or rgbd_zmq_enabled
    zstd_compressor = None
    if depth_enabled and args.depth_format == "zstd16":
        if zstd is None:
            raise RuntimeError("depth-format=zstd16 requires: python -m pip install zstandard")
        zstd_compressor = zstd.ZstdCompressor(level=int(args.zstd_level))
    wrist_topics = tuple(args.wrist_topic or ("/xlerobot/wrist_left/rgb/image_raw", "/xlerobot/wrist_right/rgb/image_raw"))

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.setsockopt(zmq.SNDHWM, 64)
    pub.setsockopt(zmq.SNDBUF, 4 * 1024 * 1024)
    pub.setsockopt(zmq.SNDTIMEO, 0)
    pub.bind(str(args.bind))
    pub_lock = threading.Lock()

    rgbd_zmq_pub = None
    rgbd_zmq_topic = str(args.rgbd_zmq_topic or "/xlerobot/head/rgbd")
    if rgbd_zmq_enabled:
        rgbd_zmq_pub = ctx.socket(zmq.PUB)
        rgbd_zmq_pub.setsockopt(zmq.LINGER, 0)
        rgbd_zmq_pub.setsockopt(zmq.SNDHWM, 8)
        rgbd_zmq_pub.setsockopt(zmq.SNDBUF, 4 * 1024 * 1024)
        rgbd_zmq_pub.setsockopt(zmq.SNDTIMEO, 0)
        rgbd_zmq_pub.bind(str(args.rgbd_zmq_bind))

    sender = (
        BinaryRgbdSender(str(args.binary_host), int(args.binary_port), float(args.binary_fps))
        if binary_enabled
        else None
    )
    if sender is not None:
        sender.start()
    wrist_proxy: WristProxy | None = None
    camera_workers: list[threading.Thread] = []
    legacy_pub = None
    legacy_lock = None
    wrist_mode = str(args.wrist_capture_mode or "proxy").strip().lower()
    if wrist_mode == "proxy":
        wrist_proxy = WristProxy(args, pub, pub_lock, wrist_topics)
        wrist_proxy.start()
    elif wrist_mode == "direct":
        if str(args.wrist_jpeg_bind or "").strip():
            legacy_pub = ctx.socket(zmq.PUB)
            legacy_pub.setsockopt(zmq.LINGER, 0)
            legacy_pub.setsockopt(zmq.SNDHWM, 64)
            legacy_pub.setsockopt(zmq.SNDBUF, 4 * 1024 * 1024)
            legacy_pub.setsockopt(zmq.SNDTIMEO, 0)
            legacy_pub.bind(str(args.wrist_jpeg_bind))
            legacy_lock = threading.Lock()
        floor_topic = str(args.floor_topic or f"rgb.floor.{int(args.robot_id)}")
        direct_specs = (
            ("wrist_left", wrist_topics[0] if len(wrist_topics) > 0 else "/xlerobot/wrist_left/rgb/image_raw", args.wrist_left_device, args.wrist_left_input_format, args.wrist_left_flip, float(args.wrist_left_max_fps or args.wrist_max_fps)),
            ("wrist_right", wrist_topics[1] if len(wrist_topics) > 1 else "/xlerobot/wrist_right/rgb/image_raw", args.wrist_right_device, args.wrist_right_input_format, args.wrist_right_flip, float(args.wrist_right_max_fps or args.wrist_max_fps)),
            ("floor", floor_topic, args.floor_device, args.floor_input_format, args.floor_flip, float(args.floor_max_fps or args.wrist_max_fps)),
        )
        used_devices: dict[str, str] = {}
        for name, topic, device, input_format, flip, fps in direct_specs:
            if not _is_enabled_device(str(device)):
                continue
            resolved = _resolve_device(str(device))
            if resolved in used_devices:
                print(
                    f"[rgb_rgbd_sidechannel] skip {name}: device {resolved} already used by {used_devices[resolved]}",
                    flush=True,
                )
                continue
            used_devices[resolved] = name
            worker_cls = (
                FfmpegMjpegCamera
                if str(args.wrist_wire_format) == "jpeg"
                and str(input_format).strip().upper() in {"MJPG", "MJPEG"}
                else DirectWristCamera
            )
            worker = worker_cls(
                args,
                name=name,
                topic=str(topic),
                device=str(device),
                input_format=str(input_format),
                flip=str(flip),
                fps=float(fps),
                pub=pub,
                pub_lock=pub_lock,
                legacy_pub=legacy_pub,
                legacy_lock=legacy_lock,
            )
            worker.start()
            camera_workers.append(worker)

    pipeline, align, depth_units = start_realsense(args, enable_depth=depth_enabled)
    rs_clock = RealSenseFrameClock()
    depth_filters = build_depth_filters(args) if depth_enabled else []
    print(
        "[rgb_rgbd_sidechannel] "
        f"rgb_pub={args.bind} front_topic=/xlerobot/head/rgb/image_raw "
        f"rgbd_zmq={'%s topic=%s fps=%g' % (args.rgbd_zmq_bind, rgbd_zmq_topic, float(args.rgbd_zmq_fps)) if rgbd_zmq_enabled else 'disabled'} "
        f"rgbd_color_mode={args.rgbd_zmq_color_mode} "
        f"wrist_mode={wrist_mode} wrist_source={args.wrist_source} "
        f"wrist_jpeg_bind={args.wrist_jpeg_bind or 'disabled'} "
        f"wrist_topics={','.join(wrist_topics)} floor_topic={args.floor_topic or 'rgb.floor.' + str(args.robot_id)} "
        f"rgb_wire={args.rgb_wire_format} wrist_wire={args.wrist_wire_format} "
        f"h264_input={args.h264_input_format} "
        f"front_size={args.rgb_width}x{args.rgb_height} "
        f"wrist_size={args.wrist_width}x{args.wrist_height} "
        f"binary={'tcp://' + str(args.binary_host) + ':' + str(args.binary_port) if binary_enabled else 'disabled'} "
        f"binary_fps={args.binary_fps:g} "
        f"binary_size={args.binary_width}x{args.binary_height} "
        f"depth_filter={args.depth_filter_mode} depth_range={args.depth_min_m:g}-{args.depth_max_m:g}m",
        flush=True,
    )

    min_rgb_period = 1.0 / max(0.1, float(args.rgb_max_fps))
    min_rgbd_zmq_period = 1.0 / max(0.1, float(args.rgbd_zmq_fps))
    last_front_rgb = 0.0
    last_rgbd_zmq = 0.0
    last_stats = time.monotonic()
    seq = 0
    stats = {
        "rs_frames": 0,
        "front_sent": 0,
        "front_hwm_drop": 0,
        "binary_submit": 0,
        "rgbd_zmq_sent": 0,
        "rgbd_zmq_hwm_drop": 0,
        "encode_drop": 0,
    }
    color_frame_id = "head_camera_rgb_optical_frame"
    depth_frame_id = "head_camera_depth_optical_frame"
    front_encoder = (
        H264Fmp4Encoder(
            width=int(args.rgb_width),
            height=int(args.rgb_height),
            fps=float(args.rgb_max_fps),
            crf=int(args.h264_crf),
            keyint_frames=int(args.h264_keyint_frames),
            init_interval_frames=int(args.h264_init_interval_frames),
            input_format=str(args.h264_input_format),
        )
        if args.rgb_wire_format == "h264_fmp4"
        else None
    )
    front_rtsp_publisher = (
        H264RtspPublisher(
            name="head",
            url=_rtsp_publish_url(args, "head"),
            width=int(args.rgb_width),
            height=int(args.rgb_height),
            fps=float(args.rgb_max_fps),
            crf=int(args.rtsp_h264_crf),
            keyint_frames=int(args.rtsp_h264_keyint_frames),
            transport=str(args.rtsp_publish_transport),
        )
        if _rtsp_publish_url(args, "head")
        else None
    )

    last_rs_timeout_log = 0.0
    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(1000)
            except RuntimeError as exc:
                stats["rs_timeout"] = int(stats.get("rs_timeout", 0)) + 1
                now = time.monotonic()
                if now - last_rs_timeout_log >= 2.0:
                    print(f"[rgb_rgbd_sidechannel] RealSense wait timeout; reconnecting: {exc}", flush=True)
                    last_rs_timeout_log = now
                try:
                    pipeline.stop()
                except Exception:
                    pass
                pipeline, align, depth_units = start_realsense(args, enable_depth=depth_enabled)
                rs_clock = RealSenseFrameClock()
                depth_filters = build_depth_filters(args) if depth_enabled else []
                continue
            if align is not None:
                frames = align.process(frames)
            depth_frame = frames.get_depth_frame() if depth_enabled else None
            if depth_frame is not None and depth_filters:
                depth_frame = apply_depth_filters(depth_frame, depth_filters)
            color_frame = frames.get_color_frame()
            if not color_frame or (depth_enabled and not depth_frame):
                continue
            stats["rs_frames"] += 1
            seq += 1
            now = time.monotonic()
            stamp_ns = rs_clock.stamp_ns(color_frame)
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data()) if depth_frame is not None else None

            if now - last_front_rgb >= min_rgb_period:
                if front_rtsp_publisher is not None:
                    front_rtsp_publisher.submit(color)
                packed = rgb_payload(
                    topic="/xlerobot/head/rgb/image_raw",
                    image=color,
                    robot_id=int(args.robot_id),
                    frame_id=color_frame_id,
                    width=int(args.rgb_width),
                    height=int(args.rgb_height),
                    jpeg_quality=int(args.rgb_jpeg_quality),
                    seq=seq,
                    wire_format=str(args.rgb_wire_format),
                    encoder=front_encoder,
                    extra={"camera": "front", "device": "realsense"},
                )
                if packed is not None:
                    try:
                        topic_b = "/xlerobot/head/rgb/image_raw".encode("utf-8")
                        with pub_lock:
                            pub.send_multipart([topic_b, packed], flags=zmq.NOBLOCK)
                        stats["front_sent"] += 1
                        last_front_rgb = now
                    except zmq.Again:
                        stats["front_hwm_drop"] += 1

            rgbd_zmq_due = (
                rgbd_zmq_pub is not None
                and now - last_rgbd_zmq >= min_rgbd_zmq_period
            )
            if (sender is not None or rgbd_zmq_due) and depth is not None and depth_frame is not None:
                need_color_jpeg = sender is not None or str(args.rgbd_zmq_color_mode) == "jpeg"
                if need_color_jpeg:
                    binary_color_image, binary_depth_image = resize_color_depth(
                        color,
                        depth,
                        width=int(args.binary_width),
                        height=int(args.binary_height),
                    )
                    binary_color = encode_jpeg(binary_color_image, int(args.binary_jpeg_quality))
                    if binary_color is None:
                        stats["encode_drop"] += 1
                        continue
                else:
                    binary_color_image = color
                    binary_color = b""
                    depth_h, depth_w = depth.shape[:2]
                    if int(args.binary_width) > 0 and int(args.binary_height) > 0 and (
                        depth_w != int(args.binary_width) or depth_h != int(args.binary_height)
                    ):
                        binary_depth_image = cv2.resize(
                            depth,
                            (int(args.binary_width), int(args.binary_height)),
                            interpolation=cv2.INTER_NEAREST,
                        )
                    else:
                        binary_depth_image = depth
                depth_u16 = np.ascontiguousarray(binary_depth_image.astype(np.uint16, copy=False))
                depth_raw = depth_u16.tobytes(order="C")
                if args.depth_format == "png16":
                    ok, encoded_depth = cv2.imencode(
                        ".png",
                        depth_u16,
                        [int(cv2.IMWRITE_PNG_COMPRESSION), int(args.png_compress)],
                    )
                    if not ok:
                        stats["encode_drop"] += 1
                        depth_bytes = None
                        depth_format = "png;16UC1"
                    else:
                        depth_bytes = encoded_depth.tobytes()
                        depth_format = "png;16UC1"
                elif args.depth_format == "zstd16":
                    assert zstd_compressor is not None
                    depth_bytes = zstd_compressor.compress(depth_raw)
                    depth_format = "zstd;16UC1"
                elif args.depth_format == "rvl16":
                    depth_bytes = encode_rvl_depth(depth_u16)
                    depth_format = "rvl;16UC1"
                else:
                    depth_bytes = depth_raw
                    depth_format = "raw16uc1-le"
                if depth_bytes is not None:
                    st = stamp_from_ns(stamp_ns)
                    header = {
                        "type": "rgbd",
                        "stamp": st,
                        "color_format": "jpeg",
                        "color_len": len(binary_color),
                        "color_encoding": "bgr8",
                        "color_frame_id": color_frame_id,
                        "depth_format": depth_format,
                        "depth_len": len(depth_bytes),
                        "depth_encoding": "16UC1",
                        "aligned_depth_to_color": True,
                        "depth_frame_id": color_frame_id,
                        "depth_width": int(depth_u16.shape[1]),
                        "depth_height": int(depth_u16.shape[0]),
                        "depth_step": int(depth_u16.shape[1] * 2),
                        "depth_uncompressed_len": len(depth_raw),
                        "depth_units": float(depth_units),
                        "color_camera_info": camera_info_from_frame(color_frame, st, color_frame_id),
                        "depth_camera_info": camera_info_from_frame(color_frame, st, color_frame_id),
                        "stamp_source": "realsense_color_frame",
                        "color_frame_timestamp_ms": float(color_frame.get_timestamp()),
                    }
                    if sender is not None:
                        sender.submit(header, binary_color + depth_bytes)
                        stats["binary_submit"] += 1
                    if rgbd_zmq_due and rgbd_zmq_pub is not None:
                        color_mode = str(args.rgbd_zmq_color_mode)
                        rgbd_payload = {
                                "schema": "indoory_rgbd_zmq_v1",
                                "topic": rgbd_zmq_topic,
                                "robot_id": int(args.robot_id),
                                "stamp_ns": int(stamp_ns),
                                "encoding": "jpeg+depth" if color_mode == "jpeg" else "rgb_ref+depth",
                                "color_mode": color_mode,
                                "color_topic": "/xlerobot/head/rgb/image_raw",
                                "color_format": "jpeg" if color_mode == "jpeg" else "external_topic",
                                "color_encoding": "bgr8" if color_mode == "jpeg" else str(args.rgb_wire_format),
                                "color_width": int(binary_color_image.shape[1]) if color_mode == "jpeg" else int(args.rgb_width),
                                "color_height": int(binary_color_image.shape[0]) if color_mode == "jpeg" else int(args.rgb_height),
                                "color_len": len(binary_color) if color_mode == "jpeg" else 0,
                                "color_frame_id": color_frame_id,
                                "depth_format": depth_format,
                                "depth_encoding": "16UC1",
                                "depth_width": int(depth_u16.shape[1]),
                                "depth_height": int(depth_u16.shape[0]),
                                "depth_step": int(depth_u16.shape[1] * 2),
                                "depth_uncompressed_len": len(depth_raw),
                                "depth_len": len(depth_bytes),
                                "depth_data": depth_bytes,
                                "depth_units": float(depth_units),
                                "aligned_depth_to_color": True,
                                "depth_frame_id": color_frame_id,
                                "color_camera_info": header["color_camera_info"],
                                "depth_camera_info": header["depth_camera_info"],
                                "optimized": True,
                                "stamp_source": "realsense_color_frame",
                                "color_frame_timestamp_ms": float(color_frame.get_timestamp()),
                            }
                        if color_mode == "jpeg":
                            rgbd_payload["color_data"] = binary_color
                        try:
                            rgbd_zmq_pub.send_multipart(
                                [
                                    rgbd_zmq_topic.encode("utf-8"),
                                    msgpack.packb(rgbd_payload, use_bin_type=True),
                                ],
                                flags=zmq.NOBLOCK,
                            )
                            stats["rgbd_zmq_sent"] += 1
                            last_rgbd_zmq = now
                        except zmq.Again:
                            stats["rgbd_zmq_hwm_drop"] += 1

            if now - last_stats >= float(args.stats_interval_s):
                print(
                    f"[rgb_rgbd_sidechannel] stats={stats} cameras="
                    f"{wrist_proxy.stats if wrist_proxy is not None else {w.camera_name: w.stats for w in camera_workers}} "
                    f"binary_sent={sender.frames_sent if sender is not None else 0} "
                    f"rgbd_zmq={'enabled' if rgbd_zmq_pub is not None else 'disabled'}",
                    flush=True,
                )
                last_stats = now
    finally:
        if wrist_proxy is not None:
            wrist_proxy.stop_event.set()
        for worker in camera_workers:
            worker.stop_event.set()
        if sender is not None:
            sender.stop_event.set()
        if rgbd_zmq_pub is not None:
            rgbd_zmq_pub.close(linger=0)
        if front_encoder is not None:
            front_encoder.close()
        if front_rtsp_publisher is not None:
            front_rtsp_publisher.close()
        if legacy_pub is not None:
            legacy_pub.close(linger=0)
        pipeline.stop()


if __name__ == "__main__":
    raise SystemExit(main())
