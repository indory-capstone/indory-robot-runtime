#!/usr/bin/env python3
"""Publish an optimized RGB-D ZMQ stream using existing RGB plus RealSense depth."""

from __future__ import annotations

import argparse
import threading
import time
from typing import Any

import cv2
import msgpack
import numpy as np
import pyrealsense2 as rs
import zmq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb-source", default="tcp://127.0.0.1:8866")
    parser.add_argument("--rgb-topic", default="rgb.front.0")
    parser.add_argument("--bind", default="tcp://0.0.0.0:8867")
    parser.add_argument("--topic", default="rgbd.front.0")
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--max-fps", type=float, default=3.0)
    parser.add_argument("--width", type=int, default=424)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--depth-fps", type=int, default=15)
    parser.add_argument("--png-compress", type=int, default=3)
    parser.add_argument("--stats-interval-s", type=float, default=10.0)
    return parser.parse_args()


class LatestRgbSubscriber(threading.Thread):
    def __init__(self, endpoint: str, topic: str) -> None:
        super().__init__(name="latest-rgb-zmq", daemon=True)
        self.endpoint = endpoint
        self.topic = topic
        self.lock = threading.Lock()
        self.latest: dict[str, Any] | None = None
        self.stop = threading.Event()

    def run(self) -> None:
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVHWM, 2)
        sock.setsockopt(zmq.SUBSCRIBE, self.topic.encode("utf-8"))
        sock.connect(self.endpoint)
        while not self.stop.is_set():
            try:
                _topic_raw, payload_raw = sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.002)
                continue
            try:
                payload = msgpack.unpackb(payload_raw, raw=False)
            except Exception:
                continue
            if isinstance(payload, dict) and isinstance(payload.get("data"), (bytes, bytearray)):
                with self.lock:
                    self.latest = payload
        sock.close(linger=0)

    def snapshot(self) -> dict[str, Any] | None:
        with self.lock:
            return dict(self.latest) if self.latest is not None else None


def start_depth_pipeline(args: argparse.Namespace) -> tuple[rs.pipeline, float]:
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(
        rs.stream.depth,
        int(args.depth_width),
        int(args.depth_height),
        rs.format.z16,
        int(args.depth_fps),
    )
    profile = pipeline.start(cfg)
    sensor = profile.get_device().first_depth_sensor()
    try:
        depth_units = float(sensor.get_depth_scale())
    except Exception:
        depth_units = 0.001
    return pipeline, depth_units


def encode_depth_png(depth_frame: Any, width: int, height: int, png_compress: int) -> tuple[bytes, dict[str, Any]] | None:
    depth = np.asanyarray(depth_frame.get_data())
    original_height, original_width = depth.shape[:2]
    if width > 0 and height > 0 and (original_width != width or original_height != height):
        depth = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
    depth = np.ascontiguousarray(depth.astype(np.uint16, copy=False))
    ok, encoded = cv2.imencode(".png", depth, [int(cv2.IMWRITE_PNG_COMPRESSION), int(png_compress)])
    if not ok:
        return None
    meta = {
        "depth_width": int(depth.shape[1]),
        "depth_height": int(depth.shape[0]),
        "depth_original_width": int(original_width),
        "depth_original_height": int(original_height),
        "depth_step": int(depth.shape[1] * 2),
    }
    return encoded.tobytes(), meta


def main() -> int:
    args = parse_args()
    min_period = 1.0 / max(0.1, float(args.max_fps))
    png_compress = max(0, min(9, int(args.png_compress)))

    rgb_sub = LatestRgbSubscriber(args.rgb_source, args.rgb_topic)
    rgb_sub.start()

    pipeline, depth_units = start_depth_pipeline(args)

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.setsockopt(zmq.SNDHWM, 1)
    pub.setsockopt(zmq.SNDTIMEO, 0)
    pub.bind(args.bind)

    print(
        "[rgbd_zmq_optimized_publisher] "
        f"bind={args.bind} topic={args.topic} rgb={args.rgb_source}/{args.rgb_topic} "
        f"max_fps={args.max_fps:g} size={args.width}x{args.height} "
        f"depth_capture={args.depth_width}x{args.depth_height}@{args.depth_fps} depth_units={depth_units}",
        flush=True,
    )

    stats = {"depth_frames": 0, "sent": 0, "missing_rgb": 0, "encode_drop": 0, "hwm_drop": 0}
    last_sent = 0.0
    last_stats = time.monotonic()

    try:
        while True:
            frames = pipeline.wait_for_frames(1000)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            stats["depth_frames"] += 1
            now = time.monotonic()
            if now - last_sent < min_period:
                continue

            rgb = rgb_sub.snapshot()
            if rgb is None:
                stats["missing_rgb"] += 1
                continue

            encoded = encode_depth_png(depth_frame, int(args.width), int(args.height), png_compress)
            if encoded is None:
                stats["encode_drop"] += 1
                continue
            depth_png, depth_meta = encoded

            color_data = rgb.get("data", b"")
            payload: dict[str, Any] = {
                "schema": "indoory_rgbd_zmq_v1",
                "topic": str(args.topic),
                "robot_id": int(args.robot_id),
                "stamp_ns": time.time_ns(),
                "encoding": "jpeg+png16",
                "color_topic": str(args.rgb_topic),
                "color_stamp_ns": rgb.get("stamp_ns"),
                "color_encoding": rgb.get("encoding", "jpeg"),
                "color_width": int(rgb.get("width") or args.width),
                "color_height": int(rgb.get("height") or args.height),
                "color_len": len(color_data),
                "color_data": bytes(color_data),
                "depth_encoding": "png;16UC1",
                "depth_len": len(depth_png),
                "depth_data": depth_png,
                "depth_units": depth_units,
                "depth_frame_id": "head_camera_depth_optical_frame",
                "color_frame_id": rgb.get("frame_id", "front_rgb"),
                "optimized": True,
                **depth_meta,
            }
            packed = msgpack.packb(payload, use_bin_type=True)
            try:
                pub.send_multipart([str(args.topic).encode("utf-8"), packed], flags=zmq.NOBLOCK)
                stats["sent"] += 1
                last_sent = now
            except zmq.Again:
                stats["hwm_drop"] += 1

            if now - last_stats >= float(args.stats_interval_s):
                print(f"[rgbd_zmq_optimized_publisher] stats={stats}", flush=True)
                last_stats = now
    finally:
        rgb_sub.stop.set()
        pipeline.stop()


if __name__ == "__main__":
    raise SystemExit(main())
