#!/usr/bin/env python3
"""Optimized external ZMQ proxy for local camera JPEG streams."""

from __future__ import annotations

import argparse
import time
from typing import Any

import cv2
import msgpack
import numpy as np
import zmq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="tcp://127.0.0.1:8864")
    parser.add_argument("--bind", default="tcp://0.0.0.0:8865")
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help="Topic allowlist. Repeatable. Default: rgb.front.0 and rgb.wrist_right.0.",
    )
    parser.add_argument("--max-fps", type=float, default=4.0)
    parser.add_argument("--width", type=int, default=424)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--jpeg-quality", type=int, default=45)
    parser.add_argument("--stats-interval-s", type=float, default=10.0)
    return parser.parse_args()


def optimize_payload(
    topic: str,
    payload_raw: bytes,
    *,
    width: int,
    height: int,
    jpeg_quality: int,
) -> bytes | None:
    payload = msgpack.unpackb(payload_raw, raw=False)
    if not isinstance(payload, dict):
        return None
    data = payload.get("data")
    if not isinstance(data, (bytes, bytearray)):
        return None
    if payload.get("encoding") != "jpeg":
        return None

    encoded = np.frombuffer(data, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        return None
    original_height, original_width = image.shape[:2]
    if width > 0 and height > 0 and (original_width != width or original_height != height):
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

    ok, out = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok:
        return None

    out_payload: dict[str, Any] = dict(payload)
    out_payload.update(
        {
            "topic": topic,
            "encoding": "jpeg",
            "width": int(image.shape[1]),
            "height": int(image.shape[0]),
            "data": out.tobytes(),
            "optimized": True,
            "original_width": int(original_width),
            "original_height": int(original_height),
            "original_bytes": len(data),
            "proxy_stamp_ns": time.time_ns(),
        }
    )
    return msgpack.packb(out_payload, use_bin_type=True)


def main() -> int:
    args = parse_args()
    allowed = tuple(args.topic or ("rgb.front.0", "rgb.wrist_right.0"))
    min_period = 1.0 / max(0.1, float(args.max_fps))
    quality = max(1, min(95, int(args.jpeg_quality)))

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.RCVHWM, 4)
    for topic in allowed:
        sub.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))
    sub.connect(args.source)

    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.setsockopt(zmq.SNDHWM, 1)
    pub.setsockopt(zmq.SNDTIMEO, 0)
    pub.bind(args.bind)

    print(
        "[camera_zmq_optimized_proxy] "
        f"source={args.source} bind={args.bind} topics={','.join(allowed)} "
        f"max_fps={args.max_fps:g} size={args.width}x{args.height} quality={quality}",
        flush=True,
    )

    last_sent_by_topic: dict[str, float] = {}
    stats = {"recv": 0, "sent": 0, "dropped_rate": 0, "dropped_decode": 0, "dropped_hwm": 0}
    last_stats = time.monotonic()

    while True:
        topic_raw, payload_raw = sub.recv_multipart()
        now = time.monotonic()
        topic = topic_raw.decode("utf-8", "replace")
        stats["recv"] += 1

        last_sent = last_sent_by_topic.get(topic, 0.0)
        if now - last_sent < min_period:
            stats["dropped_rate"] += 1
            continue

        try:
            optimized = optimize_payload(
                topic,
                payload_raw,
                width=int(args.width),
                height=int(args.height),
                jpeg_quality=quality,
            )
        except Exception:
            optimized = None
        if optimized is None:
            stats["dropped_decode"] += 1
            continue

        try:
            pub.send_multipart([topic_raw, optimized], flags=zmq.NOBLOCK)
            last_sent_by_topic[topic] = now
            stats["sent"] += 1
        except zmq.Again:
            stats["dropped_hwm"] += 1

        if now - last_stats >= float(args.stats_interval_s):
            print(f"[camera_zmq_optimized_proxy] stats={stats}", flush=True)
            last_stats = now


if __name__ == "__main__":
    raise SystemExit(main())
