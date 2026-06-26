#!/usr/bin/env python3
"""Benchmark camera transport startup and throughput by wire format.

The tool probes both layers used by the robot:

* ZMQ camera topics on the optimized RGB PUB socket, usually tcp://127.0.0.1:8866.
* ZMQ RGB-D topic on the optimized RGB-D PUB socket, usually tcp://127.0.0.1:8867.
* WebXR HTTP camera endpoints, usually https://127.0.0.1:8443/api/....

It prints Markdown tables so the output can be pasted directly into docs.
"""

from __future__ import annotations

import argparse
import http.client
import json
import shutil
import socket
import ssl
import subprocess
import time
from dataclasses import dataclass
from typing import Any
from urllib.parse import urlparse

import cv2
import msgpack
import numpy as np
import zmq

try:
    import zstandard as zstd
except Exception:  # pragma: no cover - optional; reported in benchmark output
    zstd = None


DEFAULT_ZMQ_TOPICS = (
    "rgb.front.0",
    "rgb.wrist_left.0",
    "rgb.wrist_right.0",
    "rgb.floor.0",
)
DEFAULT_RGBD_ZMQ_TOPICS = ("rgbd.front.0",)
DEFAULT_LOSSLESS_TOPICS = (
    "rgb.wrist_left.0",
    "rgb.wrist_right.0",
    "rgb.floor.0",
)
DEFAULT_HTTP_PATHS = (
    "/api/head_rgb.mp4",
    "/api/wrist_rgb.mp4?side=left",
    "/api/wrist_rgb.mp4?side=right",
    "/api/floor_rgb.mp4",
    "/api/head_rgb.jpg",
    "/api/wrist_rgb.jpg?side=left",
    "/api/wrist_rgb.jpg?side=right",
    "/api/floor_rgb.jpg",
    "/api/head_rgb.mjpg",
    "/api/wrist_rgb.mjpg?side=left",
    "/api/wrist_rgb.mjpg?side=right",
    "/api/floor_rgb.mjpg",
)


@dataclass(frozen=True)
class ZmqResult:
    topic: str
    status: str
    encoding: str
    messages: int
    first_msg_ms: float | None
    first_init_ms: float | None
    data_bytes: int
    init_bytes: int
    duration_s: float
    first_seq: Any = None
    last_seq: Any = None
    error: str = ""


@dataclass(frozen=True)
class HttpResult:
    path: str
    status: str
    http_code: int | None
    video_format: str
    headers_ms: float | None
    first_byte_ms: float | None
    bytes_read: int
    duration_s: float
    error: str = ""


@dataclass(frozen=True)
class RgbdZmqResult:
    topic: str
    status: str
    encoding: str
    depth_format: str
    messages: int
    first_msg_ms: float | None
    payload_bytes: int
    color_bytes: int
    depth_bytes: int
    duration_s: float
    color_size: str = ""
    depth_size: str = ""
    depth_units: Any = None
    error: str = ""


@dataclass(frozen=True)
class LosslessReencodeResult:
    topic: str
    status: str
    source_encoding: str
    frames: int
    width: int | None
    height: int | None
    duration_s: float
    source_kib_per_frame: float | None = None
    raw_kib_per_frame: float | None = None
    zstd_kib_per_frame: float | None = None
    zstd_encode_ms: float | None = None
    png1_kib_per_frame: float | None = None
    png1_encode_ms: float | None = None
    png3_kib_per_frame: float | None = None
    png3_encode_ms: float | None = None
    ffv1_kib_per_frame: float | None = None
    ffv1_encode_fps: float | None = None
    error: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zmq-endpoint", default="tcp://127.0.0.1:8866")
    parser.add_argument("--topic", action="append", default=[])
    parser.add_argument("--rgbd-zmq-endpoint", default="tcp://127.0.0.1:8867")
    parser.add_argument("--rgbd-topic", action="append", default=[])
    parser.add_argument("--http-base", default="https://127.0.0.1:8443")
    parser.add_argument("--http-path", action="append", default=[])
    parser.add_argument("--lossless-topic", action="append", default=[])
    parser.add_argument("--lossless-sample-frames", type=int, default=30)
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--startup-timeout-s", type=float, default=4.0)
    parser.add_argument("--read-chunk-bytes", type=int, default=4096)
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of Markdown.")
    parser.add_argument("--skip-zmq", action="store_true")
    parser.add_argument("--skip-rgbd-zmq", action="store_true")
    parser.add_argument("--skip-http", action="store_true")
    parser.add_argument("--skip-lossless", action="store_true")
    return parser.parse_args()


def measure_zmq_topic(endpoint: str, topic: str, *, duration_s: float, startup_timeout_s: float) -> ZmqResult:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.setsockopt(zmq.RCVTIMEO, 250)
    sock.connect(endpoint)
    sock.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))

    start = time.monotonic()
    deadline = start + max(duration_s, startup_timeout_s)
    measure_until: float | None = None
    messages = 0
    data_bytes = 0
    init_bytes = 0
    first_msg_ms: float | None = None
    first_init_ms: float | None = None
    encoding = ""
    first_seq: Any = None
    last_seq: Any = None
    error = ""
    try:
        while time.monotonic() < deadline:
            try:
                topic_b, payload_b = sock.recv_multipart()
            except zmq.Again:
                if messages == 0 and time.monotonic() - start >= startup_timeout_s:
                    break
                continue
            now = time.monotonic()
            if topic_b.decode("utf-8", "replace") != topic:
                continue
            try:
                msg = msgpack.unpackb(payload_b, raw=False)
            except Exception as exc:
                error = f"msgpack: {exc}"
                break
            if not isinstance(msg, dict):
                error = "payload is not a dict"
                break
            if messages == 0:
                first_msg_ms = (now - start) * 1000.0
                measure_until = now + duration_s
            messages += 1
            encoding = str(msg.get("encoding") or encoding)
            seq = msg.get("chunk_seq", msg.get("seq"))
            if first_seq is None:
                first_seq = seq
            last_seq = seq
            data_len = len(bytes(msg.get("data") or b""))
            init_len = len(bytes(msg.get("init") or b""))
            data_bytes += data_len
            init_bytes += init_len
            if init_len and first_init_ms is None:
                first_init_ms = (now - start) * 1000.0
            if measure_until is not None and now >= measure_until:
                break
    finally:
        sock.close(0)
    duration = max(0.0, time.monotonic() - start)
    status = "ok" if messages else "no_data"
    if error:
        status = "error"
    return ZmqResult(
        topic=topic,
        status=status,
        encoding=encoding,
        messages=messages,
        first_msg_ms=first_msg_ms,
        first_init_ms=first_init_ms,
        data_bytes=data_bytes,
        init_bytes=init_bytes,
        duration_s=duration,
        first_seq=first_seq,
        last_seq=last_seq,
        error=error,
    )


def measure_rgbd_zmq_topic(endpoint: str, topic: str, *, duration_s: float, startup_timeout_s: float) -> RgbdZmqResult:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.setsockopt(zmq.RCVTIMEO, 250)
    sock.connect(endpoint)
    sock.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))

    start = time.monotonic()
    deadline = start + max(duration_s, startup_timeout_s)
    measure_until: float | None = None
    messages = 0
    payload_bytes = 0
    color_bytes = 0
    depth_bytes = 0
    first_msg_ms: float | None = None
    encoding = ""
    depth_format = ""
    color_size = ""
    depth_size = ""
    depth_units: Any = None
    error = ""
    try:
        while time.monotonic() < deadline:
            try:
                topic_b, payload_b = sock.recv_multipart()
            except zmq.Again:
                if messages == 0 and time.monotonic() - start >= startup_timeout_s:
                    break
                continue
            now = time.monotonic()
            if topic_b.decode("utf-8", "replace") != topic:
                continue
            try:
                msg = msgpack.unpackb(payload_b, raw=False)
            except Exception as exc:
                error = f"msgpack: {exc}"
                break
            if not isinstance(msg, dict):
                error = "payload is not a dict"
                break
            if messages == 0:
                first_msg_ms = (now - start) * 1000.0
                measure_until = now + duration_s
            messages += 1
            payload_bytes += len(payload_b)
            color_len = len(bytes(msg.get("color_data") or b""))
            depth_len = len(bytes(msg.get("depth_data") or b""))
            color_bytes += color_len
            depth_bytes += depth_len
            encoding = str(msg.get("encoding") or encoding)
            depth_format = str(msg.get("depth_format") or depth_format)
            color_size = f"{msg.get('color_width')}x{msg.get('color_height')}"
            depth_size = f"{msg.get('depth_width')}x{msg.get('depth_height')}"
            depth_units = msg.get("depth_units", depth_units)
            if measure_until is not None and now >= measure_until:
                break
    finally:
        sock.close(0)
    duration = max(0.0, time.monotonic() - start)
    status = "ok" if messages else "no_data"
    if error:
        status = "error"
    return RgbdZmqResult(
        topic=topic,
        status=status,
        encoding=encoding,
        depth_format=depth_format,
        messages=messages,
        first_msg_ms=first_msg_ms,
        payload_bytes=payload_bytes,
        color_bytes=color_bytes,
        depth_bytes=depth_bytes,
        duration_s=duration,
        color_size=color_size,
        depth_size=depth_size,
        depth_units=depth_units,
        error=error,
    )


def _kib_per_frame(total_bytes: int, frames: int) -> float | None:
    if frames <= 0:
        return None
    return total_bytes / frames / 1024.0


def _fmt_num(value: float | None, digits: int = 1) -> str:
    return "-" if value is None else f"{value:.{digits}f}"


def _ffv1_bytes(frames: list[np.ndarray], fps: float) -> tuple[int | None, float | None, str]:
    if not frames:
        return None, None, "no frames"
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        return None, None, "ffmpeg not found"
    h, w = frames[0].shape[:2]
    raw = b"".join(np.ascontiguousarray(frame).tobytes(order="C") for frame in frames)
    cmd = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s:v",
        f"{w}x{h}",
        "-r",
        str(max(1.0, float(fps))),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "ffv1",
        "-level",
        "3",
        "-g",
        "1",
        "-coder",
        "1",
        "-context",
        "1",
        "-slicecrc",
        "0",
        "-f",
        "matroska",
        "pipe:1",
    ]
    start = time.monotonic()
    try:
        proc = subprocess.run(cmd, input=raw, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20.0)
    except Exception as exc:
        return None, None, str(exc)
    elapsed = max(1e-6, time.monotonic() - start)
    if proc.returncode != 0:
        return None, None, proc.stderr.decode("utf-8", "replace").strip()
    return len(proc.stdout), len(frames) / elapsed, ""


def measure_lossless_reencode(
    endpoint: str,
    topic: str,
    *,
    sample_frames: int,
    startup_timeout_s: float,
) -> LosslessReencodeResult:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVHWM, 4)
    sock.setsockopt(zmq.RCVTIMEO, 250)
    sock.connect(endpoint)
    sock.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))

    start = time.monotonic()
    deadline = start + max(1.0, startup_timeout_s)
    frames: list[np.ndarray] = []
    source_bytes = 0
    source_encoding = ""
    error = ""
    try:
        while len(frames) < max(1, int(sample_frames)) and time.monotonic() < deadline:
            try:
                topic_b, payload_b = sock.recv_multipart()
            except zmq.Again:
                continue
            if topic_b.decode("utf-8", "replace") != topic:
                continue
            try:
                msg = msgpack.unpackb(payload_b, raw=False)
            except Exception as exc:
                error = f"msgpack: {exc}"
                break
            if not isinstance(msg, dict):
                error = "payload is not a dict"
                break
            source_encoding = str(msg.get("encoding") or source_encoding)
            data = msg.get("data")
            if source_encoding != "jpeg" or not isinstance(data, (bytes, bytearray)):
                error = f"source encoding is {source_encoding or '-'}, expected jpeg"
                break
            image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                error = "jpeg decode failed"
                break
            frames.append(np.ascontiguousarray(image))
            source_bytes += len(data)
    finally:
        sock.close(0)

    if not frames:
        return LosslessReencodeResult(
            topic=topic,
            status="no_data" if not error else "error",
            source_encoding=source_encoding,
            frames=0,
            width=None,
            height=None,
            duration_s=time.monotonic() - start,
            error=error,
        )

    h, w = frames[0].shape[:2]
    raw_bytes = 0
    zstd_bytes = 0
    zstd_ms = 0.0
    png1_bytes = 0
    png1_ms = 0.0
    png3_bytes = 0
    png3_ms = 0.0
    zstd_error = ""

    compressor = zstd.ZstdCompressor(level=1) if zstd is not None else None
    for frame in frames:
        raw = frame.tobytes(order="C")
        raw_bytes += len(raw)
        if compressor is not None:
            t0 = time.monotonic()
            zstd_bytes += len(compressor.compress(raw))
            zstd_ms += (time.monotonic() - t0) * 1000.0
        else:
            zstd_error = "zstandard not available"
        t0 = time.monotonic()
        ok, encoded = cv2.imencode(".png", frame, [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
        png1_ms += (time.monotonic() - t0) * 1000.0
        if ok:
            png1_bytes += len(encoded)
        t0 = time.monotonic()
        ok, encoded = cv2.imencode(".png", frame, [int(cv2.IMWRITE_PNG_COMPRESSION), 3])
        png3_ms += (time.monotonic() - t0) * 1000.0
        if ok:
            png3_bytes += len(encoded)

    ffv1_bytes, ffv1_fps, ffv1_error = _ffv1_bytes(frames, fps=max(1.0, len(frames) / max(1e-6, time.monotonic() - start)))
    note = "; ".join(part for part in (zstd_error, ffv1_error) if part)
    duration = time.monotonic() - start
    return LosslessReencodeResult(
        topic=topic,
        status="ok",
        source_encoding=source_encoding,
        frames=len(frames),
        width=w,
        height=h,
        duration_s=duration,
        source_kib_per_frame=_kib_per_frame(source_bytes, len(frames)),
        raw_kib_per_frame=_kib_per_frame(raw_bytes, len(frames)),
        zstd_kib_per_frame=_kib_per_frame(zstd_bytes, len(frames)) if compressor is not None else None,
        zstd_encode_ms=zstd_ms / len(frames) if compressor is not None else None,
        png1_kib_per_frame=_kib_per_frame(png1_bytes, len(frames)),
        png1_encode_ms=png1_ms / len(frames),
        png3_kib_per_frame=_kib_per_frame(png3_bytes, len(frames)),
        png3_encode_ms=png3_ms / len(frames),
        ffv1_kib_per_frame=_kib_per_frame(ffv1_bytes or 0, len(frames)) if ffv1_bytes is not None else None,
        ffv1_encode_fps=ffv1_fps,
        error=note,
    )


def http_connection(parsed: Any, *, timeout_s: float) -> http.client.HTTPConnection:
    if parsed.scheme == "https":
        ctx = ssl._create_unverified_context()
        return http.client.HTTPSConnection(parsed.hostname, parsed.port or 443, timeout=timeout_s, context=ctx)
    return http.client.HTTPConnection(parsed.hostname, parsed.port or 80, timeout=timeout_s)


def measure_http_path(
    base_url: str,
    path: str,
    *,
    duration_s: float,
    startup_timeout_s: float,
    read_chunk_bytes: int,
) -> HttpResult:
    url = base_url.rstrip("/") + path
    parsed = urlparse(url)
    request_path = parsed.path or "/"
    if parsed.query:
        request_path += "?" + parsed.query
    conn = http_connection(parsed, timeout_s=startup_timeout_s)
    start = time.monotonic()
    status = "ok"
    code: int | None = None
    headers_ms: float | None = None
    first_byte_ms: float | None = None
    video_format = ""
    bytes_read = 0
    error = ""
    try:
        conn.request("GET", request_path, headers={"Connection": "close"})
        response = conn.getresponse()
        headers_ms = (time.monotonic() - start) * 1000.0
        code = int(response.status)
        video_format = response.getheader("X-Indory-Video-Format", "") or response.getheader("Content-Type", "")
        if code >= 400:
            body = response.read(read_chunk_bytes)
            bytes_read = len(body)
            status = "http_error"
            return HttpResult(path, status, code, video_format, headers_ms, None, bytes_read, time.monotonic() - start, "")
        conn.sock.settimeout(0.25) if conn.sock is not None else None
        first_deadline = time.monotonic() + startup_timeout_s
        while first_byte_ms is None and time.monotonic() < first_deadline:
            try:
                chunk = response.read(1)
            except (OSError, socket.timeout):
                status = "no_body"
                return HttpResult(path, status, code, video_format, headers_ms, None, bytes_read, time.monotonic() - start, "")
            if not chunk:
                break
            first_byte_ms = (time.monotonic() - start) * 1000.0
            bytes_read += len(chunk)
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            try:
                chunk = response.read(read_chunk_bytes)
            except (OSError, socket.timeout):
                break
            if not chunk:
                break
            bytes_read += len(chunk)
        if first_byte_ms is None and status == "ok":
            status = "no_body"
    except Exception as exc:
        status = "error"
        error = str(exc)
    finally:
        conn.close()
    return HttpResult(path, status, code, video_format, headers_ms, first_byte_ms, bytes_read, time.monotonic() - start, error)


def fmt_ms(value: float | None) -> str:
    return "-" if value is None else f"{value:.1f}"


def fmt_rate(bytes_count: int, duration_s: float) -> str:
    if duration_s <= 0:
        return "-"
    return f"{bytes_count / duration_s / 1024.0:.1f}"


def markdown_zmq(results: list[ZmqResult]) -> str:
    lines = [
        "### ZMQ topic benchmark",
        "",
        "| topic | status | encoding | first msg ms | first init ms | msgs/s | payload KiB/s | init KiB/s | first seq | last seq | note |",
        "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for r in results:
        msg_rate = "-" if r.duration_s <= 0 else f"{r.messages / r.duration_s:.1f}"
        note = r.error
        lines.append(
            "| "
            + " | ".join(
                [
                    r.topic,
                    r.status,
                    r.encoding or "-",
                    fmt_ms(r.first_msg_ms),
                    fmt_ms(r.first_init_ms),
                    msg_rate,
                    fmt_rate(r.data_bytes, r.duration_s),
                    fmt_rate(r.init_bytes, r.duration_s),
                    str(r.first_seq) if r.first_seq is not None else "-",
                    str(r.last_seq) if r.last_seq is not None else "-",
                    note or "-",
                ]
            )
            + " |"
        )
    return "\n".join(lines)


def markdown_http(results: list[HttpResult]) -> str:
    lines = [
        "### WebXR HTTP endpoint benchmark",
        "",
        "| endpoint | status | HTTP | format | headers ms | first byte ms | KiB/s | bytes | note |",
        "|---|---:|---:|---|---:|---:|---:|---:|---|",
    ]
    for r in results:
        lines.append(
            "| "
            + " | ".join(
                [
                    r.path,
                    r.status,
                    str(r.http_code) if r.http_code is not None else "-",
                    r.video_format or "-",
                    fmt_ms(r.headers_ms),
                    fmt_ms(r.first_byte_ms),
                    fmt_rate(r.bytes_read, r.duration_s),
                    str(r.bytes_read),
                    r.error or "-",
                ]
            )
            + " |"
        )
    return "\n".join(lines)


def markdown_rgbd_zmq(results: list[RgbdZmqResult]) -> str:
    lines = [
        "### RGB-D ZMQ topic benchmark",
        "",
        "| topic | status | encoding | depth format | first msg ms | msgs/s | payload KiB/s | color KiB/s | depth KiB/s | color size | depth size | depth units | note |",
        "|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for r in results:
        msg_rate = "-" if r.duration_s <= 0 else f"{r.messages / r.duration_s:.1f}"
        lines.append(
            "| "
            + " | ".join(
                [
                    r.topic,
                    r.status,
                    r.encoding or "-",
                    r.depth_format or "-",
                    fmt_ms(r.first_msg_ms),
                    msg_rate,
                    fmt_rate(r.payload_bytes, r.duration_s),
                    fmt_rate(r.color_bytes, r.duration_s),
                    fmt_rate(r.depth_bytes, r.duration_s),
                    r.color_size or "-",
                    r.depth_size or "-",
                    str(r.depth_units) if r.depth_units is not None else "-",
                    r.error or "-",
                ]
            )
            + " |"
        )
    return "\n".join(lines)


def markdown_lossless(results: list[LosslessReencodeResult]) -> str:
    lines = [
        "### Lossless re-encode estimate",
        "",
        "This section samples JPEG-backed ZMQ frames and re-encodes the decoded BGR frame losslessly. "
        "It measures transport/encoder cost for lossless packaging; it does not recover information already lost by the camera MJPEG/JPEG source.",
        "",
        "| topic | status | source | frames | size | source KiB/frame | raw BGR KiB/frame | zstd raw KiB/frame | zstd ms/frame | PNG level1 KiB/frame | PNG level1 ms/frame | PNG level3 KiB/frame | PNG level3 ms/frame | FFV1 KiB/frame | FFV1 encode fps | note |",
        "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for r in results:
        size = f"{r.width}x{r.height}" if r.width and r.height else "-"
        lines.append(
            "| "
            + " | ".join(
                [
                    r.topic,
                    r.status,
                    r.source_encoding or "-",
                    str(r.frames),
                    size,
                    _fmt_num(r.source_kib_per_frame),
                    _fmt_num(r.raw_kib_per_frame),
                    _fmt_num(r.zstd_kib_per_frame),
                    _fmt_num(r.zstd_encode_ms, 2),
                    _fmt_num(r.png1_kib_per_frame),
                    _fmt_num(r.png1_encode_ms, 2),
                    _fmt_num(r.png3_kib_per_frame),
                    _fmt_num(r.png3_encode_ms, 2),
                    _fmt_num(r.ffv1_kib_per_frame),
                    _fmt_num(r.ffv1_encode_fps),
                    r.error or "-",
                ]
            )
            + " |"
        )
    return "\n".join(lines)


def as_json(
    zmq_results: list[ZmqResult],
    rgbd_zmq_results: list[RgbdZmqResult],
    http_results: list[HttpResult],
    lossless_results: list[LosslessReencodeResult],
) -> str:
    return json.dumps(
        {
            "zmq": [r.__dict__ for r in zmq_results],
            "rgbd_zmq": [r.__dict__ for r in rgbd_zmq_results],
            "http": [r.__dict__ for r in http_results],
            "lossless_reencode": [r.__dict__ for r in lossless_results],
        },
        indent=2,
        sort_keys=True,
    )


def main() -> int:
    args = parse_args()
    topics = tuple(args.topic or DEFAULT_ZMQ_TOPICS)
    rgbd_topics = tuple(args.rgbd_topic or DEFAULT_RGBD_ZMQ_TOPICS)
    paths = tuple(args.http_path or DEFAULT_HTTP_PATHS)
    lossless_topics = tuple(args.lossless_topic or DEFAULT_LOSSLESS_TOPICS)
    zmq_results: list[ZmqResult] = []
    rgbd_zmq_results: list[RgbdZmqResult] = []
    http_results: list[HttpResult] = []
    lossless_results: list[LosslessReencodeResult] = []
    if not args.skip_zmq:
        for topic in topics:
            zmq_results.append(
                measure_zmq_topic(
                    args.zmq_endpoint,
                    topic,
                    duration_s=float(args.duration_s),
                    startup_timeout_s=float(args.startup_timeout_s),
                )
            )
    if not args.skip_rgbd_zmq:
        for topic in rgbd_topics:
            rgbd_zmq_results.append(
                measure_rgbd_zmq_topic(
                    args.rgbd_zmq_endpoint,
                    topic,
                    duration_s=float(args.duration_s),
                    startup_timeout_s=float(args.startup_timeout_s),
                )
            )
    if not args.skip_http:
        for path in paths:
            http_results.append(
                measure_http_path(
                    args.http_base,
                    path,
                    duration_s=float(args.duration_s),
                    startup_timeout_s=float(args.startup_timeout_s),
                    read_chunk_bytes=int(args.read_chunk_bytes),
                )
            )
    if not args.skip_lossless:
        for topic in lossless_topics:
            lossless_results.append(
                measure_lossless_reencode(
                    args.zmq_endpoint,
                    topic,
                    sample_frames=int(args.lossless_sample_frames),
                    startup_timeout_s=float(args.startup_timeout_s),
                )
            )
    if args.json:
        print(as_json(zmq_results, rgbd_zmq_results, http_results, lossless_results))
    else:
        print(f"Camera format benchmark: duration={float(args.duration_s):g}s startup_timeout={float(args.startup_timeout_s):g}s")
        if zmq_results:
            print()
            print(markdown_zmq(zmq_results))
        if rgbd_zmq_results:
            print()
            print(markdown_rgbd_zmq(rgbd_zmq_results))
        if http_results:
            print()
            print(markdown_http(http_results))
        if lossless_results:
            print()
            print(markdown_lossless(lossless_results))
    failed = (
        [r for r in zmq_results if r.status == "error"]
        + [r for r in rgbd_zmq_results if r.status == "error"]
        + [r for r in http_results if r.status == "error"]
        + [r for r in lossless_results if r.status == "error"]
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
