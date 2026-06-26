#!/usr/bin/env python3
"""Small ZMQ client for the direct low-latency robot ports."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import defaultdict
from typing import Any

try:
    import msgpack
    import zmq
except Exception as exc:  # pragma: no cover - CLI startup path
    if os.environ.get("FAST_ROBOT_CLIENT_REEXEC") != "1":
        candidates = [
            os.path.join(os.path.expanduser(os.environ.get("XLE_ROBOT_VENV", "~/xlerobot-io-venv")), "bin", "python3"),
            os.path.expanduser("~/.miniforge3/envs/lerobot/bin/python3"),
        ]
        for python in candidates:
            if os.path.exists(python) and os.path.realpath(python) != os.path.realpath(sys.executable):
                env = os.environ.copy()
                env["FAST_ROBOT_CLIENT_REEXEC"] = "1"
                os.execve(python, [python, *sys.argv], env)
    print(f"[err] pyzmq and msgpack are required: {exc}", file=sys.stderr)
    raise SystemExit(1)


SCHEMA_VERSION_V11 = "xlerobot_v1.1"


def pack(payload: dict[str, Any]) -> bytes:
    return msgpack.packb(payload, use_bin_type=True)


def unpack(payload: bytes) -> dict[str, Any]:
    decoded = msgpack.unpackb(payload, raw=False)
    if not isinstance(decoded, dict):
        raise ValueError("payload is not a dict")
    return decoded


def endpoint(host: str, port: int) -> str:
    return f"tcp://{host}:{port}"


def print_json(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False))


def make_req_socket(ctx: zmq.Context, host: str, port: int, timeout_ms: int) -> zmq.Socket:
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
    sock.connect(endpoint(host, port))
    return sock


def rpc(args: argparse.Namespace, op: str, **payload: Any) -> dict[str, Any]:
    ctx = zmq.Context.instance()
    sock = make_req_socket(ctx, args.host, args.rep_port, args.timeout_ms)
    try:
        sock.send(pack({"op": op, **payload}))
        return unpack(sock.recv())
    finally:
        sock.close(0)


def make_push_socket(ctx: zmq.Context, host: str, port: int) -> zmq.Socket:
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.setsockopt(zmq.SNDTIMEO, 0)
    try:
        sock.setsockopt(zmq.CONFLATE, 1)
    except zmq.ZMQError:
        pass
    sock.connect(endpoint(host, port))
    return sock


def send_base_command(
    sock: zmq.Socket,
    vx: float,
    vy: float,
    wz: float,
    *,
    seq: int,
    source: str,
) -> bool:
    payload = {
        "schema": SCHEMA_VERSION_V11,
        "source_id": source,
        "seq": seq,
        "stamp_ns": time.time_ns(),
        "frame": "body",
        "base_cmd_vel": [vx, vy, wz],
    }
    try:
        sock.send(pack(payload), flags=zmq.NOBLOCK)
        return True
    except zmq.Again:
        return False


def cmd_health(args: argparse.Namespace) -> int:
    print_json(rpc(args, "health"))
    return 0


def cmd_topics(args: argparse.Namespace) -> int:
    print_json(rpc(args, "topic_list"))
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    print_json(rpc(args, "command_status"))
    return 0


def cmd_head_debug(args: argparse.Namespace) -> int:
    print_json(rpc(args, "head_debug"))
    return 0


def cmd_rescan(args: argparse.Namespace) -> int:
    print_json(rpc(args, "request_rescan", force=args.force))
    return 0


def cmd_stop(args: argparse.Namespace) -> int:
    ctx = zmq.Context.instance()
    sock = make_push_socket(ctx, args.host, args.pull_port)
    try:
        for seq in range(3):
            send_base_command(sock, 0.0, 0.0, 0.0, seq=seq, source="fast_robot_client.stop")
            time.sleep(0.02)
    finally:
        sock.close(0)
    print_json(rpc(args, "stop"))
    return 0


def cmd_estop(args: argparse.Namespace) -> int:
    print_json(rpc(args, "set_estop", enabled=args.enabled))
    return 0


def cmd_move(args: argparse.Namespace) -> int:
    for name, value in (("vx", args.vx), ("vy", args.vy), ("wz", args.wz)):
        if not math.isfinite(value):
            raise SystemExit(f"{name} must be finite")
    if args.duration < 0.0:
        raise SystemExit("duration must be >= 0")

    ctx = zmq.Context.instance()
    sock = make_push_socket(ctx, args.host, args.pull_port)
    seq = 0
    sent = 0
    dropped = 0
    delay = 1.0 / max(1.0, args.rate_hz)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            ok = send_base_command(
                sock,
                args.vx,
                args.vy,
                args.wz,
                seq=seq,
                source="fast_robot_client.move",
            )
            sent += int(ok)
            dropped += int(not ok)
            seq += 1
            time.sleep(delay)
        for _ in range(3):
            send_base_command(sock, 0.0, 0.0, 0.0, seq=seq, source="fast_robot_client.move.stop")
            seq += 1
            time.sleep(0.02)
    finally:
        sock.close(0)
    print_json({"ok": True, "sent": sent, "dropped_local": dropped, "duration": args.duration})
    return 0


def send_head_relative_command(
    sock: zmq.Socket,
    pan: float,
    tilt: float,
    *,
    source: str,
) -> bool:
    payload = {
        "schema": SCHEMA_VERSION_V11,
        "source_id": source,
        "stamp_ns": time.time_ns(),
        "frame": "body",
        "head_joint_relative_target": {
            "head_pan": float(pan),
            "head_tilt": float(tilt),
        },
    }
    try:
        sock.send(pack(payload), flags=zmq.NOBLOCK)
        return True
    except zmq.Again:
        return False


def send_head_sparse_command(
    sock: zmq.Socket,
    pan_tick: float | None,
    tilt_tick: float | None,
    *,
    source: str,
) -> bool:
    targets: list[float | None] = [None] * 14
    if pan_tick is not None:
        targets[12] = float(pan_tick)
    if tilt_tick is not None:
        targets[13] = float(tilt_tick)
    payload = {
        "schema": SCHEMA_VERSION_V11,
        "source_id": source,
        "stamp_ns": time.time_ns(),
        "frame": "body",
        "joint_targets_sparse": targets,
    }
    try:
        sock.send(pack(payload), flags=zmq.NOBLOCK)
        return True
    except zmq.Again:
        return False


def cmd_head_nudge(args: argparse.Namespace) -> int:
    for name, value in (("pan", args.pan), ("tilt", args.tilt)):
        if not math.isfinite(value):
            raise SystemExit(f"{name} must be finite")
    ctx = zmq.Context.instance()
    sock = make_push_socket(ctx, args.host, args.pull_port)
    try:
        ok = send_head_relative_command(
            sock,
            args.pan,
            args.tilt,
            source=args.source_id,
        )
        time.sleep(0.05)
    finally:
        sock.close(0)
    print_json({"ok": ok, "head_pan_rad": args.pan, "head_tilt_rad": args.tilt})
    return 0 if ok else 1


def cmd_head_raw(args: argparse.Namespace) -> int:
    pan_tick = args.pan_tick
    tilt_tick = args.tilt_tick
    if pan_tick is None and tilt_tick is None:
        raise SystemExit("provide --pan-tick and/or --tilt-tick")
    for name, value in (("pan_tick", pan_tick), ("tilt_tick", tilt_tick)):
        if value is not None and not math.isfinite(value):
            raise SystemExit(f"{name} must be finite")
    ctx = zmq.Context.instance()
    sock = make_push_socket(ctx, args.host, args.pull_port)
    try:
        ok = send_head_sparse_command(
            sock,
            pan_tick,
            tilt_tick,
            source=args.source_id,
        )
        time.sleep(0.05)
    finally:
        sock.close(0)
    print_json({"ok": ok, "head_pan_tick": pan_tick, "head_tilt_tick": tilt_tick})
    return 0 if ok else 1


def describe(topic: str, payload: dict[str, Any]) -> str:
    stamp_ns = payload.get("stamp_ns")
    age_ms = None
    if isinstance(stamp_ns, int):
        age_ms = (time.time_ns() - stamp_ns) / 1_000_000.0
    prefix = f"{topic} age={age_ms:.1f}ms" if age_ms is not None else topic
    if topic.startswith("scan."):
        return f"{prefix} ranges={payload.get('num_ranges')}"
    if topic.startswith("proprio."):
        return f"{prefix} base={payload.get('base_joint_vel')} cmd_age={payload.get('base_command_age_ms')}"
    if topic.startswith("joint_states."):
        return f"{prefix} joints={len(payload.get('position') or [])}"
    if topic.startswith("odom."):
        msg = payload.get("msg") or {}
        twist = ((msg.get("twist") or {}).get("twist") or {})
        return f"{prefix} twist={twist}"
    return f"{prefix} keys={','.join(sorted(payload.keys()))}"


def cmd_watch(args: argparse.Namespace) -> int:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVHWM, args.hwm)
    subscriptions = args.topic or [""]
    for topic in subscriptions:
        sock.setsockopt(zmq.SUBSCRIBE, topic.encode("ascii"))
    sock.connect(endpoint(args.host, args.pub_port))

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    counts: dict[str, int] = defaultdict(int)
    first_seen: dict[str, float] = {}
    last_seen: dict[str, float] = {}
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            events = dict(poller.poll(100))
            if sock not in events:
                continue
            try:
                topic_raw, payload_raw = sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                continue
            topic = topic_raw.decode("ascii", errors="replace")
            payload = unpack(payload_raw)
            now = time.monotonic()
            counts[topic] += 1
            first_seen.setdefault(topic, now)
            last_seen[topic] = now
            if args.verbose or counts[topic] == 1:
                print(describe(topic, payload), flush=True)
    finally:
        sock.close(0)

    summary = {}
    for topic, count in sorted(counts.items()):
        elapsed = max(1e-9, last_seen[topic] - first_seen[topic])
        summary[topic] = {"count": count, "hz": count / elapsed if count > 1 else 0.0}
    print_json({"ok": True, "duration": args.duration, "topics": summary})
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate direct fast ZMQ robot ports")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--pub-port", type=int, default=8855)
    parser.add_argument("--pull-port", type=int, default=8856)
    parser.add_argument("--rep-port", type=int, default=8857)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("health").set_defaults(func=cmd_health)
    sub.add_parser("topics").set_defaults(func=cmd_topics)
    sub.add_parser("status").set_defaults(func=cmd_status)
    sub.add_parser("head-debug").set_defaults(func=cmd_head_debug)
    rescan = sub.add_parser("rescan")
    rescan.add_argument("--force", action=argparse.BooleanOptionalAction, default=True)
    rescan.set_defaults(func=cmd_rescan)
    sub.add_parser("stop").set_defaults(func=cmd_stop)

    estop = sub.add_parser("estop")
    estop.add_argument("enabled", type=lambda raw: raw.lower() in ("1", "true", "yes", "on"))
    estop.set_defaults(func=cmd_estop)

    move = sub.add_parser("move")
    move.add_argument("--vx", type=float, default=0.0, help="body-frame x velocity in m/s")
    move.add_argument("--vy", type=float, default=0.0, help="body-frame y velocity in m/s")
    move.add_argument("--wz", type=float, default=0.0, help="body-frame yaw velocity in rad/s")
    move.add_argument("--duration", type=float, default=0.5)
    move.add_argument("--rate-hz", type=float, default=60.0)
    move.set_defaults(func=cmd_move)

    head_nudge = sub.add_parser("head-nudge")
    head_nudge.add_argument("--pan", type=float, default=0.0, help="relative head pan delta in radians")
    head_nudge.add_argument("--tilt", type=float, default=0.0, help="relative head tilt delta in radians")
    head_nudge.add_argument("--source-id", default="fast_robot_client.head_nudge")
    head_nudge.set_defaults(func=cmd_head_nudge)

    head_raw = sub.add_parser("head-raw")
    head_raw.add_argument("--pan-tick", type=float, default=None, help="absolute raw tick target for logical head_pan")
    head_raw.add_argument("--tilt-tick", type=float, default=None, help="absolute raw tick target for logical head_tilt")
    head_raw.add_argument("--source-id", default="fast_robot_client.head_raw")
    head_raw.set_defaults(func=cmd_head_raw)

    watch = sub.add_parser("watch")
    watch.add_argument("--duration", type=float, default=5.0)
    watch.add_argument("--topic", action="append", help="topic prefix to subscribe; repeatable")
    watch.add_argument("--hwm", type=int, default=64)
    watch.add_argument("--verbose", action="store_true")
    watch.set_defaults(func=cmd_watch)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
