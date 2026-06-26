#!/usr/bin/env python3
"""URDF-backed IK solver for the direct XLeRobot fast ZMQ protocol.

Examples:
  python3 -m tools.ik_solver --side right --delta 0.02 0 0
  python3 -m tools.ik_solver --side right --xyz 0.12 -0.36 0.88 --send
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Sequence


def _ensure_paths() -> None:
    roots = (
        Path(__file__).resolve().parents[1],
        Path("/home/pi/teleoperation/src"),
        Path("/home/pi/teleoperation"),
    )
    for root in reversed(roots):
        if root.exists():
            value = str(root)
            if value not in sys.path:
                sys.path.insert(0, value)


_ensure_paths()

try:
    import msgpack
    import zmq
except Exception as exc:  # pragma: no cover - CLI startup path
    if os.environ.get("INDOORY_IK_SOLVER_REEXEC") != "1":
        candidates = [
            Path(os.environ.get("XLE_ROBOT_VENV", "~/xlerobot-io-venv")).expanduser()
            / "bin"
            / "python3",
            Path("/home/pi/indory_isaac_sim/.venv-client/bin/python"),
            Path("~/.miniforge3/envs/lerobot/bin/python3").expanduser(),
        ]
        for python in candidates:
            if python.exists() and os.path.realpath(python) != os.path.realpath(sys.executable):
                env = os.environ.copy()
                env["INDOORY_IK_SOLVER_REEXEC"] = "1"
                os.execve(str(python), [str(python), *sys.argv], env)
    print(f"[err] pyzmq and msgpack are required: {exc}", file=sys.stderr)
    raise SystemExit(1)

try:
    import numpy as np
    from scipy.optimize import least_squares

    from indoory_isaac_sim.vr_teleop.workspace import (
        DEFAULT_SEED_SAMPLE_COUNT,
        VR_HOME_IK_SEED,
        _seed_cloud,
        _workspace_model,
        fk_ee_position_for_joints,
        project_ee_pose_to_kinematic_workspace,
        project_ee_pose_to_kinematic_workspace_accurate,
        project_ee_pose_to_kinematic_workspace_fast,
    )
    from indoory_isaac_sim.vr_teleop.bridge import (
        ROBOT_ARM_IK_JOINT_COUNT,
        ROBOT_ARM_IK_TOLERANCE_M,
        ROBOT_EXTERNAL_URDF_JOINT_NAMES,
        _robot_joint_rad_to_tick,
    )
    from indoory_isaac_sim.wire.schema import clamp_ee_pose_to_workspace
except Exception as exc:  # pragma: no cover - local environment dependent
    print(
        "[err] teleoperation IK modules are required. "
        "Expected /home/pi/teleoperation/src to be importable: "
        f"{type(exc).__name__}: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(1)


SCHEMA_VERSION_V11 = "xlerobot_v1.1"
FAST_JOINT_POS_ORDER = (
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
    "Rotation_2",
    "Pitch_2",
    "Elbow_2",
    "Wrist_Pitch_2",
    "Wrist_Roll_2",
    "Jaw_2",
    "head_pan_joint",
    "head_tilt_joint",
)
ARM_IK_NAMES = {
    "right": ("Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"),
    "left": ("Rotation_2", "Pitch_2", "Elbow_2", "Wrist_Pitch_2", "Wrist_Roll_2"),
}
ARM_CANONICAL_INDICES = {
    "right": (0, 1, 2, 3, 4),
    "left": (6, 7, 8, 9, 10),
}
TF_TARGET_NAMES = {"right": "gripper_right", "left": "gripper_left"}
SOURCE_ID = "indoory_ros.tools.ik_solver"


def _endpoint(host: str, port: int) -> str:
    return f"tcp://{host}:{port}"


def _pack(payload: dict[str, Any]) -> bytes:
    return msgpack.packb(payload, use_bin_type=True)


def _unpack(payload: bytes) -> dict[str, Any]:
    decoded = msgpack.unpackb(payload, raw=False)
    if not isinstance(decoded, dict):
        raise ValueError("message payload is not a dict")
    return decoded


def _finite_float_list(values: Any, length: int, name: str) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise SystemExit(f"{name} must contain exactly {length} numbers")
    out: list[float] = []
    for value in values:
        f = float(value)
        if not math.isfinite(f):
            raise SystemExit(f"{name} must contain finite numbers")
        out.append(f)
    return out


def _latest_fast_state(args: argparse.Namespace) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, 25)
    sock.connect(_endpoint(args.host, args.pub_port))
    topics = (f"proprio.{args.robot_id}", f"tf.links.{args.robot_id}")
    for topic in topics:
        sock.setsockopt(zmq.SUBSCRIBE, topic.encode())
    proprio: dict[str, Any] | None = None
    tf_links: dict[str, Any] | None = None
    deadline = time.monotonic() + max(0.05, float(args.timeout_s))
    try:
        while time.monotonic() < deadline:
            try:
                topic_b, payload_b = sock.recv_multipart()
            except zmq.Again:
                continue
            topic = topic_b.decode("utf-8", errors="replace")
            msg = _unpack(payload_b)
            if topic == topics[0]:
                proprio = msg
            elif topic == topics[1]:
                tf_links = msg
            if proprio is not None and tf_links is not None:
                break
    finally:
        sock.close(0)
    return proprio, tf_links


def _pose_from_tf_links(tf_links: dict[str, Any] | None, side: str) -> list[float] | None:
    if not isinstance(tf_links, dict):
        return None
    target_name = TF_TARGET_NAMES[side]
    for entry in tf_links.get("targets") or []:
        if not isinstance(entry, dict) or entry.get("name") != target_name:
            continue
        pose = entry.get("pose")
        if isinstance(pose, list) and len(pose) >= 7:
            return [float(v) for v in pose[:7]]
    return None


def _seed_from_proprio(proprio: dict[str, Any] | None, side: str) -> tuple[float, ...] | None:
    if not isinstance(proprio, dict):
        return None
    names = proprio.get("joint_names_urdf")
    values = proprio.get("joint_pos_urdf_rad")
    if not isinstance(names, list) or not isinstance(values, list):
        return None
    by_name: dict[str, float] = {}
    for idx, name in enumerate(names):
        if idx >= len(values):
            continue
        try:
            value = float(values[idx])
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            by_name[str(name)] = value
    seed = tuple(by_name.get(name) for name in ARM_IK_NAMES[side])
    if any(value is None for value in seed):
        return None
    return tuple(float(value) for value in seed if value is not None)


def _current_ticks_from_proprio(proprio: dict[str, Any] | None) -> list[float] | None:
    if not isinstance(proprio, dict):
        return None
    names = proprio.get("joint_names_pos")
    values = proprio.get("joint_pos")
    if not isinstance(names, list) or not isinstance(values, list):
        return None
    by_name: dict[str, float] = {}
    for idx, name in enumerate(names):
        if idx >= len(values):
            continue
        try:
            value = float(values[idx])
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            by_name[str(name)] = value
    if not all(name in by_name for name in FAST_JOINT_POS_ORDER):
        return None
    return [by_name[name] for name in FAST_JOINT_POS_ORDER]


def _bridge_index_for_joint(joint_name: str) -> int:
    try:
        return tuple(ROBOT_EXTERNAL_URDF_JOINT_NAMES).index(joint_name)
    except ValueError as exc:
        raise SystemExit(f"unknown URDF joint {joint_name!r}") from exc


def _joint_radians_to_ticks(side: str, q_values: tuple[float, ...]) -> dict[str, float]:
    out: dict[str, float] = {}
    for joint_name, value in zip(ARM_IK_NAMES[side], q_values, strict=True):
        tick = _robot_joint_rad_to_tick(_bridge_index_for_joint(joint_name), float(value))
        if tick is None or not math.isfinite(tick):
            raise SystemExit(f"could not map {joint_name}={value} rad to a Feetech tick")
        out[joint_name] = float(tick)
    return out


def _merged_canonical_targets(
    side: str,
    joint_ticks: dict[str, float],
    current_ticks: list[float] | None,
) -> list[float]:
    merged = list(current_ticks) if current_ticks is not None else [2048.0] * len(FAST_JOINT_POS_ORDER)
    for canonical_idx, joint_name in zip(ARM_CANONICAL_INDICES[side], ARM_IK_NAMES[side], strict=True):
        merged[canonical_idx] = float(joint_ticks[joint_name])
    return merged


def _sparse_canonical_targets(side: str, joint_ticks: dict[str, float]) -> list[float | None]:
    sparse: list[float | None] = [None] * len(FAST_JOINT_POS_ORDER)
    for canonical_idx, joint_name in zip(ARM_CANONICAL_INDICES[side], ARM_IK_NAMES[side], strict=True):
        sparse[canonical_idx] = float(joint_ticks[joint_name])
    return sparse


def _target_pose(args: argparse.Namespace, current_pose: list[float] | None) -> list[float]:
    if args.pose is not None:
        return _finite_float_list(args.pose, 7, "--pose")
    if args.xyz is not None:
        xyz = _finite_float_list(args.xyz, 3, "--xyz")
        quat = (
            _finite_float_list(args.quat, 4, "--quat")
            if args.quat is not None
            else (current_pose[3:7] if current_pose else [0.0, 0.0, 0.0, 1.0])
        )
        return [*xyz, *quat]
    if args.delta is not None:
        if current_pose is None:
            raise SystemExit("--delta requires a fresh tf.links current gripper pose")
        delta = _finite_float_list(args.delta, 3, "--delta")
        return [
            current_pose[0] + delta[0],
            current_pose[1] + delta[1],
            current_pose[2] + delta[2],
            *current_pose[3:7],
        ]
    if current_pose is None:
        raise SystemExit("no target specified and no current tf.links gripper pose is available")
    return list(current_pose)


def _send_target(args: argparse.Namespace, canonical_targets: list[float]) -> dict[str, Any]:
    payload = {
        "schema": SCHEMA_VERSION_V11,
        "source_id": args.source_id,
        "source_role": "tool",
        "priority": int(args.priority),
        "lease_ms": int(args.lease_ms),
        "stamp_ns": time.time_ns(),
        "arm_joint_pos_target": canonical_targets,
        "arm_joint_pos_target_units": "feetech_ticks",
    }
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUSH)
    timeout_ms = int(max(50.0, args.timeout_s * 1000))
    sock.setsockopt(zmq.LINGER, timeout_ms)
    sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
    sock.connect(_endpoint(args.host, args.pull_port))
    try:
        time.sleep(max(0.0, float(args.connect_settle_s)))
        sends = max(1, int(args.repeat))
        for idx in range(sends):
            payload["seq"] = idx
            payload["stamp_ns"] = time.time_ns()
            sock.send(_pack(payload))
            if idx + 1 < sends:
                time.sleep(max(0.0, float(args.repeat_dt_s)))
        time.sleep(max(0.0, float(args.flush_s)))
    finally:
        sock.close(0)
    return payload


# High-accuracy position IK for the debug tooling. The realtime teleop fast
# path (project_ee_pose_to_kinematic_workspace_fast) trades accuracy for a tiny
# solve budget and leaves a sizeable fraction of *reachable* targets above
# tolerance. This tool is not latency-bound, so it runs a bounded multi-start +
# tight least-squares polish that drives reachable targets to sub-millimetre and
# returns the true nearest reachable point otherwise. Cost is fixed (a bounded
# number of polishes), so unreachable targets do not blow up the solve time.
ACCURATE_FINE_TOLERANCE_M = 3.0e-4
ACCURATE_NEAREST_SEEDS = 8
ACCURATE_COARSE_MAX_NFEV = 25
ACCURATE_POLISH_MAX_NFEV = 100
# Rescue: a target that looks reachable (residual under this band) but is stuck
# above fine_m after the main scan gets extra restarts (further FK-cloud seeds
# plus deterministic random seeds). Catches the rare multi-basin miss without
# slowing the common path or genuinely-unreachable targets (which fall outside
# the band and skip it).
ACCURATE_RESCUE_BAND_M = 0.08
ACCURATE_RESCUE_RESTARTS = 8


def _polish_arm_joints(model: Any, q0: Any, target: Any, *, max_nfev: int) -> tuple[Any, float]:
    """Tight bounded least-squares from one seed; returns (q, position residual_m)."""
    lower, upper = model.lower, model.upper

    def residual(q: Any) -> Any:
        return model.fk(q) - target

    result = least_squares(
        residual,
        np.clip(np.asarray(q0, dtype=np.float64), lower, upper),
        bounds=(lower, upper),
        max_nfev=max(1, int(max_nfev)),
        xtol=1e-13,
        ftol=1e-13,
        gtol=1e-13,
    )
    reached = model.fk(result.x)
    return result.x, float(np.linalg.norm(reached - target))


def _target_rng_seed(target: Any) -> int:
    """Deterministic RNG seed from a target xyz so rescue restarts are repeatable."""
    key = tuple(int(round(float(v) * 1e4)) for v in np.asarray(target, dtype=np.float64).ravel()[:3])
    return abs(hash(key)) % (2**32)


def _accurate_arm_solution(
    side: str,
    target_pose: Sequence[float],
    q_seed: Sequence[float] | None,
    *,
    tolerance_m: float,
    fine_m: float = ACCURATE_FINE_TOLERANCE_M,
    nearest_seeds: int = ACCURATE_NEAREST_SEEDS,
    coarse_max_nfev: int = ACCURATE_COARSE_MAX_NFEV,
    polish_max_nfev: int = ACCURATE_POLISH_MAX_NFEV,
    rescue_band_m: float = ACCURATE_RESCUE_BAND_M,
    rescue_restarts: int = ACCURATE_RESCUE_RESTARTS,
) -> tuple[tuple[float, ...], float, str, bool, list[float]]:
    """Bounded multi-start position IK.

    Returns ``(joint_positions, residual_m, projection_mode, projected, pose)``.
    Reachable targets converge to sub-millimetre; unreachable targets return the
    nearest reachable point. The seed set is the live seed (when given), the VR
    home pose, the joint-range midpoint, and the ``nearest_seeds`` FK-cloud seeds
    closest to the target. A cheap coarse polish scans every seed to find the
    right basin (early-out once ``fine_m`` is reached); if none reaches it, one
    tight polish refines the best basin. This keeps cost bounded even for
    unreachable targets that never converge.
    """
    model = _workspace_model(side)
    lower, upper = model.lower, model.upper
    # Same coarse reach-sphere guard the fast/full paths apply (clamps only
    # out-of-sphere targets; reachable targets pass through unchanged).
    coarse = clamp_ee_pose_to_workspace(side, list(target_pose))
    target = np.asarray(coarse[:3], dtype=np.float64)

    samples, points = _seed_cloud(side, DEFAULT_SEED_SAMPLE_COUNT)
    order = np.argsort(np.linalg.norm(points - target, axis=1))

    nearest_seeds = max(0, int(nearest_seeds))
    seeds: list[Any] = []
    if q_seed is not None and len(q_seed) == len(model.joint_names):
        seeds.append(np.asarray(q_seed, dtype=np.float64))
    seeds.append(np.asarray(VR_HOME_IK_SEED, dtype=np.float64))
    seeds.append((lower + upper) * 0.5)
    seeds.extend(samples[idx] for idx in order[:nearest_seeds])

    best_q = np.clip(seeds[0], lower, upper)
    best_residual_m = float(np.linalg.norm(model.fk(best_q) - target))

    def scan(seed_list: list[Any]) -> bool:
        nonlocal best_q, best_residual_m
        for seed in seed_list:
            q, residual_m = _polish_arm_joints(model, seed, target, max_nfev=coarse_max_nfev)
            if residual_m < best_residual_m:
                best_q, best_residual_m = q, residual_m
            if best_residual_m <= fine_m:
                return True
        return False

    # Cheap coarse scan to find the right basin.
    hit_fine = best_residual_m <= fine_m or scan(seeds)
    # Rescue a target that looks reachable but is stuck in the wrong basin:
    # extra FK-cloud seeds plus deterministic random restarts. Skipped for
    # genuinely-unreachable targets (residual above the band) so they stay cheap.
    if not hit_fine and fine_m < best_residual_m <= rescue_band_m and rescue_restarts > 0:
        rng = np.random.default_rng(_target_rng_seed(target))
        extra = [samples[idx] for idx in order[nearest_seeds : nearest_seeds + int(rescue_restarts)]]
        extra.extend(rng.uniform(lower, upper) for _ in range(int(rescue_restarts)))
        hit_fine = scan(extra)
    if not hit_fine:
        # Tight polish on the best basin found — final convergence for reachable
        # targets, true nearest reachable point for unreachable ones.
        q, residual_m = _polish_arm_joints(model, best_q, target, max_nfev=polish_max_nfev)
        if residual_m < best_residual_m:
            best_q, best_residual_m = q, residual_m

    q_solution = tuple(float(v) for v in np.clip(best_q, lower, upper).tolist())
    projected = best_residual_m > float(tolerance_m)
    pose = list(coarse)
    if projected:
        reached = model.fk(np.asarray(q_solution, dtype=np.float64))
        pose[0:3] = [float(v) for v in reached.tolist()]
        mode = "accurate_nearest"
    else:
        mode = "accurate"
    return q_solution, best_residual_m, mode, projected, pose


def solve(args: argparse.Namespace) -> dict[str, Any]:
    proprio, tf_links = _latest_fast_state(args)
    current_pose = _pose_from_tf_links(tf_links, args.side)
    target_pose = _target_pose(args, current_pose)
    q_seed = args.seed if args.seed is not None else _seed_from_proprio(proprio, args.side)
    if q_seed is not None:
        q_seed = tuple(_finite_float_list(q_seed, ROBOT_ARM_IK_JOINT_COUNT, "--seed"))

    solver_mode = str(getattr(args, "solver", "accurate") or "accurate")
    if solver_mode == "fast":
        projection = project_ee_pose_to_kinematic_workspace_fast(
            args.side,
            target_pose,
            q_seed=q_seed,
            tolerance_m=float(args.tolerance_m),
            allow_emergency_seed=not args.no_emergency_seed,
        )
        if projection.joint_positions is None:
            raise SystemExit("IK solver did not return joint positions")
        q_solution = tuple(float(v) for v in projection.joint_positions)
        projected_pose = list(projection.pose)
        projected_flag = bool(projection.projected)
        projection_mode = projection.projection_mode
        solver_residual_m = float(projection.residual_m)
    else:
        projection = project_ee_pose_to_kinematic_workspace_accurate(
            args.side,
            target_pose,
            q_seed=q_seed,
            tolerance_m=float(args.tolerance_m),
            fine_tolerance_m=float(
                getattr(args, "fine_tolerance_m", ACCURATE_FINE_TOLERANCE_M)
            ),
            nearest_seeds=int(getattr(args, "nearest_seeds", ACCURATE_NEAREST_SEEDS)),
        )
        if projection.joint_positions is None:
            raise SystemExit("IK solver did not return joint positions")
        q_solution = tuple(float(v) for v in projection.joint_positions)
        projected_pose = list(projection.pose)
        projected_flag = bool(projection.projected)
        projection_mode = projection.projection_mode
        solver_residual_m = float(projection.residual_m)

    reached_xyz = list(fk_ee_position_for_joints(args.side, q_solution))
    ticks_by_joint = _joint_radians_to_ticks(args.side, q_solution)
    current_ticks = _current_ticks_from_proprio(proprio)
    merged_targets = _merged_canonical_targets(args.side, ticks_by_joint, current_ticks)
    sparse_targets = _sparse_canonical_targets(args.side, ticks_by_joint)
    target_xyz = [float(v) for v in target_pose[:3]]

    sent_payload = _send_target(args, merged_targets) if args.send else None
    return {
        "ok": True,
        "schema": "indoory_ros.tools.ik_solver.v1",
        "side": args.side,
        "robot_id": int(args.robot_id),
        "source": "teleoperation.vr_teleop.workspace",
        "seed_source": "cli" if args.seed is not None else ("proprio.joint_pos_urdf_rad" if q_seed is not None else "workspace_default"),
        "solver": solver_mode,
        "current_pose_base": current_pose,
        "target_pose_base": target_pose,
        "projected_pose_base": projected_pose,
        "projected": projected_flag,
        "projection_mode": projection_mode,
        "solver_residual_m": solver_residual_m,
        "target_error_m": float(math.dist(reached_xyz, target_xyz)),
        "joint_names_urdf": list(ARM_IK_NAMES[args.side]),
        "joint_solution_rad": list(q_solution),
        "fk_reached_xyz": reached_xyz,
        "joint_ticks_by_name": ticks_by_joint,
        "arm_joint_pos_target_sparse": sparse_targets,
        "arm_joint_pos_target": merged_targets,
        "arm_joint_pos_target_units": "feetech_ticks",
        "sent": sent_payload is not None,
        "sent_payload": sent_payload,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("FAST_ZMQ_HOST", "127.0.0.1"))
    parser.add_argument("--pub-port", type=int, default=int(os.environ.get("FAST_ZMQ_PUB_PORT", "8855")))
    parser.add_argument("--pull-port", type=int, default=int(os.environ.get("FAST_ZMQ_PULL_PORT", "8856")))
    parser.add_argument("--robot-id", type=int, default=int(os.environ.get("FAST_ZMQ_ROBOT_ID", "0")))
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--side", choices=("right", "left"), default="right")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    target.add_argument("--xyz", nargs=3, type=float, metavar=("X", "Y", "Z"))
    target.add_argument("--delta", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--quat", nargs=4, type=float, metavar=("QX", "QY", "QZ", "QW"))
    parser.add_argument("--seed", nargs=ROBOT_ARM_IK_JOINT_COUNT, type=float, metavar=("J0", "J1", "J2", "J3", "J4"))
    parser.add_argument("--tolerance-m", type=float, default=ROBOT_ARM_IK_TOLERANCE_M)
    parser.add_argument(
        "--solver",
        choices=("accurate", "fast"),
        default=os.environ.get("IK_SOLVER_MODE", "accurate"),
        help="accurate: bounded multi-start + polish (default, sub-mm on reachable targets); "
        "fast: the realtime teleop budget (lower accuracy, for comparison).",
    )
    parser.add_argument("--fine-tolerance-m", type=float, default=ACCURATE_FINE_TOLERANCE_M,
                        help="Accurate solver: stop polishing seeds once within this residual.")
    parser.add_argument("--nearest-seeds", type=int, default=ACCURATE_NEAREST_SEEDS,
                        help="Accurate solver: number of FK-cloud restart seeds nearest the target.")
    parser.add_argument("--no-emergency-seed", action="store_true")
    parser.add_argument("--send", action="store_true", help="Send the merged full arm_joint_pos_target to the fast PULL port.")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--repeat-dt-s", type=float, default=0.02)
    parser.add_argument("--connect-settle-s", type=float, default=float(os.environ.get("IK_ZMQ_CONNECT_SETTLE_S", "0.03")))
    parser.add_argument("--flush-s", type=float, default=float(os.environ.get("IK_ZMQ_FLUSH_S", "0.02")))
    parser.add_argument("--source-id", default=SOURCE_ID)
    parser.add_argument("--priority", type=int, default=80)
    parser.add_argument("--lease-ms", type=int, default=250)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    if args.quat is not None and args.xyz is None:
        parser.error("--quat can only be used with --xyz")
    if args.repeat < 1:
        parser.error("--repeat must be >= 1")
    return args


def main() -> int:
    args = parse_args()
    result = solve(args)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, separators=(",", ":") if args.compact else None, indent=None if args.compact else 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
