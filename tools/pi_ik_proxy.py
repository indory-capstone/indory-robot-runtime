#!/usr/bin/env python3
"""Pi-side IK proxy for VR teleop.

Sits between vr_teleop.bridge and the super-server pi_isaac_compat_gateway.

Data flow:
  vr_teleop.bridge  --PUSH--> pi_ik_proxy PULL (local 5566)
  pi_ik_proxy       --PUSH--> super pi_isaac_compat_gateway PULL (super:8856)

For commands that contain arm_ee_pose_target:
  - Compute IK using ikpy + xlerobot URDF
  - Replace arm_ee_pose_target with arm_joint_pos_target (feetech ticks)
  - Forward to super

For commands that do NOT contain arm_ee_pose_target (base_cmd_vel, etc.):
  - Forward as-is
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import sys
import time
import warnings
from pathlib import Path
from typing import Any

import msgpack
import zmq

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)
logger = logging.getLogger("pi_ik_proxy")

# ──────────────────────────────────────────────────────────────────────────────
# IK setup
# ──────────────────────────────────────────────────────────────────────────────

# XLeRobot arm joint names in URDF order (right arm = joints 0-5, left arm = joints 6-11)
# In pi_isaac_compat_gateway JOINT_POS_ORDER:
#   [Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw,          <- right arm (indices 0-5)
#    Rotation_2, Pitch_2, Elbow_2, Wrist_Pitch_2, Wrist_Roll_2, Jaw_2,  <- left arm (indices 6-11)
#    head_pan_joint, head_tilt_joint]
#
# robot_io/xlerobot_fast_io.py local order:
#   [left_hand_1..6, right_hand_1..6, head_pan, head_tilt]
# _canonical_joint_ticks_to_local swaps right/left:
#   right = joints[:6], left = joints[6:12] -> [left, right, head]
# So we need to fill:
#   joints[0..5]  = right arm (Rotation..Jaw)
#   joints[6..11] = left arm  (Rotation_2..Jaw_2)

FEETECH_CENTER = 2048
FEETECH_PER_RAD = 4096.0 / (2.0 * math.pi)
HEAD_TILT_RELATIVE_TICK_SIGN = -1.0

# IK chain link sequences in URDF
RIGHT_ARM_ACTIVE_JOINTS = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
LEFT_ARM_ACTIVE_JOINTS  = ["Rotation_2", "Pitch_2", "Elbow_2", "Wrist_Pitch_2", "Wrist_Roll_2", "Jaw_2"]

# Default URDF path (override with --urdf)
DEFAULT_URDF = "/home/pi/indory_isaac_sim/src/indoory_isaac_sim/assets/data/robots/xlerobot/xlerobot.urdf"

HAS_IK = False
_chains: dict[str, Any] = {}
_np = None


def _try_init_ik(urdf_path: str) -> bool:
    global HAS_IK, _np
    try:
        import numpy as np
        import ikpy.chain
        _np = np
    except ImportError as e:
        logger.warning("ikpy/numpy not available — arm IK disabled: %s", e)
        return False

    if not os.path.exists(urdf_path):
        logger.warning("URDF not found at %s — arm IK disabled", urdf_path)
        return False

    logger.info("Loading IK chains from %s", urdf_path)
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            for side, active_joints in [("right", RIGHT_ARM_ACTIVE_JOINTS),
                                         ("left",  LEFT_ARM_ACTIVE_JOINTS)]:
                chain = ikpy.chain.Chain.from_urdf_file(
                    urdf_path,
                    active_links_mask=None,
                )
                # Build chain from only the joints we want
                # Re-create chain with only active joints
                chain = ikpy.chain.Chain.from_urdf_file(
                    urdf_path,
                    active_links_mask=_build_active_mask(urdf_path, active_joints),
                )
                _chains[side] = chain
                logger.info("IK chain loaded for %s arm (%d active joints)", side, len(active_joints))
        HAS_IK = True
        return True
    except Exception as e:
        logger.warning("Failed to load IK chains: %s — arm IK disabled", e)
        return False


def _build_active_mask(urdf_path: str, active_joint_names: list[str]) -> list[bool] | None:
    """Build active_links_mask for ikpy Chain from URDF joint names."""
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        mask = [False]  # ikpy adds a dummy base element
        for joint in root.findall("joint"):
            jtype = joint.get("type", "fixed")
            jname = joint.get("name", "")
            if jtype == "fixed":
                continue  # ikpy skips fixed joints
            mask.append(jname in active_joint_names)
        return mask if any(mask) else None
    except Exception as e:
        logger.warning("Could not build IK mask: %s", e)
        return None


def _rad_to_ticks(rad: float) -> float:
    return float(FEETECH_CENTER + rad * FEETECH_PER_RAD)


def _ee_pose_to_joint_ticks(side: str, pose: list[float]) -> list[float] | None:
    """Convert EE pose [x,y,z, qx,qy,qz,qw] to 6 feetech ticks for the arm."""
    if not HAS_IK or side not in _chains:
        return None
    np = _np
    try:
        x, y, z = pose[0], pose[1], pose[2]
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]

        # Build 4×4 target matrix
        # Rotation from quaternion (xyzw → scipy convention)
        # Manual quaternion → rotation matrix
        r11 = 1 - 2*(qy*qy + qz*qz)
        r12 = 2*(qx*qy - qz*qw)
        r13 = 2*(qx*qz + qy*qw)
        r21 = 2*(qx*qy + qz*qw)
        r22 = 1 - 2*(qx*qx + qz*qz)
        r23 = 2*(qy*qz - qx*qw)
        r31 = 2*(qx*qz - qy*qw)
        r32 = 2*(qy*qz + qx*qw)
        r33 = 1 - 2*(qx*qx + qy*qy)

        target = np.array([
            [r11, r12, r13, x],
            [r21, r22, r23, y],
            [r31, r32, r33, z],
            [0,   0,   0,   1],
        ], dtype=np.float64)

        chain = _chains[side]
        angles = chain.inverse_kinematics(target)
        # angles has one entry per link in the chain (including fixed base/tip)
        # Extract the active ones
        active_mask = chain.active_links_mask
        active_angles = [a for a, m in zip(angles, active_mask) if m]

        if len(active_angles) < 5:
            logger.warning("IK returned fewer angles than expected: %d", len(active_angles))
            return None

        ticks = [_rad_to_ticks(a) for a in active_angles[:6]]
        # Pad to 6 if fewer returned
        while len(ticks) < 6:
            ticks.append(float(FEETECH_CENTER))
        return ticks[:6]
    except Exception as e:
        logger.warning("IK failed for %s arm: %s", side, e)
        return None


# ──────────────────────────────────────────────────────────────────────────────
# Command processing
# ──────────────────────────────────────────────────────────────────────────────

def _process_command(command: dict[str, Any]) -> dict[str, Any] | None:
    """
    If arm_ee_pose_target present: compute IK → replace with arm_joint_pos_target.
    Otherwise: pass through unchanged.
    Returns None to drop the command.
    """
    arm_ee = command.get("arm_ee_pose_target")
    if not isinstance(arm_ee, dict) or not arm_ee:
        # No EE target — forward as-is (base vel, joint relative, etc.)
        return command

    # EE pose → joint ticks
    joint_targets = [float(FEETECH_CENTER)] * 14  # [right×6, left×6, head×2]

    ee_resolved = False
    for side in ("right", "left"):
        entry = arm_ee.get(side)
        if not isinstance(entry, dict):
            continue
        pose = entry.get("pose")
        if not isinstance(pose, list) or len(pose) < 7:
            continue

        ticks = _ee_pose_to_joint_ticks(side, pose)
        if ticks is None:
            logger.warning("IK returned None for %s — skipping arm side", side)
            continue

        ee_resolved = True
        if side == "right":
            joint_targets[0:6] = ticks
        else:  # left
            joint_targets[6:12] = ticks

    if not ee_resolved:
        # Nothing could be resolved — drop to avoid sending garbage
        return None

    # Handle gripper from arm_joint_relative_target if present
    arm_rel = command.get("arm_joint_relative_target") or {}
    for side in ("right", "left"):
        delta = (arm_rel.get(side) or {}).get("gripper", 0.0)
        if abs(delta) > 1e-6:
            idx = 5 if side == "right" else 11
            joint_targets[idx] = float(FEETECH_CENTER) + delta * FEETECH_PER_RAD

    # Head
    head = command.get("head_joint_relative_target") or {}
    head_pan_delta = float(head.get("head_pan", 0.0))
    head_tilt_delta = float(head.get("head_tilt", 0.0))
    joint_targets[12] = float(FEETECH_CENTER) + head_pan_delta * FEETECH_PER_RAD
    joint_targets[13] = (
        float(FEETECH_CENTER)
        + head_tilt_delta * HEAD_TILT_RELATIVE_TICK_SIGN * FEETECH_PER_RAD
    )

    # Build new command replacing arm_ee_pose_target with arm_joint_pos_target
    new_cmd = {k: v for k, v in command.items()
               if k not in ("arm_ee_pose_target", "arm_joint_relative_target",
                            "head_joint_relative_target")}
    new_cmd["schema"] = "xlerobot_v1"          # gateway requires v1 for raw joint targets
    new_cmd["arm_joint_pos_target"] = joint_targets
    new_cmd["arm_joint_pos_target_units"] = "feetech_ticks"

    # base_cmd_vel must be present for v1 schema
    if "base_cmd_vel" not in new_cmd:
        new_cmd["base_cmd_vel"] = [0.0, 0.0, 0.0]

    return new_cmd


# ──────────────────────────────────────────────────────────────────────────────
# Main proxy loop
# ──────────────────────────────────────────────────────────────────────────────

def run(args: argparse.Namespace) -> None:
    _try_init_ik(args.urdf)

    ctx = zmq.Context.instance()

    pull = ctx.socket(zmq.PULL)
    pull.setsockopt(zmq.LINGER, 0)
    pull.setsockopt(zmq.RCVHWM, 16)
    pull.bind(f"tcp://{args.bind_host}:{args.pull_port}")
    logger.info("PULL listening on tcp://%s:%s", args.bind_host, args.pull_port)

    push = ctx.socket(zmq.PUSH)
    push.setsockopt(zmq.LINGER, 0)
    push.setsockopt(zmq.SNDHWM, 16)
    push.connect(f"tcp://{args.sim_host}:{args.sim_pull_port}")
    logger.info("PUSH forwarding to tcp://%s:%s", args.sim_host, args.sim_pull_port)

    stop = False

    def _handle_signal(sig, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    poller = zmq.Poller()
    poller.register(pull, zmq.POLLIN)

    stats = {"received": 0, "ik_converted": 0, "forwarded": 0, "dropped": 0}
    last_log = time.monotonic()

    while not stop:
        try:
            events = dict(poller.poll(200))
        except zmq.ZMQError:
            break
        if pull not in events:
            now = time.monotonic()
            if now - last_log > 30.0:
                logger.info("pi_ik_proxy stats: %s", stats)
                last_log = now
            continue

        try:
            raw = pull.recv(flags=zmq.NOBLOCK)
            command = msgpack.unpackb(raw, raw=False)
        except (zmq.Again, Exception) as e:
            logger.warning("Receive/decode error: %s", e)
            continue

        stats["received"] += 1

        if not isinstance(command, dict):
            stats["dropped"] += 1
            continue

        had_ee = "arm_ee_pose_target" in command
        out = _process_command(command)

        if out is None:
            stats["dropped"] += 1
            logger.warning("Dropped command (IK failed and no fallback)")
            continue

        if had_ee:
            stats["ik_converted"] += 1

        try:
            push.send(msgpack.packb(out, use_bin_type=True), flags=zmq.NOBLOCK)
            stats["forwarded"] += 1
        except zmq.Again:
            stats["dropped"] += 1
            logger.warning("PUSH queue full — dropped command")

        now = time.monotonic()
        if now - last_log > 30.0:
            logger.info("pi_ik_proxy stats: %s", stats)
            last_log = now

    pull.close(0)
    push.close(0)
    ctx.term()
    logger.info("pi_ik_proxy stopped. Final stats: %s", stats)


def main() -> int:
    parser = argparse.ArgumentParser(description="Pi-side IK proxy for VR teleop commands")
    parser.add_argument("--bind-host", default="127.0.0.1",
                        help="Host to bind the local PULL socket on")
    parser.add_argument("--pull-port", type=int, default=5566,
                        help="Local port that vr_teleop.bridge pushes to (default: 5566)")
    parser.add_argument("--sim-host", default="127.0.0.1",
                        help="Control server host for forwarding")
    parser.add_argument("--sim-pull-port", type=int, default=8856,
                        help="Super server PULL port (pi_isaac_compat_gateway)")
    parser.add_argument("--urdf", default=DEFAULT_URDF,
                        help="Path to xlerobot URDF for IK")
    args = parser.parse_args()
    run(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
