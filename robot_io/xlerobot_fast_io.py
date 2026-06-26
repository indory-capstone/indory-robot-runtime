#!/usr/bin/env python3
"""Hardware-facing XLeRobot I/O over the direct fast ZMQ protocol.

This script is meant for the Raspberry Pi / robot computer. It does not import
ROS 2, start DDS, or talk to rosbridge. ROS compatibility lives in
``ros_bridge`` and wraps the fast ZMQ ports exposed here.
"""

from __future__ import annotations

import argparse
import array
import json
import math
import os
import queue
import signal
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from functools import lru_cache
from typing import Any, Optional

import numpy as np

try:
    import serial
except Exception as exc:  # pragma: no cover - runtime optional
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

try:
    import pyrealsense2 as rs
except Exception as exc:  # pragma: no cover - runtime optional
    rs = None
    DEPTH_SENSOR_IMPORT_ERROR = exc
else:
    DEPTH_SENSOR_IMPORT_ERROR = None

try:
    import msgpack
    import zmq
except Exception as exc:  # pragma: no cover - runtime optional
    msgpack = None
    zmq = None
    FAST_ZMQ_IMPORT_ERROR = exc
else:
    FAST_ZMQ_IMPORT_ERROR = None


CMD_STOP = 0x25
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52

SCHEMA_VERSION = "xlerobot_v1"
SCHEMA_VERSION_V11 = "xlerobot_v1.1"
FAST_ZMQ_SCHEMA = "indoory_robot_fast_v1"
COMMAND_SOURCE_ROLE_PRIORITY = {
    "safety": 100,
    "teleop": 50,
    "policy": 30,
    "script": 10,
}
FEETECH_TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)
HEAD_RELATIVE_TICK_SIGNS = {
    "head_pan": 1.0,
    "head_tilt": -1.0,
}
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
FAST_BASE_VEL_ORDER = (
    "root_x_axis_joint",
    "root_y_axis_joint",
    "root_z_rotation_joint",
)
FAST_TF_STATIC_ANCHORS = {
    "head_pan": [0.0, 0.0, 0.44, 0.0, 0.0, 0.0, 1.0],
    "head_tilt": [0.05, 0.0, 0.46, 0.0, 0.0, 0.0, 1.0],
}
FAST_TF_IMPORTANT_LINKS = (
    "root",
    "root_arm_1_link_1",
    "root_arm_1_link_2",
    "base_link",
    "top_base_link",
    "Base",
    "Base_2",
    "Fixed_Jaw_tip",
    "Fixed_Jaw_tip_2",
    "head_pan_link",
    "head_tilt_link",
    "head_camera_link",
    "head_camera_rgb_frame",
    "head_camera_rgb_optical_frame",
    "head_camera_depth_frame",
    "head_camera_depth_optical_frame",
)
FAST_TF_DEFAULT_INVERT_URDF_JOINTS = (
    # Shoulder lift raw ticks move opposite the URDF Pitch axis on the hardware.
    "Pitch",
    "Pitch_2",
    "head_pan_joint",
)
FAST_TF_LIMIT_MAPPED_URDF_JOINTS = frozenset(
    (
        # Lift/elbow joints span roughly 180 degrees mechanically. Map their
        # calibrated raw tick ranges onto the URDF limits instead of using a
        # centered +/- tick-to-radian conversion.
        "Pitch",
        "Elbow",
        "Pitch_2",
        "Elbow_2",
    )
)
FAST_TF_HEAD_ZERO_TICKS: dict[str, float] = {}
FAST_TF_HEAD_DELTA_SIGNS = {
    "head_pan_joint": 1.0,
    # Hardware head-up must also be TF/head-up. The previous negative sign made
    # physical up/down appear inverted in Foxglove and depth projection.
    "head_tilt_joint": 1.0,
}
FAST_TF_HEAD_JOINT_TO_XLE_MOTOR = {
    "head_pan_joint": "head_motor_2",
    "head_tilt_joint": "head_motor_1",
}

FAST_TF_ALIAS_LINKS = {
    "gripper_right": "Fixed_Jaw_tip",
    "jaw_right": "Fixed_Jaw_tip",
    "gripper_left": "Fixed_Jaw_tip_2",
    "jaw_left": "Fixed_Jaw_tip_2",
    "head_pan": "head_pan_link",
    "head_tilt": "head_tilt_link",
}
FAST_ARM_FK_JOINTS = {
    "right": (
        ("right_hand_1", "right_arm_shoulder_pan"),
        ("right_hand_2", "right_arm_shoulder_lift"),
        ("right_hand_3", "right_arm_elbow_flex"),
        ("right_hand_4", "right_arm_wrist_flex"),
        ("right_hand_5", "right_arm_wrist_roll"),
    ),
    "left": (
        ("left_hand_1", "left_arm_shoulder_pan"),
        ("left_hand_2", "left_arm_shoulder_lift"),
        ("left_hand_3", "left_arm_elbow_flex"),
        ("left_hand_4", "left_arm_wrist_flex"),
        ("left_hand_5", "left_arm_wrist_roll"),
    ),
}
_FK_EE_POSITION_FOR_JOINTS = None
_FK_EE_IMPORT_ATTEMPTED = False
_FK_EE_IMPORT_ERROR: Exception | None = None
_IK_PROJECT_FAST = None
_IK_IMPORT_ATTEMPTED = False
_IK_IMPORT_ERROR: Exception | None = None
_MODEL_IMPORT_ERROR: Exception | None = None

# URDF arm IK joints per side, in the order the workspace solver returns them,
# and their slot in ROS_JOINT_NAMES (left_hand_1..6, right_hand_1..6, head_*).
ARM_IK_URDF_NAMES = {
    "right": ("Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"),
    "left": ("Rotation_2", "Pitch_2", "Elbow_2", "Wrist_Pitch_2", "Wrist_Roll_2"),
}
ARM_IK_URDF_TO_ROS_INDEX = {
    "Rotation": 6, "Pitch": 7, "Elbow": 8, "Wrist_Pitch": 9, "Wrist_Roll": 10,
    "Rotation_2": 0, "Pitch_2": 1, "Elbow_2": 2, "Wrist_Pitch_2": 3, "Wrist_Roll_2": 4,
}
# Safety clamp on the per-command raw-tick step for EE-target IK so a single bad
# or large target cannot fling the arm (~200 ticks ≈ 17.6 deg). 0 disables.
try:
    EE_IK_MAX_TICK_DELTA = float(os.environ.get("XLEROBOT_EE_IK_MAX_TICK_DELTA", "200"))
except (TypeError, ValueError):
    EE_IK_MAX_TICK_DELTA = 200.0


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def env_int_list(name: str, default: tuple[int, ...]) -> list[int]:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return list(default)
    values: list[int] = []
    for item in raw.replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue
        try:
            values.append(int(item))
        except ValueError:
            return list(default)
    return values or list(default)


def env_first(names: tuple[str, ...], default: str) -> str:
    for name in names:
        value = os.environ.get(name)
        if value is not None:
            return value
    return default


def stamp() -> dict[str, int]:
    now = time.time()
    sec = int(now)
    return {"sec": sec, "nanosec": int((now - sec) * 1_000_000_000)}


def yaw_quat(yaw: float) -> dict[str, float]:
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw * 0.5),
        "w": math.cos(yaw * 0.5),
    }


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def vector3(msg: dict[str, Any], key: str, axis: str) -> float:
    try:
        value = float(msg.get(key, {}).get(axis, 0.0))
    except (TypeError, ValueError):
        return 0.0
    return value if math.isfinite(value) else 0.0


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def finite_float_list(values: Any, width: int, field: str) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) < width:
        raise ValueError(f"{field} must contain {width} numbers")
    out: list[float] = []
    for value in values[:width]:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{field} entries must be numbers")
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError(f"{field} entries must be finite")
        out.append(parsed)
    return out


def finite_optional_float_list(values: Any, width: int, field: str) -> list[float | None]:
    if not isinstance(values, (list, tuple)) or len(values) < width:
        raise ValueError(f"{field} must contain {width} entries")
    out: list[float | None] = []
    for value in values[:width]:
        if value is None:
            out.append(None)
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{field} entries must be numbers or null")
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError(f"{field} entries must be finite")
        out.append(parsed)
    return out


def _load_fk_ee_position_for_joints():
    global _FK_EE_POSITION_FOR_JOINTS, _FK_EE_IMPORT_ATTEMPTED, _FK_EE_IMPORT_ERROR
    if _FK_EE_IMPORT_ATTEMPTED:
        return _FK_EE_POSITION_FOR_JOINTS
    _FK_EE_IMPORT_ATTEMPTED = True
    candidates = [
        os.environ.get("TELEOPERATION_SRC"),
        os.path.join(os.environ.get("TELEOPERATION_ROOT", ""), "src")
        if os.environ.get("TELEOPERATION_ROOT")
        else None,
        os.path.expanduser("~/teleoperation/src"),
        os.path.expanduser("~/indory_isaac_sim/src"),
    ]
    for candidate in reversed(candidates):
        if candidate and os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)
    try:
        from indoory_isaac_sim.vr_teleop.workspace import (
            fk_ee_position_for_joints,
        )
    except Exception as exc:
        _FK_EE_IMPORT_ERROR = exc
        return None
    _FK_EE_POSITION_FOR_JOINTS = fk_ee_position_for_joints
    return _FK_EE_POSITION_FOR_JOINTS


def _load_ik_project_fast():
    """Lazily import the realtime workspace IK projector (radian solution)."""
    global _IK_PROJECT_FAST, _IK_IMPORT_ATTEMPTED, _IK_IMPORT_ERROR
    if _IK_IMPORT_ATTEMPTED:
        return _IK_PROJECT_FAST
    _IK_IMPORT_ATTEMPTED = True
    _extend_teleoperation_path()
    try:
        from indoory_isaac_sim.vr_teleop.workspace import (
            project_ee_pose_to_kinematic_workspace_fast,
        )
    except Exception as exc:
        _IK_IMPORT_ERROR = exc
        return None
    _IK_PROJECT_FAST = project_ee_pose_to_kinematic_workspace_fast
    return _IK_PROJECT_FAST


def _arm_ee_pose_target_to_ros_ticks(
    latest_joint_state: dict[str, Any] | None,
    ee_target: Any,
    *,
    max_tick_delta: float = EE_IK_MAX_TICK_DELTA,
) -> list[float | None] | None:
    """Solve ``arm_ee_pose_target`` with seeded IK -> sparse ROS-order raw ticks.

    The realtime teleop bridge sends base-frame EE pose targets; robot I/O only
    drives joint ticks, so it must run the IK itself. Each side's 5 IK joints are
    seeded from the live URDF joint angles (fast convergence) and the radian
    solution is converted to raw ticks via the same calibration the proprio
    ``joint_pos_urdf_rad`` path uses, expressed as ``current_tick + delta`` so it
    stays consistent with measured state. Returns ``None`` (caller ignores the
    target) when IK or live joint state is unavailable, so the arm never moves
    from a blind guess.
    """
    if not isinstance(ee_target, dict) or not ee_target:
        return None
    project_fast = _load_ik_project_fast()
    desc = _xlerobot_model_description_cached()
    if project_fast is None or not isinstance(desc, dict):
        return None
    raw_ticks = _urdf_joint_raw_ticks_from_joint_state(latest_joint_state)
    if not raw_ticks:
        return None
    urdf_rad = _urdf_joint_values_from_joint_state(latest_joint_state, desc)
    inverted = _tf_inverted_urdf_joints()
    joints_by_name = {
        str(joint.get("name")): joint
        for joint in (desc.get("joints") or [])
        if isinstance(joint, dict) and isinstance(joint.get("name"), str)
    }
    targets: list[float | None] = [None] * len(ROS_JOINT_NAMES)
    any_set = False
    for side, names in ARM_IK_URDF_NAMES.items():
        side_target = ee_target.get(side)
        if not isinstance(side_target, dict):
            continue
        pose = side_target.get("pose")
        if not isinstance(pose, (list, tuple)) or len(pose) < 3:
            continue
        try:
            pose_vals = [float(v) for v in pose[:7]]
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(v) for v in pose_vals[:3]):
            continue
        if len(pose_vals) < 7:
            pose_vals = pose_vals[:3] + [0.0, 0.0, 0.0, 1.0]
        # Safety: only drive an arm whose encoders report plausible values. A
        # joint tick outside its calibrated range (e.g. a dead encoder reporting
        # 0) means proprio/seed are bogus, so skip that arm entirely instead of
        # commanding it from a wrong state.
        cur_ticks: dict[str, float] = {}
        encoders_ok = True
        for name in names:
            ct = raw_ticks.get(name)
            joint = joints_by_name.get(name)
            bounds = _encoding_tick_bounds(joint.get("encoding") if isinstance(joint, dict) else None)
            if ct is None or (bounds is not None and not (bounds[0] - 1.0 <= ct <= bounds[1] + 1.0)):
                encoders_ok = False
                break
            cur_ticks[name] = float(ct)
        if not encoders_ok:
            continue
        seed = tuple(float(urdf_rad.get(name, 0.0)) for name in names)
        try:
            projection = project_fast(side, pose_vals, q_seed=seed)
        except Exception:
            continue
        solution = getattr(projection, "joint_positions", None)
        if not solution or len(solution) < len(names):
            continue
        for name, target_rad in zip(names, solution):
            tick = _urdf_joint_rad_to_raw_tick(name, float(target_rad), joints_by_name.get(name), inverted)
            if tick is None or not math.isfinite(tick):
                continue
            if max_tick_delta > 0.0:
                cur = cur_ticks[name]
                tick = clamp(tick, cur - max_tick_delta, cur + max_tick_delta)
            ros_idx = ARM_IK_URDF_TO_ROS_INDEX.get(name)
            if ros_idx is None:
                continue
            targets[ros_idx] = float(tick)
            any_set = True
    return targets if any_set else None


def _extend_teleoperation_path() -> None:
    candidates = [
        os.environ.get("TELEOPERATION_SRC"),
        os.path.join(os.environ.get("TELEOPERATION_ROOT", ""), "src")
        if os.environ.get("TELEOPERATION_ROOT")
        else None,
        os.path.expanduser("~/teleoperation/src"),
        os.path.expanduser("~/indory_isaac_sim/src"),
    ]
    for candidate in reversed(candidates):
        if candidate and os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)


@lru_cache(maxsize=1)
def _xlerobot_model_description_cached() -> dict[str, Any] | None:
    global _MODEL_IMPORT_ERROR
    _extend_teleoperation_path()
    try:
        from indoory_isaac_sim.apps.teleop.vr_web_teleop_overlay import (
            xlerobot_model_description,
        )
        return xlerobot_model_description()
    except Exception as exc:
        _MODEL_IMPORT_ERROR = exc
        return None


def _mat4_identity() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def _mat4_translation(x: float, y: float, z: float) -> np.ndarray:
    out = _mat4_identity()
    out[:3, 3] = [float(x), float(y), float(z)]
    return out


def _mat4_rot_x(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.array(
        [[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _mat4_rot_y(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.array(
        [[c, 0.0, s, 0.0], [0.0, 1.0, 0.0, 0.0], [-s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _mat4_rot_z(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.array(
        [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _mat4_axis_angle(axis: Any, angle: float) -> np.ndarray:
    values = list(axis or [1.0, 0.0, 0.0])
    x = finite_float(values[0] if len(values) > 0 else 1.0)
    y = finite_float(values[1] if len(values) > 1 else 0.0)
    z = finite_float(values[2] if len(values) > 2 else 0.0)
    length = math.sqrt(x * x + y * y + z * z)
    if length < 1e-12:
        return _mat4_identity()
    x /= length
    y /= length
    z /= length
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    t = 1.0 - c
    return np.array(
        [
            [x * x * t + c, x * y * t - z * s, x * z * t + y * s, 0.0],
            [y * x * t + z * s, y * y * t + c, y * z * t - x * s, 0.0],
            [z * x * t - y * s, z * y * t + x * s, z * z * t + c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _mat4_from_origin(origin: Any) -> np.ndarray:
    origin = origin if isinstance(origin, dict) else {}
    xyz = origin.get("xyz") if isinstance(origin.get("xyz"), list) else [0.0, 0.0, 0.0]
    rpy = origin.get("rpy") if isinstance(origin.get("rpy"), list) else [0.0, 0.0, 0.0]
    return (
        _mat4_translation(
            finite_float(xyz[0] if len(xyz) > 0 else 0.0),
            finite_float(xyz[1] if len(xyz) > 1 else 0.0),
            finite_float(xyz[2] if len(xyz) > 2 else 0.0),
        )
        @ _mat4_rot_z(finite_float(rpy[2] if len(rpy) > 2 else 0.0))
        @ _mat4_rot_y(finite_float(rpy[1] if len(rpy) > 1 else 0.0))
        @ _mat4_rot_x(finite_float(rpy[0] if len(rpy) > 0 else 0.0))
    )


def _mat4_from_joint_motion(joint: dict[str, Any], value: float) -> np.ndarray:
    joint_type = str(joint.get("type") or "fixed")
    axis = joint.get("axis") if isinstance(joint.get("axis"), list) else [1.0, 0.0, 0.0]
    if joint_type in ("revolute", "continuous"):
        return _mat4_axis_angle(axis, float(value))
    if joint_type == "prismatic":
        return _mat4_translation(
            finite_float(axis[0] if len(axis) > 0 else 0.0) * float(value),
            finite_float(axis[1] if len(axis) > 1 else 0.0) * float(value),
            finite_float(axis[2] if len(axis) > 2 else 0.0) * float(value),
        )
    return _mat4_identity()


def _quat_xyzw_from_matrix(matrix: np.ndarray) -> list[float]:
    r = np.asarray(matrix, dtype=np.float64)[:3, :3]
    trace = float(r[0, 0] + r[1, 1] + r[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + r[1, 1] - r[0, 0] - r[2, 2])) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + r[2, 2] - r[0, 0] - r[1, 1])) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [qx / norm, qy / norm, qz / norm, qw / norm]


def _pose_xyzw_from_matrix(matrix: np.ndarray) -> list[float]:
    return [
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        float(matrix[2, 3]),
        *_quat_xyzw_from_matrix(matrix),
    ]


def _raw_tick_to_encoded_rad(raw_tick: float, encoding: dict[str, Any] | None) -> float:
    if not isinstance(encoding, dict) or encoding.get("kind") != "feetech_raw_ticks_centered":
        return finite_float(raw_tick)
    range_min = finite_float(encoding.get("range_min"), 0.0)
    range_max = finite_float(encoding.get("range_max"), 4095.0)
    if range_max <= range_min:
        range_min, range_max = 0.0, 4095.0
    bounded = clamp(float(raw_tick), range_min, range_max)
    center = finite_float(encoding.get("center_tick"), (range_min + range_max) * 0.5)
    rad_per_tick = finite_float(encoding.get("rad_per_tick"), 1.0 / FEETECH_TICKS_PER_RAD)
    if rad_per_tick <= 0.0:
        rad_per_tick = 1.0 / FEETECH_TICKS_PER_RAD
    try:
        drive_mode = int(encoding.get("drive_mode", 0) or 0)
    except (TypeError, ValueError):
        drive_mode = 0
    radians = (bounded - center) * rad_per_tick
    return -radians if drive_mode else radians


def _encoding_tick_bounds(encoding: dict[str, Any] | None) -> tuple[float, float] | None:
    if not isinstance(encoding, dict) or encoding.get("kind") != "feetech_raw_ticks_centered":
        return None
    range_min = finite_float(encoding.get("range_min"), math.nan)
    range_max = finite_float(encoding.get("range_max"), math.nan)
    if not math.isfinite(range_min) or not math.isfinite(range_max) or range_max <= range_min:
        return 0.0, 4095.0
    return range_min, range_max


def _joint_limit_bounds(joint: dict[str, Any] | None) -> tuple[float, float] | None:
    if not isinstance(joint, dict):
        return None
    limit = joint.get("limit")
    if not isinstance(limit, dict):
        return None
    lower = finite_float(limit.get("lower"), math.nan)
    upper = finite_float(limit.get("upper"), math.nan)
    if not math.isfinite(lower) or not math.isfinite(upper) or upper <= lower:
        return None
    return lower, upper


def _joint_tick_reversed(
    joint_name: str,
    encoding: dict[str, Any] | None,
    inverted_joints: set[str],
) -> bool:
    try:
        drive_mode = int((encoding or {}).get("drive_mode", 0) or 0)
    except (TypeError, ValueError):
        drive_mode = 0
    return bool(drive_mode) ^ (joint_name in inverted_joints)


def _raw_tick_to_urdf_joint_rad(
    joint_name: str,
    raw_tick: float,
    joint: dict[str, Any] | None,
    inverted_joints: set[str],
) -> float:
    encoding = joint.get("encoding") if isinstance(joint, dict) else None
    tick_bounds = _encoding_tick_bounds(encoding if isinstance(encoding, dict) else None)
    limit_bounds = _joint_limit_bounds(joint)
    if joint_name in FAST_TF_LIMIT_MAPPED_URDF_JOINTS and tick_bounds is not None and limit_bounds is not None:
        tick_min, tick_max = tick_bounds
        lower, upper = limit_bounds
        bounded = clamp(float(raw_tick), tick_min, tick_max)
        ratio = (bounded - tick_min) / (tick_max - tick_min)
        if _joint_tick_reversed(joint_name, encoding if isinstance(encoding, dict) else None, inverted_joints):
            ratio = 1.0 - ratio
        return lower + ratio * (upper - lower)
    value = _raw_tick_to_encoded_rad(raw_tick, encoding if isinstance(encoding, dict) else None)
    if joint_name in inverted_joints:
        value = -value
    return value


def _head_tf_zero_tick(
    joint_name: str,
    raw_tick: float,
    calibration: dict[str, dict[str, Any]] | None = None,
) -> float:
    # Camera/head TF zero is a robot mounting calibration, not motor homing
    # metadata. Prefer explicit operator-calibrated zero ticks when present,
    # otherwise fall back to the original live behavior: first observed tick.
    _ = calibration
    env_name = {
        "head_pan_joint": "FAST_TF_HEAD_PAN_ZERO_TICK",
        "head_tilt_joint": "FAST_TF_HEAD_TILT_ZERO_TICK",
    }.get(joint_name)
    if env_name:
        raw = os.environ.get(env_name, "").strip()
        if raw:
            try:
                value = float(raw)
                if math.isfinite(value):
                    return value
            except ValueError:
                pass
    if joint_name not in FAST_TF_HEAD_ZERO_TICKS:
        FAST_TF_HEAD_ZERO_TICKS[joint_name] = float(raw_tick)
    return FAST_TF_HEAD_ZERO_TICKS[joint_name]

def _raw_head_tick_to_relative_urdf_rad(
    joint_name: str,
    raw_tick: float,
    calibration: dict[str, dict[str, Any]] | None = None,
) -> float | None:
    if joint_name not in FAST_TF_HEAD_DELTA_SIGNS:
        return None
    value = float(raw_tick)
    if not math.isfinite(value):
        return None
    zero_tick = _head_tf_zero_tick(joint_name, value, calibration)
    delta_tick = value - zero_tick
    radians = delta_tick / FEETECH_TICKS_PER_RAD * float(FAST_TF_HEAD_DELTA_SIGNS[joint_name])
    if joint_name == "head_tilt_joint":
        radians += math.radians(env_float("FAST_TF_HEAD_TILT_OFFSET_DEG", 0.0))
    elif joint_name == "head_pan_joint":
        radians += math.radians(env_float("FAST_TF_HEAD_PAN_OFFSET_DEG", 0.0))
    return radians


def _urdf_joint_rad_delta_to_raw_tick_delta(
    joint_name: str,
    delta_rad: float,
    joint: dict[str, Any] | None,
    inverted_joints: set[str],
) -> float:
    encoding = joint.get("encoding") if isinstance(joint, dict) else None
    tick_bounds = _encoding_tick_bounds(encoding if isinstance(encoding, dict) else None)
    limit_bounds = _joint_limit_bounds(joint)
    reversed_axis = _joint_tick_reversed(
        joint_name,
        encoding if isinstance(encoding, dict) else None,
        inverted_joints,
    )
    if joint_name in FAST_TF_LIMIT_MAPPED_URDF_JOINTS and tick_bounds is not None and limit_bounds is not None:
        tick_min, tick_max = tick_bounds
        lower, upper = limit_bounds
        tick_per_rad = (tick_max - tick_min) / (upper - lower)
        return float(delta_rad) * tick_per_rad * (-1.0 if reversed_axis else 1.0)
    rad_per_tick = finite_float(
        (encoding or {}).get("rad_per_tick") if isinstance(encoding, dict) else None,
        1.0 / FEETECH_TICKS_PER_RAD,
    )
    if rad_per_tick <= 0.0:
        rad_per_tick = 1.0 / FEETECH_TICKS_PER_RAD
    return float(delta_rad) / rad_per_tick * (-1.0 if reversed_axis else 1.0)


def _urdf_joint_rad_to_raw_tick(
    joint_name: str,
    rad: float,
    joint: dict[str, Any] | None,
    inverted_joints: set[str],
) -> float | None:
    """Absolute inverse of :func:`_raw_tick_to_urdf_joint_rad` (URDF rad -> raw tick).

    Used to drive an EE-IK joint solution to motor ticks without depending on a
    possibly stale/out-of-range measured tick.
    """
    encoding = joint.get("encoding") if isinstance(joint, dict) else None
    encoding = encoding if isinstance(encoding, dict) else None
    tick_bounds = _encoding_tick_bounds(encoding)
    limit_bounds = _joint_limit_bounds(joint)
    if joint_name in FAST_TF_LIMIT_MAPPED_URDF_JOINTS and tick_bounds is not None and limit_bounds is not None:
        tick_min, tick_max = tick_bounds
        lower, upper = limit_bounds
        if upper <= lower:
            return None
        ratio = clamp((float(rad) - lower) / (upper - lower), 0.0, 1.0)
        if _joint_tick_reversed(joint_name, encoding, inverted_joints):
            ratio = 1.0 - ratio
        return tick_min + ratio * (tick_max - tick_min)
    if encoding is None or encoding.get("kind") != "feetech_raw_ticks_centered":
        return None
    range_min = finite_float(encoding.get("range_min"), 0.0)
    range_max = finite_float(encoding.get("range_max"), 4095.0)
    if range_max <= range_min:
        range_min, range_max = 0.0, 4095.0
    center = finite_float(encoding.get("center_tick"), (range_min + range_max) * 0.5)
    rad_per_tick = finite_float(encoding.get("rad_per_tick"), 1.0 / FEETECH_TICKS_PER_RAD)
    if rad_per_tick <= 0.0:
        rad_per_tick = 1.0 / FEETECH_TICKS_PER_RAD
    try:
        drive_mode = int(encoding.get("drive_mode", 0) or 0)
    except (TypeError, ValueError):
        drive_mode = 0
    sign = (-1.0 if drive_mode else 1.0) * (-1.0 if joint_name in inverted_joints else 1.0)
    return center + (float(rad) * sign) / rad_per_tick


def _tf_inverted_urdf_joints() -> set[str]:
    raw = os.environ.get("XLEROBOT_TF_INVERT_URDF_JOINTS")
    if raw is None:
        return set(FAST_TF_DEFAULT_INVERT_URDF_JOINTS)
    out = {
        item.strip()
        for item in raw.replace(";", ",").split(",")
        if item.strip()
    }
    return out



def _urdf_joint_raw_ticks_from_joint_state(joint_state: dict[str, Any] | None) -> dict[str, float]:
    by_ros = _joint_state_positions_by_name(joint_state)
    mapping = {
        "right_hand_1": "Rotation",
        "right_hand_2": "Pitch",
        "right_hand_3": "Elbow",
        "right_hand_4": "Wrist_Pitch",
        "right_hand_5": "Wrist_Roll",
        "right_hand_6": "Jaw",
        "left_hand_1": "Rotation_2",
        "left_hand_2": "Pitch_2",
        "left_hand_3": "Elbow_2",
        "left_hand_4": "Wrist_Pitch_2",
        "left_hand_5": "Wrist_Roll_2",
        "left_hand_6": "Jaw_2",
        "head_pan": "head_pan_joint",
        "head_tilt": "head_tilt_joint",
    }
    out: dict[str, float] = {}
    for ros_name, urdf_name in mapping.items():
        if ros_name in by_ros:
            out[urdf_name] = by_ros[ros_name]
    return out


def _urdf_joint_values_from_joint_state(
    joint_state: dict[str, Any] | None,
    desc: dict[str, Any],
    calibration: dict[str, dict[str, Any]] | None = None,
) -> dict[str, float]:
    joints: dict[str, dict[str, Any]] = {}
    for joint in desc.get("joints") or []:
        if isinstance(joint, dict) and isinstance(joint.get("name"), str):
            joints[str(joint["name"])] = joint
    raw_ticks = _urdf_joint_raw_ticks_from_joint_state(joint_state)
    inverted_joints = _tf_inverted_urdf_joints()
    values: dict[str, float] = {
        "root_x_axis_joint": 0.0,
        "root_y_axis_joint": 0.0,
        "root_z_rotation_joint": 0.0,
    }
    for name, raw_tick in raw_ticks.items():
        head_relative = _raw_head_tick_to_relative_urdf_rad(name, raw_tick, calibration)
        if head_relative is not None:
            values[name] = head_relative
            continue
        values[name] = _raw_tick_to_urdf_joint_rad(
            name,
            raw_tick,
            joints.get(name),
            inverted_joints,
        )
    return values


def _xlerobot_link_matrices_from_joint_state(
    joint_state: dict[str, Any] | None,
    calibration: dict[str, dict[str, Any]] | None = None,
) -> tuple[dict[str, np.ndarray], list[tuple[str, str]], str]:
    desc = _xlerobot_model_description_cached()
    if not isinstance(desc, dict):
        note = "model_description_unavailable"
        if _MODEL_IMPORT_ERROR is not None:
            note += f": {type(_MODEL_IMPORT_ERROR).__name__}"
        return {}, [], note
    links = desc.get("links") if isinstance(desc.get("links"), dict) else {}
    nodes: dict[str, dict[str, Any]] = {
        str(name): {"name": str(name), "children": []}
        for name in links.keys()
    }
    edges: list[tuple[str, str]] = []
    for joint in desc.get("joints") or []:
        if not isinstance(joint, dict):
            continue
        parent = str(joint.get("parent") or "")
        child = str(joint.get("child") or "")
        name = str(joint.get("name") or "")
        if not parent or not child or parent not in nodes or child not in nodes:
            continue
        nodes[parent]["children"].append(
            {
                "joint": joint,
                "name": name,
                "child": child,
                "origin": _mat4_from_origin(joint.get("origin")),
            }
        )
        edges.append((parent, child))
    root = str(desc.get("root") or "root")
    if root not in nodes:
        return {}, edges, "model_root_missing"
    values = _urdf_joint_values_from_joint_state(joint_state, desc, calibration)
    matrices: dict[str, np.ndarray] = {}

    def visit(link_name: str, matrix: np.ndarray) -> None:
        matrices[link_name] = matrix
        for child in nodes[link_name]["children"]:
            joint = child["joint"]
            joint_name = str(joint.get("name") or "")
            child_matrix = matrix @ child["origin"] @ _mat4_from_joint_motion(
                joint,
                values.get(joint_name, 0.0),
            )
            visit(str(child["child"]), child_matrix)

    visit(root, _mat4_identity())
    return matrices, edges, "urdf_encoder_fk"


def _odom_base_pose_xyzw(odom_msg: dict[str, Any]) -> list[float]:
    pose_msg = ((odom_msg.get("pose") or {}).get("pose") or {})
    position = pose_msg.get("position") or {}
    orientation = pose_msg.get("orientation") or {}
    return [
        finite_float(position.get("x")),
        finite_float(position.get("y")),
        finite_float(position.get("z")),
        finite_float(orientation.get("x")),
        finite_float(orientation.get("y")),
        finite_float(orientation.get("z")),
        finite_float(orientation.get("w"), 1.0),
    ]


def _joint_state_positions_by_name(joint_state: dict[str, Any] | None) -> dict[str, float]:
    if not isinstance(joint_state, dict):
        return {}
    names = joint_state.get("name") or joint_state.get("names")
    positions = joint_state.get("position")
    if not isinstance(names, list) or not isinstance(positions, list):
        return {}
    out: dict[str, float] = {}
    for index, name in enumerate(names):
        if index >= len(positions):
            break
        if not isinstance(name, str):
            continue
        value = finite_float(positions[index], math.nan)
        if math.isfinite(value):
            out[name] = value
    return out


def _raw_motor_tick_to_urdf_rad(
    raw_tick: float,
    calibration: dict[str, dict[str, Any]],
    motor_name: str,
) -> float:
    cal = calibration.get(motor_name) or {}
    try:
        range_min = float(cal.get("range_min", 0.0))
        range_max = float(cal.get("range_max", 4095.0))
    except (TypeError, ValueError):
        range_min, range_max = 0.0, 4095.0
    if not math.isfinite(range_min) or not math.isfinite(range_max) or range_max <= range_min:
        range_min, range_max = 0.0, 4095.0
    bounded = clamp(float(raw_tick), range_min, range_max)
    radians = (bounded - ((range_min + range_max) * 0.5)) / FEETECH_TICKS_PER_RAD
    try:
        drive_mode = int(cal.get("drive_mode", 0) or 0)
    except (TypeError, ValueError):
        drive_mode = 0
    return -radians if drive_mode else radians


def _arm_fk_targets_from_joint_state(
    joint_state: dict[str, Any] | None,
    calibration: dict[str, dict[str, Any]],
) -> list[dict[str, Any]]:
    fk_ee_position_for_joints = _load_fk_ee_position_for_joints()
    if fk_ee_position_for_joints is None:
        return []
    by_name = _joint_state_positions_by_name(joint_state)
    targets: list[dict[str, Any]] = []
    for side, joints in FAST_ARM_FK_JOINTS.items():
        q: list[float] = []
        missing = False
        for ros_name, motor_name in joints:
            if ros_name not in by_name:
                missing = True
                break
            q.append(_raw_motor_tick_to_urdf_rad(by_name[ros_name], calibration, motor_name))
        if missing:
            continue
        try:
            xyz = fk_ee_position_for_joints(side, tuple(q))
        except Exception:
            continue
        pose = [float(xyz[0]), float(xyz[1]), float(xyz[2]), 0.0, 0.0, 0.0, 1.0]
        suffix = "right" if side == "right" else "left"
        targets.append({"name": f"gripper_{suffix}", "pose": pose, "source": "encoder_fk"})
        targets.append({"name": f"jaw_{suffix}", "pose": pose, "source": "encoder_fk"})
    return targets


def load_xlerobot_calibration(robot_id: str) -> tuple[dict[str, dict[str, Any]], str | None]:
    explicit_path = os.environ.get("XLEROBOT_CALIBRATION_PATH")
    base_dir = os.path.expanduser(
        os.environ.get(
            "XLEROBOT_CALIBRATION_DIR",
            "~/.cache/huggingface/lerobot/calibration/robots/xlerobot",
        )
    )
    candidate_paths: list[str] = []
    if explicit_path:
        candidate_paths.append(os.path.expanduser(explicit_path))
    for candidate_id in (
        os.environ.get("XLEROBOT_CALIBRATION_ID"),
        robot_id,
        "my_xlerobot_pc",
        "None",
    ):
        if candidate_id:
            candidate_paths.append(os.path.join(base_dir, f"{candidate_id}.json"))

    seen: set[str] = set()
    for path in candidate_paths:
        path = os.path.abspath(os.path.expanduser(path))
        if path in seen:
            continue
        seen.add(path)
        if not os.path.isfile(path):
            continue
        try:
            with open(path) as f:
                raw = json.load(f)
        except Exception as exc:
            print(f"[motors] calibration load failed {path}: {exc}", flush=True)
            continue
        if isinstance(raw, dict):
            cleaned = {
                str(name): dict(value)
                for name, value in raw.items()
                if isinstance(value, dict)
            }
            return cleaned, path
    return {}, None


def stamp_ns_from_header(msg: dict[str, Any]) -> int:
    header = msg.get("header") if isinstance(msg, dict) else None
    stamp_msg = header.get("stamp") if isinstance(header, dict) else None
    if isinstance(stamp_msg, dict):
        try:
            sec = int(stamp_msg.get("sec", 0))
            nanosec = int(stamp_msg.get("nanosec", 0))
            if sec or nanosec:
                return sec * 1_000_000_000 + nanosec
        except (TypeError, ValueError):
            pass
    return time.time_ns()


def frame_id(msg: dict[str, Any], fallback: str) -> str:
    header = msg.get("header") if isinstance(msg, dict) else None
    if isinstance(header, dict):
        frame = header.get("frame_id")
        if isinstance(frame, str) and frame:
            return frame
    return fallback


def quat_yaw(q: dict[str, Any]) -> float:
    x = finite_float(q.get("x"))
    y = finite_float(q.get("y"))
    z = finite_float(q.get("z"))
    w = finite_float(q.get("w"), 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def extend_pythonpath_from_env() -> None:
    for key in ("LEROBOT_ROOT", "XLE_LEROBOT_ROOT"):
        root = os.environ.get(key, "").strip()
        if not root:
            continue
        src = os.path.join(os.path.expanduser(root), "src")
        if os.path.isdir(src) and src not in sys.path:
            sys.path.insert(0, src)


def import_lerobot_xlerobot():
    extend_pythonpath_from_env()
    from lerobot.motors.feetech import OperatingMode
    from lerobot.robots.xlerobot import XLerobot, XLerobotConfig

    return XLerobot, XLerobotConfig, OperatingMode


def make_command(cmd: int, payload: bytes = b"") -> bytes:
    if not payload:
        return bytes([0xA5, cmd])
    packet = bytearray([0xA5, cmd, len(payload)])
    packet.extend(payload)
    checksum = 0
    for byte in packet:
        checksum ^= byte
    packet.append(checksum)
    return bytes(packet)


def parse_scan_point(data: bytes) -> Optional[tuple[int, float, float, int]]:
    if len(data) != 5:
        return None
    b0, b1, b2, b3, b4 = data
    start = b0 & 1
    inverted_start = (b0 >> 1) & 1
    if start == inverted_start:
        return None
    if (b1 & 1) != 1:
        return None

    quality = b0 >> 2
    angle_deg = (((b1 >> 1) | (b2 << 7)) / 64.0) % 360.0
    distance_mm = (b3 | (b4 << 8)) / 4.0
    return start, angle_deg, distance_mm, quality


class StatusPublisher:
    def __init__(self, topic_name: str):
        self.topic_name = topic_name
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {}

    def update(self, **kwargs: Any) -> None:
        with self._lock:
            self._state.update(kwargs)
            payload = dict(self._state)
        payload["stamp"] = time.time()
        if env_bool("STATUS_LOG_JSON", False):
            print(
                f"[status] {json.dumps(payload, separators=(',', ':'))}",
                flush=True,
            )

    def close(self) -> None:
        pass


class MessageSink:
    """No-op sink for ROS-shaped messages produced for fast ZMQ payloads."""

    def __init__(self, name: str, msg_type: str):
        self.name = name
        self.msg_type = msg_type

    def publish(self, _payload: dict[str, Any]) -> None:
        pass

    def close(self) -> None:
        pass


ROS_LEFT_JOINT_NAMES = tuple(f"left_hand_{i}" for i in range(1, 7))
ROS_RIGHT_JOINT_NAMES = tuple(f"right_hand_{i}" for i in range(1, 7))
ROS_HEAD_JOINT_NAMES = ("head_pan", "head_tilt")
ROS_JOINT_NAMES = ROS_LEFT_JOINT_NAMES + ROS_RIGHT_JOINT_NAMES + ROS_HEAD_JOINT_NAMES

XLE_LEFT_JOINT_NAMES = (
    "left_arm_shoulder_pan",
    "left_arm_shoulder_lift",
    "left_arm_elbow_flex",
    "left_arm_wrist_flex",
    "left_arm_wrist_roll",
    "left_arm_gripper",
)
XLE_RIGHT_JOINT_NAMES = (
    "right_arm_shoulder_pan",
    "right_arm_shoulder_lift",
    "right_arm_elbow_flex",
    "right_arm_wrist_flex",
    "right_arm_wrist_roll",
    "right_arm_gripper",
)
XLEROBOT_SWAP_HEAD_IDS = env_bool("XLEROBOT_SWAP_HEAD_IDS", False)
XLE_HEAD_JOINT_NAMES = (
    ("head_motor_2", "head_motor_1")
    if XLEROBOT_SWAP_HEAD_IDS
    else ("head_motor_1", "head_motor_2")
)
XLE_BASE_WHEEL_NAMES = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
MOTOR_LEFT_HEAD_SERIAL_SHORT = os.environ.get("XLEROBOT_LEFT_HEAD_SERIAL_SHORT", "5B14032190").strip()
MOTOR_RIGHT_BASE_SERIAL_SHORT = os.environ.get("XLEROBOT_RIGHT_BASE_SERIAL_SHORT", "5B3D046415").strip()


def motor_driver_by_serial(serial_short: str, fallback: str) -> str:
    serial_short = (serial_short or "").strip()
    if serial_short:
        by_id = "/dev/serial/by-id"
        try:
            for name in sorted(os.listdir(by_id)):
                if serial_short in name:
                    return os.path.join(by_id, name)
        except OSError:
            pass
    return fallback


ROS_TO_XLE_JOINT = dict(
    zip(
        ROS_JOINT_NAMES,
        XLE_LEFT_JOINT_NAMES + XLE_RIGHT_JOINT_NAMES + XLE_HEAD_JOINT_NAMES,
        strict=True,
    )
)


def canonical_joint_ticks_to_ros_order(joints: list[float]) -> list[float]:
    right = joints[:6]
    left = joints[6:12]
    head = joints[12:14]
    return [*left, *right, *head]


def ros_joint_positions_to_canonical(positions: list[float]) -> list[float]:
    left = positions[:6]
    right = positions[6:12]
    head = positions[12:14]
    return [*right, *left, *head]


def fast_scan_payload(ros_msg: dict[str, Any], robot_id: int, fallback_frame: str) -> tuple[str, dict[str, Any]]:
    topic = f"scan.{robot_id}"
    raw_ranges = ros_msg.get("ranges") or []
    ranges = array.array("f")
    for raw in raw_ranges:
        ranges.append(finite_float(raw, math.inf))
    count = len(ranges)
    angle_min = finite_float(ros_msg.get("angle_min"), -math.pi)
    angle_increment = finite_float(ros_msg.get("angle_increment"), 0.0)
    if angle_increment == 0.0 and count > 1:
        angle_max_raw = finite_float(ros_msg.get("angle_max"), math.pi)
        angle_increment = (angle_max_raw - angle_min) / float(count - 1)
    angle_max = finite_float(
        ros_msg.get("angle_max"),
        angle_min + max(0, count - 1) * angle_increment,
    )
    return topic, {
        "schema": SCHEMA_VERSION,
        "stamp_ns": stamp_ns_from_header(ros_msg),
        "topic": topic,
        "frame": frame_id(ros_msg, fallback_frame),
        "encoding": "f32",
        "ranges": ranges.tobytes(),
        "num_ranges": count,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": angle_increment,
        "range_min": finite_float(ros_msg.get("range_min"), 0.05),
        "range_max": finite_float(ros_msg.get("range_max"), 12.0),
    }


def fast_joint_state_payload(ros_msg: dict[str, Any], robot_id: int, fallback_frame: str) -> tuple[str, dict[str, Any]]:
    topic = f"joint_states.{robot_id}"
    names = [str(name) for name in ros_msg.get("name", [])]
    positions = [finite_float(value) for value in ros_msg.get("position", [])]
    velocities = [finite_float(value) for value in ros_msg.get("velocity", [])]
    return topic, {
        "schema": FAST_ZMQ_SCHEMA,
        "stamp_ns": stamp_ns_from_header(ros_msg),
        "topic": topic,
        "frame": frame_id(ros_msg, fallback_frame),
        "robot_id": robot_id,
        "names": names,
        "position": positions,
        "velocity": velocities,
    }


def fast_proprio_payload(
    odom_msg: dict[str, Any],
    robot_id: int,
    fallback_frame: str,
    latest_joint_state: dict[str, Any] | None,
    last_command: list[float] | None,
    command_age_ms: float | None,
    last_command_source: str | None = None,
    active_source_id: str | None = None,
    active_source_priority: int | None = None,
    active_source_ttl_ms: float | None = None,
    calibration: dict[str, dict[str, Any]] | None = None,
) -> tuple[str, dict[str, Any]]:
    topic = f"proprio.{robot_id}"
    pose_msg = ((odom_msg.get("pose") or {}).get("pose") or {})
    twist_msg = ((odom_msg.get("twist") or {}).get("twist") or {})
    position = pose_msg.get("position") or {}
    orientation = pose_msg.get("orientation") or {}
    linear = twist_msg.get("linear") or {}
    angular = twist_msg.get("angular") or {}
    yaw = quat_yaw(orientation)

    joint_pos = [0.0] * len(FAST_JOINT_POS_ORDER)
    joint_vel = [0.0] * len(FAST_JOINT_POS_ORDER)
    if isinstance(latest_joint_state, dict):
        positions = [finite_float(value) for value in latest_joint_state.get("position", [])]
        velocities = [finite_float(value) for value in latest_joint_state.get("velocity", [])]
        if len(positions) >= len(ROS_JOINT_NAMES):
            joint_pos = ros_joint_positions_to_canonical(positions[:len(ROS_JOINT_NAMES)])
        if len(velocities) >= len(ROS_JOINT_NAMES):
            joint_vel = ros_joint_positions_to_canonical(velocities[:len(ROS_JOINT_NAMES)])
    joint_pos_urdf_rad: list[float] | None = None
    desc = _xlerobot_model_description_cached()
    if isinstance(desc, dict):
        urdf_values = _urdf_joint_values_from_joint_state(latest_joint_state, desc, calibration)
        joint_pos_urdf_rad = [
            float(urdf_values.get(name, 0.0))
            for name in FAST_JOINT_POS_ORDER
        ]

    base_twist = [
        finite_float(linear.get("x")),
        finite_float(linear.get("y")),
        finite_float(linear.get("z")),
        finite_float(angular.get("x")),
        finite_float(angular.get("y")),
        finite_float(angular.get("z")),
    ]
    return topic, {
        "schema": SCHEMA_VERSION,
        "stamp_ns": stamp_ns_from_header(odom_msg),
        "topic": topic,
        "frame": f"robot_state_{robot_id}",
        "robot_id": robot_id,
        "joint_names_pos": list(FAST_JOINT_POS_ORDER),
        "joint_pos": joint_pos,
        "joint_vel": joint_vel,
        "joint_names_urdf": list(FAST_JOINT_POS_ORDER),
        "joint_pos_urdf_rad": joint_pos_urdf_rad,
        "joint_pos_urdf_source": "server_encoder_fk",
        "joint_pos_urdf_mapping": "motor_centered_ticks_with_lift_elbow_limit_mapping",
        "joint_pos_urdf_inverted_joints": sorted(_tf_inverted_urdf_joints()),
        "gripper_state": {"right": joint_pos[5], "left": joint_pos[11]},
        "gripper_velocity": {"right": joint_vel[5], "left": joint_vel[11]},
        "joint_vel_arm_sample": [
            finite_float(position.get("x")),
            finite_float(position.get("y")),
            yaw,
        ],
        "joint_names_base": list(FAST_BASE_VEL_ORDER),
        "base_joint_vel": [
            finite_float(linear.get("x")),
            finite_float(linear.get("y")),
            finite_float(angular.get("z")),
        ],
        "base_pose": [
            finite_float(position.get("x")),
            finite_float(position.get("y")),
            finite_float(position.get("z")),
            finite_float(orientation.get("x")),
            finite_float(orientation.get("y")),
            finite_float(orientation.get("z")),
            finite_float(orientation.get("w"), 1.0),
        ],
        "base_twist": base_twist,
        "base_forward_w": [math.cos(yaw), math.sin(yaw), 0.0],
        "base_motion_model": "xlerobot_direct_fast_zmq",
        "base_state_source": frame_id(odom_msg, fallback_frame),
        "base_command_frame": "body",
        "base_command_age_ms": command_age_ms,
        "base_cmd_vel_applied": last_command,
        "base_command_source": last_command_source,
        "active_source_id": active_source_id,
        "active_source_priority": active_source_priority,
        "active_source_ttl_ms": active_source_ttl_ms,
        "base_recenter_count": 0,
    }


def fast_tf_links_payload(
    odom_msg: dict[str, Any],
    robot_id: int,
    fallback_frame: str,
    latest_joint_state: dict[str, Any] | None = None,
    calibration: dict[str, dict[str, Any]] | None = None,
) -> tuple[str, dict[str, Any]]:
    topic = f"tf.links.{robot_id}"
    matrices, edges, fk_source = _xlerobot_link_matrices_from_joint_state(latest_joint_state, calibration)
    link_frames = {
        name: _pose_xyzw_from_matrix(matrix)
        for name, matrix in matrices.items()
    }
    targets: list[dict[str, Any]] = []
    for alias, link_name in FAST_TF_ALIAS_LINKS.items():
        pose = link_frames.get(link_name)
        if pose is not None:
            targets.append(
                {
                    "name": alias,
                    "pose": list(pose),
                    "source": "urdf_encoder_fk",
                    "link": link_name,
                }
            )
    for link_name in FAST_TF_IMPORTANT_LINKS:
        pose = link_frames.get(link_name)
        if pose is not None:
            targets.append(
                {
                    "name": link_name,
                    "pose": list(pose),
                    "source": "urdf_link_frame",
                    "link": link_name,
                }
            )
    if not targets:
        targets = _arm_fk_targets_from_joint_state(latest_joint_state, calibration or {})
        targets.extend(
            {"name": name, "pose": list(pose), "source": "static_anchor"}
            for name, pose in FAST_TF_STATIC_ANCHORS.items()
        )
        source_note = "fallback static/FK anchors; URDF link frames unavailable"
        if _MODEL_IMPORT_ERROR is not None:
            source_note += f": {type(_MODEL_IMPORT_ERROR).__name__}"
    else:
        targets.append(
            {
                "name": "base_link_odom",
                "pose": _odom_base_pose_xyzw(odom_msg),
                "source": "odom",
                "frame": frame_id(odom_msg, fallback_frame),
            }
        )
        source_note = (
            "URDF encoder FK link frames; joint mapping: motor_centered_ticks_with_lift_elbow_limit_mapping; inverted joints: "
            + ",".join(sorted(_tf_inverted_urdf_joints()))
        )
    return topic, {
        "schema": SCHEMA_VERSION,
        "stamp_ns": stamp_ns_from_header(odom_msg),
        "topic": topic,
        "frame": "base_link",
        "source": fk_source,
        "odom_frame": frame_id(odom_msg, fallback_frame),
        "base_pose_odom": _odom_base_pose_xyzw(odom_msg),
        "targets": targets,
        "link_frames": link_frames,
        "tree_edges": [[parent, child] for parent, child in edges],
        "source_note": source_note,
    }


class FastRobotZmqServer:
    """Direct low-latency ZMQ ports for command intake and sensor streams."""

    def __init__(self, stop_event: threading.Event):
        self.stop_event = stop_event
        self.enabled = True
        self.bind_host = os.environ.get("FAST_ZMQ_BIND_HOST", os.environ.get("ISAAC_COMPAT_BIND_HOST", "0.0.0.0"))
        self.pub_port = env_int("FAST_ZMQ_PUB_PORT", env_int("ISAAC_COMPAT_PUB_PORT", 8855))
        self.pull_port = env_int("FAST_ZMQ_PULL_PORT", env_int("ISAAC_COMPAT_PULL_PORT", 8856))
        self.rep_port = env_int("FAST_ZMQ_REP_PORT", env_int("ISAAC_COMPAT_REP_PORT", 8857))
        self.robot_id = env_int("FAST_ZMQ_ROBOT_ID", env_int("ISAAC_COMPAT_ROBOT_ID", 0))
        self.pub_hwm = max(1, env_int("FAST_ZMQ_PUB_HWM", 4))
        self.pull_hwm = max(1, env_int("FAST_ZMQ_PULL_HWM", 16))
        self.pull_conflate = env_bool("FAST_ZMQ_PULL_CONFLATE", False)
        self.web_linear_speed = env_float(
            "FAST_ZMQ_WEB_LINEAR_SPEED_MPS",
            env_float("XLEROBOT_WEB_LINEAR_SPEED_MPS", 0.2),
        )
        self.web_strafe_speed = env_float(
            "FAST_ZMQ_WEB_STRAFE_SPEED_MPS",
            env_float("XLEROBOT_WEB_STRAFE_SPEED_MPS", self.web_linear_speed),
        )
        self.web_yaw_speed = math.radians(env_float(
            "FAST_ZMQ_WEB_YAW_SPEED_DEGPS",
            env_float("XLEROBOT_WEB_YAW_SPEED_DEGPS", 60.0),
        ))
        self._ctx = None
        self._pub_socket = None
        self._pull_socket = None
        self._rep_socket = None
        self._pub_lock = threading.Lock()
        self._threads: list[threading.Thread] = []
        self._base: Any | None = None
        self._lock = threading.Lock()
        self._latest_joint_state: dict[str, Any] | None = None
        self._last_command: list[float] | None = None
        self._last_command_ns: int | None = None
        self._last_command_source: str | None = None
        self._last_command_source_role: str | None = None
        self._last_command_priority: int | None = None
        self._active_source_id: str | None = None
        self._active_source_priority: int = -1
        self._active_source_until_ns: int | None = None
        self._last_redundant_stop_ns_by_source: dict[str, int] = {}
        self._last_rejected_reason: str | None = None
        self._last_scan_ns: int | None = None
        self._last_odom_ns: int | None = None
        self._last_joint_state_ns: int | None = None
        self._latest_odom_msg: dict[str, Any] | None = None
        self._latest_odom_frame = "base_link"
        self._tf_rate_hz = max(1.0, env_float("FAST_TF_PUBLISH_RATE_HZ", 20.0))
        self._dropped_pub = 0
        self._dropped_commands = 0
        self._accepted_commands = 0
        self._skipped_redundant_stop_commands = 0
        self._estop = False
        self._default_command_priority = env_int("FAST_ZMQ_DEFAULT_COMMAND_PRIORITY", 0)
        self._default_command_lease_ms = max(50, env_int("FAST_ZMQ_DEFAULT_COMMAND_LEASE_MS", 300))
        self._max_command_lease_ms = max(self._default_command_lease_ms, env_int("FAST_ZMQ_MAX_COMMAND_LEASE_MS", 3000))
        self._redundant_stop_dedupe_ms = max(0, env_int("FAST_ZMQ_REDUNDANT_STOP_DEDUPE_MS", 100))

    def start(self) -> None:
        if not self.enabled:
            return
        if zmq is None or msgpack is None:
            raise RuntimeError(f"FAST ZMQ requires pyzmq and msgpack: {FAST_ZMQ_IMPORT_ERROR}")
        self._ctx = zmq.Context.instance()
        self._pub_socket = self._ctx.socket(zmq.PUB)
        self._pub_socket.setsockopt(zmq.LINGER, 0)
        self._pub_socket.setsockopt(zmq.SNDHWM, self.pub_hwm)
        self._pub_socket.setsockopt(zmq.SNDTIMEO, 0)
        self._pub_socket.bind(f"tcp://{self.bind_host}:{self.pub_port}")

        self._pull_socket = self._ctx.socket(zmq.PULL)
        self._pull_socket.setsockopt(zmq.LINGER, 0)
        self._pull_socket.setsockopt(zmq.RCVHWM, self.pull_hwm)
        self._pull_socket.setsockopt(zmq.RCVTIMEO, 0)
        if self.pull_conflate:
            try:
                self._pull_socket.setsockopt(zmq.CONFLATE, 1)
            except zmq.ZMQError:
                pass
        self._pull_socket.bind(f"tcp://{self.bind_host}:{self.pull_port}")

        self._rep_socket = self._ctx.socket(zmq.REP)
        self._rep_socket.setsockopt(zmq.LINGER, 0)
        self._rep_socket.bind(f"tcp://{self.bind_host}:{self.rep_port}")

        self._threads = [
            threading.Thread(target=self._command_loop, name="fast-zmq-command", daemon=True),
            threading.Thread(target=self._rpc_loop, name="fast-zmq-rpc", daemon=True),
            threading.Thread(target=self._tf_loop, name="fast-zmq-tf", daemon=True),
        ]
        for thread in self._threads:
            thread.start()
        print(
            f"[fast_zmq] PUB tcp://{self.bind_host}:{self.pub_port} "
            f"PULL tcp://{self.bind_host}:{self.pull_port} "
            f"REP tcp://{self.bind_host}:{self.rep_port} "
            f"TF={self._tf_rate_hz:g}Hz",
            flush=True,
        )

    def stop(self) -> None:
        for sock in (self._pub_socket, self._pull_socket, self._rep_socket):
            if sock is not None:
                try:
                    sock.close(0)
                except Exception:
                    pass
        self._pub_socket = None
        self._pull_socket = None
        self._rep_socket = None

    def attach_base(self, base: Any | None) -> None:
        self._base = base

    def _base_ready_state(self) -> bool:
        base = self._base
        if base is None:
            return False
        return bool(getattr(base, "_base_ready", True))

    def _request_base_rescan(self, *, force: bool = False) -> None:
        base = self._base
        event = getattr(base, "_base_rescan_requested", None) if base is not None else None
        if force and base is not None and hasattr(base, "_last_base_missing_rescan_at"):
            try:
                setattr(base, "_last_base_missing_rescan_at", 0.0)
            except Exception:
                pass
        if event is not None and hasattr(event, "set"):
            event.set()

    def _base_attr_list(self, name: str) -> list[Any]:
        base = self._base
        value = getattr(base, name, []) if base is not None else []
        return list(value or [])

    def publish_scan(self, ros_msg: dict[str, Any], fallback_frame: str) -> None:
        topic, payload = fast_scan_payload(ros_msg, self.robot_id, fallback_frame)
        self._last_scan_ns = int(payload["stamp_ns"])
        self._publish(topic, payload)

    def publish_joint_states(self, ros_msg: dict[str, Any], fallback_frame: str) -> None:
        topic, payload = fast_joint_state_payload(ros_msg, self.robot_id, fallback_frame)
        with self._lock:
            self._latest_joint_state = dict(ros_msg)
            self._last_joint_state_ns = int(payload["stamp_ns"])
        self._publish(topic, payload)

    def publish_odom(self, ros_msg: dict[str, Any], fallback_frame: str) -> None:
        now_ns = time.time_ns()
        with self._lock:
            latest_joint_state = self._latest_joint_state
            last_command = list(self._last_command) if self._last_command is not None else None
            command_age_ms = (
                (now_ns - self._last_command_ns) / 1_000_000.0
                if self._last_command_ns is not None
                else None
            )
            last_command_source = self._last_command_source
            active_source_id, active_source_priority, active_source_ttl_ms = self._active_source_snapshot_locked(now_ns)
            self._latest_odom_msg = dict(ros_msg)
            self._latest_odom_frame = fallback_frame
            base = self._base
        calibration = getattr(base, "_calibration", {}) if base is not None else {}
        odom_topic = f"odom.{self.robot_id}"
        self._last_odom_ns = stamp_ns_from_header(ros_msg)
        self._publish(odom_topic, {
            "schema": FAST_ZMQ_SCHEMA,
            "stamp_ns": self._last_odom_ns,
            "topic": odom_topic,
            "robot_id": self.robot_id,
            "msg": ros_msg,
        })
        topic, payload = fast_proprio_payload(
            ros_msg,
            self.robot_id,
            fallback_frame,
            latest_joint_state,
            last_command,
            command_age_ms,
            last_command_source,
            active_source_id,
            active_source_priority,
            active_source_ttl_ms,
            calibration=calibration,
        )
        self._publish(topic, payload)
        # Camera/head TF is intentionally published by the dedicated 20Hz TF
        # loop below. Keeping it out of the odom path prevents base feedback or
        # wheel serial reads from throttling depth projection TF.

    @staticmethod
    def _odom_msg_for_tf(odom_msg: dict[str, Any] | None) -> dict[str, Any]:
        msg = dict(odom_msg) if isinstance(odom_msg, dict) else {}
        header = dict(msg.get("header") or {})
        header["stamp"] = stamp()
        header.setdefault("frame_id", "odom")
        msg["header"] = header
        msg.setdefault("child_frame_id", "base_link")
        msg.setdefault(
            "pose",
            {
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": yaw_quat(0.0),
                }
            },
        )
        msg.setdefault(
            "twist",
            {
                "twist": {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
            },
        )
        return msg

    def publish_tf_links(self) -> None:
        with self._lock:
            latest_joint_state = self._latest_joint_state
            latest_odom_msg = self._latest_odom_msg
            fallback_frame = self._latest_odom_frame
            base = self._base
        calibration = getattr(base, "_calibration", {}) if base is not None else {}
        tf_odom_msg = self._odom_msg_for_tf(latest_odom_msg)
        tf_topic, tf_payload = fast_tf_links_payload(
            tf_odom_msg,
            self.robot_id,
            fallback_frame,
            latest_joint_state,
            calibration,
        )
        self._publish(tf_topic, tf_payload)

    def _tf_loop(self) -> None:
        delay = 1.0 / self._tf_rate_hz
        next_publish = time.monotonic()
        while not self.stop_event.is_set():
            self.publish_tf_links()
            next_publish += delay
            sleep_s = next_publish - time.monotonic()
            if sleep_s <= 0.0:
                next_publish = time.monotonic()
                sleep_s = delay
            self.stop_event.wait(sleep_s)

    def _publish(self, topic: str, payload: dict[str, Any]) -> None:
        if self._pub_socket is None or msgpack is None or zmq is None:
            return
        try:
            topic_b = topic.encode("ascii")
            payload_b = msgpack.packb(payload, use_bin_type=True)
            with self._pub_lock:
                if self._pub_socket is None:
                    return
                self._pub_socket.send_multipart([topic_b, payload_b], flags=zmq.NOBLOCK)
        except zmq.Again:
            self._dropped_pub += 1
        except Exception:
            self._dropped_pub += 1

    def _command_loop(self) -> None:
        assert zmq is not None
        poller = zmq.Poller()
        poller.register(self._pull_socket, zmq.POLLIN)
        while not self.stop_event.is_set():
            try:
                events = dict(poller.poll(5))
            except zmq.ZMQError:
                return
            if self._pull_socket not in events:
                continue
            batch: list[bytes] = []
            while True:
                try:
                    batch.append(self._pull_socket.recv(flags=zmq.NOBLOCK))
                except zmq.Again:
                    break
                except zmq.ZMQError:
                    return
            if batch:
                self._handle_command_raw(self._select_command_raw(batch))

    def _decode_command_raw(self, raw: bytes) -> tuple[dict[str, Any] | None, str | None]:
        try:
            command = msgpack.unpackb(raw, raw=False) if msgpack is not None else json.loads(raw.decode("utf-8"))
        except Exception:
            try:
                command = json.loads(raw.decode("utf-8"))
            except Exception as exc:
                return None, f"decode_failed:{exc}"
        if not isinstance(command, dict):
            return None, "command_must_be_dict"
        return command, None

    def _select_command_raw(self, batch: list[bytes]) -> bytes:
        if len(batch) == 1:
            return batch[0]
        now_ns = time.time_ns()
        with self._lock:
            active_id, active_priority, _ttl_ms = self._active_source_snapshot_locked(now_ns)
        best_raw = batch[-1]
        best_key = (-1, -10**9, -1)
        for index, raw in enumerate(batch):
            command, error = self._decode_command_raw(raw)
            if error is not None or command is None:
                key = (0, self._default_command_priority - 1, index)
            else:
                source_id, _source_role, priority, _lease_ms = self._command_source_metadata(command)
                allowed = (
                    active_id is None
                    or source_id == active_id
                    or priority > int(active_priority or 0)
                )
                key = (1 if allowed else 0, priority, index)
            if key > best_key:
                best_key = key
                best_raw = raw
        return best_raw

    def _handle_command_raw(self, raw: bytes) -> None:
        command, error = self._decode_command_raw(raw)
        if error is not None or command is None:
            self._reject_command(error or "command_must_be_dict")
            return

        try:
            twist = self._command_to_twist(command)
            joint_targets = self._command_to_joint_targets(command, allow_unsupported=twist is not None)
        except ValueError as exc:
            self._reject_command(str(exc))
            return
        if twist is None and joint_targets is None:
            self._reject_command("no_supported_command_fields")
            return

        nonzero_twist = bool(twist is not None and any(abs(value) > 1e-9 for value in twist))
        joint_target_command = bool(
            joint_targets is not None and any(value is not None for value in joint_targets)
        )
        source_id, source_role, priority, lease_ms = self._command_source_metadata(command)
        if nonzero_twist and self._estop:
            self._reject_command("estop_enabled")
            return
        base = self._base
        if base is None:
            self._reject_command("base_not_attached")
            return
        if nonzero_twist and not self._base_ready_state():
            self._request_base_rescan()
            self._reject_command("base_not_ready")
            twist = None
            if joint_targets is None:
                return
        if not self._claim_command_source(
            source_id,
            priority,
            lease_ms,
            material=bool(
                twist is not None and any(abs(value) > 1e-9 for value in twist)
            ) or joint_target_command,
            stop=twist is not None and not nonzero_twist and not joint_target_command,
        ):
            return
        if (
            twist is not None
            and not nonzero_twist
            and not joint_target_command
            and self._should_skip_redundant_stop(source_id, source_role, priority)
        ):
            return
        if twist is not None:
            linear_x, linear_y, angular_z = twist
            base.set_base_twist(linear_x, linear_y, angular_z, source="fast_zmq")
        if joint_targets is not None:
            base.set_joint_targets_raw(joint_targets, source="fast_zmq")
        with self._lock:
            if twist is not None:
                self._last_command = [linear_x, linear_y, angular_z]
                self._last_command_ns = time.time_ns()
            self._last_command_source = source_id
            self._last_command_source_role = source_role
            self._last_command_priority = priority
            self._last_rejected_reason = None
            self._accepted_commands += 1

    def _command_source_metadata(self, command: dict[str, Any]) -> tuple[str, str | None, int, int]:
        source = command.get("source_id") or command.get("source") or "anonymous"
        source_id = str(source)
        raw_role = command.get("source_role")
        source_role = str(raw_role) if isinstance(raw_role, str) and raw_role else None
        priority = self._default_command_priority
        raw_priority = command.get("priority")
        if not isinstance(raw_priority, bool) and isinstance(raw_priority, (int, float)):
            raw_priority_float = float(raw_priority)
            if math.isfinite(raw_priority_float):
                priority = int(raw_priority_float)
        elif source_role is not None:
            priority = COMMAND_SOURCE_ROLE_PRIORITY.get(source_role, priority)
        raw_lease_ms = command.get("lease_ms")
        lease_ms = self._default_command_lease_ms
        if not isinstance(raw_lease_ms, bool) and isinstance(raw_lease_ms, (int, float)):
            parsed_lease = int(raw_lease_ms)
            if parsed_lease > 0:
                lease_ms = parsed_lease
        lease_ms = int(clamp(float(lease_ms), 50.0, float(self._max_command_lease_ms)))
        return source_id, source_role, priority, lease_ms

    def _active_source_snapshot_locked(self, now_ns: int) -> tuple[str | None, int | None, float | None]:
        active_id = self._active_source_id
        active_until = self._active_source_until_ns
        if active_id is None or active_until is None or now_ns >= active_until:
            return None, None, None
        return active_id, self._active_source_priority, (active_until - now_ns) / 1_000_000.0

    def _clear_active_source_locked(self) -> None:
        self._active_source_id = None
        self._active_source_priority = -1
        self._active_source_until_ns = None

    def _claim_command_source(
        self,
        source_id: str,
        priority: int,
        lease_ms: int,
        *,
        material: bool,
        stop: bool,
    ) -> bool:
        now_ns = time.time_ns()
        with self._lock:
            active_id, active_priority, _ttl_ms = self._active_source_snapshot_locked(now_ns)
            if active_id is not None and source_id != active_id and priority <= int(active_priority or 0):
                self._last_rejected_reason = (
                    f"source_lease_active:{active_id}:priority={active_priority}:"
                    f"rejected={source_id}:priority={priority}"
                )
                self._dropped_commands += 1
                return False
            if material:
                self._active_source_id = source_id
                self._active_source_priority = priority
                self._active_source_until_ns = now_ns + int(lease_ms) * 1_000_000
            elif stop:
                self._clear_active_source_locked()
        return True

    def _should_skip_redundant_stop(
        self,
        source_id: str,
        source_role: str | None,
        priority: int,
    ) -> bool:
        if self._redundant_stop_dedupe_ms <= 0:
            return False
        if source_role in {"safety", "teleop"}:
            return False
        if priority >= COMMAND_SOURCE_ROLE_PRIORITY["safety"]:
            return False
        now_ns = time.time_ns()
        window_ns = int(self._redundant_stop_dedupe_ms) * 1_000_000
        with self._lock:
            last_command = self._last_command
            if last_command is None or any(abs(value) > 1e-9 for value in last_command):
                self._last_redundant_stop_ns_by_source[source_id] = now_ns
                return False
            last_seen_ns = self._last_redundant_stop_ns_by_source.get(source_id)
            if last_seen_ns is not None and now_ns - last_seen_ns < window_ns:
                self._skipped_redundant_stop_commands += 1
                self._last_rejected_reason = None
                return True
            self._last_redundant_stop_ns_by_source[source_id] = now_ns
        return False

    def _command_to_twist(self, command: dict[str, Any]) -> tuple[float, float, float] | None:
        frame = str(command.get("frame") or "body")
        if frame != "body":
            raise ValueError("only body-frame base commands are supported")
        if "base_cmd_vel" in command:
            base = finite_float_list(command["base_cmd_vel"], 3, "base_cmd_vel")
            return base[0], base[1], base[2]
        if "linear" in command or "angular" in command:
            return (
                vector3(command, "linear", "x"),
                vector3(command, "linear", "y"),
                vector3(command, "angular", "z"),
            )
        if "direction" in command:
            return self._direction_to_twist(str(command.get("direction")), command.get("speed", 1.0))
        return None

    def _direction_to_twist(self, direction: str, speed: Any) -> tuple[float, float, float]:
        clamped = clamp(finite_float(speed, 1.0), 0.0, 1.0)
        if direction == "forward":
            return self.web_linear_speed * clamped, 0.0, 0.0
        if direction == "backward":
            return -self.web_linear_speed * clamped, 0.0, 0.0
        if direction == "left":
            return 0.0, self.web_strafe_speed * clamped, 0.0
        if direction == "right":
            return 0.0, -self.web_strafe_speed * clamped, 0.0
        if direction == "rotate_left":
            return 0.0, 0.0, self.web_yaw_speed * clamped
        if direction == "rotate_right":
            return 0.0, 0.0, -self.web_yaw_speed * clamped
        return 0.0, 0.0, 0.0

    def _command_to_joint_targets(self, command: dict[str, Any], *, allow_unsupported: bool = False) -> list[float | None] | None:
        targets: list[float | None] | None = None
        if "joint_targets_sparse" in command:
            targets = finite_optional_float_list(
                command["joint_targets_sparse"],
                len(ROS_JOINT_NAMES),
                "joint_targets_sparse",
            )
        elif "joint_targets" in command:
            targets = finite_float_list(command["joint_targets"], len(ROS_JOINT_NAMES), "joint_targets")
        elif "arm_joint_pos_target" in command:
            if command.get("arm_joint_pos_target_units") != "feetech_ticks":
                if allow_unsupported:
                    return None
                raise ValueError("arm_joint_pos_target_units_must_be_feetech_ticks")
            canonical = finite_float_list(
                command["arm_joint_pos_target"],
                len(FAST_JOINT_POS_ORDER),
                "arm_joint_pos_target",
            )
            targets = canonical_joint_ticks_to_ros_order(canonical)
        elif "arm_ee_pose_target" in command:
            # Base-frame EE pose target (e.g. from the teleop bridge): solve IK
            # here and drive joint ticks. Unsolvable/unavailable -> None (ignored).
            targets = _arm_ee_pose_target_to_ros_ticks(
                self._latest_joint_state,
                command["arm_ee_pose_target"],
            )
        if "head_joint_relative_target" in command:
            head_targets = self._head_relative_to_sparse_targets(
                command["head_joint_relative_target"]
            )
            if targets is None:
                return head_targets
            merged = list(targets)
            for index, value in enumerate(head_targets):
                if value is not None:
                    merged[index] = value
            return merged
        return targets

    def _head_relative_to_sparse_targets(self, head: Any) -> list[float | None]:
        if not isinstance(head, dict):
            raise ValueError("head_joint_relative_target_must_be_dict")
        with self._lock:
            latest_joint_state = self._latest_joint_state
        positions = (
            latest_joint_state.get("position")
            if isinstance(latest_joint_state, dict)
            else None
        )
        if not isinstance(positions, list) or len(positions) < len(ROS_JOINT_NAMES):
            raise ValueError("head_joint_relative_target_requires_joint_state")
        targets: list[float | None] = [None] * len(ROS_JOINT_NAMES)
        desc = _xlerobot_model_description_cached()
        joints = {
            str(joint["name"]): joint
            for joint in (desc.get("joints") if isinstance(desc, dict) else []) or []
            if isinstance(joint, dict) and isinstance(joint.get("name"), str)
        }
        inverted_joints = _tf_inverted_urdf_joints()
        for field, index, joint_name in (
            ("head_pan", 12, "head_pan_joint"),
            ("head_tilt", 13, "head_tilt_joint"),
        ):
            raw_delta = head.get(field, 0.0)
            if isinstance(raw_delta, bool) or not isinstance(raw_delta, (int, float)):
                raise ValueError(f"head_joint_relative_target.{field}_must_be_number")
            delta_rad = float(raw_delta)
            if not math.isfinite(delta_rad):
                raise ValueError(f"head_joint_relative_target.{field}_must_be_finite")
            current = finite_float(positions[index])
            if joints:
                tick_delta = _urdf_joint_rad_delta_to_raw_tick_delta(
                    joint_name,
                    delta_rad,
                    joints.get(joint_name),
                    inverted_joints,
                )
            else:
                tick_delta = delta_rad * HEAD_RELATIVE_TICK_SIGNS.get(field, 1.0) * FEETECH_TICKS_PER_RAD
            targets[index] = clamp(
                current + tick_delta,
                0.0,
                4095.0,
            )
        return targets

    def _reject_command(self, reason: str) -> None:
        with self._lock:
            self._last_rejected_reason = reason
            self._dropped_commands += 1

    def _rpc_loop(self) -> None:
        assert zmq is not None
        poller = zmq.Poller()
        poller.register(self._rep_socket, zmq.POLLIN)
        while not self.stop_event.is_set():
            try:
                events = dict(poller.poll(100))
            except zmq.ZMQError:
                return
            if self._rep_socket not in events:
                continue
            try:
                raw = self._rep_socket.recv(flags=zmq.NOBLOCK)
                reply = self._handle_rpc(raw)
            except Exception as exc:
                reply = self._pack_reply(False, error=str(exc))
            try:
                self._rep_socket.send(reply)
            except zmq.ZMQError:
                return

    def _handle_rpc(self, raw: bytes) -> bytes:
        try:
            request = msgpack.unpackb(raw, raw=False) if msgpack is not None else json.loads(raw.decode("utf-8"))
        except Exception:
            request = json.loads(raw.decode("utf-8"))
        if not isinstance(request, dict):
            return self._pack_reply(False, error="request must be a dict")
        op = str(request.get("op") or "health")
        if op == "health":
            return self._pack_reply(True, health=self._health())
        if op == "topic_list":
            return self._pack_reply(True, topics=self._topics())
        if op == "fleet_info":
            return self._pack_reply(
                True,
                num_robots=1,
                command_schema=SCHEMA_VERSION_V11,
                action_dim_per_robot=23,
                base_model={"source": "xlerobot_direct_fast_zmq"},
            )
        if op == "joint_names":
            return self._pack_reply(
                True,
                joint_pos_order=list(FAST_JOINT_POS_ORDER),
                joint_vel_order=list(FAST_BASE_VEL_ORDER),
            )
        if op == "command_status":
            return self._pack_reply(True, **self._command_status())
        if op == "head_debug":
            with self._lock:
                latest_joint_state = dict(self._latest_joint_state or {})
            head_position = {}
            names = latest_joint_state.get("name") or latest_joint_state.get("names")
            positions = latest_joint_state.get("position")
            if isinstance(names, list) and isinstance(positions, list):
                by_name = {
                    str(name): positions[idx]
                    for idx, name in enumerate(names)
                    if idx < len(positions)
                }
                head_position = {
                    "head_pan": by_name.get("head_pan"),
                    "head_tilt": by_name.get("head_tilt"),
                    "head_pan_joint": by_name.get("head_pan_joint"),
                    "head_tilt_joint": by_name.get("head_tilt_joint"),
                }
            base = self._base
            calibration = (
                base.head_debug_status()
                if base is not None and hasattr(base, "head_debug_status")
                else {}
            )
            return self._pack_reply(
                True,
                calibration=calibration,
                head_position=head_position,
                joint_state_age_ms=self._age_ms(time.time_ns(), self._last_joint_state_ns),
            )
        if op in {"request_rescan", "rescan_motors", "motor_rescan"}:
            self._request_base_rescan(force=bool(request.get("force", True)))
            return self._pack_reply(True, requested=True, health=self._health())
        if op == "stop":
            base = self._base
            if base is not None:
                base.set_base_twist(0.0, 0.0, 0.0, source="fast_zmq_rpc")
            return self._pack_reply(True)
        if op == "set_estop":
            self._estop = bool(request.get("enabled", request.get("estop", True)))
            base = self._base
            if self._estop and base is not None:
                base.set_base_twist(0.0, 0.0, 0.0, source="fast_zmq_estop")
            return self._pack_reply(True, estop=self._estop)
        return self._pack_reply(False, error=f"unsupported op {op!r}")

    def _topics(self) -> list[str]:
        rid = self.robot_id
        return [f"scan.{rid}", f"proprio.{rid}", f"tf.links.{rid}", f"odom.{rid}", f"joint_states.{rid}"]

    def _health(self) -> dict[str, Any]:
        now = time.time_ns()
        return {
            "ok": True,
            "source": "xlerobot_direct_fast_zmq",
            "robot_id": self.robot_id,
            "pub": f"tcp://{self.bind_host}:{self.pub_port}",
            "pull": f"tcp://{self.bind_host}:{self.pull_port}",
            "rep": f"tcp://{self.bind_host}:{self.rep_port}",
            "base_attached": bool(self._base is not None),
            "base_ready": self._base_ready_state(),
            "base_controller_attached": self._base is not None,
            "motor_connected": bool(getattr(self._base, "_connected", False)) if self._base is not None else False,
            "bus1_detected_ids": self._base_attr_list("_bus1_detected_ids"),
            "bus1_missing_ids": self._base_attr_list("_bus1_missing_ids"),
            "bus2_detected_ids": self._base_attr_list("_bus2_detected_ids"),
            "bus2_missing_ids": self._base_attr_list("_bus2_missing_ids"),
            "estop": self._estop,
            "scan_age_ms": self._age_ms(now, self._last_scan_ns),
            "odom_age_ms": self._age_ms(now, self._last_odom_ns),
            "joint_state_age_ms": self._age_ms(now, self._last_joint_state_ns),
            "command_age_ms": self._age_ms(now, self._last_command_ns),
            "last_command_source": self._last_command_source,
            "last_command_source_role": self._last_command_source_role,
            "last_command_priority": self._last_command_priority,
            "active_source_id": self._active_source_snapshot_locked(now)[0],
            "active_source_priority": self._active_source_snapshot_locked(now)[1],
            "active_source_ttl_ms": self._active_source_snapshot_locked(now)[2],
            "accepted_commands": self._accepted_commands,
            "dropped_commands": self._dropped_commands,
            "skipped_redundant_stop_commands": self._skipped_redundant_stop_commands,
            "dropped_pub": self._dropped_pub,
            "topics": self._topics(),
        }

    def _command_status(self) -> dict[str, Any]:
        now = time.time_ns()
        with self._lock:
            return {
                "robot_id": self.robot_id,
                "last_command": self._last_command,
                "last_command_source": self._last_command_source,
                "last_command_source_role": self._last_command_source_role,
                "last_command_priority": self._last_command_priority,
                "last_command_age_ms": self._age_ms(now, self._last_command_ns),
                "last_rejected_reason": self._last_rejected_reason,
                "active_source_id": self._active_source_snapshot_locked(now)[0],
                "active_source_priority": self._active_source_snapshot_locked(now)[1],
                "active_source_ttl_ms": self._active_source_snapshot_locked(now)[2],
                "accepted_commands": self._accepted_commands,
                "dropped_commands": self._dropped_commands,
                "skipped_redundant_stop_commands": self._skipped_redundant_stop_commands,
                "estop": self._estop,
                "base_ready": self._base_ready_state(),
                "base_controller_attached": self._base is not None,
                "bus2_detected_ids": self._base_attr_list("_bus2_detected_ids"),
                "bus2_missing_ids": self._base_attr_list("_bus2_missing_ids"),
            }

    @staticmethod
    def _age_ms(now_ns: int, stamp_ns: int | None) -> float | None:
        if stamp_ns is None:
            return None
        return (now_ns - stamp_ns) / 1_000_000.0

    @staticmethod
    def _pack_reply(ok: bool, *, error: str | None = None, **payload: Any) -> bytes:
        body = {"ok": bool(ok), **payload}
        if error is not None:
            body["error"] = error
        if msgpack is not None:
            return msgpack.packb(body, use_bin_type=True)
        return json.dumps(body, separators=(",", ":")).encode("utf-8")


class XLeRobotHardware:
    JOINT_NAMES = ROS_JOINT_NAMES
    BASE_WHEEL_NAMES = XLE_BASE_WHEEL_NAMES
    JOINT_STATE_NAMES = tuple(JOINT_NAMES) + BASE_WHEEL_NAMES

    def __init__(
        self,
        stop_event: threading.Event,
        status: StatusPublisher,
        dry_run: bool,
        fast_bus: FastRobotZmqServer | None = None,
    ):
        self.stop_event = stop_event
        self.status = status
        self.dry_run = dry_run
        self.fast_bus = fast_bus

        self.port1 = motor_driver_by_serial(
            MOTOR_LEFT_HEAD_SERIAL_SHORT,
            env_first(("XLEROBOT_PORT1", "PORT1", "LEFT_HEAD_PORT"), "/dev/ttyACM1"),
        )
        self.port2 = motor_driver_by_serial(
            MOTOR_RIGHT_BASE_SERIAL_SHORT,
            env_first(("XLEROBOT_PORT2", "PORT2", "RIGHT_BASE_PORT", "ROBOT_PORT"), "/dev/ttyACM0"),
        )
        self.robot_id = os.environ.get("XLEROBOT_ID", "indoory_fast")
        self.use_calibration_limits = env_bool("XLEROBOT_USE_CALIBRATION_LIMITS", True)
        self._calibration, self._calibration_path = load_xlerobot_calibration(self.robot_id)

        self.wheel_radius = env_float("BASE_WHEEL_RADIUS_M", 0.05)
        self.base_radius = env_float("BASE_RADIUS_M", 0.125)
        self.left_sign = env_float("BASE_LEFT_SIGN", 1.0)
        self.back_sign = env_float("BASE_BACK_SIGN", 1.0)
        self.right_sign = env_float("BASE_RIGHT_SIGN", 1.0)
        self._wheel_signs = {
            "base_left_wheel": self.left_sign,
            "base_back_wheel": self.back_sign,
            "base_right_wheel": self.right_sign,
        }
        self.max_raw = env_int("BASE_MAX_RAW_COMMAND", 3000)
        self.command_rate_hz = max(1.0, env_float("BASE_COMMAND_RATE_HZ", 200.0))
        self.joint_command_rate_hz = max(1.0, env_float("JOINT_COMMAND_RATE_HZ", 50.0))
        self.joint_state_rate_hz = max(1.0, env_float("JOINT_STATE_RATE_HZ", 20.0))
        self.odom_rate_hz = max(1.0, env_float("BASE_FEEDBACK_RATE_HZ", 20.0))
        self.watchdog_timeout = max(0.05, env_float("BASE_WATCHDOG_TIMEOUT_S", 0.3))
        self.reconnect_delay_s = max(0.2, env_float("BASE_RECONNECT_DELAY_S", 2.0))
        self.base_missing_rescan_delay_s = max(0.5, env_float("BASE_MISSING_RESCAN_DELAY_S", 5.0))
        self.bus_scan_settle_s = max(0.0, env_float("MOTOR_BUS_SCAN_SETTLE_S", 0.25))
        self.bus_scan_passes = max(1, env_int("MOTOR_BUS_SCAN_PASSES", 4))
        self.bus_scan_retry = max(0, env_int("MOTOR_BUS_SCAN_RETRY", 5))
        self.bus_scan_pass_delay_s = max(0.0, env_float("MOTOR_BUS_SCAN_PASS_DELAY_S", 0.08))
        self.joint_target_min = env_int("JOINT_TARGET_MIN", 0)
        self.joint_target_max = env_int("JOINT_TARGET_MAX", 4095)
        self.odom_frame = os.environ.get("ODOM_FRAME", "odom")
        self.base_frame = os.environ.get("BASE_FRAME", "base_link")
        self.publish_odom = env_bool("BASE_PUBLISH_ODOM", False)
        self.use_feedback_odom = env_bool("BASE_USE_FEEDBACK_ODOM", True)
        self.disable_torque_on_shutdown = env_bool("BASE_DISABLE_TORQUE_ON_SHUTDOWN", False)

        self._cmd_lock = threading.Lock()
        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_z = 0.0
        self._last_cmd_at = 0.0
        self._last_base_written: Optional[tuple[int, int, int]] = None
        self._last_status_at = 0.0
        self._last_joint_target_at = 0.0
        self._last_connect_attempt = 0.0
        self._last_base_missing_rescan_at = 0.0
        self._connected = False
        self._cmd_event = threading.Event()
        self._base_rescan_requested = threading.Event()
        self._bus_lock = threading.RLock()
        self._robot = None
        self._operating_mode = None
        self._bus1_available: set[str] = set()
        self._bus2_available: set[str] = set()
        self._bus1_detected_ids: list[int] = []
        self._bus2_detected_ids: list[int] = []
        self._bus1_missing_ids: list[int] = []
        self._bus2_missing_ids: list[int] = []
        self._base_ready = False
        self._joint_lock = threading.Lock()
        self._joint_targets: list[Optional[int]] = [None] * len(self.JOINT_NAMES)
        self._joint_written: dict[str, int] = {}
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_odom_at = time.monotonic()
        self._threads: list[threading.Thread] = []

        angles = np.radians(np.array([240.0, 0.0, 120.0]) - 90.0)
        self._omni_matrix = np.array(
            [[math.cos(angle), math.sin(angle), self.base_radius] for angle in angles],
            dtype=float,
        )
        self._omni_inverse = np.linalg.pinv(self._omni_matrix)

    def start(self) -> None:
        self._threads = [
            threading.Thread(target=self._command_loop, name="motor-command", daemon=True),
            threading.Thread(target=self._joint_state_loop, name="motor-joint-state", daemon=True),
        ]
        if self.publish_odom or self.fast_bus is not None:
            self._threads.append(threading.Thread(target=self._odom_loop, name="base-odom", daemon=True))
        for thread in self._threads:
            thread.start()
        mode = "dry-run" if self.dry_run else "hardware"
        odom_desc = "fast_zmq" if self.publish_odom or self.fast_bus is not None else "off"
        print(
            f"[motors] started ({mode}), command=fast_zmq, "
            f"joint_states=fast_zmq, odom={odom_desc}",
            flush=True,
        )
        self.status.update(
            motors="started",
            base="started",
            motor_mode=mode,
            motor_bus1_role="left_arm+head",
            motor_bus2_role="right_arm+base",
            motor_port1=self.port1,
            motor_port2=self.port2,
            bus1_expected_motors=list(XLE_LEFT_JOINT_NAMES + XLE_HEAD_JOINT_NAMES),
            bus2_expected_motors=list(XLE_RIGHT_JOINT_NAMES + XLE_BASE_WHEEL_NAMES),
        )

    def stop(self) -> None:
        self._safe_stop()
        self._disconnect_buses()

    def set_base_twist(self, linear_x: float, linear_y: float, angular_z: float, *, source: str) -> None:
        with self._cmd_lock:
            self._linear_x = finite_float(linear_x)
            self._linear_y = finite_float(linear_y)
            self._angular_z = finite_float(angular_z)
            self._last_cmd_at = time.monotonic()
        self._cmd_event.set()

    def set_joint_targets_raw(self, data: Any, *, source: str) -> None:
        if not isinstance(data, list):
            return
        changed = False
        with self._joint_lock:
            for index, raw_value in enumerate(data[:len(self.JOINT_NAMES)]):
                external_name = self.JOINT_NAMES[index]
                try:
                    value = float(raw_value)
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(value):
                    continue
                lo, hi = self._joint_target_limits(external_name)
                target = int(round(clamp(value, lo, hi)))
                if self._joint_targets[index] != target:
                    self._joint_targets[index] = target
                    changed = True
            if changed:
                self._last_joint_target_at = time.monotonic()
        if changed:
            self._cmd_event.set()

    def _joint_target_limits(self, external_name: str) -> tuple[float, float]:
        if not self.use_calibration_limits:
            return float(self.joint_target_min), float(self.joint_target_max)
        xle_name = ROS_TO_XLE_JOINT.get(external_name)
        cal = self._calibration.get(xle_name or "")
        if not isinstance(cal, dict):
            return float(self.joint_target_min), float(self.joint_target_max)
        try:
            lo = float(cal["range_min"])
            hi = float(cal["range_max"])
        except (KeyError, TypeError, ValueError):
            return float(self.joint_target_min), float(self.joint_target_max)
        if not math.isfinite(lo) or not math.isfinite(hi) or hi <= lo:
            return float(self.joint_target_min), float(self.joint_target_max)
        return lo, hi

    def head_debug_status(self) -> dict[str, Any]:
        heads: dict[str, Any] = {}
        for external_name in ROS_HEAD_JOINT_NAMES:
            xle_name = ROS_TO_XLE_JOINT.get(external_name, external_name)
            cal = self._calibration.get(xle_name) or {}
            motor_id = cal.get("id")
            if motor_id is None and self._robot is not None:
                for bus in (self._robot.bus1, self._robot.bus2):
                    motor = getattr(bus, "motors", {}).get(xle_name)
                    if motor is not None:
                        motor_id = getattr(motor, "id", None)
                        break
            lo, hi = self._joint_target_limits(external_name)
            heads[external_name] = {
                "xlerobot_motor": xle_name,
                "id": motor_id,
                "range_min": lo,
                "range_max": hi,
                "range_width": hi - lo,
                "homing_offset": cal.get("homing_offset") if isinstance(cal, dict) else None,
                "tf_zero_tick": _head_tf_zero_tick(
                    "head_pan_joint" if external_name == "head_pan" else "head_tilt_joint",
                    (lo + hi) * 0.5,
                    self._calibration,
                ),
                "calibration_loaded": bool(cal),
            }
        return {
            "calibration_path": self._calibration_path,
            "calibration_limits": self.use_calibration_limits,
            "head": heads,
        }

    def _connect_buses(self) -> None:
        self._last_connect_attempt = time.monotonic()
        with self._bus_lock:
            self._disconnect_buses()
            self._last_base_written = None
            self._joint_written = {}

            if self.dry_run:
                self._bus1_available = set(XLE_LEFT_JOINT_NAMES + XLE_HEAD_JOINT_NAMES)
                self._bus2_available = set(XLE_RIGHT_JOINT_NAMES + XLE_BASE_WHEEL_NAMES)
                self._bus1_detected_ids = list(range(1, 9))
                self._bus2_detected_ids = list(range(1, 10))
                self._bus1_missing_ids = []
                self._bus2_missing_ids = []
                self._base_ready = True
                self._connected = True
                self.status.update(motors="connected", base="connected", base_ready=True)
                return

            try:
                XLerobot, XLerobotConfig, OperatingMode = import_lerobot_xlerobot()
            except Exception as exc:
                raise RuntimeError(f"LeRobot XLeRobot import failed: {exc}") from exc

            self._operating_mode = OperatingMode
            config = XLerobotConfig(
                id=self.robot_id,
                port1=self.port1,
                port2=self.port2,
                cameras={},
                use_degrees=env_bool("XLEROBOT_USE_DEGREES", False),
            )
            self._robot = XLerobot(config)
            if self._calibration_path:
                print(f"[motors] calibration loaded {self._calibration_path}", flush=True)

            errors: list[str] = []
            bus1_ok = False
            bus2_ok = False
            try:
                bus1_ok = self._connect_pruned_bus("bus1", self._robot.bus1)
            except Exception as exc:
                errors.append(f"bus1 {self.port1}: {exc}")
                self._disconnect_bus_quietly(self._robot.bus1)
            try:
                bus2_ok = self._connect_pruned_bus("bus2", self._robot.bus2)
            except Exception as exc:
                errors.append(f"bus2 {self.port2}: {exc}")
                self._disconnect_bus_quietly(self._robot.bus2)

            self._refresh_robot_motor_lists()
            if bus1_ok:
                self._configure_bus(self._robot.bus1, base_motors=set())
            if bus2_ok:
                self._configure_bus(self._robot.bus2, base_motors=set(XLE_BASE_WHEEL_NAMES))

            self._bus1_available = set(self._robot.bus1.motors) if bus1_ok else set()
            self._bus2_available = set(self._robot.bus2.motors) if bus2_ok else set()
            self._base_ready = all(name in self._bus2_available for name in XLE_BASE_WHEEL_NAMES)
            self._connected = bus1_ok or bus2_ok
            if not self._connected:
                raise RuntimeError("; ".join(errors) or "no Feetech motors detected on either XLeRobot bus")

            status = "connected" if self._base_ready else "error"
            base_error = "" if self._base_ready else "base wheel ID missing on bus2; base velocity writes disabled"
            self.status.update(
                motors="connected",
                base=status,
                base_ready=self._base_ready,
                base_error=base_error,
                bus1_role="left_arm+head",
                bus2_role="right_arm+base",
                calibration_path=self._calibration_path,
                calibration_limits=self.use_calibration_limits,
                bus1_detected_ids=self._bus1_detected_ids,
                bus1_missing_ids=self._bus1_missing_ids,
                bus2_detected_ids=self._bus2_detected_ids,
                bus2_missing_ids=self._bus2_missing_ids,
                motor_errors=errors,
            )
            print(
                f"[motors] connected bus1(left+head) {self.port1} ids={self._bus1_detected_ids} "
                f"missing={self._bus1_missing_ids}; bus2(right+base) {self.port2} "
                f"ids={self._bus2_detected_ids} missing={self._bus2_missing_ids}",
                flush=True,
            )

    def _connect_pruned_bus(self, label: str, bus) -> bool:
        expected_ids = {motor.id for motor in bus.motors.values()}
        bus.connect(handshake=False)
        if self.bus_scan_settle_s > 0.0:
            time.sleep(self.bus_scan_settle_s)
        detected_by_id: set[int] = set()
        for scan_pass in range(self.bus_scan_passes):
            missing_before = expected_ids - detected_by_id
            for motor_id in sorted(missing_before):
                if bus.ping(motor_id, num_retry=self.bus_scan_retry) is not None:
                    detected_by_id.add(motor_id)
            if detected_by_id >= expected_ids:
                break
            if scan_pass + 1 < self.bus_scan_passes and self.bus_scan_pass_delay_s > 0.0:
                time.sleep(self.bus_scan_pass_delay_s)

        missing = sorted(expected_ids - detected_by_id)
        if label == "bus1":
            self._bus1_detected_ids = sorted(detected_by_id)
            self._bus1_missing_ids = missing
        else:
            self._bus2_detected_ids = sorted(detected_by_id)
            self._bus2_missing_ids = missing

        available_motors = {
            name: motor for name, motor in bus.motors.items()
            if motor.id in detected_by_id
        }
        self._prune_bus(bus, available_motors)
        if available_motors:
            return True
        try:
            bus.disconnect(False)
        except Exception:
            pass
        return False

    @staticmethod
    def _disconnect_bus_quietly(bus) -> None:
        try:
            if bus.is_connected:
                bus.disconnect(False)
        except Exception:
            pass

    @staticmethod
    def _prune_bus(bus, available_motors: dict[str, Any]) -> None:
        bus.motors = available_motors
        bus.calibration = {
            name: calibration for name, calibration in bus.calibration.items()
            if name in available_motors
        }
        bus._id_to_model_dict = {motor.id: motor.model for motor in bus.motors.values()}
        bus._id_to_name_dict = {motor.id: name for name, motor in bus.motors.items()}
        for attr in ("models", "ids", "_has_different_ctrl_tables"):
            bus.__dict__.pop(attr, None)

    def _refresh_robot_motor_lists(self) -> None:
        if self._robot is None:
            return
        self._robot.left_arm_motors = [name for name in self._robot.bus1.motors if name.startswith("left_arm")]
        self._robot.head_motors = [name for name in self._robot.bus1.motors if name.startswith("head")]
        self._robot.right_arm_motors = [name for name in self._robot.bus2.motors if name.startswith("right_arm")]
        self._robot.base_motors = [name for name in self._robot.bus2.motors if name.startswith("base")]

    def _configure_bus(self, bus, base_motors: set[str]) -> None:
        if self._operating_mode is None:
            raise RuntimeError("XLeRobot OperatingMode is not loaded")
        bus.disable_torque()
        bus.configure_motors()
        for name in bus.motors:
            mode = (
                self._operating_mode.VELOCITY.value
                if name in base_motors
                else self._operating_mode.POSITION.value
            )
            bus.write("Operating_Mode", name, mode, normalize=False)
            if name not in base_motors:
                try:
                    bus.write("P_Coefficient", name, 16, normalize=False)
                    bus.write("I_Coefficient", name, 0, normalize=False)
                    bus.write("D_Coefficient", name, 43, normalize=False)
                except Exception:
                    pass
        bus.enable_torque()

    def _disconnect_buses(self) -> None:
        robot = self._robot
        if robot is not None:
            for bus in (robot.bus1, robot.bus2):
                try:
                    if bus.is_connected:
                        bus.disconnect(self.disable_torque_on_shutdown)
                except Exception:
                    pass
        self._robot = None
        self._bus1_available = set()
        self._bus2_available = set()
        self._base_ready = False
        self._connected = False

    def _command_loop(self) -> None:
        delay = min(1.0 / self.command_rate_hz, 1.0 / self.joint_command_rate_hz)
        last_error = ""
        while not self.stop_event.is_set():
            if not self._connected:
                if time.monotonic() - self._last_connect_attempt >= self.reconnect_delay_s:
                    try:
                        self._connect_buses()
                        last_error = ""
                    except Exception as exc:
                        msg = str(exc)
                        if msg != last_error:
                            print(f"[motors] connect failed: {msg}", flush=True)
                            self.status.update(motors="error", base="error", motor_error=msg)
                            last_error = msg
                time.sleep(0.05)
                continue

            if not self._base_ready and self._base_rescan_requested.is_set():
                now = time.monotonic()
                if now - self._last_base_missing_rescan_at >= self.base_missing_rescan_delay_s:
                    self._last_base_missing_rescan_at = now
                    self._base_rescan_requested.clear()
                    try:
                        self._connect_buses()
                        last_error = ""
                    except Exception as exc:
                        msg = str(exc)
                        if msg != last_error:
                            print(f"[motors] base rescan failed: {msg}", flush=True)
                            self.status.update(motors="error", base="error", motor_error=msg)
                            last_error = msg
                    continue

            try:
                self._write_base_velocity()
                self._write_joint_targets()
                self._publish_motor_status()
            except Exception as exc:
                print(f"[motors] command loop failed: {exc}", flush=True)
                self.status.update(motors="error", motor_error=str(exc), base="error")
                self._disconnect_buses()
                self._last_base_written = None
            self._cmd_event.wait(delay)
            self._cmd_event.clear()

    def _write_base_velocity(self) -> None:
        linear_x, linear_y, angular_z = self._commanded_twist()
        wheel_goal = self._twist_to_wheel_goal(linear_x, linear_y, angular_z)
        triple = (
            wheel_goal["base_left_wheel"],
            wheel_goal["base_back_wheel"],
            wheel_goal["base_right_wheel"],
        )
        if triple == self._last_base_written:
            return
        if self.dry_run:
            self._last_base_written = triple
            return
        with self._bus_lock:
            if not self._base_ready or self._robot is None:
                return
            self._robot.bus2.sync_write("Goal_Velocity", wheel_goal, normalize=False)
            self._last_base_written = triple

    def _write_joint_targets(self) -> None:
        with self._joint_lock:
            targets = list(self._joint_targets)
        bus1_values: dict[str, int] = {}
        bus2_values: dict[str, int] = {}
        written_external: dict[str, int] = {}
        for external_name, target in zip(self.JOINT_NAMES, targets):
            if target is None or self._joint_written.get(external_name) == target:
                continue
            xle_name = ROS_TO_XLE_JOINT[external_name]
            if xle_name in self._bus1_available:
                bus1_values[xle_name] = target
                written_external[external_name] = target
            elif xle_name in self._bus2_available:
                bus2_values[xle_name] = target
                written_external[external_name] = target

        if self.dry_run:
            self._joint_written.update(written_external)
            return
        with self._bus_lock:
            if self._robot is None:
                return
            if bus1_values and self._robot.bus1.is_connected:
                self._robot.bus1.sync_write("Goal_Position", bus1_values, normalize=False)
            if bus2_values and self._robot.bus2.is_connected:
                self._robot.bus2.sync_write("Goal_Position", bus2_values, normalize=False)
            self._joint_written.update(written_external)

    def _publish_motor_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_at < 1.0:
            return
        self._last_status_at = now
        with self._cmd_lock:
            cmd_age = now - self._last_cmd_at if self._last_cmd_at else math.inf
        with self._joint_lock:
            joint_age = now - self._last_joint_target_at if self._last_joint_target_at else math.inf
        self.status.update(
            motors="connected",
            base="connected" if self._base_ready else "error",
            base_ready=self._base_ready,
            bus1_role="left_arm+head",
            bus2_role="right_arm+base",
            bus1_detected_ids=self._bus1_detected_ids,
            bus1_missing_ids=self._bus1_missing_ids,
            bus2_detected_ids=self._bus2_detected_ids,
            bus2_missing_ids=self._bus2_missing_ids,
            last_cmd_age_s=round(cmd_age, 3) if math.isfinite(cmd_age) else None,
            last_joint_target_age_s=round(joint_age, 3) if math.isfinite(joint_age) else None,
        )

    def _odom_loop(self) -> None:
        delay = 1.0 / self.odom_rate_hz
        while not self.stop_event.is_set():
            now = time.monotonic()
            dt = max(0.0, min(now - self._last_odom_at, 0.2))
            self._last_odom_at = now

            if self._connected and self.use_feedback_odom and not self.dry_run and self._base_ready:
                try:
                    with self._bus_lock:
                        assert self._robot is not None
                        raw = self._robot.bus2.sync_read(
                            "Present_Velocity",
                            list(XLE_BASE_WHEEL_NAMES),
                            normalize=False,
                        )
                    linear_x, linear_y, angular_z = self._raw_to_twist(
                        int(raw["base_left_wheel"]),
                        int(raw["base_back_wheel"]),
                        int(raw["base_right_wheel"]),
                    )
                except Exception:
                    linear_x, linear_y, angular_z = self._commanded_twist()
            else:
                linear_x, linear_y, angular_z = self._commanded_twist()

            if dt > 0.0:
                yaw_for_xy = self._yaw
                self._yaw = math.atan2(
                    math.sin(self._yaw + angular_z * dt),
                    math.cos(self._yaw + angular_z * dt),
                )
                self._x += (linear_x * math.cos(yaw_for_xy) - linear_y * math.sin(yaw_for_xy)) * dt
                self._y += (linear_x * math.sin(yaw_for_xy) + linear_y * math.cos(yaw_for_xy)) * dt
            self._publish_odom(linear_x, linear_y, angular_z)
            time.sleep(delay)

    def _joint_state_loop(self) -> None:
        delay = 1.0 / self.joint_state_rate_hz
        while not self.stop_event.is_set():
            self._publish_joint_states()
            time.sleep(delay)

    def _commanded_twist(self) -> tuple[float, float, float]:
        with self._cmd_lock:
            age = time.monotonic() - self._last_cmd_at if self._last_cmd_at else math.inf
            linear_x = self._linear_x
            linear_y = self._linear_y
            angular_z = self._angular_z
        if age > self.watchdog_timeout:
            return 0.0, 0.0, 0.0
        return linear_x, linear_y, angular_z

    def _publish_joint_states(self) -> None:
        positions: list[float] = []
        velocities: list[float] = []
        with self._bus_lock:
            robot = self._robot
            for name in self.JOINT_STATE_NAMES:
                position = 0.0
                velocity = 0.0
                try:
                    if robot is not None and name in XLE_BASE_WHEEL_NAMES and name in self._bus2_available:
                        raw_velocity = robot.bus2.read(
                            "Present_Velocity", name, normalize=False, num_retry=1)
                        velocity = self._raw_to_radps(int(raw_velocity)) * self._wheel_signs.get(name, 1.0)
                    elif robot is not None and name in ROS_TO_XLE_JOINT:
                        xle_name = ROS_TO_XLE_JOINT[name]
                        if xle_name in self._bus1_available:
                            position = float(robot.bus1.read(
                                "Present_Position", xle_name, normalize=False, num_retry=1))
                        elif xle_name in self._bus2_available:
                            position = float(robot.bus2.read(
                                "Present_Position", xle_name, normalize=False, num_retry=1))
                except Exception:
                    pass
                positions.append(position)
                velocities.append(velocity)
        msg = {
            "header": {"stamp": stamp(), "frame_id": self.base_frame},
            "name": list(self.JOINT_STATE_NAMES),
            "position": positions,
            "velocity": velocities,
            "effort": [0.0] * len(self.JOINT_STATE_NAMES),
        }
        if self.fast_bus is not None:
            self.fast_bus.publish_joint_states(msg, self.base_frame)

    def _publish_odom(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        quat = yaw_quat(self._yaw)
        msg = {
            "header": {"stamp": stamp(), "frame_id": self.odom_frame},
            "child_frame_id": self.base_frame,
            "pose": {
                "pose": {
                    "position": {"x": self._x, "y": self._y, "z": 0.0},
                    "orientation": quat,
                },
                "covariance": [
                    0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.10,
                ],
            },
            "twist": {
                "twist": {
                    "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
                },
                "covariance": [
                    0.10, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.10, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.20,
                ],
            },
        }
        if self.fast_bus is not None:
            self.fast_bus.publish_odom(msg, self.base_frame)

    def _twist_to_wheel_goal(self, linear_x: float, linear_y: float, angular_z: float) -> dict[str, int]:
        if self._robot is not None:
            wheel_goal = self._robot._body_to_wheel_raw(
                linear_x,
                linear_y,
                math.degrees(angular_z),
                wheel_radius=self.wheel_radius,
                base_radius=self.base_radius,
                max_raw=self.max_raw,
            )
        else:
            left_raw, back_raw, right_raw = self._twist_to_raw(linear_x, linear_y, angular_z)
            wheel_goal = {
                "base_left_wheel": left_raw,
                "base_back_wheel": back_raw,
                "base_right_wheel": right_raw,
            }
        return {
            "base_left_wheel": int(clamp(wheel_goal["base_left_wheel"] * self.left_sign, -self.max_raw, self.max_raw)),
            "base_back_wheel": int(clamp(wheel_goal["base_back_wheel"] * self.back_sign, -self.max_raw, self.max_raw)),
            "base_right_wheel": int(clamp(wheel_goal["base_right_wheel"] * self.right_sign, -self.max_raw, self.max_raw)),
        }

    def _twist_to_raw(self, linear_x: float, linear_y: float, angular_z: float) -> tuple[int, int, int]:
        velocity_vector = np.array([linear_x, linear_y, angular_z], dtype=float)
        wheel_linear_speeds = self._omni_matrix.dot(velocity_vector)
        wheel_radps = wheel_linear_speeds / self.wheel_radius
        raw_values = np.array([self._radps_to_raw_raw(radps) for radps in wheel_radps], dtype=float)
        max_abs = float(np.max(np.abs(raw_values))) if raw_values.size else 0.0
        if max_abs > self.max_raw:
            raw_values *= float(self.max_raw) / max_abs
        return tuple(int(clamp(value, -self.max_raw, self.max_raw)) for value in raw_values)

    def _raw_to_twist(self, left_raw: int, back_raw: int, right_raw: int) -> tuple[float, float, float]:
        raw_values = np.array([
            float(left_raw) * self.left_sign,
            float(back_raw) * self.back_sign,
            float(right_raw) * self.right_sign,
        ])
        if self._robot is not None:
            body = self._robot._wheel_raw_to_body(
                int(raw_values[0]),
                int(raw_values[1]),
                int(raw_values[2]),
                wheel_radius=self.wheel_radius,
                base_radius=self.base_radius,
            )
            return float(body["x.vel"]), float(body["y.vel"]), math.radians(float(body["theta.vel"]))
        wheel_radps = np.array([self._raw_to_radps(int(raw)) for raw in raw_values])
        wheel_linear_speeds = wheel_radps * self.wheel_radius
        body = self._omni_inverse.dot(wheel_linear_speeds)
        return float(body[0]), float(body[1]), float(body[2])

    @staticmethod
    def _radps_to_raw_raw(radps: float) -> int:
        degps = radps * 180.0 / math.pi
        return int(round(degps * 4096.0 / 360.0))

    @staticmethod
    def _raw_to_radps(raw: int) -> float:
        degps = float(raw) / (4096.0 / 360.0)
        return degps * math.pi / 180.0

    def _safe_stop(self) -> None:
        if self.dry_run or self._robot is None or not self._base_ready:
            return
        available = {
            name: 0
            for name in XLE_BASE_WHEEL_NAMES
            if name in self._bus2_available
        }
        if not available:
            return
        try:
            self._robot.bus2.sync_write(
                "Goal_Velocity",
                available,
                normalize=False,
                num_retry=5,
            )
        except Exception:
            pass


class RPLidarPublisher:
    def __init__(
        self,
        stop_event: threading.Event,
        status: StatusPublisher,
        dry_run: bool,
        fast_bus: FastRobotZmqServer | None = None,
    ):
        self.stop_event = stop_event
        self.status = status
        self.dry_run = dry_run
        self.fast_bus = fast_bus
        self.topic = os.environ.get("SCAN_TOPIC", "/xlerobot/scan")
        self.serial_port = os.environ.get(
            "LIDAR_SERIAL",
            "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_12703f59806eef11ba3ee8c2c169b110-if00-port0",
        )
        self.baud = env_int("LIDAR_BAUD", 460800)
        self.frame = os.environ.get("LIDAR_FRAME", "base_link")
        self.samples = max(90, env_int("LIDAR_SAMPLES", 240))
        self.publish_rate_hz = max(0.2, env_float("LIDAR_PUBLISH_RATE_HZ", 8.0))
        self.angle_min = env_float("LIDAR_ANGLE_MIN", -math.pi)
        self.angle_max = env_float("LIDAR_ANGLE_MAX", math.pi)
        self.angle_offset = math.radians(env_float("LIDAR_ANGLE_OFFSET_DEG", 0.0))
        self.invert = env_bool("LIDAR_INVERT", False)
        self.range_min = env_float("LIDAR_RANGE_MIN", 0.12)
        self.range_max = env_float("LIDAR_RANGE_MAX", 12.0)
        self.min_quality = env_int("LIDAR_MIN_QUALITY", 0)
        self.min_points = env_int("LIDAR_MIN_POINTS_PER_ROTATION", 30)
        self.serial_timeout_s = env_float("LIDAR_SERIAL_TIMEOUT_S", 0.02)
        self.reconnect_delay_s = env_float("LIDAR_RECONNECT_DELAY_S", 1.0)
        self.fake_hz = max(0.2, env_float("LIDAR_FAKE_HZ", 5.0))

        self._thread = threading.Thread(target=self._run, name="rplidar", daemon=True)
        self._ser = None
        self._last_scan_publish_at = 0.0

    def start(self) -> None:
        self._thread.start()
        mode = "dry-run" if self.dry_run else "serial"
        print(f"[lidar] started ({mode}), topic={self.topic}", flush=True)
        self.status.update(lidar="started", lidar_mode=mode, lidar_port=self.serial_port)

    def stop(self) -> None:
        try:
            self._send(CMD_STOP)
        except Exception:
            pass
        pass

    def _run(self) -> None:
        if self.dry_run:
            self._run_fake()
            return
        if serial is None:
            msg = f"pyserial import failed: {SERIAL_IMPORT_ERROR}"
            print(f"[lidar] {msg}", flush=True)
            self.status.update(lidar="error", lidar_error=msg)
            return

        last_error = ""
        while not self.stop_event.is_set():
            try:
                print(f"[lidar] connecting {self.serial_port} @ {self.baud}", flush=True)
                with serial.Serial(
                    self.serial_port,
                    self.baud,
                    timeout=self.serial_timeout_s,
                    write_timeout=0.5,
                ) as ser:
                    self._ser = ser
                    ser.dtr = False
                    ser.rts = False
                    time.sleep(0.1)
                    self._send(CMD_STOP)
                    time.sleep(0.1)
                    self._read_info()
                    self._read_health()
                    self._start_scan()
                    last_error = ""
                    self.status.update(lidar="connected")

                    current: list[tuple[float, float, int]] = []
                    buffer = bytearray()
                    rotation_started_at: Optional[float] = None

                    while not self.stop_event.is_set():
                        chunk = ser.read(4096)
                        if not chunk:
                            continue
                        buffer.extend(chunk)
                        while len(buffer) >= 5:
                            raw = bytes(buffer[:5])
                            parsed = parse_scan_point(raw)
                            if parsed is None:
                                del buffer[0]
                                continue
                            del buffer[:5]
                            start, angle_deg, distance_mm, quality = parsed
                            now = time.monotonic()
                            if start:
                                if len(current) >= self.min_points:
                                    scan_time = 0.0
                                    if rotation_started_at is not None:
                                        scan_time = max(now - rotation_started_at, 1e-6)
                                    self._publish_rotation(current, scan_time)
                                current = []
                                rotation_started_at = now
                            if distance_mm > 0:
                                current.append((angle_deg, distance_mm, quality))
            except Exception as exc:
                msg = str(exc)
                if msg != last_error:
                    print(f"[lidar] error: {msg}", flush=True)
                    self.status.update(lidar="error", lidar_error=msg)
                    last_error = msg
                time.sleep(self.reconnect_delay_s)
            finally:
                try:
                    self._send(CMD_STOP)
                except Exception:
                    pass
                self._ser = None

    def _run_fake(self) -> None:
        delay = 1.0 / self.fake_hz
        scan_time = delay
        while not self.stop_event.is_set():
            ranges = [4.0] * self.samples
            for i in range(self.samples):
                angle = self.angle_min + (self.angle_max - self.angle_min) * i / self.samples
                if abs(angle) < 0.35:
                    ranges[i] = 6.0
                elif abs(abs(angle) - math.pi * 0.5) < 0.3:
                    ranges[i] = 1.2
            self._publish_scan(ranges, [20.0] * self.samples, scan_time)
            time.sleep(delay)

    def _send(self, cmd: int, payload: bytes = b"") -> None:
        if self._ser is None:
            return
        self._ser.write(make_command(cmd, payload))
        self._ser.flush()

    def _read_descriptor(self, timeout_s: float = 1.0) -> Optional[dict[str, Any]]:
        deadline = time.monotonic() + timeout_s
        window = bytearray()
        while time.monotonic() < deadline and not self.stop_event.is_set():
            byte = self._ser.read(1)
            if not byte:
                continue
            window += byte
            while len(window) >= 2 and not (window[0] == 0xA5 and window[1] == 0x5A):
                del window[0]
            if len(window) == 7:
                raw_len = int.from_bytes(window[2:6], "little")
                return {
                    "size": raw_len & 0x3FFFFFFF,
                    "mode": (raw_len >> 30) & 0x03,
                    "type": window[6],
                    "raw": bytes(window),
                }
        return None

    def _read_exact(self, size: int, timeout_s: float = 1.0) -> bytes:
        deadline = time.monotonic() + timeout_s
        data = bytearray()
        while len(data) < size and time.monotonic() < deadline and not self.stop_event.is_set():
            chunk = self._ser.read(size - len(data))
            if chunk:
                data.extend(chunk)
        return bytes(data)

    def _command_response(self, cmd: int, timeout_s: float = 1.0) -> tuple[Optional[dict[str, Any]], bytes]:
        self._ser.reset_input_buffer()
        self._send(cmd)
        descriptor = self._read_descriptor(timeout_s)
        if not descriptor:
            return None, b""
        return descriptor, self._read_exact(int(descriptor["size"]), timeout_s)

    def _read_info(self) -> None:
        descriptor, data = self._command_response(CMD_GET_INFO, 1.0)
        if not descriptor or len(data) != 20:
            return
        print(
            "[lidar] info: "
            f"model=0x{data[0]:02x}, firmware={data[2]}.{data[1]}, "
            f"hardware=0x{data[3]:02x}, serial={data[4:].hex()}",
            flush=True,
        )

    def _read_health(self) -> None:
        descriptor, data = self._command_response(CMD_GET_HEALTH, 1.0)
        if not descriptor or len(data) != 3:
            return
        labels = {0: "good", 1: "warning", 2: "error"}
        error_code = data[1] | (data[2] << 8)
        print(
            f"[lidar] health: {labels.get(data[0], str(data[0]))}, error_code={error_code}",
            flush=True,
        )

    def _start_scan(self) -> None:
        self._ser.reset_input_buffer()
        self._send(CMD_SCAN)
        descriptor = self._read_descriptor(1.0)
        if not descriptor:
            raise RuntimeError("scan descriptor timeout")
        if descriptor["size"] != 5 or descriptor["type"] != 0x81:
            raise RuntimeError(
                f"unexpected scan descriptor: size={descriptor['size']} "
                f"type=0x{descriptor['type']:02x}")

    def _publish_rotation(self, points: list[tuple[float, float, int]], scan_time: float) -> None:
        if self.angle_max <= self.angle_min:
            return
        now = time.monotonic()
        if now - self._last_scan_publish_at < 1.0 / self.publish_rate_hz:
            return
        self._last_scan_publish_at = now
        count = self.samples
        span = self.angle_max - self.angle_min
        angle_increment = span / float(count)
        ranges = [math.inf] * count
        intensities = [0.0] * count
        for angle_deg, distance_mm, quality in points:
            if quality < self.min_quality:
                continue
            distance_m = distance_mm / 1000.0
            if distance_m < self.range_min or distance_m > self.range_max:
                continue
            angle = math.radians(angle_deg) + self.angle_offset
            if self.invert:
                angle = -angle
            while angle < self.angle_min:
                angle += math.tau
            while angle >= self.angle_max:
                angle -= math.tau
            index = int((angle - self.angle_min) / angle_increment)
            if index < 0 or index >= count:
                continue
            if not math.isfinite(ranges[index]) or distance_m < ranges[index]:
                ranges[index] = distance_m
                intensities[index] = float(quality)
        self._publish_scan(ranges, intensities, scan_time)

    def _publish_scan(self, ranges: list[float], intensities: list[float], scan_time: float) -> None:
        count = len(ranges)
        angle_increment = (self.angle_max - self.angle_min) / float(count)
        msg = {
            "header": {"stamp": stamp(), "frame_id": self.frame},
            "angle_min": self.angle_min,
            "angle_max": self.angle_min + angle_increment * float(count - 1),
            "angle_increment": angle_increment,
            "time_increment": scan_time / float(count) if scan_time > 0.0 else 0.0,
            "scan_time": scan_time,
            "range_min": self.range_min,
            "range_max": self.range_max,
            "ranges": ranges,
            "intensities": intensities,
        }
        if self.fast_bus is not None:
            self.fast_bus.publish_scan(msg, self.frame)


class CompressedCameraPublisher:
    def __init__(self, stop_event: threading.Event, status: StatusPublisher):
        self.stop_event = stop_event
        self.status = status
        self.device = os.environ.get("CAMERA_DEVICE", "/dev/video0")
        self.image_topic = os.environ.get("CAMERA_TOPIC", "/xlerobot/base_camera/image/compressed")
        self.info_topic = os.environ.get("CAMERA_INFO_TOPIC", "/xlerobot/base_camera/camera_info")
        self.frame = os.environ.get("CAMERA_FRAME", "base_camera_optical_frame")
        self.width = env_int("CAMERA_WIDTH", 320)
        self.height = env_int("CAMERA_HEIGHT", 240)
        self.fps = max(0.2, env_float("CAMERA_RATE_HZ", env_float("CAMERA_FPS", 8.0)))
        self.quality = int(clamp(env_int("CAMERA_JPEG_QUALITY", 60), 10, 95))
        self.hfov_deg = env_float("CAMERA_HFOV_DEG", 70.0)
        self._image_pub = MessageSink(self.image_topic, "sensor_msgs/CompressedImage")
        self._info_pub = MessageSink(self.info_topic, "sensor_msgs/CameraInfo")
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)

    def start(self) -> None:
        self._thread.start()
        print(
            f"[camera] started, topic={self.image_topic}, "
            f"{self.width}x{self.height}@{self.fps}fps jpeg={self.quality}",
            flush=True,
        )
        self.status.update(
            camera="started",
            camera_device=self.device,
            camera_topic=self.image_topic,
            camera_info_topic=self.info_topic,
            camera_rate_hz=self.fps,
            camera_jpeg_quality=self.quality,
        )

    def stop(self) -> None:
        self._image_pub.close()
        self._info_pub.close()

    def _run(self) -> None:
        try:
            import cv2
        except Exception as exc:
            print(f"[camera] cv2 import failed: {exc}", flush=True)
            self.status.update(camera="error", camera_error=str(exc))
            return

        cap = cv2.VideoCapture(self.device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not cap.isOpened():
            msg = f"camera open failed: {self.device}"
            print(f"[camera] {msg}", flush=True)
            self.status.update(camera="error", camera_error=msg)
            return

        delay = 1.0 / self.fps
        params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        while not self.stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(delay)
                continue
            ok, encoded = cv2.imencode(".jpg", frame, params)
            if not ok:
                time.sleep(delay)
                continue
            st = stamp()
            self._image_pub.publish({
                "header": {"stamp": st, "frame_id": self.frame},
                "format": "jpeg",
                "data": encoded.reshape(-1).tolist(),
            })
            self._info_pub.publish(self._camera_info(st))
            time.sleep(delay)
        cap.release()

    def _camera_info(self, st: dict[str, int]) -> dict[str, Any]:
        fx = self.width / (2.0 * math.tan(math.radians(self.hfov_deg) * 0.5))
        fy = fx
        cx = (self.width - 1.0) * 0.5
        cy = (self.height - 1.0) * 0.5
        return {
            "header": {"stamp": st, "frame_id": self.frame},
            "height": self.height,
            "width": self.width,
            "distortion_model": "plumb_bob",
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        }


class RtspH264Publisher:
    def __init__(self, stop_event: threading.Event, status: StatusPublisher):
        self.stop_event = stop_event
        self.status = status
        self.url = os.environ.get("DEPTH_SENSOR_RTSP_URL", "").strip()
        self.fps = max(1.0, env_float(
            "DEPTH_SENSOR_RTSP_FPS", env_float("DEPTH_SENSOR_COLOR_FPS", 15.0)))
        self.bitrate_kbps = max(250, env_int("DEPTH_SENSOR_RTSP_BITRATE_KBPS", 3000))
        self.preset = os.environ.get("DEPTH_SENSOR_RTSP_X264_PRESET", "ultrafast")
        self.profile = os.environ.get("DEPTH_SENSOR_RTSP_H264_PROFILE", "baseline")
        self.transport = os.environ.get("DEPTH_SENSOR_RTSP_TRANSPORT", "tcp")
        self._queue: queue.Queue[np.ndarray] = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._run, name="depth_sensor-rtsp", daemon=True)
        self._process: Optional[subprocess.Popen] = None
        self._started = False
        self._last_frame_at = 0.0
        self._last_error_log_at = 0.0
        self._size: Optional[tuple[int, int]] = None

    @property
    def enabled(self) -> bool:
        return bool(self.url)

    def start(self) -> None:
        if not self.enabled:
            return
        if shutil.which("ffmpeg") is None:
            msg = "ffmpeg not found; depth sensor RTSP/H.264 video disabled"
            print(f"[depth_sensor-rtsp] {msg}", flush=True)
            self.status.update(depth_sensor_rtsp="error", depth_sensor_rtsp_error=msg)
            return
        self._started = True
        self._thread.start()
        self.status.update(
            depth_sensor_rtsp="starting",
            depth_sensor_rtsp_url=self.url,
            depth_sensor_rtsp_fps=self.fps,
            depth_sensor_rtsp_bitrate_kbps=self.bitrate_kbps,
        )
        print(
            f"[depth_sensor-rtsp] publishing H.264 to {self.url} "
            f"@{self.fps:g}fps {self.bitrate_kbps}kbps",
            flush=True,
        )

    def stop(self) -> None:
        if self._process is not None:
            try:
                if self._process.stdin:
                    self._process.stdin.close()
            except Exception:
                pass
            try:
                self._process.terminate()
            except Exception:
                pass

    def submit_bgr(self, image: np.ndarray) -> None:
        if not self._started or not self.enabled:
            return
        now = time.monotonic()
        if now - self._last_frame_at < 1.0 / self.fps:
            return
        self._last_frame_at = now
        frame = np.ascontiguousarray(image).copy()
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(frame)
            except queue.Full:
                pass

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                frame = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue
            h, w = frame.shape[:2]
            size = (w, h)
            if self._process is None or self._process.poll() is not None or self._size != size:
                self._restart_ffmpeg(size)
            if self._process is None or self._process.stdin is None:
                continue
            try:
                self._process.stdin.write(frame.tobytes())
            except (BrokenPipeError, OSError) as exc:
                self._log_error(f"ffmpeg pipe failed: {exc}")
                self._stop_process()
                time.sleep(0.5)

    def _restart_ffmpeg(self, size: tuple[int, int]) -> None:
        self._stop_process()
        width, height = size
        gop = max(1, int(round(self.fps)))
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{width}x{height}",
            "-r",
            f"{self.fps:g}",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            self.preset,
            "-tune",
            "zerolatency",
            "-profile:v",
            self.profile,
            "-pix_fmt",
            "yuv420p",
            "-g",
            str(gop),
            "-bf",
            "0",
            "-b:v",
            f"{self.bitrate_kbps}k",
            "-maxrate",
            f"{self.bitrate_kbps}k",
            "-bufsize",
            f"{max(self.bitrate_kbps // 2, 250)}k",
            "-f",
            "rtsp",
            "-rtsp_transport",
            self.transport,
            self.url,
        ]
        try:
            self._process = subprocess.Popen(cmd, stdin=subprocess.PIPE)
            self._size = size
            self.status.update(depth_sensor_rtsp="publishing", depth_sensor_rtsp_size=f"{width}x{height}")
            print(f"[depth_sensor-rtsp] ffmpeg started for {width}x{height}", flush=True)
        except Exception as exc:
            self._process = None
            self._size = None
            self._log_error(f"ffmpeg start failed: {exc}")

    def _stop_process(self) -> None:
        proc = self._process
        self._process = None
        self._size = None
        if proc is None:
            return
        try:
            if proc.stdin:
                proc.stdin.close()
        except Exception:
            pass
        try:
            proc.terminate()
        except Exception:
            pass

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_at > 2.0:
            print(f"[depth_sensor-rtsp] {msg}", flush=True)
            self.status.update(depth_sensor_rtsp="error", depth_sensor_rtsp_error=msg)
            self._last_error_log_at = now


class UsbCameraRtspProcess:
    def __init__(self, name: str, stop_event: threading.Event, status: StatusPublisher):
        self.name = name
        self.stop_event = stop_event
        self.status = status
        prefix = name.upper()
        self.enabled = env_bool(
            f"{prefix}_CAMERA_RTSP_ENABLE",
            env_bool("USB_CAMERA_RTSP_ENABLE", env_bool("ENABLE_USB_CAMERA_RTSP", False)),
        )
        self.device = os.environ.get(f"{prefix}_CAMERA_DEVICE", "").strip()
        self.url = os.environ.get(f"{prefix}_CAMERA_RTSP_URL", "").strip()
        self.width = env_int(f"{prefix}_CAMERA_WIDTH", env_int("USB_CAMERA_WIDTH", 640))
        self.height = env_int(f"{prefix}_CAMERA_HEIGHT", env_int("USB_CAMERA_HEIGHT", 480))
        self.fps = max(1.0, env_float(f"{prefix}_CAMERA_FPS", env_float("USB_CAMERA_FPS", 10.0)))
        self.input_format = os.environ.get(
            f"{prefix}_CAMERA_INPUT_FORMAT",
            os.environ.get("USB_CAMERA_INPUT_FORMAT", "mjpeg"),
        ).strip()
        self.bitrate_kbps = max(
            250,
            env_int(f"{prefix}_CAMERA_RTSP_BITRATE_KBPS", env_int("USB_CAMERA_RTSP_BITRATE_KBPS", 1500)),
        )
        self.transport = os.environ.get(f"{prefix}_CAMERA_RTSP_TRANSPORT", "tcp")
        self.preset = os.environ.get(f"{prefix}_CAMERA_RTSP_X264_PRESET", "ultrafast")
        self.profile = os.environ.get(f"{prefix}_CAMERA_RTSP_H264_PROFILE", "baseline")
        self.rotate_deg = env_int(f"{prefix}_CAMERA_ROTATE_DEG", env_int("USB_CAMERA_ROTATE_DEG", 0)) % 360
        self.video_filter = os.environ.get(f"{prefix}_CAMERA_FFMPEG_FILTER", "").strip()
        if not self.video_filter and self.rotate_deg == 180:
            self.video_filter = "hflip,vflip"
        self._thread = threading.Thread(target=self._run, name=f"{name}-camera-rtsp", daemon=True)
        self._process: Optional[subprocess.Popen] = None
        self._last_error_log_at = 0.0

    def start(self) -> None:
        if not self.enabled:
            return
        if not self.device or not self.url:
            self._log_error("disabled; missing device or RTSP URL")
            return
        if shutil.which("ffmpeg") is None:
            self._log_error("ffmpeg not found")
            return
        self._thread.start()
        self.status.update(**{
            f"{self.name}_camera_rtsp": "starting",
            f"{self.name}_camera_device": self.device,
            f"{self.name}_camera_rtsp_url": self.url,
        })
        print(
            f"[{self.name}_camera-rtsp] publishing {self.device} to {self.url} "
            f"@{self.width}x{self.height}/{self.fps:g}fps {self.bitrate_kbps}kbps "
            f"input={self.input_format or 'default'} rotate={self.rotate_deg}",
            flush=True,
        )

    def stop(self) -> None:
        self._stop_process()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            if not os.path.exists(self.device):
                self._log_error(f"device not found: {self.device}")
                time.sleep(1.0)
                continue
            self._start_ffmpeg()
            proc = self._process
            if proc is None:
                time.sleep(1.0)
                continue
            while not self.stop_event.is_set() and proc.poll() is None:
                time.sleep(0.5)
            if not self.stop_event.is_set():
                self._log_error("ffmpeg exited; restarting")
                self._stop_process()
                time.sleep(1.0)

    def _start_ffmpeg(self) -> None:
        self._stop_process()
        gop = max(1, int(round(self.fps)))
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-f",
            "v4l2",
        ]
        if self.input_format:
            cmd.extend(["-input_format", self.input_format])
        cmd.extend([
            "-framerate",
            f"{self.fps:g}",
            "-video_size",
            f"{self.width}x{self.height}",
            "-i",
            self.device,
            "-an",
        ])
        if self.video_filter:
            cmd.extend(["-vf", self.video_filter])
        cmd.extend([
            "-c:v",
            "libx264",
            "-preset",
            self.preset,
            "-tune",
            "zerolatency",
            "-profile:v",
            self.profile,
            "-pix_fmt",
            "yuv420p",
            "-g",
            str(gop),
            "-bf",
            "0",
            "-b:v",
            f"{self.bitrate_kbps}k",
            "-maxrate",
            f"{self.bitrate_kbps}k",
            "-bufsize",
            f"{max(self.bitrate_kbps // 2, 250)}k",
            "-f",
            "rtsp",
            "-rtsp_transport",
            self.transport,
            self.url,
        ])
        try:
            self._process = subprocess.Popen(cmd, stdin=subprocess.DEVNULL)
            self.status.update(**{f"{self.name}_camera_rtsp": "publishing"})
        except Exception as exc:
            self._process = None
            self._log_error(f"ffmpeg start failed: {exc}")

    def _stop_process(self) -> None:
        proc = self._process
        self._process = None
        if proc is None:
            return
        try:
            proc.terminate()
        except Exception:
            pass

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_at > 2.0:
            print(f"[{self.name}_camera-rtsp] {msg}", flush=True)
            self.status.update(**{
                f"{self.name}_camera_rtsp": "error",
                f"{self.name}_camera_rtsp_error": msg,
            })
            self._last_error_log_at = now


class BinaryRgbdTcpPublisher:
    """Length-framed RGB-D side channel for ROS 2 consumers on the compute PC."""

    def __init__(self, stop_event: threading.Event, status: StatusPublisher):
        self.stop_event = stop_event
        self.status = status
        self.host = env_first(("DEPTH_SENSOR_BINARY_HOST", "COMPUTE_PC_HOST"), "").strip()
        self.port = env_int("DEPTH_SENSOR_BINARY_PORT", 9102)
        self.max_fps = max(0.0, env_float("DEPTH_SENSOR_BINARY_FPS", 0.0))
        self._queue: queue.Queue[tuple[dict[str, Any], bytes]] = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._run, name="depth_sensor-binary-rgbd", daemon=True)
        self._sock: Optional[socket.socket] = None
        self._started = False
        self._last_submit_at = 0.0
        self._last_error_log_at = 0.0
        self._frames_sent = 0
        self._bytes_sent = 0

    @property
    def enabled(self) -> bool:
        return bool(self.host) and self.port > 0

    def start(self) -> None:
        if not self.enabled:
            print("[depth_sensor-binary] disabled; missing DEPTH_SENSOR_BINARY_HOST", flush=True)
            self.status.update(depth_sensor_binary="disabled")
            return
        self._started = True
        self._thread.start()
        self.status.update(
            depth_sensor_binary="starting",
            depth_sensor_binary_target=f"{self.host}:{self.port}",
            depth_sensor_binary_max_fps=self.max_fps,
        )
        print(
            f"[depth_sensor-binary] publishing RGB-D to tcp://{self.host}:{self.port} "
            f"(max_fps={self.max_fps:g}, 0 means camera rate)",
            flush=True,
        )

    def stop(self) -> None:
        self._close_socket()

    def submit(self, header: dict[str, Any], payload: bytes) -> None:
        if not self._started or not self.enabled:
            return
        now = time.monotonic()
        if self.max_fps > 0.0 and now - self._last_submit_at < 1.0 / self.max_fps:
            return
        self._last_submit_at = now
        item = (header, payload)
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                pass

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                header, payload = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if self._sock is None and not self._connect():
                continue
            try:
                header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
                packet = struct.pack("!I", len(header_bytes)) + header_bytes + payload
                assert self._sock is not None
                self._sock.sendall(packet)
                self._frames_sent += 1
                self._bytes_sent += len(packet)
                if self._frames_sent % 150 == 0:
                    self.status.update(
                        depth_sensor_binary="publishing",
                        depth_sensor_binary_frames=self._frames_sent,
                        depth_sensor_binary_mb=round(self._bytes_sent / 1_000_000.0, 1),
                    )
            except Exception as exc:
                self._log_error(f"send failed: {exc}")
                self._close_socket()

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=2.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(None)
            self._sock = sock
            self.status.update(depth_sensor_binary="connected")
            print(f"[depth_sensor-binary] connected to tcp://{self.host}:{self.port}", flush=True)
            return True
        except Exception as exc:
            self._log_error(f"connect failed: {exc}")
            self._close_socket()
            time.sleep(0.5)
            return False

    def _close_socket(self) -> None:
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    def _log_error(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_at > 2.0:
            print(f"[depth_sensor-binary] {msg}", flush=True)
            self.status.update(depth_sensor_binary="error", depth_sensor_binary_error=msg)
            self._last_error_log_at = now


class DepthSensorPublisher:
    def __init__(self, stop_event: threading.Event, status: StatusPublisher, dry_run: bool):
        self.stop_event = stop_event
        self.status = status
        self.dry_run = dry_run
        self.serial = os.environ.get("DEPTH_SENSOR_SERIAL", "").strip()
        self.enable_color = env_bool("DEPTH_SENSOR_ENABLE_COLOR", True)
        self.enable_imu = env_bool("DEPTH_SENSOR_ENABLE_IMU", True)
        self.align_depth_to_color = env_bool("DEPTH_SENSOR_ALIGN_DEPTH_TO_COLOR", True)
        self.depth_width = env_int("DEPTH_SENSOR_DEPTH_WIDTH", 640)
        self.depth_height = env_int("DEPTH_SENSOR_DEPTH_HEIGHT", 360)
        self.depth_fps = env_int("DEPTH_SENSOR_DEPTH_FPS", 15)
        self.color_width = env_int("DEPTH_SENSOR_COLOR_WIDTH", 640)
        self.color_height = env_int("DEPTH_SENSOR_COLOR_HEIGHT", 360)
        self.color_fps = env_int("DEPTH_SENSOR_COLOR_FPS", 15)
        self.require_usb3 = env_bool("DEPTH_SENSOR_REQUIRE_USB3", False)
        self.depth_publish_hz = max(
            0.0,
            env_float("DEPTH_SENSOR_DEPTH_PUBLISH_HZ", self.depth_fps),
        )
        self.color_publish_hz = max(
            0.0,
            env_float("DEPTH_SENSOR_COLOR_PUBLISH_HZ", self.color_fps),
        )
        self.imu_rate_hz = max(1.0, env_float("DEPTH_SENSOR_IMU_PUBLISH_HZ", 100.0))
        self.png_compress_level = int(clamp(env_int("DEPTH_SENSOR_PNG_COMPRESS_LEVEL", 1), 0, 9))
        self.jpeg_quality = int(clamp(env_int("DEPTH_SENSOR_JPEG_QUALITY", 60), 10, 95))
        self.binary_enabled = env_bool("DEPTH_SENSOR_BINARY_ENABLE", False)
        self.ros_image_enable = env_bool(
            "DEPTH_SENSOR_ROSBRIDGE_IMAGE_ENABLE", not self.binary_enabled)
        self.binary_depth_format = os.environ.get(
            "DEPTH_SENSOR_BINARY_DEPTH_FORMAT", "raw16").strip().lower()
        if self.binary_depth_format not in ("raw16", "png16"):
            self.binary_depth_format = "raw16"
        self.binary_color_mode = os.environ.get(
            "DEPTH_SENSOR_BINARY_COLOR_MODE", "bgr8").strip().lower()
        if self.binary_color_mode in ("gray", "grey", "grayscale", "mono"):
            self.binary_color_mode = "mono8"
        if self.binary_color_mode not in ("mono8", "bgr8"):
            self.binary_color_mode = "mono8"
        self.depth_topic = os.environ.get("DEPTH_SENSOR_DEPTH_TOPIC", "/xlerobot/head_camera/depth/image")
        self.depth_info_topic = os.environ.get(
            "DEPTH_SENSOR_DEPTH_CAMERA_INFO_TOPIC",
            "/xlerobot/head_camera/depth/camera_info",
        )
        self.color_topic = os.environ.get("DEPTH_SENSOR_COLOR_TOPIC", "/xlerobot/head_camera/color/image")
        self.color_info_topic = os.environ.get(
            "DEPTH_SENSOR_COLOR_CAMERA_INFO_TOPIC",
            "/xlerobot/head_camera/color/camera_info",
        )
        self.imu_topic = os.environ.get("DEPTH_SENSOR_IMU_TOPIC", "/xlerobot/head_camera/imu")
        self.depth_frame = os.environ.get(
            "DEPTH_SENSOR_DEPTH_FRAME", "head_camera_depth_optical_frame")
        self.color_frame = os.environ.get(
            "DEPTH_SENSOR_COLOR_FRAME", "head_camera_rgb_optical_frame")
        self.imu_frame = os.environ.get("DEPTH_SENSOR_IMU_FRAME", "head_camera_imu_frame")
        self._align = None
        self._depth_publish_frame = self.depth_frame

        self._depth_pub = None
        self._depth_info_pub = None
        self._color_pub = None
        self._color_info_pub = None
        if self.ros_image_enable:
            self._depth_pub = MessageSink(self.depth_topic, "sensor_msgs/CompressedImage")
            self._depth_info_pub = MessageSink(
                self.depth_info_topic, "sensor_msgs/CameraInfo")
        if self.ros_image_enable and self.enable_color:
            self._color_pub = MessageSink(self.color_topic, "sensor_msgs/CompressedImage")
            self._color_info_pub = MessageSink(self.color_info_topic, "sensor_msgs/CameraInfo")
        self._imu_pub = None
        if self.enable_imu:
            self._imu_pub = MessageSink(self.imu_topic, "sensor_msgs/Imu")
        self._pipeline = None
        self._thread = threading.Thread(target=self._run, name="depth_sensor", daemon=True)
        self._latest_accel: Optional[tuple[float, float, float]] = None
        self._latest_gyro: Optional[tuple[float, float, float]] = None
        self._last_imu_publish_at = 0.0
        self._last_depth_publish_at = 0.0
        self._last_color_publish_at = 0.0
        self._last_frame_error_log_at = 0.0
        self._last_rs_frame_at = 0.0
        self._color_active = False
        self._imu_active = False
        self._rtsp = None
        rtsp_enabled = env_bool("DEPTH_SENSOR_RTSP_ENABLE", False)
        if rtsp_enabled:
            self._rtsp = RtspH264Publisher(stop_event, status)
        self._binary = None
        if self.binary_enabled:
            self._binary = BinaryRgbdTcpPublisher(stop_event, status)

    def start(self) -> None:
        self._thread.start()
        print(
            f"[depth_sensor] started, depth={self.depth_topic}, imu={self.imu_topic}, "
            f"color={self.color_topic if self.enable_color else 'off'}, "
            f"ros_images={self.ros_image_enable}, "
            f"binary={self._binary.enabled if self._binary is not None else False}",
            flush=True,
        )
        self.status.update(
            depth_sensor="started",
            depth_sensor_depth_topic=self.depth_topic,
            depth_sensor_depth_info_topic=self.depth_info_topic,
            depth_sensor_imu_topic=self.imu_topic if self.enable_imu else "",
            depth_sensor_color_topic=self.color_topic if self.enable_color else "",
            depth_sensor_depth_aligned_to_color=self.align_depth_to_color,
            depth_sensor_rtsp="enabled" if self._rtsp is not None else "disabled",
            depth_sensor_ros_images=self.ros_image_enable,
            depth_sensor_binary="enabled" if self._binary is not None else "disabled",
            depth_sensor_binary_color_mode=self.binary_color_mode,
        )
        if self._rtsp is not None:
            self._rtsp.start()
        if self._binary is not None:
            self._binary.start()

    def stop(self) -> None:
        if self._rtsp is not None:
            self._rtsp.stop()
        if self._binary is not None:
            self._binary.stop()
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        for pub in (
            self._depth_pub,
            self._depth_info_pub,
            self._color_pub,
            self._color_info_pub,
            self._imu_pub,
        ):
            if pub is not None:
                pub.close()

    def _run(self) -> None:
        if self.dry_run:
            self._run_fake()
            return
        if rs is None:
            msg = f"pyrealsense2 import failed: {DEPTH_SENSOR_IMPORT_ERROR}"
            print(f"[depth_sensor] {msg}", flush=True)
            self.status.update(depth_sensor="error", depth_sensor_error=msg)
            return

        last_error = ""
        reconnect_delay_s = max(0.5, env_float("DEPTH_SENSOR_RECONNECT_DELAY_S", 2.0))
        while not self.stop_event.is_set():
            try:
                self._start_pipeline()
                last_error = ""
                self.status.update(
                    depth_sensor="connected",
                    depth_sensor_color=self._color_active,
                    depth_sensor_imu=self._imu_active,
                    depth_sensor_depth_aligned_to_color=self._align is not None,
                )
                while not self.stop_event.is_set():
                    time.sleep(0.2)
                    if self._last_rs_frame_at and time.monotonic() - self._last_rs_frame_at > 5.0:
                        now = time.monotonic()
                        if now - self._last_frame_error_log_at > 2.0:
                            print("[depth_sensor] no frames for 5s; restarting pipeline", flush=True)
                            self.status.update(depth_sensor_frame_error="no frames for 5s")
                            self._last_frame_error_log_at = now
                        raise RuntimeError("depth sensor produced no frames for 5s")
            except Exception as exc:
                msg = str(exc)
                if msg != last_error:
                    print(f"[depth_sensor] error: {msg}", flush=True)
                    self.status.update(depth_sensor="error", depth_sensor_error=msg)
                    last_error = msg
            finally:
                if self._pipeline is not None:
                    try:
                        self._pipeline.stop()
                    except Exception:
                        pass
                    self._pipeline = None
            if not self.stop_event.is_set():
                time.sleep(reconnect_delay_s)

    def _start_pipeline(self) -> None:
        attempts = [
            (self.enable_color, self.enable_imu),
            (self.enable_color, False),
            (False, self.enable_imu),
            (False, False),
        ]
        errors = []
        for color_enabled, imu_enabled in attempts:
            pipeline = rs.pipeline()
            config = rs.config()
            if self.serial:
                config.enable_device(self.serial)
            config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.depth_fps,
            )
            if color_enabled:
                config.enable_stream(
                    rs.stream.color,
                    self.color_width,
                    self.color_height,
                    rs.format.bgr8,
                    self.color_fps,
                )
            if imu_enabled:
                config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
                config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
            try:
                self._align = (
                    rs.align(rs.stream.color)
                    if color_enabled and self.align_depth_to_color
                    else None
                )
                self._depth_publish_frame = (
                    self.color_frame if self._align is not None else self.depth_frame
                )
                profile = pipeline.start(config, self._on_rs_frame)
            except Exception as exc:
                errors.append(f"color={color_enabled} imu={imu_enabled}: {exc}")
                try:
                    pipeline.stop()
                except Exception:
                    pass
                self._align = None
                self._depth_publish_frame = self.depth_frame
                continue
            usb_type = self._usb_type_from_profile(profile)
            if self.require_usb3 and usb_type and not usb_type.startswith("3"):
                try:
                    pipeline.stop()
                except Exception:
                    pass
                errors.append(
                    f"color={color_enabled} imu={imu_enabled}: "
                    f"USB3 required, but sensor negotiated USB {usb_type}"
                )
                self._align = None
                self._depth_publish_frame = self.depth_frame
                continue
            self._pipeline = pipeline
            self._last_rs_frame_at = time.monotonic()
            self._color_active = color_enabled
            self._imu_active = imu_enabled
            self.status.update(depth_sensor_usb_type=usb_type or "unknown")
            print(
                f"[depth_sensor] pipeline connected "
                f"(color={color_enabled}, imu={imu_enabled}, "
                f"align_depth_to_color={self._align is not None}, "
                f"usb={usb_type or 'unknown'}, require_usb3={self.require_usb3})",
                flush=True,
            )
            if self.enable_color and not color_enabled:
                print("[depth_sensor] color stream unavailable; continuing with depth/IMU", flush=True)
            if self.enable_imu and not imu_enabled:
                print("[depth_sensor] IMU stream unavailable; continuing with depth only", flush=True)
            return
        raise RuntimeError("could not start depth sensor pipeline; " + " | ".join(errors))

    def _usb_type_from_profile(self, profile) -> str:
        try:
            device = profile.get_device()
            return str(device.get_info(rs.camera_info.usb_type_descriptor))
        except Exception:
            return ""

    def _on_rs_frame(self, frame) -> None:
        try:
            self._last_rs_frame_at = time.monotonic()
            if frame.is_frameset():
                frameset = frame.as_frameset()
                if self._align is not None:
                    frameset = self._align.process(frameset)
                depth_frame = frameset.get_depth_frame()
                color_frame = frameset.get_color_frame()
                if color_frame and self._rtsp is not None:
                    self._submit_color_rtsp(color_frame)
                if depth_frame and color_frame and self._binary is not None:
                    self._publish_binary_rgbd(depth_frame, color_frame)
                if depth_frame and self.ros_image_enable:
                    self._publish_depth(depth_frame)
                if color_frame and self._color_pub is not None:
                    self._publish_color(color_frame)
            elif frame.is_motion_frame():
                self._publish_motion(frame.as_motion_frame())
        except Exception as exc:
            print(f"[depth_sensor] frame handling failed: {exc}", flush=True)

    def _submit_color_rtsp(self, frame) -> None:
        if self._rtsp is None:
            return
        image = np.asanyarray(frame.get_data())
        self._rtsp.submit_bgr(image)

    def _publish_depth(self, frame) -> None:
        if self._depth_pub is None or self._depth_info_pub is None:
            return
        if not self._should_publish("depth"):
            return
        try:
            import cv2
        except Exception as exc:
            print(f"[depth_sensor] cv2 import failed for depth PNG: {exc}", flush=True)
            return
        depth = np.asanyarray(frame.get_data())
        ok, encoded = cv2.imencode(
            ".png",
            depth,
            [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_compress_level],
        )
        if not ok:
            return
        st = stamp()
        self._depth_pub.publish({
            "header": {"stamp": st, "frame_id": self._depth_publish_frame},
            "format": "png; 16UC1",
            "data": encoded.reshape(-1).tolist(),
        })
        self._depth_info_pub.publish(
            self._camera_info_from_video_frame(frame, st, self._depth_publish_frame))

    def _publish_color(self, frame) -> None:
        if self._color_pub is None or self._color_info_pub is None:
            return
        if not self._should_publish("color"):
            return
        try:
            import cv2
        except Exception as exc:
            print(f"[depth_sensor] cv2 import failed for color JPEG: {exc}", flush=True)
            return
        image = np.asanyarray(frame.get_data())
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return
        st = stamp()
        self._color_pub.publish({
            "header": {"stamp": st, "frame_id": self.color_frame},
            "format": "jpeg",
            "data": encoded.reshape(-1).tolist(),
        })
        self._color_info_pub.publish(
            self._camera_info_from_video_frame(frame, st, self.color_frame))

    def _publish_binary_rgbd(self, depth_frame, color_frame) -> None:
        if self._binary is None:
            return
        try:
            import cv2
        except Exception as exc:
            print(f"[depth_sensor] cv2 import failed for binary RGB-D: {exc}", flush=True)
            return

        st = stamp()
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        if self.binary_color_mode == "mono8":
            color_payload = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            color_encoding = "mono8"
        else:
            color_payload = color
            color_encoding = "bgr8"
        ok, color_encoded = cv2.imencode(
            ".jpg",
            color_payload,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return

        depth = np.ascontiguousarray(depth.astype(np.uint16, copy=False))
        if self.binary_depth_format == "png16":
            ok, depth_encoded = cv2.imencode(
                ".png",
                depth,
                [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_compress_level],
            )
            if not ok:
                return
            depth_bytes = depth_encoded.tobytes()
            depth_format = "png;16UC1"
        else:
            depth_bytes = depth.tobytes(order="C")
            depth_format = "raw16uc1-le"

        color_bytes = color_encoded.tobytes()
        depth_units = 0.001
        try:
            depth_units = float(depth_frame.get_units())
        except Exception:
            pass
        aligned_depth_to_color = bool(self._align is not None)
        depth_publish_frame = self.color_frame if aligned_depth_to_color else self._depth_publish_frame
        depth_info_frame = color_frame if aligned_depth_to_color else depth_frame
        header = {
            "type": "rgbd",
            "stamp": st,
            "color_format": "jpeg",
            "color_len": len(color_bytes),
            "color_encoding": color_encoding,
            "color_frame_id": self.color_frame,
            "depth_format": depth_format,
            "depth_len": len(depth_bytes),
            "depth_encoding": "16UC1",
            "aligned_depth_to_color": aligned_depth_to_color,
            "depth_frame_id": depth_publish_frame,
            "depth_width": int(depth.shape[1]),
            "depth_height": int(depth.shape[0]),
            "depth_step": int(depth.shape[1] * 2),
            "depth_units": depth_units,
            "color_camera_info": self._camera_info_from_video_frame(
                color_frame, st, self.color_frame),
            "depth_camera_info": self._camera_info_from_video_frame(
                depth_info_frame, st, depth_publish_frame),
        }
        self._binary.submit(header, color_bytes + depth_bytes)

    def _should_publish(self, stream: str) -> bool:
        if stream == "depth":
            hz = self.depth_publish_hz
            if hz <= 0:
                return False
            last = self._last_depth_publish_at
        else:
            hz = self.color_publish_hz
            if hz <= 0:
                return False
            last = self._last_color_publish_at

        now = time.monotonic()
        if last and now - last < 1.0 / hz:
            return False
        if stream == "depth":
            self._last_depth_publish_at = now
        else:
            self._last_color_publish_at = now
        return True

    def _publish_motion(self, frame) -> None:
        if self._imu_pub is None:
            return
        data = frame.get_motion_data()
        values = (float(data.x), float(data.y), float(data.z))
        stream_type = frame.get_profile().stream_type()
        if stream_type == rs.stream.accel:
            self._latest_accel = values
        elif stream_type == rs.stream.gyro:
            self._latest_gyro = values
        now = time.monotonic()
        if now - self._last_imu_publish_at < 1.0 / self.imu_rate_hz:
            return
        self._last_imu_publish_at = now
        accel = self._latest_accel or (0.0, 0.0, 0.0)
        gyro = self._latest_gyro or (0.0, 0.0, 0.0)
        self._imu_pub.publish({
            "header": {"stamp": stamp(), "frame_id": self.imu_frame},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "orientation_covariance": [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "angular_velocity": {"x": gyro[0], "y": gyro[1], "z": gyro[2]},
            "angular_velocity_covariance": [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ],
            "linear_acceleration": {"x": accel[0], "y": accel[1], "z": accel[2]},
            "linear_acceleration_covariance": [
                0.10, 0.0, 0.0,
                0.0, 0.10, 0.0,
                0.0, 0.0, 0.10,
            ],
        })

    def _camera_info_from_video_frame(self, frame, st: dict[str, int], frame_id: str) -> dict[str, Any]:
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

    def _run_fake(self) -> None:
        try:
            import cv2
        except Exception:
            cv2 = None
        delay = 1.0 / max(0.2, float(self.depth_fps))
        while not self.stop_event.is_set():
            st = stamp()
            depth = np.full((self.depth_height, self.depth_width), 1500, dtype=np.uint16)
            if cv2 is not None and self._depth_pub is not None:
                ok, encoded = cv2.imencode(".png", depth)
                if ok:
                    self._depth_pub.publish({
                        "header": {"stamp": st, "frame_id": self.depth_frame},
                        "format": "png; 16UC1",
                        "data": encoded.reshape(-1).tolist(),
                    })
            if self._depth_info_pub is not None:
                self._depth_info_pub.publish(self._fake_camera_info(st, self.depth_frame))
            if self._imu_pub is not None:
                self._imu_pub.publish({
                    "header": {"stamp": st, "frame_id": self.imu_frame},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "orientation_covariance": [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular_velocity_covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
                    "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.80665},
                    "linear_acceleration_covariance": [0.10, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.10],
                })
            time.sleep(delay)

    def _fake_camera_info(self, st: dict[str, int], frame_id: str) -> dict[str, Any]:
        fx = self.depth_width / (2.0 * math.tan(math.radians(86.0) * 0.5))
        fy = fx
        cx = (self.depth_width - 1.0) * 0.5
        cy = (self.depth_height - 1.0) * 0.5
        return {
            "header": {"stamp": st, "frame_id": frame_id},
            "height": self.depth_height,
            "width": self.depth_width,
            "distortion_model": "plumb_bob",
            "d": [0.0, 0.0, 0.0, 0.0, 0.0],
            "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        }


class DepthSensorProcess:
    """Run RealSense capture out-of-process so USB stalls cannot freeze motor I/O."""

    def __init__(
        self,
        stop_event: threading.Event,
        status: StatusPublisher,
        dry_run: bool,
    ):
        self.stop_event = stop_event
        self.status = status
        self.dry_run = dry_run
        self._process: Optional[subprocess.Popen] = None

    def start(self) -> None:
        env = os.environ.copy()
        env.update({
            "ENABLE_BASE": "0",
            "ENABLE_LIDAR": "0",
            "ENABLE_CAMERA": "0",
            "ENABLE_USB_CAMERA_RTSP": "0",
            "ENABLE_DEPTH_SENSOR": "1",
            "DEPTH_SENSOR_SEPARATE_PROCESS": "0",
            "DEPTH_SENSOR_CHILD": "1",
            "ENABLE_FAST_ZMQ": "0",
        })
        cmd = [
            sys.executable,
            os.path.abspath(__file__),
        ]
        if self.dry_run:
            cmd.append("--dry-run")
        self._process = subprocess.Popen(cmd, env=env)
        self.status.update(
            depth_sensor_process="started",
            depth_sensor_process_pid=self._process.pid,
        )
        print(f"[depth_sensor] child process pid={self._process.pid}", flush=True)

    def stop(self) -> None:
        proc = self._process
        self._process = None
        if proc is None:
            return
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            proc.wait(timeout=5)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="XLeRobot hardware I/O over direct fast ZMQ")
    parser.add_argument("--rosbridge-uri", default="", help=argparse.SUPPRESS)
    parser.add_argument("--host", default="", help=argparse.SUPPRESS)
    parser.add_argument("--port", type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true", default=env_bool("DRY_RUN", False))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stop_event = threading.Event()

    def handle_signal(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    enable_base = env_bool("ENABLE_BASE", True)
    enable_lidar = env_bool("ENABLE_LIDAR", True)
    enable_camera = env_bool("ENABLE_CAMERA", False)
    enable_usb_camera_rtsp = env_bool("ENABLE_USB_CAMERA_RTSP", False)
    enable_depth_sensor = env_bool("ENABLE_DEPTH_SENSOR", True)
    enable_fast_zmq = env_bool("ENABLE_FAST_ZMQ", True)
    separate_depth_sensor = (
        enable_depth_sensor
        and env_bool("DEPTH_SENSOR_SEPARATE_PROCESS", True)
        and not env_bool("DEPTH_SENSOR_CHILD", False)
    )
    status_topic = os.environ.get("STATUS_TOPIC", "/xlerobot/io_status")

    print("============================================================", flush=True)
    print("XLeRobot fast ZMQ hardware I/O", flush=True)
    print("============================================================", flush=True)
    print(f"dry run   : {args.dry_run}", flush=True)
    print(f"base      : {enable_base}", flush=True)
    print(f"lidar     : {enable_lidar}", flush=True)
    print(f"depth     : {enable_depth_sensor}", flush=True)
    print(f"camera    : {enable_camera}", flush=True)
    print(f"usb video : {enable_usb_camera_rtsp}", flush=True)
    print(f"fast zmq  : {enable_fast_zmq}", flush=True)
    print("============================================================", flush=True)

    fast_bus = FastRobotZmqServer(stop_event) if enable_fast_zmq else None
    components: list[Any] = []
    status = StatusPublisher(status_topic)
    try:
        if fast_bus is not None:
            fast_bus.start()
        status.update(state="started")

        if enable_base:
            base = XLeRobotHardware(stop_event, status, args.dry_run, fast_bus)
            if fast_bus is not None:
                fast_bus.attach_base(base)
            base.start()
            components.append(base)
        if enable_lidar:
            lidar = RPLidarPublisher(stop_event, status, args.dry_run, fast_bus)
            lidar.start()
            components.append(lidar)
        if enable_depth_sensor:
            if separate_depth_sensor:
                depth_sensor = DepthSensorProcess(stop_event, status, args.dry_run)
            else:
                depth_sensor = DepthSensorPublisher(stop_event, status, args.dry_run)
            depth_sensor.start()
            components.append(depth_sensor)
        if enable_camera:
            camera = CompressedCameraPublisher(stop_event, status)
            camera.start()
            components.append(camera)
        if enable_usb_camera_rtsp:
            names = [
                item.strip()
                for item in os.environ.get(
                    "USB_CAMERA_RTSP_CAMERAS",
                    "base,wrist_left,wrist_right",
                ).split(",")
                if item.strip()
            ]
            for name in names:
                usb_camera = UsbCameraRtspProcess(name, stop_event, status)
                usb_camera.start()
                components.append(usb_camera)

        while not stop_event.is_set():
            time.sleep(0.5)
    finally:
        if fast_bus is not None:
            fast_bus.attach_base(None)
        for component in reversed(components):
            try:
                component.stop()
            except Exception:
                pass
        status.update(state="stopped")
        status.close()
        if fast_bus is not None:
            fast_bus.stop()

    print("[exit] stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
