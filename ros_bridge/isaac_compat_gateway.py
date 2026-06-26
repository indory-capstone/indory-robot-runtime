#!/usr/bin/env python3
"""Pi-side compatibility gateway for indory_isaac_sim ZMQ clients.

This process keeps the existing Pi rosbridge I/O path intact:

  Pi hardware -> rosbridge /xlerobot/scan,/xlerobot/odom
  rosbridge /xlerobot/cmd_vel -> Pi hardware

It adds an indory_isaac_sim-shaped ZMQ endpoint next to it:

  PUB  scan.<robot_id>, proprio.<robot_id>   -> ZMQ clients
  PULL base_cmd_vel commands                 <- ZMQ clients
  REP  fleet_info/topic_list/health          <-> ZMQ clients

The defaults follow indory_isaac_sim's homeserver_udp profile (8855/8856/8857)
and stay separate from ROS/rosbridge ports.
"""

from __future__ import annotations

import argparse
import array
import logging
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import msgpack
import zmq

import xml.etree.ElementTree as ET
try:
    import ikpy.chain
    import numpy as np
    from pyquaternion import Quaternion
    HAS_IK = True
except ImportError:
    HAS_IK = False

def build_ik_chain(urdf_path, active_links):
    if not HAS_IK: return None
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        links_to_keep = set(active_links)
        for joint in root.findall('joint'):
            child = joint.find('child').get('link')
            if child not in links_to_keep:
                root.remove(joint)
        for link in root.findall('link'):
            name = link.get('name')
            if name not in links_to_keep:
                root.remove(link)
        temp_urdf = f'/tmp/temp_ik_{active_links[-1]}.urdf'
        tree.write(temp_urdf)
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            return ikpy.chain.Chain.from_urdf_file(temp_urdf)
    except Exception as e:
        logging.error(f"Failed to build IK chain: {e}")
        return None

def quaternion_to_matrix(q):
    # q is [x, y, z, w]
    quat = Quaternion(q[3], q[0], q[1], q[2])
    return quat.rotation_matrix

# Global IK state
ik_chains = {}
def init_ik():
    if not HAS_IK: return
    urdf = os.environ.get("XLEROBOT_URDF", str(ROOT_DIR / "robot" / "xlerobot.urdf"))
    ik_chains["left"] = build_ik_chain(urdf, ['base_link', 'base_turn', 'base_tilt', 'hand_l_pan', 'hand_l_lift', 'hand_l_roll', 'hand_l_grip'])
    ik_chains["right"] = build_ik_chain(urdf, ['base_link', 'base_turn', 'base_tilt', 'hand_r_pan', 'hand_r_lift', 'hand_r_roll', 'hand_r_grip'])


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

try:
    from .ws_client import SimpleWebSocket, WebSocketClosed
except ImportError:  # pragma: no cover - direct script execution
    from ws_client import SimpleWebSocket, WebSocketClosed

SCHEMA_VERSION = "xlerobot_v1"
SCHEMA_VERSION_V11 = "xlerobot_v1.1"
SUPPORTED_SCHEMAS = (SCHEMA_VERSION, SCHEMA_VERSION_V11)
MAX_NUM_ROBOTS = 16
COMMAND_FRAMES = ("body", "world")
COMMAND_SOURCE_ROLES = ("teleop", "policy", "safety", "script")
COMMAND_SOURCE_ID_MAX_LEN = 128
COMMAND_PRIORITY_RANGE = (0, 100)
COMMAND_LEASE_MS_RANGE = (1, 60_000)
LEGACY_SOURCE_ID = "<legacy>"
DEFAULT_COMMAND_SOURCE_LEASE_MS = 500
SAFETY_SOURCE_PRIORITY = 100
JOINT_POS_ORDER = [
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
]
JOINT_VEL_ORDER = [
    "root_x_axis_joint",
    "root_y_axis_joint",
    "root_z_rotation_joint",
]
TF_TARGET_POSES_BASE = {
    "gripper_right": [0.22, -0.18, 0.35, 0.0, 0.0, 0.0, 1.0],
    "gripper_left": [0.22, 0.18, 0.35, 0.0, 0.0, 0.0, 1.0],
    "jaw_right": [0.22, -0.18, 0.35, 0.0, 0.0, 0.0, 1.0],
    "jaw_left": [0.22, 0.18, 0.35, 0.0, 0.0, 0.0, 1.0],
    "wrist_right": [0.16, -0.16, 0.34, 0.0, 0.0, 0.0, 1.0],
    "wrist_left": [0.16, 0.16, 0.34, 0.0, 0.0, 0.0, 1.0],
    "head_pan": [0.0, 0.0, 0.44, 0.0, 0.0, 0.0, 1.0],
    "head_tilt": [0.05, 0.0, 0.46, 0.0, 0.0, 0.0, 1.0],
}


def _stamp_ns_from_header(msg: dict[str, Any]) -> int:
    header = msg.get("header") if isinstance(msg, dict) else None
    stamp = header.get("stamp") if isinstance(header, dict) else None
    if isinstance(stamp, dict):
        try:
            sec = int(stamp.get("sec", 0))
            nanosec = int(stamp.get("nanosec", 0))
            if sec or nanosec:
                return sec * 1_000_000_000 + nanosec
        except (TypeError, ValueError):
            pass
    return time.time_ns()


def _frame_id(msg: dict[str, Any], fallback: str) -> str:
    header = msg.get("header") if isinstance(msg, dict) else None
    if isinstance(header, dict):
        frame = header.get("frame_id")
        if isinstance(frame, str) and frame:
            return frame
    return fallback


def _float(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(parsed):
        return default
    return parsed


def _float_list(values: Any, width: int, default: float = 0.0) -> list[float]:
    if not isinstance(values, list):
        return [default] * width
    out = [_float(v, default) for v in values[:width]]
    if len(out) < width:
        out.extend([default] * (width - len(out)))
    return out


def _finite_number_list(values: Any, width: int, field: str) -> list[float]:
    if not isinstance(values, list) or len(values) != width:
        raise ValueError(f"{field} must be a list of {width} numbers")
    out: list[float] = []
    for value in values:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ValueError(f"{field} entries must be numbers")
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError(f"{field} entries must be finite")
        out.append(parsed)
    return out


def _boolish(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.lower()
        if lowered in ("1", "true", "yes", "on"):
            return True
        if lowered in ("0", "false", "no", "off"):
            return False
    return default


def _robot_id(value: Any, default: int = 0) -> int | None:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    if parsed < 0 or parsed >= MAX_NUM_ROBOTS:
        return None
    return parsed


def _quat_yaw(q: dict[str, Any]) -> float:
    x = _float(q.get("x"))
    y = _float(q.get("y"))
    z = _float(q.get("z"))
    w = _float(q.get("w"), 1.0)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _scan_payload(ros_msg: dict[str, Any], robot_id: int, fallback_frame: str) -> tuple[str, dict[str, Any]]:
    topic = f"scan.{robot_id}"
    raw_ranges = ros_msg.get("ranges") or []
    ranges = array.array("f")
    for raw in raw_ranges:
        value = _float(raw, math.inf)
        ranges.append(value)
    num_ranges = len(ranges)
    angle_min = _float(ros_msg.get("angle_min"), -math.pi)
    angle_increment = _float(ros_msg.get("angle_increment"), 0.0)
    if angle_increment == 0.0 and num_ranges > 1:
        angle_max_raw = _float(ros_msg.get("angle_max"), math.pi)
        angle_increment = (angle_max_raw - angle_min) / float(num_ranges - 1)
    angle_max = _float(
        ros_msg.get("angle_max"),
        angle_min + max(0, num_ranges - 1) * angle_increment,
    )
    payload = {
        "schema": SCHEMA_VERSION,
        "stamp_ns": _stamp_ns_from_header(ros_msg),
        "topic": topic,
        "frame": _frame_id(ros_msg, fallback_frame),
        "encoding": "f32",
        "ranges": ranges.tobytes(),
        "num_ranges": num_ranges,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": angle_increment,
        "range_min": _float(ros_msg.get("range_min"), 0.05),
        "range_max": _float(ros_msg.get("range_max"), 12.0),
    }
    return topic, payload


def _proprio_payload(ros_msg: dict[str, Any], robot_id: int, fallback_frame: str) -> tuple[str, dict[str, Any]]:
    topic = f"proprio.{robot_id}"
    pose_msg = ((ros_msg.get("pose") or {}).get("pose") or {})
    twist_msg = ((ros_msg.get("twist") or {}).get("twist") or {})
    position = pose_msg.get("position") or {}
    orientation = pose_msg.get("orientation") or {}
    linear = twist_msg.get("linear") or {}
    angular = twist_msg.get("angular") or {}
    yaw = _quat_yaw(orientation)
    payload = {
        "schema": SCHEMA_VERSION,
        "stamp_ns": _stamp_ns_from_header(ros_msg),
        "topic": topic,
        "frame": f"proprio_{robot_id}",
        "robot_id": robot_id,
        "joint_names_pos": JOINT_POS_ORDER,
        "joint_pos": [0.0] * len(JOINT_POS_ORDER),
        "joint_vel": [0.0] * len(JOINT_POS_ORDER),
        "gripper_state": {"right": 0.0, "left": 0.0},
        "gripper_velocity": {"right": 0.0, "left": 0.0},
        "joint_vel_arm_sample": [
            _float(position.get("x")),
            _float(position.get("y")),
            yaw,
        ],
        "joint_names_base": JOINT_VEL_ORDER,
        "base_joint_vel": [
            _float(linear.get("x")),
            _float(linear.get("y")),
            _float(angular.get("z")),
        ],
        "base_pose": [
            _float(position.get("x")),
            _float(position.get("y")),
            _float(position.get("z")),
            _float(orientation.get("x")),
            _float(orientation.get("y")),
            _float(orientation.get("z")),
            _float(orientation.get("w"), 1.0),
        ],
        "base_twist": [
            _float(linear.get("x")),
            _float(linear.get("y")),
            _float(linear.get("z")),
            _float(angular.get("x")),
            _float(angular.get("y")),
            _float(angular.get("z")),
        ],
        "base_forward_w": [math.cos(yaw), math.sin(yaw), 0.0],
        "base_motion_model": "pi_rosbridge_hardware",
        "base_state_source": _frame_id(ros_msg, fallback_frame),
        "base_command_frame": "body",
        "base_command_age_ms": None,
        "base_cmd_vel_applied": None,
        "base_recenter_count": 0,
    }
    return topic, payload


def _tf_links_payload(ros_msg: dict[str, Any], robot_id: int, fallback_frame: str) -> tuple[str, dict[str, Any]]:
    topic = f"tf.links.{robot_id}"
    payload = {
        "schema": SCHEMA_VERSION,
        "stamp_ns": _stamp_ns_from_header(ros_msg),
        "topic": topic,
        "frame": f"tf_links_{robot_id}",
        "source": _frame_id(ros_msg, fallback_frame),
        "targets": [
            {"name": name, "pose": list(pose)}
            for name, pose in TF_TARGET_POSES_BASE.items()
        ],
        "source_note": "static base-frame anchors for real-robot base driving; arm EE actuation is not supported here",
    }
    return topic, payload


def _canonical_joint_ticks_to_local(joints: list[float]) -> list[float]:
    right = joints[:6]
    left = joints[6:12]
    head = joints[12:14]
    return [*left, *right, *head]


def _validate_command(command: dict[str, Any]) -> int:
    schema = command.get("schema")
    if schema not in SUPPORTED_SCHEMAS:
        raise ValueError(f"unsupported schema {schema!r}")
    rid = command.get("robot_id", 0)
    if not isinstance(rid, int) or isinstance(rid, bool):
        raise ValueError("robot_id must be an int")
    if not (0 <= rid < MAX_NUM_ROBOTS):
        raise ValueError("robot_id out of protocol range")
    frame = command.get("frame", "body")
    if frame not in COMMAND_FRAMES:
        raise ValueError(f"frame {frame!r} is not supported by the wire contract")

    _validate_command_metadata(command)
    if schema == SCHEMA_VERSION:
        _finite_number_list(command.get("arm_joint_pos_target"), len(JOINT_POS_ORDER), "arm_joint_pos_target")
        _finite_number_list(command.get("base_cmd_vel"), len(JOINT_VEL_ORDER), "base_cmd_vel")
        _validate_v11_optional_fields(command)
    else:
        if "base_cmd_vel" in command:
            _finite_number_list(command["base_cmd_vel"], len(JOINT_VEL_ORDER), "base_cmd_vel")
        if "arm_joint_pos_target" in command:
            _finite_number_list(command["arm_joint_pos_target"], len(JOINT_POS_ORDER), "arm_joint_pos_target")
        _validate_v11_optional_fields(command)
        if "arm_joint_pos_target" in command and "arm_ee_pose_target" in command:
            raise ValueError("v1.1 forbids arm_joint_pos_target and arm_ee_pose_target together")
    return int(rid)


def _validate_command_metadata(command: dict[str, Any]) -> None:
    metadata_keys = ("source_id", "source_role", "priority", "lease_ms")
    if not any(key in command for key in metadata_keys):
        return
    source_id = command.get("source_id")
    if not isinstance(source_id, str) or not source_id or len(source_id) > COMMAND_SOURCE_ID_MAX_LEN:
        raise ValueError(f"source_id must be 1..{COMMAND_SOURCE_ID_MAX_LEN} characters")
    if "source_role" in command and command["source_role"] not in COMMAND_SOURCE_ROLES:
        raise ValueError(f"source_role must be one of {COMMAND_SOURCE_ROLES}")
    if "priority" in command:
        priority = command["priority"]
        if not isinstance(priority, int) or isinstance(priority, bool):
            raise ValueError("priority must be an int")
        lo, hi = COMMAND_PRIORITY_RANGE
        if not (lo <= priority <= hi):
            raise ValueError(f"priority must be in [{lo}, {hi}]")
    if "lease_ms" in command:
        lease_ms = command["lease_ms"]
        if not isinstance(lease_ms, int) or isinstance(lease_ms, bool):
            raise ValueError("lease_ms must be an int")
        lo, hi = COMMAND_LEASE_MS_RANGE
        if not (lo <= lease_ms <= hi):
            raise ValueError(f"lease_ms must be in [{lo}, {hi}]")


def _validate_v11_optional_fields(command: dict[str, Any]) -> None:
    if "arm_ee_pose_target" in command:
        body = command["arm_ee_pose_target"]
        if not isinstance(body, dict) or not body:
            raise ValueError("arm_ee_pose_target must be a non-empty dict")
        for side, entry in body.items():
            if side not in ("right", "left"):
                raise ValueError("arm_ee_pose_target side must be right or left")
            if not isinstance(entry, dict):
                raise ValueError("arm_ee_pose_target entries must be dicts")
            _finite_number_list(entry.get("pose"), 7, f"arm_ee_pose_target[{side}].pose")
            if entry.get("mode", "absolute") not in ("absolute",):
                raise ValueError("arm_ee_pose_target mode must be absolute")
            if entry.get("frame", "base") not in ("base",):
                raise ValueError("arm_ee_pose_target frame must be base")
    if "arm_joint_relative_target" in command:
        body = command["arm_joint_relative_target"]
        if not isinstance(body, dict):
            raise ValueError("arm_joint_relative_target must be a dict")
        for side, entry in body.items():
            if side not in ("right", "left"):
                raise ValueError("arm_joint_relative_target side must be right or left")
            if not isinstance(entry, dict):
                raise ValueError("arm_joint_relative_target entries must be dicts")
            for field, value in entry.items():
                if field not in ("shoulder_pan", "gripper"):
                    raise ValueError("unsupported arm_joint_relative_target field")
                if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
                    raise ValueError("arm_joint_relative_target values must be finite numbers")
    if "head_joint_relative_target" in command:
        body = command["head_joint_relative_target"]
        if not isinstance(body, dict):
            raise ValueError("head_joint_relative_target must be a dict")
        for field, value in body.items():
            if field not in ("head_pan", "head_tilt"):
                raise ValueError("unsupported head_joint_relative_target field")
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
                raise ValueError("head_joint_relative_target values must be finite numbers")


class IsaacCompatGateway:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.stop_event = threading.Event()
        self.context = zmq.Context.instance()
        self.ws: SimpleWebSocket | None = None
        self.ws_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.rosbridge_connected = False
        self.estop_enabled = False
        self.estop_reason: str | None = None
        self.estop_source_id: str | None = None
        self.active_source_id: str | None = None
        self.active_source_role: str | None = None
        self.active_priority: int | None = None
        self.lease_until_mono: float | None = None
        self.last_rejected_reason: str | None = None
        self.last_rejected_source_id: str | None = None
        self.last_accepted_reason: str | None = None
        self.last_ignored_fields: list[str] = []
        self.last_command_schema: str | None = None
        self.last_command_frame: str | None = None
        self.last_scan_ns: int | None = None
        self.last_proprio_ns: int | None = None
        self.last_tf_links_ns: int | None = None
        self.last_command_ns: int | None = None

        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.setsockopt(zmq.LINGER, 0)
        self.pub_socket.setsockopt(zmq.SNDHWM, int(args.pub_hwm))
        self.pub_socket.bind(f"tcp://{args.bind_host}:{args.pub_port}")

        self.pull_socket = self.context.socket(zmq.PULL)
        self.pull_socket.setsockopt(zmq.LINGER, 0)
        self.pull_socket.setsockopt(zmq.RCVHWM, int(args.pull_hwm))
        self.pull_socket.bind(f"tcp://{args.bind_host}:{args.pull_port}")

        self.rep_socket = self.context.socket(zmq.REP)
        self.rep_socket.setsockopt(zmq.LINGER, 0)
        self.rep_socket.bind(f"tcp://{args.bind_host}:{args.rep_port}")

    def run(self) -> None:
        logging.info(
            "indory_isaac_sim compat gateway: PUB tcp://%s:%s, PULL tcp://%s:%s, REP tcp://%s:%s",
            self.args.bind_host,
            self.args.pub_port,
            self.args.bind_host,
            self.args.pull_port,
            self.args.bind_host,
            self.args.rep_port,
        )
        threads = [
            threading.Thread(target=self._command_loop, name="zmq-command", daemon=True),
            threading.Thread(target=self._rpc_loop, name="zmq-rpc", daemon=True),
        ]
        for thread in threads:
            thread.start()
        self._rosbridge_loop()

    def close(self) -> None:
        self.stop_event.set()
        with self.ws_lock:
            if self.ws is not None:
                close = getattr(self.ws, "close", None)
                if callable(close):
                    close()
                self.ws = None
            self.rosbridge_connected = False
        for socket_obj in (self.pub_socket, self.pull_socket, self.rep_socket):
            socket_obj.close(0)

    def _rosbridge_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                ws = SimpleWebSocket(
                    self.args.rosbridge_url,
                    connect_timeout=self.args.connect_timeout_s,
                    read_timeout=self.args.read_timeout_s,
                )
                ws.connect()
                with self.ws_lock:
                    self.ws = ws
                    self.rosbridge_connected = True
                    self._setup_rosbridge_locked()
                logging.info("Connected to rosbridge at %s", self.args.rosbridge_url)
                self._connected_rosbridge_loop()
            except (OSError, WebSocketClosed, RuntimeError, ValueError) as exc:
                logging.warning("rosbridge connection lost: %s", exc)
            finally:
                with self.ws_lock:
                    if self.ws is not None:
                        self.ws.close()
                    self.ws = None
                    self.rosbridge_connected = False
            self.stop_event.wait(self.args.reconnect_s)

    def _setup_rosbridge_locked(self) -> None:
        assert self.ws is not None
        self.ws.send_json(
            {"op": "subscribe", "topic": self.args.scan_topic, "type": "sensor_msgs/LaserScan"}
        )
        self.ws.send_json(
            {"op": "subscribe", "topic": self.args.odom_topic, "type": "nav_msgs/Odometry"}
        )
        self.ws.send_json(
            {"op": "advertise", "topic": self.args.cmd_vel_topic, "type": "geometry_msgs/Twist"}
        )
        self.ws.send_json(
            {
                "op": "advertise",
                "topic": self.args.joint_target_topic,
                "type": "std_msgs/Float64MultiArray",
            }
        )

    def _connected_rosbridge_loop(self) -> None:
        while not self.stop_event.is_set():
            with self.ws_lock:
                ws = self.ws
                envelope = ws.recv_json(timeout=0.1) if ws is not None else None
            if envelope is None:
                continue
            if envelope.get("op") != "publish":
                continue
            topic = envelope.get("topic")
            msg = envelope.get("msg")
            if not isinstance(msg, dict):
                continue
            if topic == self.args.scan_topic:
                zmq_topic, payload = _scan_payload(msg, self.args.robot_id, self.args.scan_frame)
                self._publish_zmq(zmq_topic, payload)
                self.last_scan_ns = time.time_ns()
            elif topic == self.args.odom_topic:
                zmq_topic, payload = _proprio_payload(msg, self.args.robot_id, self.args.base_frame)
                self._publish_zmq(zmq_topic, payload)
                self.last_proprio_ns = time.time_ns()
                zmq_topic, payload = _tf_links_payload(msg, self.args.robot_id, self.args.base_frame)
                self._publish_zmq(zmq_topic, payload)
                self.last_tf_links_ns = time.time_ns()

    def _publish_zmq(self, topic: str, payload: dict[str, Any]) -> None:
        packed = msgpack.packb(payload, use_bin_type=True)
        try:
            self.pub_socket.send_multipart([topic.encode("ascii"), packed], flags=zmq.NOBLOCK)
        except zmq.Again:
            logging.debug("ZMQ PUB queue full; dropped %s", topic)

    def _command_loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self.pull_socket, zmq.POLLIN)
        while not self.stop_event.is_set():
            try:
                events = dict(poller.poll(100))
            except zmq.ZMQError:
                if self.stop_event.is_set():
                    return
                raise
            if self.pull_socket not in events:
                continue
            try:
                raw = self.pull_socket.recv(flags=zmq.NOBLOCK)
                command = msgpack.unpackb(raw, raw=False)
            except (zmq.Again, msgpack.ExtraData, ValueError) as exc:
                logging.warning("Could not decode ZMQ command: %s", exc)
                continue
            if not isinstance(command, dict):
                continue
            try:
                rid = _validate_command(command)
            except ValueError as exc:
                self._reject_command(str(exc), command.get("source_id"))
                continue
            if rid != self.args.robot_id:
                continue
            self._forward_command(command)

    def _forward_command(self, command: dict[str, Any]) -> None:
        base = command.get("base_cmd_vel")
        if isinstance(base, list) and len(base) >= 3 and str(command.get("frame") or "body") != "body":
            self._reject_command("unsupported_base_frame", command.get("source_id"))
            return
        joints = command.get("arm_joint_pos_target")

        arm_ee = command.get("arm_ee_pose_target")
        if isinstance(arm_ee, dict) and HAS_IK:
            if not ik_chains:
                init_ik()

            if not joints:
                joints = [2048.0] * 14

            for side in ("left", "right"):
                target = arm_ee.get(side)
                if not target or not target.get("pose"): continue
                pose = target["pose"]
                target_matrix = np.eye(4)
                target_matrix[:3, :3] = quaternion_to_matrix(pose[3:7])
                target_matrix[:3, 3] = pose[:3]

                chain = ik_chains.get(side)
                if not chain: continue

                try:
                    angles = chain.inverse_kinematics(target_matrix)
                    if side == "left":
                        joints[0] = 2048 + angles[3] * (4096 / (2 * 3.14159))
                        joints[1] = 2048 + angles[4] * (4096 / (2 * 3.14159))
                        joints[2] = 2048 + angles[5] * (4096 / (2 * 3.14159))
                        joints[5] = 2048.0 # Grip handled elsewhere?
                    else:
                        joints[6] = 2048 + angles[3] * (4096 / (2 * 3.14159))
                        joints[7] = 2048 + angles[4] * (4096 / (2 * 3.14159))
                        joints[8] = 2048 + angles[5] * (4096 / (2 * 3.14159))
                        joints[11] = 2048.0
                except Exception as e:
                    logging.error(f"IK error {side}: {e}")

        if isinstance(joints, list) and joints:
            if not self.args.allow_raw_joint_targets:
                self._reject_command("arm_joint_pos_target_requires_explicit_raw_mode", command.get("source_id"))
                return
            if command.get("arm_joint_pos_target_units") != "feetech_ticks":
                self._reject_command("arm_joint_pos_target_units_must_be_feetech_ticks", command.get("source_id"))
                return

        accepted, reason = self._consider_command(command)
        if not accepted:
            self._reject_command(reason, command.get("source_id"))
            return

        if isinstance(base, list) and len(base) >= 3:
            frame = str(command.get("frame") or "body")
            vx, vy, wz = _float_list(base, 3)
            twist = {
                "linear": {"x": vx, "y": vy, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": wz},
            }
            if self._rosbridge_publish(self.args.cmd_vel_topic, twist):
                self.last_command_ns = time.time_ns()
                self.last_command_frame = frame

        if isinstance(joints, list) and joints:
            local_joints = _canonical_joint_ticks_to_local(_float_list(joints, len(JOINT_POS_ORDER)))
            if self._rosbridge_publish(
                self.args.joint_target_topic,
                {"data": local_joints},
            ):
                self.last_command_ns = time.time_ns()

    def _rosbridge_publish(self, topic: str, msg: dict[str, Any]) -> bool:
        with self.ws_lock:
            if self.ws is None:
                return False
            self.ws.send_json({"op": "publish", "topic": topic, "msg": msg})
            return True

    def _consider_command(self, command: dict[str, Any]) -> tuple[bool, str]:
        now_mono = time.monotonic()
        source_id, source_role, priority, lease_ms, source_aware = self._source_metadata(command)
        with self.command_lock:
            if self.estop_enabled:
                return False, "estop_enabled"
            if self.lease_until_mono is not None and self.lease_until_mono <= now_mono:
                self.active_source_id = None
                self.active_source_role = None
                self.active_priority = None
                self.lease_until_mono = None
            if self.active_source_id is not None and source_id != self.active_source_id:
                active_priority = self.active_priority or 0
                if priority < active_priority:
                    return False, "lower_priority_active_lease"
                if priority == active_priority:
                    return False, "same_priority_active_lease"
            if source_aware:
                self.active_source_id = source_id
                self.active_source_role = source_role
                self.active_priority = priority
                self.lease_until_mono = now_mono + lease_ms / 1000.0
            self.last_rejected_reason = None
            self.last_rejected_source_id = None
            self.last_accepted_reason = "accepted"
            self.last_ignored_fields = [
                field
                for field in ("arm_ee_pose_target", "arm_joint_relative_target", "head_joint_relative_target")
                if field in command
            ]
            self.last_command_schema = str(command.get("schema") or "")
            return True, "accepted"

    def _source_metadata(self, command: dict[str, Any]) -> tuple[str, str | None, int, int, bool]:
        raw_source_id = command.get("source_id")
        source_aware = isinstance(raw_source_id, str) and bool(raw_source_id)
        source_id = raw_source_id if source_aware else LEGACY_SOURCE_ID
        source_role = command.get("source_role") if source_aware else None
        if not isinstance(source_role, str):
            source_role = None
        priority = command.get("priority", 0) if source_aware else 0
        if not isinstance(priority, int) or isinstance(priority, bool):
            priority = 0
        if source_role == "safety":
            priority = max(priority, SAFETY_SOURCE_PRIORITY)
        lease_ms = command.get("lease_ms", DEFAULT_COMMAND_SOURCE_LEASE_MS) if source_aware else 0
        if not isinstance(lease_ms, int) or isinstance(lease_ms, bool):
            lease_ms = DEFAULT_COMMAND_SOURCE_LEASE_MS
        lease_ms = max(1, min(60_000, int(lease_ms)))
        return source_id, source_role, int(priority), lease_ms, source_aware

    def _reject_command(self, reason: str, source_id: Any = None) -> None:
        with self.command_lock:
            self.last_rejected_reason = reason
            self.last_rejected_source_id = str(source_id) if isinstance(source_id, str) else None
        logging.warning("Dropped ZMQ command: %s", reason)

    def _zero_base(self) -> None:
        self._rosbridge_publish(
            self.args.cmd_vel_topic,
            {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
        )

    def _rpc_loop(self) -> None:
        poller = zmq.Poller()
        poller.register(self.rep_socket, zmq.POLLIN)
        while not self.stop_event.is_set():
            try:
                events = dict(poller.poll(100))
            except zmq.ZMQError:
                if self.stop_event.is_set():
                    return
                raise
            if self.rep_socket not in events:
                continue
            try:
                raw = self.rep_socket.recv(flags=zmq.NOBLOCK)
                request = msgpack.unpackb(raw, raw=False)
                reply = self._rpc_reply(request)
            except Exception as exc:  # keep REP socket synchronized
                reply = self._pack_rpc_reply(False, error=str(exc))
            self.rep_socket.send(reply)

    def _rpc_reply(self, request: Any) -> bytes:
        if not isinstance(request, dict):
            return self._pack_rpc_reply(False, error="rpc request must be a dict")
        if request.get("schema") not in (SCHEMA_VERSION, SCHEMA_VERSION_V11):
            return self._pack_rpc_reply(False, error=f"unsupported schema {request.get('schema')!r}")
        op = request.get("op")
        if op == "topic_list":
            return self._pack_rpc_reply(True, topics=self._topics())
        if op == "fleet_info":
            return self._pack_rpc_reply(
                True,
                num_robots=self.args.num_robots,
                vr_mode=True,
                command_schema=SCHEMA_VERSION_V11,
                ik_config=None,
                base_model={"source": "indory_ros_pi_compat_gateway"},
                action_dim_per_robot=23,
                action_dim=23 * int(self.args.num_robots),
            )
        if op == "joint_names":
            return self._pack_rpc_reply(
                True,
                joint_pos_order=JOINT_POS_ORDER,
                joint_vel_order=JOINT_VEL_ORDER,
            )
        if op == "health":
            now = time.time_ns()
            return self._pack_rpc_reply(
                True,
                health={
                    "ok": bool(self.rosbridge_connected),
                    "source": "indory_ros_pi_compat_gateway",
                    "rosbridge_url": self.args.rosbridge_url,
                    "rosbridge_connected": bool(self.rosbridge_connected),
                    "scan_age_ms": self._age_ms(now, self.last_scan_ns),
                    "proprio_age_ms": self._age_ms(now, self.last_proprio_ns),
                    "tf_links_age_ms": self._age_ms(now, self.last_tf_links_ns),
                    "command_age_ms": self._age_ms(now, self.last_command_ns),
                    "estop": self.estop_enabled,
                },
            )
        if op == "command_status":
            return self._pack_rpc_reply(True, **self._command_status())
        if op == "set_estop":
            rid = _robot_id(request.get("robot_id", self.args.robot_id))
            if rid is None or rid != self.args.robot_id:
                return self._pack_rpc_reply(False, error="invalid robot_id")
            enabled = _boolish(request.get("enabled", request.get("estop", True)), True)
            source_id = request.get("source_id")
            reason = request.get("reason")
            with self.command_lock:
                self.estop_enabled = enabled
                self.estop_reason = str(reason) if isinstance(reason, str) else None
                self.estop_source_id = str(source_id) if isinstance(source_id, str) else None
                if enabled:
                    self.active_source_id = None
                    self.active_source_role = None
                    self.active_priority = None
                    self.lease_until_mono = None
            if enabled:
                self._zero_base()
            return self._pack_rpc_reply(True, enabled=enabled, estop=enabled)
        if op == "stream_stats":
            return self._pack_rpc_reply(
                True,
                stats={topic: {"enabled": True} for topic in self._topics()},
            )
        if op in ("enable_stream", "disable_stream", "set_stream_rate", "set_stream_param", "reset"):
            return self._pack_rpc_reply(True)
        if op == "set_pose":
            return self._pack_rpc_reply(False, error="set_pose is not supported by the Pi hardware gateway")
        if op == "shutdown":
            return self._pack_rpc_reply(False, error="shutdown is disabled on the Pi hardware gateway")
        return self._pack_rpc_reply(False, error=f"unsupported op {op!r}")

    def _topics(self) -> list[str]:
        rid = int(self.args.robot_id)
        return [f"scan.{rid}", f"proprio.{rid}", f"tf.links.{rid}"]

    def _command_status(self) -> dict[str, Any]:
        now_mono = time.monotonic()
        now_ns = time.time_ns()
        with self.command_lock:
            if self.lease_until_mono is not None and self.lease_until_mono <= now_mono:
                self.active_source_id = None
                self.active_source_role = None
                self.active_priority = None
                self.lease_until_mono = None
            remaining_ms = (
                max(0.0, (self.lease_until_mono - now_mono) * 1000.0)
                if self.lease_until_mono is not None
                else None
            )
            return {
                "robot_id": int(self.args.robot_id),
                "estop": bool(self.estop_enabled),
                "estop_reason": self.estop_reason,
                "estop_source_id": self.estop_source_id,
                "active_source_id": self.active_source_id,
                "active_source_role": self.active_source_role,
                "active_priority": self.active_priority,
                "source_lease_remaining_ms": round(remaining_ms, 3) if remaining_ms is not None else None,
                "last_rejected_source_id": self.last_rejected_source_id,
                "last_rejected_reason": self.last_rejected_reason,
                "last_accepted_reason": self.last_accepted_reason,
                "last_ignored_fields": list(self.last_ignored_fields),
                "last_command_schema": self.last_command_schema,
                "last_command_frame": self.last_command_frame,
                "last_command_age_ms": self._age_ms(now_ns, self.last_command_ns),
                "raw_joint_targets_enabled": bool(self.args.allow_raw_joint_targets),
            }

    @staticmethod
    def _age_ms(now_ns: int, stamp_ns: int | None) -> float | None:
        if stamp_ns is None:
            return None
        return (now_ns - stamp_ns) / 1_000_000.0

    @staticmethod
    def _pack_rpc_reply(ok: bool, *, error: str | None = None, **payload: Any) -> bytes:
        body: dict[str, Any] = {"schema": SCHEMA_VERSION, "ok": bool(ok)}
        if error is not None:
            body["error"] = str(error)
        body.update(payload)
        return msgpack.packb(body, use_bin_type=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge Pi rosbridge scan/odom/cmd_vel to indory_isaac_sim ZMQ clients."
    )
    parser.add_argument("--rosbridge-url", default="ws://127.0.0.1:9090")
    parser.add_argument("--reconnect-s", type=float, default=2.0)
    parser.add_argument("--connect-timeout-s", type=float, default=5.0)
    parser.add_argument("--read-timeout-s", type=float, default=5.0)
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--pub-port", type=int, default=8855)
    parser.add_argument("--pull-port", type=int, default=8856)
    parser.add_argument("--rep-port", type=int, default=8857)
    parser.add_argument("--pub-hwm", type=int, default=16)
    parser.add_argument("--pull-hwm", type=int, default=8)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--num-robots", type=int, default=1)
    parser.add_argument("--scan-topic", default="/xlerobot/scan")
    parser.add_argument("--odom-topic", default="/xlerobot/odom")
    parser.add_argument("--cmd-vel-topic", default="/xlerobot/cmd_vel")
    parser.add_argument("--joint-target-topic", default="/xlerobot/teleop/joint_targets")
    parser.add_argument("--scan-frame", default="laser")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--allow-raw-joint-targets", action="store_true")
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    gateway = IsaacCompatGateway(args)

    def _stop(_signum: int, _frame: Any) -> None:
        gateway.close()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        gateway.run()
    finally:
        gateway.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
