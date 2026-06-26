#!/usr/bin/env python3
"""ROS 2 client bridge for the indoory_isaac_sim ZMQ server contract."""

from __future__ import annotations

import array
import json
import math
import socket
import sys
import time
from typing import Any

import msgpack
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import zmq


SCHEMA_V1 = "xlerobot_v1"
SCHEMA_V11 = "xlerobot_v1.1"
COMMAND_FRAMES = ("body", "world")
JOINT_POS_ORDER = (
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
LOCAL_JOINT_ALIASES = {
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


def _clip(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def _as_float_list(value: Any, size: int, default: float = 0.0) -> list[float]:
    if not isinstance(value, (list, tuple)):
        return [default] * size
    out = [default] * size
    for idx, item in enumerate(value[:size]):
        try:
            out[idx] = float(item)
        except (TypeError, ValueError):
            out[idx] = default
    return out


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _set_socket_option(sock: zmq.Socket, option: int, value: Any) -> None:
    try:
        sock.setsockopt(option, value)
    except zmq.ZMQError:
        pass


class IsaacSimZmqBridge(Node):
    def __init__(self) -> None:
        super().__init__("xlerobot_isaac_sim_zmq_bridge")

        self.declare_parameter("sim_host", "127.0.0.1")
        self.declare_parameter("pub_port", 5555)
        self.declare_parameter("pull_port", 5556)
        self.declare_parameter("rep_port", 5557)
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("command_schema", "auto")
        self.declare_parameter("command_frame", "body")
        self.declare_parameter("source_id", "")
        self.declare_parameter("source_role", "teleop")
        self.declare_parameter("priority", 50)
        self.declare_parameter("lease_ms", 500)
        self.declare_parameter("cmd_vel_topic", "/xlerobot/cmd_vel")
        self.declare_parameter("joint_target_topic", "/xlerobot/teleop/joint_targets")
        self.declare_parameter("odom_topic", "/xlerobot/odom")
        self.declare_parameter("joint_states_topic", "/xlerobot/joint_states")
        self.declare_parameter("scan_topic", "/xlerobot/scan")
        self.declare_parameter("scan_mid_topic", "/xlerobot/scan_mid")
        self.declare_parameter("front_image_topic", "/xlerobot/camera/front/image/compressed")
        self.declare_parameter("wrist_image_topic", "/xlerobot/camera/wrist/image/compressed")
        self.declare_parameter("left_wrist_image_topic", "/xlerobot/camera/wrist_left/image/compressed")
        self.declare_parameter("front_depth_topic", "/xlerobot/depth/front/image/compressed")
        self.declare_parameter("status_topic", "/xlerobot/isaac/status")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("publish_images", True)
        self.declare_parameter("publish_depth_compressed", True)
        self.declare_parameter("subscribe_joint_targets", True)
        self.declare_parameter("stream_topics", [""])
        self.declare_parameter("command_rate_hz", 30.0)
        self.declare_parameter("stream_poll_rate_hz", 100.0)
        self.declare_parameter("status_rate_hz", 0.2)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("rpc_timeout_ms", 500)
        self.declare_parameter("max_stream_messages_per_poll", 50)
        self.declare_parameter("max_linear_x", 0.5)
        self.declare_parameter("max_linear_y", 0.5)
        self.declare_parameter("max_angular_z", 1.5)
        self.declare_parameter("arm_joint_pos_target", [0.0] * len(JOINT_POS_ORDER))

        self.sim_host = str(self.get_parameter("sim_host").value)
        self.pub_port = int(self.get_parameter("pub_port").value)
        self.pull_port = int(self.get_parameter("pull_port").value)
        self.rep_port = int(self.get_parameter("rep_port").value)
        self.robot_id = int(self.get_parameter("robot_id").value)
        requested_schema = str(self.get_parameter("command_schema").value)
        if requested_schema not in ("auto", SCHEMA_V1, SCHEMA_V11):
            raise ValueError(f"command_schema must be auto, {SCHEMA_V1}, or {SCHEMA_V11}")
        self.command_schema = requested_schema if requested_schema != "auto" else SCHEMA_V1
        self.command_schema_auto = requested_schema == "auto"
        self.command_frame = str(self.get_parameter("command_frame").value)
        if self.command_frame not in COMMAND_FRAMES:
            raise ValueError(f"command_frame must be one of {COMMAND_FRAMES}")
        self.source_id = str(self.get_parameter("source_id").value).strip()
        if not self.source_id:
            host = socket.gethostname().split(".")[0] or "host"
            self.source_id = f"ros2:{host}:robot{self.robot_id}"
        self.source_role = str(self.get_parameter("source_role").value)
        self.priority = int(self.get_parameter("priority").value)
        self.lease_ms = int(self.get_parameter("lease_ms").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_tf = _as_bool(self.get_parameter("publish_tf").value)
        self.publish_odom = _as_bool(self.get_parameter("publish_odom").value)
        self.publish_joint_states = _as_bool(self.get_parameter("publish_joint_states").value)
        self.publish_scan = _as_bool(self.get_parameter("publish_scan").value)
        self.publish_images = _as_bool(self.get_parameter("publish_images").value)
        self.publish_depth_compressed = _as_bool(
            self.get_parameter("publish_depth_compressed").value
        )
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.rpc_timeout_ms = int(self.get_parameter("rpc_timeout_ms").value)
        self.max_stream_messages_per_poll = int(
            self.get_parameter("max_stream_messages_per_poll").value
        )
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.arm_joint_pos_target = _as_float_list(
            self.get_parameter("arm_joint_pos_target").value,
            len(JOINT_POS_ORDER),
        )

        self.latest_twist = Twist()
        self.latest_cmd_time = 0.0
        self.last_stream_time = 0.0
        self.last_rpc_reply: dict[str, Any] = {}
        self.last_rpc_error = ""
        self._last_warn: dict[str, float] = {}

        self.context = zmq.Context()
        self.push_socket = self.context.socket(zmq.PUSH)
        _set_socket_option(self.push_socket, zmq.LINGER, 0)
        _set_socket_option(self.push_socket, zmq.SNDHWM, 1)
        _set_socket_option(self.push_socket, zmq.CONFLATE, 1)
        self.push_socket.connect(f"tcp://{self.sim_host}:{self.pull_port}")

        self.sub_socket = self.context.socket(zmq.SUB)
        _set_socket_option(self.sub_socket, zmq.LINGER, 0)
        _set_socket_option(self.sub_socket, zmq.RCVHWM, 16)
        self.sub_socket.connect(f"tcp://{self.sim_host}:{self.pub_port}")
        for topic in self._stream_topics():
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, topic.encode())

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        joint_target_topic = str(self.get_parameter("joint_target_topic").value)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        if _as_bool(self.get_parameter("subscribe_joint_targets").value):
            self.create_subscription(JointState, joint_target_topic, self._on_joint_target, 10)

        self.odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            10,
        )
        self.joint_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            10,
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            10,
        )
        self.scan_mid_pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter("scan_mid_topic").value),
            10,
        )
        self.image_pubs = {
            f"rgb.front.{self.robot_id}": self.create_publisher(
                CompressedImage,
                str(self.get_parameter("front_image_topic").value),
                10,
            ),
            f"rgb.wrist.{self.robot_id}": self.create_publisher(
                CompressedImage,
                str(self.get_parameter("wrist_image_topic").value),
                10,
            ),
            f"rgb.wrist.left.{self.robot_id}": self.create_publisher(
                CompressedImage,
                str(self.get_parameter("left_wrist_image_topic").value),
                10,
            ),
        }
        self.depth_pubs = {
            f"depth.front.{self.robot_id}": self.create_publisher(
                CompressedImage,
                str(self.get_parameter("front_depth_topic").value),
                10,
            ),
        }
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        command_rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        stream_poll_rate_hz = max(1.0, float(self.get_parameter("stream_poll_rate_hz").value))
        status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        self.create_timer(1.0 / command_rate_hz, self._send_latest_command)
        self.create_timer(1.0 / stream_poll_rate_hz, self._poll_stream)
        if status_rate_hz > 0.0:
            self.create_timer(1.0 / status_rate_hz, self._publish_status)

        if self.command_schema_auto:
            self._refresh_fleet_info()

        self.get_logger().info(
            "Connected Isaac Sim ZMQ client: "
            f"SUB tcp://{self.sim_host}:{self.pub_port}, "
            f"PUSH tcp://{self.sim_host}:{self.pull_port}, "
            f"REQ tcp://{self.sim_host}:{self.rep_port}, "
            f"robot_id={self.robot_id}, command_schema={self.command_schema}"
        )

    def destroy_node(self) -> bool:
        try:
            self.push_socket.send(self._pack_command([0.0, 0.0, 0.0]), flags=zmq.NOBLOCK)
        except zmq.ZMQError:
            pass
        self.push_socket.close(0)
        self.sub_socket.close(0)
        self.context.term()
        return super().destroy_node()

    def _stream_topics(self) -> list[str]:
        raw_topics = self.get_parameter("stream_topics").value
        if isinstance(raw_topics, str):
            raw_topics = [raw_topics]
        configured = [str(topic) for topic in raw_topics if str(topic)]
        if configured:
            return configured

        topics = [f"proprio.{self.robot_id}"]
        if self.publish_scan:
            topics.extend([f"scan.{self.robot_id}", f"scan.mid.{self.robot_id}"])
        if self.publish_images:
            topics.extend(
                [
                    f"rgb.front.{self.robot_id}",
                    f"rgb.wrist.{self.robot_id}",
                    f"rgb.wrist.left.{self.robot_id}",
                ]
            )
        if self.publish_depth_compressed:
            topics.append(f"depth.front.{self.robot_id}")
        return topics

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.latest_twist = msg
        self.latest_cmd_time = time.monotonic()

    def _on_joint_target(self, msg: JointState) -> None:
        if len(msg.position) >= len(JOINT_POS_ORDER) and not msg.name:
            self.arm_joint_pos_target = [float(v) for v in msg.position[: len(JOINT_POS_ORDER)]]
            return

        index_by_name = {name: idx for idx, name in enumerate(JOINT_POS_ORDER)}
        next_target = list(self.arm_joint_pos_target)
        updated = False
        for idx, raw_name in enumerate(msg.name):
            if idx >= len(msg.position):
                break
            name = LOCAL_JOINT_ALIASES.get(raw_name, raw_name)
            target_idx = index_by_name.get(name)
            if target_idx is None:
                continue
            next_target[target_idx] = float(msg.position[idx])
            updated = True
        if updated:
            self.arm_joint_pos_target = next_target

    def _send_latest_command(self) -> None:
        now = time.monotonic()
        if now - self.latest_cmd_time > self.command_timeout_s:
            base_cmd_vel = [0.0, 0.0, 0.0]
        else:
            base_cmd_vel = [
                _clip(float(self.latest_twist.linear.x), self.max_linear_x),
                _clip(float(self.latest_twist.linear.y), self.max_linear_y),
                _clip(float(self.latest_twist.angular.z), self.max_angular_z),
            ]
        try:
            self.push_socket.send(self._pack_command(base_cmd_vel), flags=zmq.NOBLOCK)
        except zmq.Again:
            self._warn_throttled("command_busy", "Command socket busy; dropping latest command.")
        except zmq.ZMQError as exc:
            self._warn_throttled("command_error", f"Failed to send Isaac command: {exc}")

    def _pack_command(self, base_cmd_vel: list[float]) -> bytes:
        body: dict[str, Any] = {
            "schema": self.command_schema,
            "stamp_ns": int(time.monotonic_ns()),
            "robot_id": self.robot_id,
            "frame": self.command_frame,
            "base_cmd_vel": [float(v) for v in base_cmd_vel],
        }
        if self.command_schema == SCHEMA_V11:
            body.update(
                {
                    "source_id": self.source_id,
                    "source_role": self.source_role,
                    "priority": self.priority,
                    "lease_ms": self.lease_ms,
                }
            )
        else:
            body["schema"] = SCHEMA_V1
            body["arm_joint_pos_target"] = [float(v) for v in self.arm_joint_pos_target]
        return msgpack.packb(body, use_bin_type=True)

    def _poll_stream(self) -> None:
        for _ in range(max(1, self.max_stream_messages_per_poll)):
            try:
                topic_b, payload_b = self.sub_socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            except ValueError:
                self._warn_throttled("stream_multipart", "Dropping non-multipart stream frame.")
                continue
            except zmq.ZMQError as exc:
                self._warn_throttled("stream_error", f"Failed to receive Isaac stream: {exc}")
                return

            try:
                topic = topic_b.decode()
                payload = msgpack.unpackb(payload_b, raw=False)
            except (UnicodeDecodeError, msgpack.ExtraData, ValueError) as exc:
                self._warn_throttled("stream_decode", f"Dropping invalid Isaac stream frame: {exc}")
                continue

            if not isinstance(payload, dict):
                continue
            self.last_stream_time = time.monotonic()
            self._handle_stream(topic, payload)

    def _handle_stream(self, topic: str, payload: dict[str, Any]) -> None:
        if topic == f"proprio.{self.robot_id}":
            if self.publish_joint_states:
                self._publish_joint_state(payload)
            if self.publish_odom:
                self._publish_odom(payload)
        elif topic == f"scan.{self.robot_id}" and self.publish_scan:
            self._publish_scan(payload, self.scan_pub)
        elif topic == f"scan.mid.{self.robot_id}" and self.publish_scan:
            self._publish_scan(payload, self.scan_mid_pub)
        elif topic in self.image_pubs and self.publish_images:
            self._publish_compressed(payload, self.image_pubs[topic])
        elif topic in self.depth_pubs and self.publish_depth_compressed:
            self._publish_compressed(payload, self.depth_pubs[topic])

    def _publish_joint_state(self, payload: dict[str, Any]) -> None:
        positions = payload.get("joint_pos")
        if not isinstance(positions, (list, tuple)):
            return
        names = payload.get("joint_names_pos")
        if not isinstance(names, (list, tuple)) or len(names) != len(positions):
            names = JOINT_POS_ORDER[: len(positions)]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [str(name) for name in names]
        msg.position = [float(value) for value in positions]
        velocities = payload.get("joint_vel")
        if isinstance(velocities, (list, tuple)):
            msg.velocity = _as_float_list(velocities, len(msg.name))
        self.joint_pub.publish(msg)

    def _publish_odom(self, payload: dict[str, Any]) -> None:
        pose = payload.get("base_pose")
        if not isinstance(pose, (list, tuple)) or len(pose) < 7:
            return
        twist = payload.get("base_twist")
        twist_values = _as_float_list(twist, 6)
        pose_values = _as_float_list(pose, 7)

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = pose_values[0]
        odom.pose.pose.position.y = pose_values[1]
        odom.pose.pose.position.z = pose_values[2]
        odom.pose.pose.orientation.x = pose_values[3]
        odom.pose.pose.orientation.y = pose_values[4]
        odom.pose.pose.orientation.z = pose_values[5]
        odom.pose.pose.orientation.w = pose_values[6]
        odom.twist.twist.linear.x = twist_values[0]
        odom.twist.twist.linear.y = twist_values[1]
        odom.twist.twist.linear.z = twist_values[2]
        odom.twist.twist.angular.x = twist_values[3]
        odom.twist.twist.angular.y = twist_values[4]
        odom.twist.twist.angular.z = twist_values[5]
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = pose_values[0]
            tf_msg.transform.translation.y = pose_values[1]
            tf_msg.transform.translation.z = pose_values[2]
            tf_msg.transform.rotation.x = pose_values[3]
            tf_msg.transform.rotation.y = pose_values[4]
            tf_msg.transform.rotation.z = pose_values[5]
            tf_msg.transform.rotation.w = pose_values[6]
            self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_scan(self, payload: dict[str, Any], publisher: Any) -> None:
        ranges = self._decode_float32_ranges(payload.get("ranges"), payload.get("num_ranges"))
        if ranges is None:
            return
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = str(payload.get("frame") or self.base_frame_id)
        scan.angle_min = float(payload.get("angle_min", -math.pi))
        scan.angle_max = float(payload.get("angle_max", math.pi))
        if "angle_increment" in payload:
            scan.angle_increment = float(payload.get("angle_increment"))
        elif len(ranges) > 1:
            scan.angle_increment = (scan.angle_max - scan.angle_min) / float(len(ranges) - 1)
        else:
            scan.angle_increment = 0.0
        scan.range_min = float(payload.get("range_min", 0.0))
        scan.range_max = float(payload.get("range_max", 0.0))
        scan.ranges = ranges
        publisher.publish(scan)

    def _publish_compressed(self, payload: dict[str, Any], publisher: Any) -> None:
        data = payload.get("data")
        if not isinstance(data, (bytes, bytearray)):
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(payload.get("frame") or "isaac_camera")
        encoding = str(payload.get("encoding") or "jpeg")
        if "depth_scale_m" in payload:
            msg.format = f"{encoding}; depth_scale_m={payload.get('depth_scale_m')}"
        else:
            msg.format = encoding
        msg.data = bytes(data)
        publisher.publish(msg)

    def _decode_float32_ranges(self, data: Any, num_ranges: Any) -> list[float] | None:
        if isinstance(data, (list, tuple)):
            return [float(value) for value in data]
        if not isinstance(data, (bytes, bytearray)):
            return None
        values = array.array("f")
        usable_len = len(data) - (len(data) % values.itemsize)
        values.frombytes(bytes(data[:usable_len]))
        if sys.byteorder != "little":
            values.byteswap()
        try:
            count = int(num_ranges)
        except (TypeError, ValueError):
            count = len(values)
        return [float(value) for value in values[:count]]

    def _refresh_fleet_info(self) -> None:
        reply = self._rpc("fleet_info")
        if not reply.get("ok"):
            return
        schema = reply.get("command_schema")
        if schema in (SCHEMA_V1, SCHEMA_V11):
            self.command_schema = str(schema)

    def _publish_status(self) -> None:
        if self.command_schema_auto:
            self._refresh_fleet_info()
        health = self._rpc("health")
        if health.get("ok"):
            self.last_rpc_reply = health
            self.last_rpc_error = ""

        now = time.monotonic()
        status = {
            "sim_host": self.sim_host,
            "pub_port": self.pub_port,
            "pull_port": self.pull_port,
            "rep_port": self.rep_port,
            "robot_id": self.robot_id,
            "command_schema": self.command_schema,
            "last_cmd_age_s": None if self.latest_cmd_time <= 0.0 else now - self.latest_cmd_time,
            "last_stream_age_s": None if self.last_stream_time <= 0.0 else now - self.last_stream_time,
            "last_rpc_ok": bool(self.last_rpc_reply),
            "last_rpc_error": self.last_rpc_error,
            "health": self.last_rpc_reply.get("health", {}),
        }
        msg = String()
        msg.data = json.dumps(status, separators=(",", ":"), sort_keys=True)
        self.status_pub.publish(msg)

    def _rpc(self, op: str, **kwargs: Any) -> dict[str, Any]:
        sock = self.context.socket(zmq.REQ)
        _set_socket_option(sock, zmq.LINGER, 0)
        _set_socket_option(sock, zmq.SNDTIMEO, self.rpc_timeout_ms)
        _set_socket_option(sock, zmq.RCVTIMEO, self.rpc_timeout_ms)
        try:
            sock.connect(f"tcp://{self.sim_host}:{self.rep_port}")
            body = {"schema": SCHEMA_V1, "op": op, **kwargs}
            sock.send(msgpack.packb(body, use_bin_type=True))
            reply = msgpack.unpackb(sock.recv(), raw=False)
            if isinstance(reply, dict):
                return reply
            self.last_rpc_error = f"{op}: non-dict reply"
            return {"ok": False, "error": self.last_rpc_error}
        except (zmq.ZMQError, msgpack.ExtraData, ValueError) as exc:
            self.last_rpc_error = f"{op}: {exc}"
            return {"ok": False, "error": self.last_rpc_error}
        finally:
            sock.close(0)

    def _warn_throttled(self, key: str, message: str, period_s: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= period_s:
            self._last_warn[key] = now
            self.get_logger().warning(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IsaacSimZmqBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
