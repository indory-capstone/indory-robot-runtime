"""Optional rosbridge monitor used by the robot webview."""

from __future__ import annotations

import threading
import time
from typing import Any

from .ws_client import SimpleWebSocket


def _now() -> float:
    return time.time()


def _with_age(entry: dict[str, Any] | None) -> dict[str, Any] | None:
    if not entry:
        return None
    copied = dict(entry)
    copied["age_s"] = max(0.0, _now() - float(copied.get("received_unix", _now())))
    return copied


def _clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def _unit_move(direction: str) -> tuple[float, float, float]:
    return {
        "forward": (1.0, 0.0, 0.0),
        "backward": (-1.0, 0.0, 0.0),
        "left": (0.0, 1.0, 0.0),
        "right": (0.0, -1.0, 0.0),
        "rotate_left": (0.0, 0.0, 1.0),
        "rotate_right": (0.0, 0.0, -1.0),
        "stop": (0.0, 0.0, 0.0),
    }.get(direction, (0.0, 0.0, 0.0))


class RosbridgeMonitor(threading.Thread):
    def __init__(
        self,
        rosbridge_url: str,
        cmd_topics: list[str],
        odom_topics: list[str],
        scan_topics: list[str],
        control_topic: str,
        max_linear_x: float,
        max_linear_y: float,
        max_angular_z: float,
        reconnect_s: float,
        rosbridge_enabled: bool = False,
        fast_command: Any | None = None,
        fast_sensor: Any | None = None,
    ):
        super().__init__(name="rosbridge-monitor", daemon=True)
        self.rosbridge_enabled = rosbridge_enabled
        self.rosbridge_url = rosbridge_url
        self.cmd_topics = cmd_topics
        self.odom_topics = odom_topics
        self.scan_topics = scan_topics
        self.control_topic = control_topic
        self.max_linear_x = max_linear_x
        self.max_linear_y = max_linear_y
        self.max_angular_z = max_angular_z
        self.reconnect_s = reconnect_s
        self.fast_command = fast_command
        self.fast_sensor = fast_sensor
        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.connected = False
        self.error = ""
        self.latest_by_topic: dict[str, dict[str, Any]] = {}
        self.stop_event = threading.Event()
        self.ws: SimpleWebSocket | None = None

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            latest = dict(self.latest_by_topic)
            rosbridge_connected = self.connected
            rosbridge_error = self.error
        latest_cmd = self._latest_for(self.cmd_topics, latest)
        latest_odom = self._latest_for(self.odom_topics, latest)
        latest_scan = self._latest_for(self.scan_topics, latest)
        fast_command_snapshot = (
            self.fast_command.snapshot()
            if self.fast_command is not None
            else {"enabled": False, "ok": False}
        )
        fast_sensor_snapshot = (
            self.fast_sensor.snapshot()
            if self.fast_sensor is not None
            else {"enabled": False, "ok": False, "latest_odom": None, "latest_scan": None}
        )
        if self.fast_command is not None:
            latest_cmd = self._newer(latest_cmd, self.fast_command.latest_command())
        latest_odom = self._newer(latest_odom, fast_sensor_snapshot.get("latest_odom"))
        latest_scan = self._newer(latest_scan, fast_sensor_snapshot.get("latest_scan"))
        connected = (
            rosbridge_connected
            if self.rosbridge_enabled
            else bool(fast_command_snapshot.get("ok") or fast_sensor_snapshot.get("ok"))
        )
        error = rosbridge_error if self.rosbridge_enabled else ""
        if not connected:
            error = (
                str(fast_command_snapshot.get("error") or "")
                or str(fast_sensor_snapshot.get("error") or "")
                or rosbridge_error
            )
        return {
            "connected": connected,
            "error": error,
            "rosbridge_enabled": self.rosbridge_enabled,
            "rosbridge_connected": rosbridge_connected,
            "rosbridge_url": self.rosbridge_url,
            "cmd_topics": self.cmd_topics,
            "odom_topics": self.odom_topics,
            "scan_topics": self.scan_topics,
            "control_topic": self.control_topic,
            "control_limits": {
                "linear_x": self.max_linear_x,
                "linear_y": self.max_linear_y,
                "angular_z": self.max_angular_z,
            },
            "latest_cmd": _with_age(latest_cmd),
            "latest_odom": _with_age(latest_odom),
            "latest_scan": _with_age(latest_scan),
            "fast_zmq": fast_command_snapshot,
            "fast_sensor": fast_sensor_snapshot,
            "updated_unix": _now(),
        }

    def run(self) -> None:
        if not self.rosbridge_enabled:
            return
        while not self.stop_event.is_set():
            try:
                ws = SimpleWebSocket(self.rosbridge_url, connect_timeout=4.0, read_timeout=4.0)
                ws.connect()
                with self.send_lock:
                    self.ws = ws
                self._set_state(True, "")
                self._setup_rosbridge()
                while not self.stop_event.is_set():
                    message = ws.recv_json(timeout=0.2)
                    if message is None:
                        continue
                    if message.get("op") != "publish":
                        continue
                    topic = message.get("topic")
                    if not isinstance(topic, str):
                        continue
                    with self.lock:
                        self.latest_by_topic[topic] = {
                            "topic": topic,
                            "msg": message.get("msg", {}),
                            "received_unix": _now(),
                        }
            except Exception as exc:
                self._set_state(False, str(exc))
                time.sleep(self.reconnect_s)
            finally:
                with self.send_lock:
                    ws_to_close = self.ws
                    self.ws = None
                if ws_to_close is not None:
                    try:
                        ws_to_close.close()
                    except Exception:
                        pass

    def publish_cmd_vel(self, x: float, y: float, z: float) -> dict[str, Any]:
        command = {
            "x": _clamp(float(x), self.max_linear_x),
            "y": _clamp(float(y), self.max_linear_y),
            "z": _clamp(float(z), self.max_angular_z),
        }
        msg = {
            "linear": {"x": command["x"], "y": command["y"], "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": command["z"]},
        }
        if self.fast_command is not None and self.fast_command.enabled:
            fast_result = self.fast_command.publish_cmd_vel(command)
            if fast_result.get("ok"):
                return {
                    "ok": True,
                    "command": command,
                    "base_cmd_vel": [command["x"], command["y"], command["z"]],
                    "transport": "fast_zmq",
                }
        else:
            fast_result = None
        if not self.rosbridge_enabled:
            return {
                "ok": False,
                "error": fast_result.get("error", "fast_zmq command failed") if fast_result else "fast_zmq disabled",
                "command": command,
                "base_cmd_vel": [command["x"], command["y"], command["z"]],
                "transport": "fast_zmq",
            }
        payload = {"op": "publish", "topic": self.control_topic, "msg": msg}
        if not self._send_json(payload):
            return {
                "ok": False,
                "error": self.error or "rosbridge is not connected",
                "fast_error": fast_result.get("error") if fast_result else "",
                "command": command,
                "base_cmd_vel": [command["x"], command["y"], command["z"]],
            }
        return {
            "ok": True,
            "command": command,
            "base_cmd_vel": [command["x"], command["y"], command["z"]],
            "topic": self.control_topic,
            "transport": "rosbridge",
            "fast_error": fast_result.get("error") if fast_result else "",
        }

    def publish_move(self, direction: str, speed: float) -> dict[str, Any]:
        clamped_speed = max(0.0, min(1.0, float(speed)))
        unit_x, unit_y, unit_z = _unit_move(str(direction))
        result = self.publish_cmd_vel(
            unit_x * self.max_linear_x * clamped_speed,
            unit_y * self.max_linear_y * clamped_speed,
            unit_z * self.max_angular_z * clamped_speed,
        )
        result["direction"] = direction
        result["speed"] = clamped_speed
        return result

    def _setup_rosbridge(self) -> None:
        self._send_json({"op": "advertise", "topic": self.control_topic, "type": "geometry_msgs/Twist"})
        for topic in self.cmd_topics:
            self._send_json({"op": "subscribe", "topic": topic, "type": "geometry_msgs/Twist", "throttle_rate": 30})
        for topic in self.odom_topics:
            self._send_json({"op": "subscribe", "topic": topic, "type": "nav_msgs/Odometry", "throttle_rate": 30})
        for topic in self.scan_topics:
            self._send_json({"op": "subscribe", "topic": topic, "type": "sensor_msgs/LaserScan", "throttle_rate": 100})

    def _send_json(self, payload: dict[str, Any]) -> bool:
        with self.send_lock:
            if self.ws is None:
                return False
            try:
                self.ws.send_json(payload)
                return True
            except Exception as exc:
                self.error = str(exc)
                self.connected = False
                return False

    def _set_state(self, connected: bool, error: str) -> None:
        with self.lock:
            self.connected = connected
            self.error = error

    @staticmethod
    def _latest_for(topics: list[str], latest: dict[str, dict[str, Any]]) -> dict[str, Any] | None:
        candidates = [latest[topic] for topic in topics if topic in latest]
        if not candidates:
            return None
        return max(candidates, key=lambda entry: float(entry.get("received_unix", 0.0)))

    @staticmethod
    def _newer(left: dict[str, Any] | None, right: dict[str, Any] | None) -> dict[str, Any] | None:
        if right is None:
            return left
        if left is None:
            return right
        return max((left, right), key=lambda entry: float(entry.get("received_unix", 0.0)))
