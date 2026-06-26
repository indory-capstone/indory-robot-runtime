# Indoory XLeRobot Fast Protocol

This document describes the customer-facing low-latency protocol for controlling
the Indoory XLeRobot and receiving robot state. The webview and custom clients
should use this protocol directly; rosbridge is not required for normal customer
teleoperation.

## Overview

The robot exposes three ZeroMQ TCP ports:

| Purpose | ZMQ pattern | Default endpoint | Direction |
| --- | --- | --- | --- |
| Sensor stream | `PUB/SUB` | `tcp://<robot-host>:8855` | robot to client |
| Commands | `PUSH/PULL` | `tcp://<robot-host>:8856` | client to robot |
| RPC/health | `REQ/REP` | `tcp://<robot-host>:8857` | client to robot |

Payloads are MessagePack dictionaries. Command and RPC request paths also accept
JSON dictionaries, but MessagePack is the supported format.

The command port is latest-value oriented: the server uses low watermarks,
nonblocking reads, and `CONFLATE` where supported. Clients should send the newest
desired command repeatedly and should not rely on every intermediate command
being applied.

## Defaults

| Variable | Default | Description |
| --- | --- | --- |
| `ENABLE_FAST_ZMQ` | `true` | Enable the fast protocol. |
| `FAST_ZMQ_BIND_HOST` | `0.0.0.0` | Interface to bind on the robot. |
| `FAST_ZMQ_PUB_PORT` | `8855` | Sensor PUB port. |
| `FAST_ZMQ_PULL_PORT` | `8856` | Command PULL port. |
| `FAST_ZMQ_REP_PORT` | `8857` | RPC REP port. |
| `FAST_ZMQ_ROBOT_ID` | `0` | Numeric robot id used in topic suffixes. |
| `FAST_ZMQ_PUB_HWM` | `4` | PUB send high-water mark. |
| `FAST_ZMQ_PULL_HWM` | `16` | PULL receive high-water mark. |
| `FAST_ZMQ_PULL_CONFLATE` | `false` | Keep false when multiple command sources may compete; the server drains each batch and chooses the highest-priority fresh command. |
| `FAST_ZMQ_REDUNDANT_STOP_DEDUPE_MS` | `100` | Coalesce repeated non-teleop/non-safety zero base stop commands while the base is already stopped, so background nav clients cannot churn the motor command loop. Set `0` to disable. |

The current robot I/O process uses the upstream XLeRobot motor grouping:
bus1 is left arm plus head, and bus2 is right arm plus base.

## Common Rules

- Timestamps use Unix nanoseconds in `stamp_ns`.
- Linear units are meters and meters per second.
- Angular units are radians and radians per second.
- Base commands are body-frame commands. Use `"frame": "body"`.
- Non-finite command values are rejected or converted to zero depending on the
  field.
- The command port is one-way. It does not return a per-command acknowledgement;
  use the RPC port for health, command status, and head calibration information.
- The base watchdog stops motion when commands are stale. Send commands at
  20 to 60 Hz during teleoperation, and send `[0, 0, 0]` on release.
- The fast e-stop is a software command gate for base motion, not a certified
  hardware safety stop.
- Commands may include `source_id`, `source_role`, `priority`, and `lease_ms`.
  The robot uses these fields to keep an active teleop source from being
  overwritten by stale lower-priority clients. Higher `priority` can take over
  an active lease; equal/lower priority waits until the lease expires. Safety
  sources should use `source_role: "safety"` or an explicit high priority.
- Repeated zero stop commands from background clients may be coalesced after the
  first accepted stop. This does not affect teleop or safety stops, and nonzero
  or joint-target commands are never coalesced.

## Command Port

Connect with a ZMQ `PUSH` socket to `tcp://<robot-host>:8856` and send one
MessagePack dictionary per command.

### Base Velocity

Preferred command shape:

```json
{
  "schema": "xlerobot_v1.1",
  "source_id": "customer_app",
  "seq": 42,
  "stamp_ns": 1780067900000000000,
  "frame": "body",
  "base_cmd_vel": [0.10, 0.00, 0.30]
}
```

`base_cmd_vel` is `[vx, vy, wz]`.

- `vx`: forward velocity in m/s
- `vy`: left strafe velocity in m/s
- `wz`: counter-clockwise yaw velocity in rad/s

Equivalent geometry-style command:

```json
{
  "schema": "xlerobot_v1.1",
  "source_id": "customer_app",
  "frame": "body",
  "linear": {"x": 0.10, "y": 0.00, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.30}
}
```

Simple direction command:

```json
{
  "schema": "xlerobot_v1.1",
  "source_id": "customer_app",
  "direction": "forward",
  "speed": 0.5
}
```

Supported directions are `forward`, `backward`, `left`, `right`,
`rotate_left`, `rotate_right`, and `stop`. `speed` is clamped to `0..1`.

### Joint Targets

Raw external joint target order:

```json
{
  "schema": "xlerobot_v1.1",
  "source_id": "customer_app",
  "joint_targets": [2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048]
}
```

`joint_targets` has 14 Feetech raw tick values in this order:

```text
left_hand_1..left_hand_6,
right_hand_1..right_hand_6,
head_pan,
head_tilt
```

Canonical XLeRobot/Isaac-style raw tick command:

```json
{
  "schema": "xlerobot_v1.1",
  "source_id": "customer_app",
  "arm_joint_pos_target_units": "feetech_ticks",
  "arm_joint_pos_target": [2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048]
}
```

The canonical order is:

```text
Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll, Jaw,
Rotation_2, Pitch_2, Elbow_2, Wrist_Pitch_2, Wrist_Roll_2, Jaw_2,
head_pan_joint, head_tilt_joint
```

This order is right arm first, then left arm, then head. The robot server
reorders it internally to the hardware bus order.

Sparse joint target command:

```json
{
  "schema": "xlerobot_v1.1",
  "source_id": "customer_app",
  "stamp_ns": 1780067900000000000,
  "frame": "body",
  "joint_targets_sparse": [
    null, null, null, null, null, null,
    null, null, null, null, null, null,
    2019, 1902
  ]
}
```

`joint_targets_sparse` has the same 14-element external order as
`joint_targets`, but `null` entries are ignored. This is the preferred raw-tick
shape for moving only selected arm or head motors because untouched joints keep
their last target. In the current external order, left arm motors are indices
`0..5`, right arm motors are `6..11`, `head_pan` is index `12`, and `head_tilt`
is index `13`.

Head camera relative command:

```json
{
  "schema": "xlerobot_v1.1",
  "source_id": "customer_app",
  "stamp_ns": 1780067900000000000,
  "frame": "body",
  "head_joint_relative_target": {
    "head_pan": 0.05,
    "head_tilt": -0.03
  }
}
```

`head_joint_relative_target` values are radians relative to the latest
`joint_states.<id>` feedback. The server converts them to Feetech ticks using
the current head position before writing the motor target. If no joint state is
available yet, this command is rejected. On the current head camera hardware,
positive `head_tilt` is implemented by decreasing the raw Feetech tick value;
raw absolute tick commands are not sign-converted.

If a command contains both `joint_targets_sparse` and
`head_joint_relative_target`, the server merges them: sparse arm/head entries
are decoded first, then the relative head conversion overwrites only the two
head slots.

Except for `head_joint_relative_target`, do not send radians in joint target
commands. Joint target values are raw Feetech ticks.

### Joint Limit and Calibration Behavior

The server clamps finite joint targets before writing `Goal_Position` to the
motor bus.

- Arm and head joint targets use the XLeRobot LeRobot calibration file when
  `XLEROBOT_USE_CALIBRATION_LIMITS=true` (default).
- If the calibration file is missing, disabled, or invalid for a motor, that
  motor falls back to `JOINT_TARGET_MIN..JOINT_TARGET_MAX` (`0..4095` by
  default).

On the current robot, the loaded `my_xlerobot_pc` calibration defines these
raw tick ranges:

| External joint | XLeRobot motor | Calibrated tick range |
| --- | --- | --- |
| `left_hand_1` | `left_arm_shoulder_pan` | `739..3283` |
| `left_hand_2` | `left_arm_shoulder_lift` | `161..2658` |
| `left_hand_3` | `left_arm_elbow_flex` | `771..3029` |
| `left_hand_4` | `left_arm_wrist_flex` | `741..3262` |
| `left_hand_5` | `left_arm_wrist_roll` | `0..4094` |
| `left_hand_6` | `left_arm_gripper` | `2041..3558` |
| `right_hand_1` | `right_arm_shoulder_pan` | `865..3449` |
| `right_hand_2` | `right_arm_shoulder_lift` | `770..3138` |
| `right_hand_3` | `right_arm_elbow_flex` | `866..3118` |
| `right_hand_4` | `right_arm_wrist_flex` | `985..3386` |
| `right_hand_5` | `right_arm_wrist_roll` | `1..4094` |
| `right_hand_6` | `right_arm_gripper` | `1983..3516` |
| `head_pan` | `head_motor_2` | `594..3444` |
| `head_tilt` | `head_motor_1` | `888..2915` |

Commands outside these ranges are accepted and clamped, not rejected. For
example, a raw `head_pan` target of `9999` is written as `3444`, and a
`left_hand_1` target of `0` is written as `739`. The command port does not send
a clamp notification back to the client; clients should read the calibration
file or expose equivalent limits before sending raw joint targets, and query
`command_status` only for validation failures such as malformed payloads,
unsupported fields, unavailable joint state, or a detached motor base.

## Sensor Port

Connect with a ZMQ `SUB` socket to `tcp://<robot-host>:8855`. Messages are
multipart frames:

```text
[topic: ascii bytes, payload: msgpack bytes]
```

Topic names include the robot id suffix. With the default `FAST_ZMQ_ROBOT_ID=0`,
topics are:

| Topic | Description |
| --- | --- |
| `odom.0` | Base odometry as a ROS-like `nav_msgs/Odometry` dictionary. |
| `proprio.0` | Compact proprioceptive state for web/VR/control clients. |
| `joint_states.0` | Joint and base wheel state. |
| `scan.0` | Laser scan ranges when lidar is active. |
| `tf.links.0` | Static base-frame link anchors. |

### `odom.<id>`

```json
{
  "schema": "indoory_robot_fast_v1",
  "stamp_ns": 1780067900000000000,
  "topic": "odom.0",
  "robot_id": 0,
  "msg": {
    "header": {"stamp": {"sec": 1780067900, "nanosec": 0}, "frame_id": "odom"},
    "child_frame_id": "base_link",
    "pose": {"pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
    "twist": {"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}
  }
}
```

### `proprio.<id>`

Important fields:

```json
{
  "schema": "xlerobot_v1",
  "stamp_ns": 1780067900000000000,
  "topic": "proprio.0",
  "robot_id": 0,
  "joint_names_pos": ["Rotation", "Pitch", "..."],
  "joint_pos": [0.0, 0.0],
  "joint_vel": [0.0, 0.0],
  "joint_names_base": ["root_x_axis_joint", "root_y_axis_joint", "root_z_rotation_joint"],
  "base_joint_vel": [0.0, 0.0, 0.0],
  "base_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
  "base_twist": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "base_command_frame": "body",
  "base_command_age_ms": 12.3,
  "base_cmd_vel_applied": [0.0, 0.0, 0.0]
}
```

`base_joint_vel` is `[vx, vy, wz]`. `base_pose` is
`[x, y, z, qx, qy, qz, qw]`.

### `joint_states.<id>`

```json
{
  "schema": "indoory_robot_fast_v1",
  "stamp_ns": 1780067900000000000,
  "topic": "joint_states.0",
  "frame": "base_link",
  "robot_id": 0,
  "names": ["left_hand_1", "...", "base_left_wheel", "base_back_wheel", "base_right_wheel"],
  "position": [0.0],
  "velocity": [0.0]
}
```

Arm and head positions are raw Feetech feedback values. Base wheel velocities are
reported in rad/s where available.

### `scan.<id>`

```json
{
  "schema": "xlerobot_v1",
  "stamp_ns": 1780067900000000000,
  "topic": "scan.0",
  "frame": "base_link",
  "encoding": "f32",
  "ranges": "<binary float32 bytes>",
  "num_ranges": 360,
  "angle_min": -3.14159,
  "angle_max": 3.12414,
  "angle_increment": 0.01745,
  "range_min": 0.12,
  "range_max": 12.0
}
```

`ranges` is a MessagePack binary field containing IEEE-754 float32 range values
in little-endian order on the Raspberry Pi. Values may include infinity for
empty bins. The scan topic is only fresh when the lidar publisher is active and
receiving rotations.

### `tf.links.<id>`

```json
{
  "schema": "xlerobot_v1",
  "stamp_ns": 1780067900000000000,
  "topic": "tf.links.0",
  "frame": "tf_links_0",
  "source": "base_link",
  "targets": [
    {"name": "gripper_right", "pose": [0.22, -0.18, 0.35, 0.0, 0.0, 0.0, 1.0]}
  ]
}
```

Each pose is `[x, y, z, qx, qy, qz, qw]`.

## RPC Port

Connect with a ZMQ `REQ` socket to `tcp://<robot-host>:8857`. Send one
MessagePack dictionary and receive one MessagePack dictionary.

### `health`

Request:

```json
{"op": "health"}
```

Response:

```json
{
  "ok": true,
  "health": {
    "ok": true,
    "source": "xlerobot_direct_fast_zmq",
    "robot_id": 0,
    "pub": "tcp://0.0.0.0:8855",
    "pull": "tcp://0.0.0.0:8856",
    "rep": "tcp://0.0.0.0:8857",
    "base_attached": true,
    "estop": false,
    "scan_age_ms": 21.4,
    "odom_age_ms": 8.1,
    "joint_state_age_ms": 9.2,
    "command_age_ms": 31.0,
    "accepted_commands": 12,
    "dropped_commands": 0,
    "dropped_pub": 0,
    "topics": ["scan.0", "proprio.0", "tf.links.0", "odom.0", "joint_states.0"]
  }
}
```

### `head_debug`

Request:

```json
{"op": "head_debug"}
```

Response:

```json
{
  "ok": true,
  "calibration_path": "<calibration-file>",
  "calibration_limits": true,
  "head": {
    "head_pan": {
      "xlerobot_motor": "head_motor_2",
      "id": 8,
      "range_min": 594.0,
      "range_max": 3444.0,
      "range_width": 2850.0,
      "calibration_loaded": true
    },
    "head_tilt": {
      "xlerobot_motor": "head_motor_1",
      "id": 7,
      "range_min": 888.0,
      "range_max": 2915.0,
      "range_width": 2027.0,
      "calibration_loaded": true
    }
  }
}
```

Use this response to set UI slider limits and to pre-clamp customer commands.
The robot server still performs its own final clamp before writing to the
motors.

### Other RPC Operations

| Request | Response fields |
| --- | --- |
| `{"op": "topic_list"}` | `ok`, `topics` |
| `{"op": "fleet_info"}` | `ok`, `num_robots`, `command_schema`, `action_dim_per_robot`, `base_model` |
| `{"op": "joint_names"}` | `ok`, `joint_pos_order`, `joint_vel_order` |
| `{"op": "command_status"}` | `ok`, `last_command`, `last_command_source`, `last_command_age_ms`, `last_rejected_reason`, counters |
| `{"op": "head_debug"}` | `ok`, `calibration_path`, `calibration_limits`, `head` |
| `{"op": "request_rescan", "force": true}` | Requests a motor-bus rescan without commanding motion; returns `ok`, `requested`, `health` |
| `{"op": "stop"}` | Sends zero base twist and returns `ok` |
| `{"op": "set_estop", "enabled": true}` | Enables/disables software e-stop and returns `ok`, `estop` |

Errors are returned as:

```json
{"ok": false, "error": "unsupported op 'example'"}
```

## Python Example

```python
import msgpack
import zmq
import time

robot_host = "<robot-host>"
ctx = zmq.Context.instance()

# Health check.
req = ctx.socket(zmq.REQ)
req.connect(f"tcp://{robot_host}:8857")
req.send(msgpack.packb({"op": "health"}, use_bin_type=True))
print(msgpack.unpackb(req.recv(), raw=False))

# Send a short forward command, then stop.
push = ctx.socket(zmq.PUSH)
push.setsockopt(zmq.LINGER, 0)
push.setsockopt(zmq.SNDHWM, 1)
push.connect(f"tcp://{robot_host}:8856")

for seq in range(10):
    push.send(msgpack.packb({
        "schema": "xlerobot_v1.1",
        "source_id": "example",
        "seq": seq,
        "stamp_ns": time.time_ns(),
        "frame": "body",
        "base_cmd_vel": [0.05, 0.0, 0.0],
    }, use_bin_type=True))
    time.sleep(0.02)

push.send(msgpack.packb({
    "schema": "xlerobot_v1.1",
    "source_id": "example",
    "stamp_ns": time.time_ns(),
    "frame": "body",
    "base_cmd_vel": [0.0, 0.0, 0.0],
}, use_bin_type=True))
```

## Validation Client

The repository includes a CLI client:

```bash
# Health and topic list.
./tools/fast_robot_client.py --host <robot-host> health
./tools/fast_robot_client.py --host <robot-host> topics

# Watch all fast sensor topics for 5 seconds.
./tools/fast_robot_client.py --host <robot-host> watch --duration 5

# Request a motor-bus rescan without moving the robot.
./tools/fast_robot_client.py --host <robot-host> rescan

# Safe short move; the client sends stop at the end.
./tools/fast_robot_client.py --host <robot-host> move --vx 0.05 --duration 0.5 --rate-hz 60

# Stop now.
./tools/fast_robot_client.py --host <robot-host> stop

# Software e-stop gate.
./tools/fast_robot_client.py --host <robot-host> estop true
./tools/fast_robot_client.py --host <robot-host> estop false
```

## Webview

The webview uses the fast protocol by default and does not need rosbridge:

```bash
./tools/robot_webview.py \
  --host 0.0.0.0 \
  --port 8765 \
  --fast-zmq-host 127.0.0.1
```

If the webview runs on a different computer, set `--fast-zmq-host` to the robot
IP address. Legacy rosbridge monitoring can be enabled explicitly with:

```bash
./tools/robot_webview.py --rosbridge-monitor
```

## Security Notes

The fast command port is unauthenticated. Do not expose ports `8855`, `8856`,
or `8857` to the public internet. Use a trusted LAN, VPN, firewall rules, or SSH
tunnel. Any client that can reach port `8856` can command the robot.

## Troubleshooting

- `health.base_attached` is `false`: the robot I/O process is up, but the motor
  base is not attached yet.
- `health.estop` is `true`: clear it with `{"op": "set_estop", "enabled": false}`
  before sending nonzero base velocity.
- `last_rejected_reason` is not null: inspect `command_status` for the latest
  command validation failure.
- Sensor ages are large: the corresponding hardware publisher is stale or not
  enabled.
- `scan.<id>` is missing: lidar may be disconnected, not rotating, or disabled.
- No reply from RPC: verify the robot process is running and that port `8857`
  is reachable.
