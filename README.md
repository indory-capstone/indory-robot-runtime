# indory-robot-runtime

`indory-robot-runtime` is the Raspberry Pi side runtime for the Indory indoor
delivery robot. It owns the hardware devices attached to the robot and exposes a
small fast ZMQ interface to the compute PC and client runtimes.

The Pi stays lightweight: it reads motors, lidar, and cameras, applies incoming
commands, and publishes state. ROS 2, SLAM, Nav2, web control, and task logic
run on the compute PC in `indory-control-server`.

## Role In The System

| Repository | Role |
| --- | --- |
| `indory-control-server` | Compute-side ROS, web, and task orchestration |
| `indory-robot-runtime` | Pi hardware runtime and fast ZMQ interface |
| `indory-vla-runtime` | LeRobot/VLA client and manipulation runtime |
| `indory-perception-server` | OCR and visual perception service |

## Interfaces

The runtime exposes these default ports:

```text
8855  PUB      robot state: scan, odom, joint state, TF, proprioception
8856  PULL     robot commands: base velocity and joint targets
8857  REP      health, topic list, command status, stop/e-stop
8866  PUB      RGB camera streams
8867  PUB      optional RGB-D stream
```

The command port is not intended for the public internet. Run it on a trusted
LAN, VPN, or SSH tunnel.

## Main Directories

```text
robot_io/       fast ZMQ hardware runtime
robot/          robot environment examples and checks
scripts/        Pi service and live-stack launch helpers
tools/          protocol clients, camera utilities, IK/debug tools
docs/           protocol and camera transport details
src/            optional ROS 2 bridge package for compatible environments
```

## Quick Start

Install Python dependencies in the robot runtime environment, then run:

```bash
./run_xlerobot_rosbridge_io.sh
```

Useful environment variables:

```text
FAST_ZMQ_BIND_HOST=0.0.0.0
FAST_ZMQ_PUB_PORT=8855
FAST_ZMQ_PULL_PORT=8856
FAST_ZMQ_REP_PORT=8857
ENABLE_LIDAR=1
ENABLE_CAMERA=1
```

Health check from another terminal:

```bash
python3 tools/fast_robot_client.py --host <robot-ip> health
```

## Hardware Scope

This repository contains the runtime code and interface definitions only. It
does not include robot calibration files, model weights, captured camera data,
or private network configuration.

## License

Add a project license before publishing this repository.
