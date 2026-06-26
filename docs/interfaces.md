# Runtime Interfaces

The robot runtime exposes a small ZMQ interface from the Raspberry Pi.

## State

```text
tcp://<robot-host>:8855
ZMQ PUB/SUB
```

Topics:

```text
joint_states.0
proprio.0
odom.0
scan.0
tf.links.0
```

## Commands

```text
tcp://<robot-host>:8856
ZMQ PUSH/PULL
```

Command types:

```text
base_cmd_vel
joint_targets
joint_targets_sparse
```

## RPC

```text
tcp://<robot-host>:8857
ZMQ REQ/REP
```

Supported operations include:

```text
health
topic_list
command_status
stop
estop
```

## Cameras

```text
tcp://<robot-host>:8866  RGB streams
tcp://<robot-host>:8867  optional RGB-D stream
```

The compute-side camera bridge in `indory-control-server` can mirror these
streams into browser and ROS-facing transports.
