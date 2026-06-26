# Indoory XLeRobot Protocol

The customer-facing fast ZMQ protocol is documented in
[docs/fast_protocol.md](docs/fast_protocol.md).

For an outside-client summary of every live robot action, RGB, RGB-D, lidar,
RPC, and optional streaming interface, start with
[docs/external_interface_summary.md](docs/external_interface_summary.md).

Head camera motor commands, sparse raw tick targets, and XLeRobot calibration
limit behavior are documented under `Joint Targets` and `Joint Limit and
Calibration Behavior`.

## External Camera Streams

The external camera and RGB-D ZMQ contract is documented in
[docs/external_zmq_streams.md](docs/external_zmq_streams.md), with the full ZMQ
interface reference in [docs/zmq_interfaces.md](docs/zmq_interfaces.md).

Current live camera endpoints:

| Endpoint | Topics | Payload |
| --- | --- | --- |
| `tcp://<robot-ip>:8866` | `rgb.front.0`, `rgb.wrist_left.0`, `rgb.wrist_right.0`, `rgb.floor.0` | Four RGB camera streams |
| `tcp://<robot-ip>:8867` | `rgbd.front.0` | Self-contained `jpeg+depth` RGB-D packet |

`rgbd.front.0` includes both `color_data` JPEG bytes and zstd-compressed
uint16 `depth_data` in the same MessagePack payload. Do not switch RGB-D
format or add a new camera transport without updating
[docs/camera_format_benchmarks.md](docs/camera_format_benchmarks.md) from a
fresh benchmark run.

Lossless RGB video is currently documented as an opt-in debug/recording
candidate, not the live default. Depth is already transported losslessly as
zstd-compressed `16UC1`; RGB lossless wrappers such as raw BGR, PNG, zstd raw,
or FFV1 are substantially heavier than the current JPEG wrist/floor streams.
