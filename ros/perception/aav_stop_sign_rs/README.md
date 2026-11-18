# aav_stop_sign_rs

## Overview

`stop_sign_watcher` in Rust (rclrs): **the default since 17 Nov** (Ben: "I don't think we
want to do this again later"), in place of `aav_detector`'s Python node. The same node
name, topics, parameters and answers. `stop_sign_impl:=python` falls back.

```
detector ─► /perception/detections ─► stop_sign_watcher ─► /perception/stop_sign  (twist_mux's lock, 30)
CameraInfo (each lens) ─────────────────────┘          └─► /perception/stop_sign/distance
```

A sign counts once its box says it is `stop_distance_m` (4.7 m) away or closer
(`focal_px x sign_height_m / box height`, each camera's own lens). Then the lock holds
navigation, the point driver and the AI slot for `hold_s`, and the sign is ignored for
`ignore_s` so the car can drive past. The rules: `aav_detector`'s README.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/stop-sign-core` | `StopRule`, `StopSignWatcher` (`State`: `Clear`, `Stopping`, `Passing`), `Sighting`, `sign_distance_m`, `sign_point`, `biggest_sign`, `lidar_sign`, `normal_label`, `is_optical`, `facing_forward` | no: `cargo test` on any laptop |
| `crates/stop-sign-node` | the process, binary `stop_sign_watcher`; with `lidar_distance`, `aav_behaviour_rs`'s TF buffer (reused) | yes (rclrs 0.8, `ros-env` 0.3) |

### Node `stop_sign_watcher`

| In / out | Topic | Type |
|---|---|---|
| in | `/perception/detections` | `vision_msgs/Detection2DArray` |
| in | `camera_info_topics` (front_left, front_right, rear) | `sensor_msgs/CameraInfo` |
| in | `/world/things`, `/tf`, `/tf_static` (only with `lidar_distance`) | `aav_interfaces/Things`, TF |
| out | `/perception/stop_sign`, 10 Hz and with each detection | `std_msgs/Bool` |
| out | `/perception/stop_sign/distance`, while CLEAR | `geometry_msgs/PointStamped`, the camera's frame |

Parameters: `stop_distance_m` 4.7, `focal_px` 343, `sign_height_m` 0.75, `min_score`
0.5, `hold_s` 4, `ignore_s` 10, `forward_words` [front, left, right], `backward_words`
[rear], `camera_info_topics`, `lidar_distance` false, `lidar_gate_m` 3.0.

### Which one runs

`stop_sign_impl:=rust|python` (`vehicle.STOP_SIGN_IMPLS`, default `rust`):
`car.launch.py` (the `stop_signs` part), `sim.launch.py` and `bench.launch.py` (with
`perception:=true`). Not in `stacks.yaml`: the same algorithm in another language.

## Invariants

- The same answers as `stop_sign.py`: `tests/vectors.rs` reads `vectors.json`, which
  `gen_vectors.py` writes from it (2,400 watcher steps under four rules, 200 sign points,
  150 biggest signs, 150 lidar matches, labels, frames, distances); states and booleans
  the same, numbers within 1e-12. `aav_detector`'s `test_stop_sign_vectors.py` fails if
  the Python drifts from the file.
- Only ever stops the car: the lock is twist_mux priority 30, never a person or a test.
  It never expires on silence.
- `stop-sign-core` has no ROS; no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## Where it differs from the Python

| Python | Rust |
|---|---|
| tf2's buffer (`lidar_distance` only) | `aav_behaviour_rs`'s TF buffer: the same lookups at the picture's stamp, interpolated; no wait for a late transform (the Python's timeout is 0 too) |
| log lines with `:g` numbers | the same words, Rust's own number format |

## Side Effects

- Publishes the two topics above; logs each change of state and each new lens. Stops
  cleanly on SIGINT, SIGTERM or SIGHUP.

## Verification

```sh
cd ros/perception/aav_stop_sign_rs && cargo test --locked -p stop-sign-core   # no ROS; tools/test.sh
python3 ros/perception/aav_stop_sign_rs/gen_vectors.py                       # after changing stop_sign.py
pixi run build && colcon test --packages-select aav_stop_sign_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `stop-sign-core` unit tests | the hold and the pass, labels and frames, the default height |
| `tests/vectors.rs` | the Python's answers |
| `test/test_stop_sign_node.py` | the process, each language: the sign's point, the lock on, off after `hold_s` while the sign is still seen, a clean Ctrl-C |
