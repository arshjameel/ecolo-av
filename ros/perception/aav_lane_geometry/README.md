# aav_lane_geometry

## Overview

Lane keeping, from the 2025-26 team (`Lane_Detection_ROS2`; see `PROVENANCE.md`). It holds:

- the `LaneGeometry` message, which [aav_lane_finder](../aav_lane_finder/README.md)
  publishes on `/perception/lane/geometry`;
- `control_interface`: the steering half. A PID on the lane offset, as a driver on
  twist_mux's AI slot (`/drivers/ai/cmd_vel`, priority 10). Its logic is
  `scripts/lane_follow.py` (no ROS); the node is the ROS around it;
- a fake lane publisher and a CSV logger, for tests without a camera.

Every other driver outranks the AI slot, so a person, a test script or navigation always wins.

**Its Rust twin runs by default** since 19 Nov:
[aav_lane_follower_rs](../../drivers/aav_lane_follower_rs/README.md), the same node, topics,
parameters and commands, with two challengers beside the PID (`lane_follower:=pure_pursuit`,
`stanley`). `lane_follower_impl:=python` runs this package's node instead.

## Interface

### Message `msg/LaneGeometry.msg`

| Field | Type | Means |
|---|---|---|
| `header` | `std_msgs/Header` | the picture's stamp; `base_link` from lane_finder |
| `left_curvature_m`, `right_curvature_m`, `mean_curvature_m` | float32 | radius, m (9999 = straight or not seen) |
| `curve_direction` | float32 | which way the road bends; 0 = straight |
| `vehicle_offset_m` | float32 | m from the lane's centre, + = car left of it |
| `lane_width_m` | float32 | m |
| `left_poly_a/b/c`, `right_poly_a/b/c` | float32 | y = a x² + b x + c (m from lane_finder; pixels from the fake) |
| `left_confidence`, `right_confidence` | float32 | 0 to 1 |
| `left_lane_detected`, `right_lane_detected`, `both_lanes_detected` | bool | |
| `steering_hint` | float32 | |

### Nodes (in `scripts/`, run with `python3`)

| Node | In | Out |
|---|---|---|
| `control_interface.py` | `/perception/lane/geometry` | `/drivers/ai/cmd_vel` (`TwistStamped`, `base_link`), `/drivers/ai/lane_status` (`String`), `/drivers/ai/lane_state` (`String`) |
| `fake_lane_geometry_pub.py` | | `/perception/lane/geometry`, at `rate_hz` |

### control_interface parameters

| Parameter | Default | Means |
|---|---|---|
| `base_speed_mps`, `min_speed_mps` | 2.0, 0.3 m/s | cruise and floor speed |
| `max_angular_z` | 0.5 rad/s | steering clamp |
| `min_curvature_m` | 20 m | tighter bends slow the car (down to 0.4 x) |
| `lane_change_offset_threshold`, `lane_change_rate_threshold` | 0.40 m, 0.05 m/s | both exceeded: LANE_CHANGING |
| `lane_change_speed`, `lane_change_timeout_s` | 1.0 m/s, 8 s | |
| `decel_rate`, `accel_rate` | 0.5, 0.3 m/s² | |
| `stop_wait_s` | 0.3 s | no lane message this long: slow down. Keep it below twist_mux's 0.5 s for the AI slot |
| `pid_kp`, `pid_ki`, `pid_kd` | 0.8, 0.01, 0.3 | |
| `invert_offset_sign` | true | error = -offset, so + offset steers right |

States: `LANE_KEEP`, `LANE_CHANGING`, `LANE_COMMITTED`, `NO_LANE_STOPPING`, `RESUMING`.

Fake modes (`mode`): `straight`, `drift_right`, `drift_left`, `sine`, `sharp_curve`,
`lane_change_right`, `lane_change_left`, `lane_loss`. Fake parameters: `mode`, `rate_hz`
(10 Hz), `lane_width_m` (3.6 m).

### Launch

| File | Starts |
|---|---|
| `launch/lane_follower.launch.py` | control_interface, Rust or Python (`lane_follower_impl:=rust`), steering by `lane_follower:=pid` (or the Rust `pure_pursuit`, `stanley`); `fake:=true` adds the fake lane |

## Invariants

- Drivers publish `TwistStamped` on `/drivers/<driver>/cmd_vel`: this one only on the AI slot.
- No lines seen, or no message for `stop_wait_s`: the car slows at `decel_rate` and
  stops (`NO_LANE_STOPPING`). The slow-down runs on a 0.1 s timer, so it goes on when
  the messages stop. The rule is `scripts/lane_speed.py` (no ROS).
- `scripts/lane_follow.py` (no ROS) holds the logic; `aav_lane_follower_rs` must give the
  same answers: after changing it, run that package's `gen_vectors.py`.
- It never starts a slow-down from speed once its own commands have been quiet for
  twist_mux's 0.5 s: by then ego is already stopping the car, and the slow-down would
  pull away again. It sends 0 instead.
- The watchdog's times are on the node's clock: the simulator's in `pixi run sim`, as
  twist_mux's and ego's are.
- Lane width, offsets and radii are metres; turn rate is rad/s, + = left.
- The scripts should run through `python3`, not their `#!` line: on a Mac, `/usr/bin/env`
  drops the library path their messages need (`docs/mac.md`). The launch file does this
  (`prefix='python3'`).
- Inherited code: record every change in `PROVENANCE.md`.

## Side Effects

- control_interface commands the car through twist_mux (lowest priority).

## Verification

| Test | Covers |
|---|---|
| `test/test_lane_speed.py` | the slow-down, its stop, and that `stop_wait_s` and the timeout match twist_mux's |
| `test/test_lane_follow.py` | the states (steering, one line, a lane change, lost lines, the watchdog's clock), and that `aav_lane_follower_rs`'s vectors are still these answers |

Quick, without ROS: `cd ros/perception/aav_lane_geometry && python3 -m pytest -q`
(`tools/test.sh` runs it).

```sh
pixi run build && colcon test --packages-select aav_lane_geometry && colcon test-result --verbose
ros2 launch aav_lane_geometry lane_follower.launch.py fake:=true   # watch /drivers/ai/cmd_vel
```

`pixi run test` runs everything; `pixi run sim-check --lane` drives it in Ravensim.

## Notes

- `package.xml` is still the 2025-26 one: maintainer `OpenAI`, `user@example.com`, MIT.
- `fake_lane_geometry_pub.py` lacks the module docstring listing topics that `CLAUDE.md`
  asks for.
- After lost lines, RESUMING doesn't ramp the speed: the lane-keeping command overwrites the
  ramp (2025-26). Kept, in both languages, for now.
