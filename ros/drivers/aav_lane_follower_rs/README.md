# aav_lane_follower_rs

## Overview

`control_interface` in Rust (rclrs): the lane follower on twist_mux's AI slot, a
**drop-in** for the 2025-26 team's Python node in
[aav_lane_geometry](../../perception/aav_lane_geometry/README.md). The same node name,
topics, QoS, parameters, states and commands. Ben, 19 Nov: "We also need to port the lane
following code.or find a better version."

```
/perception/lane/geometry ─► control_interface ─► /drivers/ai/cmd_vel (TwistStamped, priority 10)
   (lane_finder: offset,       │  five states         /drivers/ai/lane_status, lane_state
    the lines' fits in m)      │  + a steering law:
                               └─ pid (2025-26) · pure_pursuit · stanley
```

**The default** wherever lane keeping runs (`lane:=true|fake` in the sim and on the bench,
the car's `lane` part); `lane_follower_impl:=python` runs the Python one instead.
`lane_follower:=` picks how it steers; the PID stays the default (see "Scores").

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/lane-follow-core` | `Follower` (the five states), `Law` (the PID, pure pursuit, Stanley), `Geometry` (the lane centre from the fits), `fixed` (numbers as Python writes them) | no: `cargo test` on any laptop |
| `crates/lane-follow-node` | the node, binary `control_interface` | yes (rclrs 0.8, `ros-env` 0.3) |

### Node `control_interface`

| In / out | Topic | Type |
|---|---|---|
| in | `/perception/lane/geometry` | `aav_lane_geometry/LaneGeometry`, reliable, depth 10 |
| out | `/drivers/ai/cmd_vel` | `geometry_msgs/TwistStamped`, `base_link` |
| out | `/drivers/ai/lane_status` | `std_msgs/String`: the state and numbers, as the Python writes them |
| out | `/drivers/ai/lane_state` | `std_msgs/String`: the state, once per lane message |

A 0.1 s timer on the node's clock is the watchdog: no lane message for `stop_wait_s`, and
the car slows at `decel_rate` and stops.

### Parameters

The Python's, with its names and defaults (`aav_lane_geometry`'s README): `base_speed_mps`,
`max_angular_z`, `min_speed_mps`, `min_curvature_m`, `lane_change_*`, `stop_wait_s`,
`decel_rate`, `accel_rate`, `pid_kp`, `pid_ki`, `pid_kd`, `invert_offset_sign`. Ours:

| Parameter | Default | Means |
|---|---|---|
| `steering` | `pid` | `pid`, `pure_pursuit` or `stanley` (launch: `lane_follower:=`) |
| `lag_s` | 0.2 s (launch: from `vehicle.yaml`) | the picture to the wheels: `LANE_CAMERA_DELAY` (0.1 s) + `steer_dead_time` + `steer_lag` |
| `lookahead_m`, `lookahead_gain_s` | 3 m, 1 s | pure pursuit looks `lookahead_m` + `lookahead_gain_s` x speed ahead ... |
| `lookahead_min_m`, `lookahead_max_m` | 4 m, 10 m | ... within these (the camera's fit covers 4-16 m) |
| `stanley_gain`, `stanley_soft_speed` | 1 /s, 1 m/s | Stanley's cross-track gain, and its softening at low speed |
| `wheelbase`, `max_steering_angle` | 1.561 m, 0.5 rad (launch: `vehicle.yaml`) | Stanley's front axle and full lock |

The launch file is `aav_lane_geometry`'s `lane_follower.launch.py`
(`lane_follower_impl:=rust|python`, `lane_follower:=pid|pure_pursuit|stanley`).

## The challengers, and why these two

The 2025-26 PID steers on one number: the offset 3 m ahead. It ignores the lines' shape,
the speed and the car's lag, and its D term divides by the wall clock's dt between
pictures, so the turn it asks for jumps with the camera's noise.

The camera gives more: each line as y = a x² + b x + c in metres. So the challengers steer
on **the lane's centre line**, the mean of the two fits (one line alone: moved half a lane
across).

| Option | How it steers | Kept? |
|---|---|---|
| **Pure pursuit** (Coulter 1992) | the arc from the car to the centre line a look-ahead on; the look-ahead grows with speed | **built** (`pure_pursuit`): one gain, the look-ahead; geometric, so it is smooth and needs no dt; robust to the fit's noise, because a point 4-10 m ahead is where the fit is best |
| **Stanley** (Thrun et al. 2006) | the lane's heading plus atan(k e / v) for the cross-track error e at the front axle | **built** (`stanley`): the classic other answer, good at low speed; but its point (the front axle, 1.6 m ahead) is outside the camera's view (4 m on), so it leans on the fit's extrapolation |
| Curvature feed-forward + feedback on offset and heading | 2a from the fit, plus gains on c and b | not built: on our fits the three gains (and a) are noisier than one look-ahead point, and pure pursuit already follows the curvature |
| MPC on the lane | as `mppi_lag`'s `LaneCritic`, on the lane map | not here: that is navigation's follower on the map (TODO 169); this one is the camera-only driver |

**The car's lag.** Both challengers steer from **where the car will be** when the command
takes effect, not where the picture was taken: the car is moved on by `lag_s` at the
commanded speed, turning at the last command's rate, and the target is looked at from
there. `lag_s` comes from `vehicle.yaml` (the steering's dead time and lag, plus the
camera's 0.1 s). The 2025-26 PID has no idea of the lag, which is one reason it weaves.

Both run inside the same five states as the PID (lost lines, the watchdog, lane changes),
so only the steering differs. They need `lane_finder`'s fits in metres: `lane:=fake`'s
fake publisher writes pixels, so use the PID with it.

## Scores

Not yet scored: see TODO 262.

## How it matches the Python

`gen_vectors.py` runs the Python's `lane_follow.py` (the logic `control_interface.py` runs,
with no ROS) on five runs of made-up lane messages and ticks, and writes
`crates/lane-follow-core/tests/vectors.json`; `tests/vectors.rs` runs the Rust on the same
events.

| Runs | What |
|---|---|
| events | a lane kept with a jittering offset, drifts both ways, a lane change and its commit, one line, tight bends, lines lost and found, messages that stop for 0.25, 0.45 and 2 s, ties in the status line's rounding |
| clocks | the wall clock, and a node clock at 0.37, 0.8, 1 and 1.6 times its rate (a slow simulator) |
| settings | the defaults; other speeds, gains, `invert_offset_sign` false, a short `stop_wait_s`, a quick lane-change timeout, `decel_rate` 0 |

| What must agree | Allowed | Seen |
|---|---|---|
| each command's speed and turn | 1e-12 | the same |
| the status line, the state, each transition, the watchdog's trip | the same text | the same |

2,797 events, 1,364 commands. `aav_lane_geometry`'s `test_lane_follow.py` checks the vectors
are still the Python's answers. `test/test_control_interface_node.py` runs both nodes
(and the Rust challengers) on the same lane messages, then stops them: the same states,
status lines, steering side and slow-down.

## Invariants

- The same answers as `lane_follow.py` with `steering:=pid`: change one, change both, then
  `pixi run python ros/drivers/aav_lane_follower_rs/gen_vectors.py`.
- The same node name and topics as the Python: only one runs (`lane_follower_impl:=`).
- Drives only twist_mux's AI slot (priority 10): every other driver outranks it.
- `lane-follow-core` has no ROS; `cargo fmt` and `cargo clippy -D warnings` clean.
- REP-103: x forward, y left, metres; + offset = the car left of the centre; + turn = left.

## Side Effects

- Commands the car through twist_mux (the lowest priority).

## Verification

```sh
cd ros/drivers/aav_lane_follower_rs && pixi run cargo test -p lane-follow-core
pixi run build && colcon test --packages-select aav_lane_follower_rs aav_lane_geometry
pixi run measure lane --follower pure_pursuit     # a drive in Ravensim, scored
```
