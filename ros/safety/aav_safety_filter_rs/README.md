# aav_safety_filter_rs

## Overview

The **safety filter** for the automatic drivers (TODO 104; the design "How the car stops",
its filter and K1). An untrusted driver (navigation, the point driver, the AI or LLM slot)
sends `(v, w)`; the filter sends on the **nearest** command that keeps three things true.
**In shadow by default**: it drives nothing until a launch file wires it in.

| Kept | How | Hard? |
|---|---|---|
| the car can still stop before the road ahead's end | a control barrier function (CBF) on the stopping law `S(v)` | yes |
| the car stays inside its lane | CBFs on a look-ahead point, `p = e_y + ℓ·sin e_ψ` | yes, while the wheels can reach |
| the car settles on the lane's line | a Lyapunov function `V = ½·p²`, with a slack | no: a pull |

It only ever slows the car or bends its turn: never faster, never a stop into motion,
never a direction reversed. Each step is a minimum-change problem in one unknown, so each
answer is a formula: no solver, the same answer every time.

```text
shadow (default):  /drivers/<d>/cmd_vel ──► safety_filter ──► /safety/filter/shadow/<d>/cmd_vel
live (K1):         /drivers/<d>/cmd_vel_raw ──► safety_filter ──► /drivers/<d>/cmd_vel ──► twist_mux
                                                    │
                                                    └──► /safety/filter/status (every number, every command)
```

**Where it sits (K1: the relay the design proposed; still open, Ben's to decide).** In front of
twist_mux, on the automatic drivers' topics only. A person on the joystick, RC, keyboard
or the buttons never goes through it, by construction; inside ego it would need a "who is
driving" signal to be right and on time, or it would trim the safety driver.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/safety-filter` | the maths: `lane` (the car in the road ahead's frame), `filter` (the filter and its report), `sim` (a closed loop with a lagging car) | no: `cargo test` on any laptop |
| `crates/safety-filter-node` | the node, binary `safety_filter` | yes (rclrs 0.8, `ros-env` 0.3) |

It uses ego-core's `StopCurve`: one stopping law for ego, the behaviour planner and the filter.

### Node `safety_filter`

| In | Type | Use |
|---|---|---|
| `in_topic` per driver | `geometry_msgs/TwistStamped` | the driver's command |
| `/world/road_ahead` | `aav_interfaces/RoadAhead` | the line, its soft edges, the end |
| `/localization/odom/global` | `nav_msgs/Odometry` | the pose (map) |
| `/vehicle/odom` | `nav_msgs/Odometry` | the measured speed |

| Out | Type | Use |
|---|---|---|
| `out_topic` per driver | `geometry_msgs/TwistStamped` | the filtered command, the driver's header kept |
| `/safety/filter/status` | `diagnostic_msgs/DiagnosticStatus` | one per command: the driver, the rules that acted, every number |

| Parameter | Default | Means |
|---|---|---|
| `drivers` | `[nav, point, ai]` | whose commands it filters |
| `in_topic`, `out_topic` | `/drivers/{d}/cmd_vel`, `/safety/filter/shadow/{d}/cmd_vel` | `{d}`: the driver's name |
| `road_timeout`, `pose_timeout`, `speed_timeout` | 0.5 s | older: no road (no forward motion), or the last output stands in for the speed |
| `brake`, `jerk`, `delay` | 1.0 m/s², 2.0 m/s³, 0.35 s | the stopping law (the design's starting numbers; TODO: from vehicle.yaml as ego builds it) |
| `margin` | 0.5 m | kept clear past the stopping distance |
| `max_offset` | 3.0 m | further off the line, the road is lost: no forward motion |
| `alpha_speed`, `alpha_lane` | 1.0 1/s | how fast each barrier may close |
| `look_ahead` | 1.5 m | the look-ahead point's distance ahead of `base_link` |
| `half_width`, `lane_margin` | 0.5 m, 0.2 m | half the car (vehicle.yaml's body) and what is kept inside each lane line |
| `clf_rate`, `clf_weight` | 0.5 1/s, 1.0 | the Lyapunov term's decay and strength (0: off) |
| `wheelbase`, `max_steer` | 1.561 m, 0.5 rad | vehicle.yaml |
| `max_speed`, `max_reverse` | 1.0, 0.5 m/s | caps |

The lane barriers need the lane map's soft edges; without them (NaN) only the speed
barrier and the wheels' reach act. Backwards, only the caps act: the road ahead says
nothing behind the car (as ego).

## Invariants

- Never faster, never reversed: `|v_out| ≤ |v_in|`, the same sign; a stop stays a stop.
- No fresh road or pose, a car more than `max_offset` off its line, or a command that
  isn't a number: no forward motion.
- The wheels' reach (`|w| ≤ v·tan δ_max / L`) is physics: when the lane can't be kept
  within it, the reach wins, the turn goes as near the lane as it allows, and the car slows.
- Never in a person's path (K1). Not a lock: locks still stop the car; this only trims.
- `safety-filter` has no ROS. Rust API Guidelines: enums (`Rule`), `Result`
  (`SettingsError`), no `unwrap` in library code; `cargo fmt` and `cargo clippy -D
  warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## What the guarantee covers, and doesn't

The barriers are proven for the model inside the filter: the kinematic car in the lane's
frame, at the filter's rate. The real car lags; the margins absorb it, and
`tests/guarantees.rs` checks they do for a lagging car (speed lag 0.3 s, steering 0.8 rad/s).
It does **not** cover: moving obstacles (the road ahead's end must already allow for them),
reversing, a road ahead that is wrong, or a car whose steering can't follow (the
infeasible case slows it, it doesn't prove it).

## Testing

```sh
cd ros/safety/aav_safety_filter_rs && cargo test --locked -p safety-filter   # no ROS; tools/test.sh
cargo run --release -p safety-filter --example ablation                       # the table below
cargo run --release -p safety-filter --example ablation -- --csv              # one row per run
pixi run build && colcon test --packages-select aav_safety_filter_rs
```

| Test | Covers |
|---|---|
| `filter` and `lane` unit tests | each rule's own cases; the frame on lines and circles, past the end |
| `tests/guarantees.rs` (proptest, 400 cases each) | never faster or reversed; any hostile driver, on a straight road and a bend, never passes the end nor puts the body over a lane line, with a lagging car; the same drivers do both without the filter |
| `examples/ablation.rs` | the filter's parts taken away one at a time, on the same 300 hostile drivers |

The ablation (20 Nov; a 30 m bend of radius 15 m, lane lines 1.5 m from the line, 40 s a
run, the filter at 20 Hz, the car lagging):

| Variant | Runs past the end | Runs over a line | Mean progress (m) | Mean \|offset\| (m) |
|---|---|---|---|---|
| open (no filter) | 24/300 | 299/300 | 11.3 | 6.53 |
| speed barrier only | 0/300 | 299/300 | 6.8 | 3.12 |
| speed + lane barriers | 0/300 | 0/300 | 28.0 | 0.640 |
| full (+ Lyapunov) | 0/300 | 0/300 | 28.1 | 0.619 |

The barriers make it safe; the Lyapunov term only tightens the line a little (3 % less
offset). Two findings on the way: the speed barrier first trusted a closing speed of `v`,
which is wrong off the line on the inside of a bend (`closing` now uses
`v·cos e_ψ / (1 − κ·e_y)`); and a car far off its line makes the frame meaningless (the
nearest point jumps), hence `max_offset`.

Live, the node in shadow on a pretend road (20 Nov, `ROS_DOMAIN_ID` of its own): plenty
of room passed unchanged; 2 m/s near the end became 0 (`max_speed speed_barrier`); a hard
left at the left line became straight ahead (`lyapunov lane_left`). **Not yet run in
Ravensim or on the car.**

### Logging real drives

Record the filter's status with everything it judged from, and a drive can be scored
afterwards without the car:

```sh
ros2 bag record /safety/filter/status /drivers/nav/cmd_vel /drivers/point/cmd_vel \
  /drivers/ai/cmd_vel /safety/filter/shadow/nav/cmd_vel /safety/filter/shadow/point/cmd_vel \
  /safety/filter/shadow/ai/cmd_vel /world/road_ahead /localization/odom/global /vehicle/odom \
  /ego/actuator_cmd /control/cmd_vel
```

Then score it: what was trimmed and why, the closest call on each barrier, and every stop
the filter called against the metres the car really took (its `stop_m`, S(v), against the
wheel speed integrated). Predicted above measured: the stopping law is cautious; below: it
trusts too much room, and `brake`, `jerk` or `delay` need the step tests' numbers.

```sh
pixi run tools/ros.sh python3 tools/safety_filter_report.py bags/<date>_<name>
pixi run tools/ros.sh python3 tools/safety_filter_report.py bags/<date>_<name> --csv
```

The shadow ladder: unit and property tests → Ravensim in shadow → bench → the car in
shadow with a safety driver → live on one driver (the AI slot first).
