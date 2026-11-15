# aav_mpc_lag

The **lag-aware MPC in Rust**: `aav_track`'s `mpc_lag` (Python), ported step for step,
as a challenger beside it (TODO 132, 199). It plans the steering and the set speed
through the car's own lags. The Python one stays the default `mpc_lag`; no stack picks
this one yet (`docs/trials.md`, "Lag-aware MPC").

```
bt_navigator --FollowPath--> mpc_lag_rs --cmd_vel_nav--> velocity_smoother
   -> collision_monitor -> /drivers/nav/cmd_vel (twist_mux 20) -> ego
controller_server keeps running; its own action is renamed follow_path_unused
```

## Overview

| Part | What it is |
|---|---|
| `crates/mpc-lag` | the maths, no ROS: the path, the car model in the path's frame, the QP, the tracker |
| `crates/mpc-node` | the ROS node, binary `mpc_lag_rs` (rclrs): messages in and out only |
| `config/mpc_lag_rs.yaml` | its own settings: the same gains as `aav_track`'s `track.yaml` `mpc_lag:` |

| | Python `mpc_lag` | Rust `mpc_lag_rs` |
|---|---|---|
| Maths | `lag_model.py`, `qp.py`, `trackers/mpc_lag.py` | the same, ported; held to the Python by `vectors.json` |
| Estimate | `car_model.step`, Python's speed loop (f64) | `car-model`'s step, spine-core's own speed loop (f32) |
| QP | OSQP's ADMM, numpy, dense | OSQP's ADMM, dense P, sparse A (`qp.rs`) |
| A tick on a Mac (U-turn at 2 m/s) | 5.3 ms mean, 26 ms worst | 0.75 ms mean, 2.4 ms worst |
| Lane keeping (`road_ahead_topic`, TODO 169) | yes, with `behaviour:=true` and a lane map | not yet |
| Live swap | yes (`live_swap`) | no: chosen at launch |

```sh
pixi run sim world:=test_lot reversing:=false tracker:=mpc_lag_rs
pixi run measure controllers --trackers mpc_lag_rs mpc_lag --tours lot
```

### Why our own QP, not clarabel

`clarabel` (Apache-2.0, pure Rust) was the first choice. We kept a port of `qp.py`'s ADMM:

| | ADMM (ours) | clarabel |
|---|---|---|
| Warm start from the last tick | yes: the multipliers | no (an interior point method) |
| Same answers as the Python | yes: the same iterations, to 1e-9 on the model, 1e-3 on the commands | no: another algorithm, other stopping |
| Size | 90 numbers, 180 rows: dense is small | built for big sparse problems |
| New dependencies | none | clarabel and its tree |

## Interface

**`mpc_lag_rs`** (`crates/mpc-node/src/main.rs`), started by `aav_navigation`'s
`navigation.launch.py tracker:=mpc_lag_rs` (forward only: refused with `reversing:=true`)

| Item | What |
|---|---|
| Action server | `/follow_path` (`nav2_msgs/action/FollowPath`): the newest path replaces the last without a stop |
| Subscribes | `/localization/odom/local` (Odometry); `/tf` (map -> odom: held, and moved toward the newest at most 0.1 m/s and 1°/s at the car; each new map path comes into odom by the held one and the running path moves with it, TODO 186) |
| Publishes | `/cmd_vel_nav` (TwistStamped: its set speed, and `v tan(steer) / L`); `/drivers/nav/track/status` (DiagnosticStatus: `aav_track`'s keys, `solve_ms`, `iterations`, `solved`, `wheels_est_rad`, `speed_limit_mps`, `feed_forward_rad`, `map_gap_m`); `/drivers/nav/lag_aware` (Bool, latched: true while a path runs) |

Results: succeeded; aborted with `FAILED_TO_MAKE_PROGRESS`, `INVALID_PATH`, `TF_ERROR`,
`UNKNOWN` (no odometry, or a QP that can't be solved); canceled. The goal and progress
checks are Nav2's (`mppi-lag`'s `GoalCheck`, `ProgressCheck`, the same as `checks.py`).

### Parameters

From `navigation.launch.py` (`vehicle.mpc_lag_rs_params`: `track_params` without the
lookahead, and vehicle.yaml's actuators; nothing copied):

| Name | From |
|---|---|
| `rate_hz`, `odom_topic`, `stamped`, goal and progress checks | controller_server's |
| `cruise_speed` | RPP's `desired_linear_vel`, or `max_speed:=` |
| `acceleration`, `deceleration` | the velocity smoother's |
| `max_brake`, `wheelbase`, `max_steering_angle`, `actuators.*` | vehicle.yaml (dead times 0 while unset) |
| `max_steering_rate` | the lower of vehicle.yaml's `steer_rate` and ego.yaml's |
| `map_correction` | the launch's switch (never in the yaml) |

From `config/mpc_lag_rs.yaml`: `lateral_accel`, `odom_timeout`, the map correction's rates,
and `mpc_lag.*` (`horizon`, `q`, `r`, `r_rate`, `q_speed`, `r_speed`, `latency`,
`speed_loop`; also `w_slack`, `terminal`, `substeps`, `max_iter`, `rho`, `sqp` at the
Python's defaults). `aav_navigation`'s `test_launch_params.py` checks they match
`track.yaml`'s.

### Crate `mpc-lag`

| Module | Holds | Python |
|---|---|---|
| `path.rs` | `Path`: the foot (searched forward from the last), the curvature, `wrap` | `path.py` |
| `model.rs` | `step`: the car with its lags in the path's frame; `PiLoop` (the spine's PI in f64) | `lag_model.py` |
| `qp.rs` | `solve`: OSQP's ADMM; `RowMatrix` (sparse rows) | `qp.py` |
| `tracker.rs` | `MpcLag`: the estimate, the nominal plan, linearise, condense, one QP, the command | `trackers/mpc_lag.py` |

## Invariants

- Forward only. Never both followers: with it, controller_server's action is `follow_path_unused`.
- The same maths as the Python: after changing `lag_model.py`, `mpc_lag.py` or `qp.py`,
  run `gen_vectors.py` and both test suites.
- Every steering command within full lock and the steering rate; every set speed within
  0..cruise and the smoother's ramps.
- Nothing in the safety path changes: the smoother, collision_monitor, twist_mux's slot,
  the locks and e-stops all sit after `cmd_vel_nav`. It doesn't read the costmap.
- The maths has no ROS; the node has no maths. Rust API Guidelines; `cargo fmt` and
  `cargo clippy` clean.

## Side Effects

- Publishes `/cmd_vel_nav` and its status only while a path runs. No files, no hardware.

## Verification

```sh
cd ros/drivers/aav_mpc_lag && cargo test --release -p mpc-lag      # tools/test.sh runs it
cargo test --release -p mpc-lag --test closed_loop -- --nocapture   # prints the tick time
pixi run python3 ros/drivers/aav_mpc_lag/crates/mpc-lag/gen_vectors.py   # after a Python change
```

| Test | Covers |
|---|---|
| `tests/vectors.rs` | the model against `lag_model.step`: 240 states, three cars, both PI loops (worst 4e-15); the tracker's commands against the Python's on the same observations, a bend and the U-turn at 2 m/s (worst 6e-5 rad, 1.2e-3 m/s) |
| `qp.rs` | hand answers: free, a box, an equality, a coupling row; the warm start (the same 400 and 50 iterations as `qp.py`); bad problems |
| `path.rs`, `model.rs`, `tracker.rs` | the foot past the ends, a circle's curvature; the motor's rate, the brake never reversing, the drift on a bend; `interp`, `D'D`, the queues |
| `tests/closed_loop.rs` | a pretend car (ego's ramps, the shared car model at 100 Hz): straight at 1 and 3 m/s, no speed overshoot, a 1 m offset, an 8 m bend, the U-turn at 2 m/s with vehicle.yaml's lags (0.10 m) and doubled (0.18 m), the stop at the end, the dead-time queues, the tick time, bad numbers |

`colcon test --packages-select aav_mpc_lag` runs the same; `pixi run test` everything.
