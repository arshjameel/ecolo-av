# aav_mppi_lag

Nav2's MPPI path follower, ported to Rust, with the **actuators' lag in its model**.
The default follower since 14 Nov (`stack:=ours`); the only MPPI since Nav2 went on 17 Nov (`docs/trials.md`, `docs/mppi-lag.md`). A planner still plans
the path; this node follows it.

```
navigator --FollowPath--> mppi_lag --cmd_vel_nav--> velocity_smoother
   -> stop zone -> /drivers/nav/cmd_vel (twist_mux 20) -> ego
```

## Overview

| Part | What it is |
|---|---|
| `crates/mppi-lag` | the maths, no ROS: the car model, MPPI, Nav2's critics, the checks |
| `crates/mppi-node` | the ROS node, binary `mppi_lag` (rclrs): messages in and out only |
| `config/mppi_lag.yaml` | its own settings: the model, the steering noise, its correlation |

What it adds to Nav2's MPPI:

| | Nav2's MPPI | `mppi_lag` |
|---|---|---|
| Model of the car | speed and turn rate obey at once (within ax, az limits) | ego's ramps, the spine's speed loop, throttle and brake lag, steering motor rate, linkage lag, deadbands, dead times |
| State it starts from | pose and speed | pose, speed, ego's output, the actuators' levels, the commands still in their dead time |
| Noise | white, on (v, w) | on (v, steering), correlated step to step (AR(1), 0.8); the control cost weighs it by its own covariance. `sampling: smooth` (TODO 212, a challenger): the steering's noise on its change a step, summed (SMPPI's idea) |
| Cusps | reached within the inversion tolerances | the same, or stopped within the goal's tolerances, or held 3 s within 2 m (`cusp_reach`, `cusp_wait`: a stop zone short of a wall) |
| The lane | no critic for it | `LaneCritic`, with a lane map: the outline kept inside the road ahead's lines and kerbs |
| Language | C++ plugin | Rust |

```sh
pixi run sim world:=test_lot tracker:=mppi_lag
pixi run measure controllers --trackers mppi_lag --controllers mppi --tours lot
```

## Interface

**`mppi_lag`** (`crates/mppi-node/src/main.rs`), started by `aav_navigation`'s
`navigation.launch.py tracker:=mppi_lag`

| Item | What |
|---|---|
| Action server | `/follow_path` (`nav2_msgs/action/FollowPath`): the newest path replaces the last without a stop |
| Subscribes | `/localization/odom/local` (Odometry); `/tf` (map -> odom: held, and moved toward the newest at most 0.1 m/s and 1°/s at the car; each new path comes into odom by the held one and the running path moves with it, TODO 186 [Follow in odom](../../../docs/mppi-lag.md#follow-in-odom-correct-slowly); the road ahead by the same); `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake` (ActuatorState, measured, while fresh); `/local_costmap/costmap_raw`, or ours `/drivers/nav/costmap/local_raw` with `costmap:=own` (nav2_msgs/Costmap; the launch file chooses: `config/mppi_lag.yaml` must not, `aav_navigation` `test_launch_params.py`); `/world/road_ahead` (aav_interfaces/RoadAhead, map: the lane's line and edges for `LaneCritic`, while 0.5 s fresh), only with `behaviour:=true` and a lane map (`road_ahead_topic`, empty: off) |
| Publishes | `/cmd_vel_nav` (TwistStamped); `/drivers/nav/track/status` (DiagnosticStatus, aav_track's keys and more; `map_gap_m`: how far the held map -> odom is from the newest, at the car); `/drivers/nav/local_plan` (Path, odom: MPPI's chosen future, the screens' faint line); `/drivers/nav/lag_aware` (Bool, latched) |

Results: succeeded; aborted with `FAILED_TO_MAKE_PROGRESS`, `NO_VALID_CONTROL`,
`CONTROLLER_TIMED_OUT` (a costmap older than `costmap_timeout`, 0.8 s: zero first),
`INVALID_PATH`, `TF_ERROR`, `UNKNOWN` (no odometry); canceled.

### `lag_aware`

Ego's contract (0.3): `std_msgs/Bool` on `/drivers/<slot>/lag_aware`, reliable, transient
local, depth 1. This node drives through navigation's slot, so `/drivers/nav/lag_aware`:

| When | Says |
|---|---|
| start, exit | `false` |
| a path starts, `model: lag` | `true` |
| the path ends (succeeded, aborted, cancelled) | `false` |
| `model: ideal` | always `false` |

Ego's lead turns off while it is `true`, so the lag isn't cancelled twice. Every
`TwistStamped` is stamped with now.

### Parameters

From `navigation.launch.py` (`vehicle.mppi_lag_params`; nothing copied):

| Name | From |
|---|---|
| `time_steps`, `model_dt`, `batch_size`, `iteration_count`, `temperature`, `gamma`, `vx_std`, `vx_max`, `critics`, `<Critic>.*`, `inversion_xy_tolerance`, `inversion_yaw_tolerance` | nav2.yaml's `MPPI` block (`vx_max`: `max_speed:=` if given) |
| `road_ahead_topic`, `LaneCritic` in `critics`, `LaneCritic.*` | the launch's `behaviour:=true` with `lane_map:=`: `/world/road_ahead` and our critic; else empty and Nav2's critics only |
| `vx_min`, `enforce_path_inversion`, `PathAngleCritic.mode`, no `PreferForwardCritic` | the launch's `reversing`, as `nav2_tuned` sets Nav2's MPPI: 0, false, 0 and kept forward only; -`max_reverse_speed`, true, 2 and dropped with `reversing:=true` |
| `prune_distance` | MPPI's, stretched to fit 2.8 s at `vx_max` (as `nav2_tuned`) |
| `rate_hz`, `odom_topic`, goal and progress checks | `nav2.yaml`'s `controller_server` block |
| `footprint`, `footprint_padding`, `inflation_radius`, `cost_scaling_factor` | vehicle.yaml's body; the local costmap's |
| `wheelbase`, `max_steering_angle`, `actuators.*` | vehicle.yaml (dead times 0 while unset) |
| `ego.*` | ego.yaml's ramps |
| `speed_loop` | the spine's speed loop profile the car runs: the sim's `speed_loop:=` (`default`, `smooth`, `firm`: the lag model's PI; `lqi`: the shared car model); an unknown name is refused (TODO 163) |

From `config/mppi_lag.yaml`: `model` (lag or ideal), `steer_std`, `steer_std_speed` (above it
the steering noise shrinks as 1 / speed², TODO 168), `steer_std_min` (never under it: 0.024 rad),
`steer_gamma_speed` (1 m/s: with a `steer_gamma`, it holds from here up and eases to gamma at a stop), `noise_correlation`, `steer_rate_std` (0.5 rad/s: the steering's change a step with smooth sampling),
`threads`, `seed`, `use_costmap`, the topics and timeouts (`road_ahead_timeout` 0.5 s),
and the map correction (TODO 186): `map_correction_speed` (0.1 m/s; 0: off, each new path by
the newest map -> odom, as before), `map_correction_turn_rate` (1°/s),
`map_correction_snap_distance` (2 m) and `map_correction_snap_angle` (10°: a bigger gap is a
relocalisation, taken at once and logged). From the launch: `map_correction` (bool, false by default; `map_correction:=slow` gives true, as the sim's `ours` stack does since 15 Nov (it won there, TODO 168);
`navigation.launch.py map_correction:=newest` gives false, the old way; never in the yaml,
which would win); `sampling` (the launch's `mppi_lag_sampling:=`: `correlated`, the default, or `smooth`, TODO 212); `steer_gamma` (the launch's `mppi_lag_steer_gamma:=`; below 0 or unset:
gamma's, as Nav2: the steering's control cost weight, TODO 168, [the sway that was left](../../../docs/history/mppi-lag-log.md#the-sway-that-was-left)).
All read once at start.

### Crate `mppi-lag`

| Module | Holds |
|---|---|
| `model.rs` | `CarModel`: the trait any car model plugs in through (the shared car model at merge) |
| `lag.rs` | `LagModel`: the 0.3 spec |
| `shared.rs` (feature `shared`) | `SharedModel`: the shared `car-model` crate behind `CarModel`, with spine-core's own speed loop (any profile) |
| `ego.rs` | `Driven`: ego's ramps in front of any model |
| `ideal.rs` | `IdealModel`: no lag, as Nav2's MPPI |
| `optimizer.rs` | MPPI: sample, roll out (threads), score, softmax, Savitzky-Golay, shift |
| `critics.rs` | Nav2's eight critics in nav2.yaml, and our `LaneCritic` (the outline past the lane's lines, squared; past its kerbs, far more) |
| `lane.rs` | the road ahead's lane: its line, its four edges (`None` for NaN), where a point lies beside it |
| `path.rs`, `costmap.rs`, `smoothing.rs`, `rng.rs` | the path window, the costmap and outline, the filter, random numbers |
| `age.rs` | the costmap's age (Nav2's `costmap_update_timeout`) |
| `map_correction.rs` | follow in odom, correct slowly: the held map -> odom, slewed toward the newest within `SlewLimits`, measured at the car; snaps past 2 m or 10° |
| `follower.rs` | one path: Nav2's goal and progress checks, the state estimate, MPPI |

## Invariants

- Reverses only with `reversing:=true` (Ben, 14 Nov: "The car can reverse"), as Nav2's
  MPPI: down to `-max_reverse_speed`, stopping at each cusp before it backs up. The gear
  changes only at a stop, in the model as on the spine.
- Never both followers: with another `tracker:=`, this node doesn't start.
- Nothing in the safety path changes: the smoother, the stop zone, twist_mux's slot,
  the locks and e-stops all sit after `cmd_vel_nav`.
- The maths has no ROS; the node has no maths.
- `model_dt` must equal `1 / rate_hz` (the plan shifts one step a tick, as Nav2's).
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean; no dependencies in the crate
  by default. The feature `shared` adds the shared `car-model` crate (ours) as
  `SharedModel` behind `CarModel` (`shared.rs`: it drives as `LagModel` does, to 1 cm,
  with each PI profile), and spine-core's profiles for `LagModel`'s loop
  (`SpeedLoop::from_profile`); the node builds with it and runs `LagModel`, or
  `SharedModel` for the LQI. The feature `trace` adds `tracing` (below).

## Side Effects

- Publishes `/cmd_vel_nav` and its status only while a path runs. No files, no hardware.
- Uses one thread per core for the rollouts (`threads`).

## Verification

```sh
cd ros/drivers/aav_mppi_lag && cargo test --release -p mppi-lag          # tools/test.sh runs it
cargo test --release -p mppi-lag --test closed_loop explore -- --ignored --nocapture
cargo run --release -p mppi-lag --example timing                         # ms a tick
cargo bench -p mppi-lag                                                  # a tick, 1000 and 2000 samples, lane or not (criterion: says if a change made it slower)
cargo run --release -p mppi-lag --example follow_plan -- paths/*.csv     # planned paths, followed (lattice-planner's moves example writes them)
cargo run --release -p mppi-lag --example follow_plan -- bags/<drive>/<drive>_0.mcap   # a recording's own plans on /plan (TOPIC, LIMIT 10), followed again offline (TODO 202)
cargo run --release -p mppi-lag --example wiggle                         # the wiggle: each suspect at 2.5 m/s, before and after (docs/mppi-lag.md)
STEER_GAMMA=0.05 SPEED=1.0 ONLY=after cargo run --release -p mppi-lag --example wiggle   # one setting tried (also BATCH, TEMP, CORR, STD, GAMMA)
ONLY=sim-like SPEED=1.0 cargo run --release -p mppi-lag --features shared --example wiggle   # TODO 163: the car on the firm or smooth speed loop, the model told or not
```

Where a tick's time goes (TODO 196), off by default:

```sh
pixi run build --packages-select aav_mppi_lag --cmake-args -DMPPI_LAG_FEATURES=trace   # or tracy
AAV_TRACE=1 ...       # each stage's time on stderr as it closes: mppi.tick, mppi.rollout, mppi.critics, mppi.weigh, mppi.smooth
AAV_TRACE=tracy ...   # live in the Tracy viewer (feature tracy)
pixi run build --packages-select aav_mppi_lag --cmake-args -DMPPI_LAG_FEATURES=          # back to none (CMake keeps the last)
```

| Test | Covers |
|---|---|
| `lag.rs`, `ego.rs`, `ideal.rs` | each stage of the model: lags, rate, deadbands, dead times, offset; reversing, the gear changing only at a stop |
| `critics.rs`, `path.rs` | each critic on hand-made samples, PathAngle's modes on a reversing path, `LaneCritic` inside, over a line, over a kerb, without a lane; the cusps; `lane.rs`: the side, the search, NaN edges |
| `optimizer.rs`, `smoothing.rs` | the limits, the filter, a straight path, a turn, the same seed the same plan; the control cost against a brute-force covariance inverse, the steering's own weight (`steer_gamma`) and its ease by speed; the steering noise's spread by speed; smooth sampling (TODO 212): each step's change as drawn, the spread a random walk's, its control cost against a brute-force inverse, still turning to a path |
| `tests/closed_loop.rs` | a pretend car (ego + the lag model at 100 Hz): the model predicts it; circle and S-curve at vehicle.yaml's lags; a corner and an S on very slow actuators: the lag-aware one closer on the corner, the only one to keep the S; 8 m straight back; 6 m ahead, a stop at the cusp, a quarter circle (r 5 m and the planner's 3.6 m) back; the test lot's turn back (back, cusp, forward), from a stop and arriving rolling 0.5 m off; the campus's "round the bend" junction, on the planner's own path from the sim (`campus_junction_round_the_bend`); a path drawn 1 m off a lane's centre, kept within 0.1 m of the line by `LaneCritic` (0.41 m over without); the wiggle (TODO 168), set up as the launch file sets it (2000 samples, 8.4 m of path): no side to side on a straight and a bend at 2.5 m/s, less with a sim-like pose, none backing up; the calm steering (`steer_gamma` 0.05): less sway at 1 and 2.5 m/s with a sim-like pose, and the turn back and cusp still pass |
| `tests/bag.rs` | `follow_plan`'s recording reader: a path through CDR and back; a cut message is an error; plans written into an MCAP come back in order, other topics left out |
| `tests/map_correction.rs` | the node's frames rebuilt (odometry in odom, the path in map again every 1 or 10 s, a global EKF jumping 5 times a second): the held map -> odom sways the car less than the newest, on a straight and a bend; a real 0.5 m offset is still taken out within 8 s; a 3 m relocalisation snaps. `explore` (ignored) prints every case |

`colcon test --packages-select aav_mppi_lag` runs the same; `pixi run test` everything.
