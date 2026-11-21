# aav_localization_health_rs

## Overview

`localization_health` in Rust (rclrs): a port of Betsi's `aav_localization_health`
(1 Nov). The same node name, topics, QoS, parameters and answers. The Python was removed
on 20 Nov (tag `save/python-localization-health`). Ben, 14 Nov: "convert
anything that uses a lot of resources to rust" (TODO 185).

**Warn only.** It never stops the car and never touches the EKFs. The sensor watchdog does the
stopping; these are the checks it doesn't do.

| Check | WARN when |
|---|---|
| `gyro_bias` | parked, the gyro's mean turn rate stands out (0.2 deg/s = 12 deg a minute without GPS) |
| `imu_mounting` | parked, gravity says the IMU is upside down, tilted, or in the wrong units |
| `gps_quality` | no fix, an unknown covariance, or a sigma too big or too small to believe |
| `timestamps` | an input's stamp is too old, or from the future (a computer's clock is off) |
| `frames` | an input's `frame_id` isn't the one the EKFs expect |
| `tf` | `base_link -> imu_link / gps_link` missing, or `map -> base_link` old |
| `wheels` | the two front wheels disagree with the turn the gyro measures |
| `motion` | an EKF says a speed, turn or acceleration the car can't do; `ekf_local` jumps |


```
/sensors/imu/data, /vehicle/odom, /vehicle/wheels, /sensors/gps/fix ─┐
/localization/odom/local, /localization/odom/global, /tf, /tf_static ─┤
                                                                      ▼
                         localization_health ──► /localization/health (1 Hz): OK, or WARN and why
                                             ──► /diagnostics (1 Hz): one status per check
```

| Where | Default |
|---|---|
| our simulator, the car, the bench | **Rust**, since 16 Nov; the only one since 20 Nov (`localization_health_impl:=rust`, a hook for a challenger) |

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/health-core` | the checks (`health`), and the TF tree's answers (`tree`) | no: `cargo test` on any laptop |
| `crates/health-node` | the node, binary `localization_health` | yes (rclrs 0.8, `ros-env` 0.3) |

colcon builds it through `CMakeLists.txt` (`cargo build --release --locked`), installed as
`lib/aav_localization_health_rs/localization_health`. It reuses `ego-core` (Python's
`hypot` and `%g`) and `watchdog-core` (Python's `sum`, `max`, `min`). The launch file
gives it `config/localization_health.yaml` (every threshold, with its reason; moved here
from the Python on 20 Nov); the wheelbase
and track come from `aav_description`'s `vehicle.yaml`.

### Node `localization_health`

The same as the Python's: every topic, QoS, parameter and default. TF is read as tf2's
listener does (`/tf` keep last 100, `/tf_static` latched); `tree.rs` answers the two
questions the node asks (is `base_link -> imu_link / gps_link` there; how old is
`map -> base_link` at tf2's `Time()`).

## Invariants

- **The same answers as the Python**: `tests/vectors.rs` reads `vectors.json`, a
  **golden fixture** since 20 Nov (`health.py` wrote it; it and `gen_vectors.py` went with
  the Python, `save/python-localization-health`: never regenerate it). Every report: each check's level,
  words and numbers (bit for bit; `tilt_deg` within 4 ulps, libm's `acos`), the summary
  line, and each number as `/diagnostics` writes it. Change one, change both.
- **The same TF answers as tf2**: 300 random trees of the car's frames, fed to tf2_ros's
  own `Buffer` and to `tree.rs`.
- `health-core` has no ROS. Rust API Guidelines: enums (`Check`, `Input`, `Ekf`,
  `Level`), no `unwrap` in library code; `cargo fmt` and `cargo clippy -D warnings`
  clean. `Cargo.lock` committed.

## Measured

Our simulator's road world, headless, `/clock` at 100 Hz, Ben's MacBook (M4 Max),
16 Nov. CPU: the node's CPU seconds a second, sampled with `ps` every 2 s.

| | Python | Rust |
|---|---|---|
| parked, 57 s after 40 s of start-up | **9.4 %** of a core | **1.5 %** |
| `sim-check`'s road world (about 20 s, driving) | 9.9 % | 1.4 % (2.3 % with its exit) |
| the health line at the end | `OK` | `OK` |

| sim-check, `localization_health_impl:=` | Python | Rust |
|---|---|---|
| the road world (start-up, the graph against the contract, sensors, lidar, scripted drive, EKF, wheels vs IMU and GPS) | 9 of 9 pass | 9 of 9 pass |
| `--actuator-faults` | | 6 of 6 cases pass |
| `--faults` (3 runs) | | 12, 10 and 13 of 14 cases pass. Each failure: "Nav2 would not drive before the fault", after the global EKF jumps 9 m when the wheels come back (`odom stop`, `odom nan`) and the follower loses its path (mppi_lag: "no part of the path ahead is inside the costmap"). Navigation, not this node: it only publishes words. The one run with the Python health node passed 14 of 14; the jump shows in every run's log, the abort in some (TODO 207) |

## Side Effects

- Publishes `/localization/health` and `/diagnostics`; logs each change of state once.
- Stops cleanly on SIGINT, SIGTERM or SIGHUP (`ctrlc`).

## Verification

```sh
cd ros/localization/aav_localization_health_rs && cargo test --locked -p health-core   # no ROS; tools/test.sh
pixi run build && colcon test --packages-select aav_localization_health_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `tests/checks.rs` | `test_health.py`'s cases, ported |
| `src/tree.rs`, `src/lib.rs` unit tests | the TF tree's ages; `format(x, '.4g')` |
| `tests/vectors.rs` | the Python's answers, report by report; tf2's on the trees |
| `test/test_localization_health_rs_node.py` | the process: a parked car (OK), the IMU upside down (`imu_mounting WARN`), the IMU quiet (`STALE`), 8 statuses on `/diagnostics`, a clean Ctrl-C |

Not yet run on the car.
