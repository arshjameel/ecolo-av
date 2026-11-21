# aav_actuator_tests

The actuator tests: `docs/tests.md`'s stage 3 (on stands) and stage 4 (in an empty lot),
run as a twist_mux driver. Each test drives a fixed set of steps, logs what the car
did, and prints the `vehicle.yaml` lines to paste (`docs/actuator-tests.md`).

```
actuator_test -> /drivers/tests/cmd_vel -> twist_mux (60) -> ego -> output mux -> car
      ^ reads /vehicle/*, IMU, GPS, odometry          -> ~/aav-tests/<run>/
```

## Overview

```sh
pixi run actuator-test --list                       # the tests
pixi run actuator-test steer-sweep --sim            # Ravensim, this computer only
pixi run actuator-test steer-sweep --stands         # the real car, wheels off the ground
pixi run actuator-test speed-up                     # the real car, in an empty lot
pixi run actuator-test --analyse ~/aav-tests/<run>  # a run's report again, from its CSV
pixi run actuator-test preflight --stands --sim                            # the preflight (actuator-test preflight --stands)
pixi run car-test --stands                          # phase A: every stands test and fault, one report
pixi run car-test --stands --sim --no-manual        # the same against its own Ravensim
pixi run car-test --hallway                         # phase B: the transfer functions, in a hallway
pixi run car-test --hallway --sim --no-manual       # the same in a pretend hallway in Ravensim
pixi run car-test --analyse ~/aav-tests/<session>   # a hallway session's report again
```

| Test | Stage | Measures |
|---|---|---|
| `steer-sweep` | 3.10 | command vs measured steering angle |
| `steer-rate` | 3.11 | the steering's speed, centre to full lock |
| `steer-centre` | 3.12 | the wheels' straight angle; the deadband |
| `speed-up` | 4.1 | time to 1, 2, 3 m/s; acceleration |
| `coast` | 4.2 | the drag |
| `brake` | 4.3 | stopping distance and deceleration |
| `throttle-map` | 4.4 | throttle level vs acceleration |
| `circles` | 4.5 | the real wheel angle, both ways |
| `straight` | 4.7 | sideways drift over 10 m |
| `actuator-test preflight --stands` | 3.P | on stands: speed up, hold, slow, brake, steer, every sensor at once: PASS / FAIL per line |
| `wheel-steps` | 3.14 | on stands: throttle on free wheels, 0.1, 0.2, 0.3, 0.15 m/s: does the wheel speed follow |
| `hall-speed-up` | B1 | in a hallway (`--hallway LENGTH`): 10 s still, speed up, brake: throttle lag, dead time, `max_accel` |
| `hall-coast` | B2 | the same, then coast: the drag |
| `hall-brake` | B3 | the same, then brake: brake lag, dead time, `max_brake`, the stop |
| `hall-steer` | B4 | the same, then steer left, right, left on the ground: steering lag, dead time |

`car_test` (`pixi run car-test --stands`, TODO 246) runs phase A of `docs/testing-plan.md` in
one go: the preflight, the sensors alone, the three steering tests and `wheel-steps` (each an
`actuator_test` child), then the faults on purpose (the Orin's heartbeat, the GPS, IMU,
camera, lidar, a Pi's link, a pulled cable, the e-stop), into one folder with `report.md`
(`docs/actuator-tests.md`, "car-test").

`car_test --hallway` (TODO 248) runs phase B: it asks the hallway's measured length, works
out the top speed (speed up + stop + margin, within 1.0 m/s), runs every `hall-*` pass
`--repeats` times, the car turned round between passes, then fits everything with error
bars and lays the model over each pass (`hallway.py`; `docs/actuator-tests.md`,
"car-test --hallway").

## Interface

Executable `actuator_test` (`ros2 run aav_actuator_tests actuator_test`, wrapped by
`tools/actuator_test.sh`). Options: `<test>`, `--list`, `--sim`, `--bench`, `--stands`,
`--max-speed`, `--speeds`, `--angles`, `--max-distance`, `--max-time`, `--out`, `--bag`,
`--analyse RUN_FOLDER`; for `actuator-test preflight --stands` only, `--skip SENSOR...` and `--no-drive`;
for the `hall-*` tests, `--hallway LENGTH` (required), `--margin` (3 m) and `--direction there|back`.

Executable `car_test` (wrapped by `tools/car_test.sh`):
`--stands` or `--hallway` (one of them), `--ground` (refused), `--sim`, `--only STEP...`,
`--no-manual`, `--out`; with `--hallway`: `--length`, `--margin`, `--repeats`, `--kinds`,
`--one-way`, and with `--sim` `--sim-actuators "throttle_lag=0.3,..."`, `--wheel-noise`,
`--wheel-slip`; `--analyse SESSION`;
what it reads, publishes, calls and signals is in `car_test_node.py`'s docstring: the same as
the preflight, plus `/safety/sensor_watchdog/state` and `lock`, `/vehicle/status`,
`/safety/output_mux/state`; with `--sim` it sets
Ravensim's `fault.<sensor>` and pauses its own Ravensim's `output_mux` (SIGSTOP, SIGCONT).
With `--hallway --sim` its Ravensim is the empty world with no cameras, its car's own
actuator numbers set by `--sim-actuators` (`sim.launch.py actuators:=`).

| Item | What |
|---|---|
| Subscribes | `/vehicle/odom` (`nav_msgs/Odometry`); `/vehicle/steering`, `/vehicle/throttle`, `/vehicle/brake` (`aav_interfaces/ActuatorState`); `/vehicle/wheels` (`aav_interfaces/WheelSpeeds`); `/sensors/imu/data_raw` (`sensor_msgs/Imu`); `/localization/odom/local` (`nav_msgs/Odometry`); `/sensors/gps/fix` (`sensor_msgs/NavSatFix`); `/safety/estop` (`std_msgs/Bool`); `/safety/geofence/state` (`std_msgs/String`); `/control/cmd_vel` (`geometry_msgs/TwistStamped`); `/ego/actuator_cmd` (`ackermann_msgs/AckermannDriveStamped`: what ego told the backend, logged); `/system/source` (`aav_interfaces/DataSource`, latched). The preflight adds `/sensors/imu/data`, `/sensors/lidar/points`, `/sensors/radar/points` (`sensor_msgs/PointCloud2`), `/sensors/camera/front_left/image_raw` (`sensor_msgs/Image`, reliable), `/vehicle/battery` (`sensor_msgs/BatteryState`), `/sensors/gps/fix_velocity` (`geometry_msgs/TwistWithCovarianceStamped`), `/safety/legacy_uart_bridge/lock` (`std_msgs/Bool`, latched) |
| Publishes | `/drivers/tests/cmd_vel` (`geometry_msgs/TwistStamped`), 50 Hz while a test runs; the preflight: `/safety/estop` (`std_msgs/Bool`), **true only**, when the car is still moving 5 s after its stop |
| Calls | `/ego/get_parameters` (its limits); `/ego/set_parameters` (`max_steering_rate`, lifted for `steer-rate` only, then put back); `get_parameters` of the backend that runs (`legacy_uart_bridge`, `sim_adapter`, `spine_sim`, `spine_bridge`: `reports.SHAPING_PARAMS`), for `meta.yaml` |
| Runs | `ros2 bag record`, with `--bag` |

Modules (all but the node have no ROS):

| Module | Holds |
|---|---|
| `procedures.py` | the steps: `Step`, `Runner`, `Settings`, `Test`, `TESTS`, `plan()`, `estimate()`, `with_ego_limit()`; the hallway's `hall_*`, `HALL_TESTS`, `stop_room()` (the brake point, `Step.fenced`) |
| `safety.py` | the rules: `Kind`, `source_kind`, `decide`, `confirmed`, `Envelope.breach`, `taken_over`, checklists; caps `REAL_CAP` 1.0, `STANDS_CAP` 0.3, `SIM_DEFAULT_CAP` 3.0, `SIM_MAX_CAP` 5.0 m/s; the hallway: `hallway_limits()`, `HALLWAY_CHECKLIST` |
| `fusion.py` | the IMU with the wheels: `still_imu` (the offset), `slope_from` (the floor), `fuse` (a Kalman filter: speed, IMU bias), `forces` (F = m a) |
| `errorbars.py` | `Value`, `Estimate` (mean ± sd, bootstrap 90 %, resolution, floor, trust), `estimate()` |
| `hallway.py` | phase B's session: `analyse()` (fits per pass, error bars, the model replayed), `write()` (report.md, fits.csv, compare.csv, overlays/, vehicle-yaml.txt), `chosen()`, `recovery_rows()`; `main()` |
| `analysis.py` | the maths, plain Python (no numpy): `fit_line`, `fit_throttle`, `fit_lag` (the old fit), `fit_fopdt` (dead time + lag), `fit_speed_response` (through the wheel speed), `fit_steer_lag`, `fit_steer`, `steer_play`, `LagFit`; what can be resolved: `pulse_interval`, `fresh_period`, `time_resolution`, `unresolved`; `stopping`, `time_to_speeds`, `fit_circle`, `wheel_angle`, `drift` |
| `reports.py` | one run's rows to a `Report`: `analyse()`, `table()`, `yaml_lines()`; `PATHS` (each fit's from -> to), `signal()` (true arrival times), `shaping()` (what shaped the commands) |
| `preflight.py` | the preflight's judging: `SENSORS` (documented rates), `sensor_line`, `motion_lines`, `stands_lines`, `ImuWatch` (live, on stands), `table` |
| `actuator_test.py` | the ROS node and the command line |
| `preflight_node.py` | the preflight's ROS part: a counter per sensor, its refusals, the e-stop when it won't stop (`press_estop`), its report |
| `car_test.py` | phase A's steps and judging, no ROS: `STEPS`, `plan()`, `parse()`, `tool_args()`, `tool_result()`, `named()`, `judge_fault()`, `judge_heartbeat_car()`, `table()`, `markdown()`; phase B's `hall_passes()`, `hall_args()`, `sim_overrides()`, `sim_launch_args()` |
| `car_test_node.py` | phases A and B's ROS part: the runner, its slow roll and stops, each fault, the hallway's passes (`HallwayTest`), its own Ravensim (`--sim`), the report folder |

`aav_drive_to_point` reuses `safety.py` (`source_kind`, `taken_over`, `LOT_CHECKLIST`), so both
agree on what REAL means.

## Invariants

- `procedures.py`, `safety.py`, `analysis.py`, `reports.py`, `preflight.py`, `car_test.py`,
  `fusion.py`, `errorbars.py`, `hallway.py` have no ROS imports.
- In the hallway (`hall-*`, `--hallway`): the top speed from the measured length, never above
  1.0 m/s; each driving step ends at the brake point (`stop_room`: the car stands still
  before length − margin, by the wheels); past the room the envelope stops it at once.
  Indoors the geofence says STANDS (no GPS: `pixi run car ... gps:=false stands:=true`):
  only the hallway tests accept that, and NEAR, OUTSIDE, LOST, JUMP still stop them.
- It prints `vehicle.yaml` lines from trusted numbers only; an untrusted one is commented,
  with why.
- `car_test` stops at the first FAIL (the steps after: NOT RUN), and after each step sends
  zeros until still, else `/safety/estop` true (never false). It signals only its own
  Ravensim's processes (`--sim`: its launch log, its process group). Each `actuator_test`
  child still asks the person on the real car.
- Every rule errs towards "this is the real car": nobody announcing counts as REAL.
- `--sim` only when `/system/source` says SIM, and SIM only with `--sim`. REPLAY refused.
- Real car: stage 3 needs `--stands`, stage 4 refuses it. Speed cap 1.0 m/s (0.3 on
  stands). The person types the test's name to start.
- Stops (zero commands) on: Ctrl-C, any error, `/safety/estop`, someone else driving
  (twist_mux passing another command for 0.3 s), twist_mux silent for 2 s, the speed,
  distance or time leaving the test's envelope, `/vehicle/odom` stale for 0.5 s, 4 s of
  driving with no measured speed (`safety.unseen`), the speed rising while the command
  falls (`safety.runaway`), a `must` step timing out, and on the ground the geofence
  saying NEAR, OUTSIDE, JUMP or LOST (`safety.geofence_stop`). Its slot (60) is above the
  geofence's lock (51), so it stops itself. On the real car, off stands, it refuses to
  start while the geofence is silent, STARTING or in STANDS mode.
- The preflight drives the real car only with a driven-wheel speed (`vehicle.yaml`
  `speed_wheels: rear`, TODO 49b), the Rust ESP32 (no no-timeout lock) and the bridge on
  `speed_loop.mode open`; `--no-drive` checks the sensors alone. It may set `/safety/estop`,
  never clear it.
- Steering only changes while moving (ego holds it below 0.05 m/s), so stage-3 tests creep.
- It prints `vehicle.yaml` lines; it never writes `vehicle.yaml`. Each line names its
  path (from -> to) and rms; a lag or dead time the data can't resolve gets no line.
- `log.csv` stays readable by `--analyse` for older runs: new columns go at the end.
- SI units; positive steering = left.

## Side Effects

- Publishes `/drivers/tests/cmd_vel`; may change ego's `max_steering_rate` for a while.
- Writes one folder per run under `~/aav-tests` (or `$AAV_TESTS_DIR`, or `--out`):
  `log.csv` (with each value's arrival time, `t_*`), `summary.txt`, `meta.yaml` (with
  `shaping`: backend, speed loop, ego's limits), and with `--bag` a rosbag.
- Starts `ros2 bag record` with `--bag`.

## Verification

```sh
pixi run build && colcon test --packages-select aav_actuator_tests && colcon test-result --verbose
```

Quick, without ROS: `cd ros/drivers/aav_actuator_tests && python3 -m pytest -q`
(`tools/test.sh` runs it). `pixi run test` runs everything.

| Test | Covers |
|---|---|
| `test_procedures.py` | each test's steps, on a fake car |
| `test_safety.py` | who may run what, caps, envelope, takeover |
| `test_analysis.py` | the fits and figures; the resolution maths |
| `test_robustness.py` | the fits on pretend data with play, Hall pulses, noise, no brake level, a dead time (`synthetic.py`): the numbers back within a stated tolerance, or refused |
| `test_preflight.py` | the preflight's lines on the pretend car, on the ground and as if on stands; rates, values, the IMU watch |
| `test_reports.py` | true times, paths, `shaping`, old logs, the fallbacks (IMU-only brake, the ESP32's open-loop throttle) |
| `test_car_test.py` | car-test's steps, `--only`, `--no-manual`, `--sim`, the command line; the tools' results; `wheel-steps` on the pretend car and judged; the watchdog's words; each fault's judgement; the report |
| `test_node_names.py` | no attribute of the nodes hides a method (the preflight's e-stop, 19 Nov) |
| `test_hallway.py` | the IMU with the wheels, the slope, error bars, the hallway's limits and brake point, every pass inside its room, the command line; and on `fake_hallway.py`'s pretend sessions, the known numbers found within their error bars, the fitted model matching every pass |
