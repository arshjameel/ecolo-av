# Drift report: the folder READMEs against the code (25 Oct 2026)

The EGM bootstrap of 25 Oct gave 88 folders a README with Overview, Interface,
Invariants, Side Effects and Verification. Seven fresh reviewers, reading only, then
checked each README against its code.

| Result | READMEs |
|---|---|
| No drift | 39 |
| Some drift | 49 |

Most drift is a README that is incomplete or slightly wrong: fix the README. A few items
are **the code not doing what the team's rules say**. Those are listed first, for the
team to decide. Nothing in the code was changed.

## Code to look at first

| # | Where | What the code does | The rule it misses |
|---|---|---|---|
| 1 | `spine-core` `Spine::direct` (`spine.rs:526-555`) | open-loop UART commands (today's ESP32 path) have no reverse cap; with no wheel speed the overspeed check is off too | "Reverse never faster than 0.5 m/s, whoever drives" |
| 2 | `spine-core` `IDLE_BRAKE = 0.0`, used by `pico2` | after a reset the brake `init` set is released on the first tick (Manual mode); only `esp32-umod` sets 1.0 | "Safe after any reset: brake on" (one of the three open choices in `docs/firmware.md`) |
| 3 | `aav_language` `drive_tools.py:147` | checks only twist_mux's `estop` lock; a `stop_sign` or `sensors` lock doesn't block `drive`, `turn` or `go_to_gps` | "Nothing moves while twist_mux says locked" |
| 4 | `aav_language`, pixi tasks `drive-server`, `talk` | always run through `tools/ros.sh --local`, so `--car` can never reach the car | the README's "the real car: only with a safety driver" |
| 5 | `aav_scripted` `scripted_driver.py:64,89` | `busy` set in `execute`, not `on_goal`; a negative `hold_time` raises after `busy = True`, and every later goal is refused | "One goal at a time"; the server should never wedge |
| 6 | `aav_scripted` | no `/safety/estop` or takeover subscription; a `hold_time` goal runs without odometry | the other drivers stop on e-stop, takeover and stale inputs (twist_mux still holds it) |
| 7 | `aav_lane_geometry` `control_interface.py:239-242` | 2 s without a message: an instant zero, not a slow-down; `stop_wait_s` is read but never used | the README's "slows at `decel_rate` and stops" |
| 8 | `turtlesim_adapter` | no command timeout | "A command older than 0.2 s means speed 0" (sim backends) |
| 9 | `battery_monitor` `VEDirect.set_block` | merges blocks, so a block missing V, I, SOC or VS still publishes, with old values | "publishes only when the block holds V, I, SOC and VS" |

Also worth a thought: twist_mux's stop-sign and sensor locks hold `goto` (25) as well as
Nav2 and AI. Several READMEs said "Nav2 and the AI slot only".

## By README

## ros/drivers/README.md

- The invariant "The programs stop on the e-stop, when a person takes over, and when their inputs go stale" does not hold for `aav_scripted`. `scripted_driver` does not subscribe to `/safety/estop` or `/control/cmd_vel`. A `hold_time` goal also keeps running without odometry: freshness is checked only for `hold_distance` goals, and only when the goal is accepted (`ros/drivers/aav_scripted/aav_scripted/scripted_driver.py:43-44`, `:70`, `:99-129`). Only twist_mux's lock and priorities stop it.
- The invariant "On the real car … `aav_language` wait[s] for a person to confirm" is enforced only in `talk`, at its "yes" prompt (`aav_language/agent.py:67-73`). The drive server itself never asks for confirmation. With `--car` it drives whatever it finds (`aav_language/safety.py:126-127`), and `.mcp.json` starts it directly for Claude Code (`.mcp.json:8-12`).

## ros/drivers/aav_actuator_tests/README.md

No drift detected.

## ros/drivers/aav_goto/README.md

- The invariant list of stop and refusal reasons is incomplete. `goto_driver` also stops in three more cases:
  - the measured speed goes over `cruise × 1.2 + 0.3` (`aav_goto/goto_driver.py:328-330`);
  - the distance driven goes over `2·d + 2πR + 10` m (`goto_driver.py:322-325`);
  - the elapsed time goes over `2·ideal_time + 30` s (`goto_driver.py:326-327`).

  It also refuses REPLAY (`aav_goto/rules.py:73-74`) and refuses a B less than 0.5 m away (`rules.py:116-117`). Without an answer from ego, cruise is capped at 1.0 m/s (`goto_driver.py:240`). ego's `max_acceleration` and `max_deceleration` also cap the acceleration and deceleration (`goto_driver.py:241-242`), not just `max_speed`.

## ros/drivers/aav_language/README.md

- `pixi run -e mcp drive-server --car` can't reach the real car, although the README says it drives "the real car". The `drive-server` task always runs through `tools/ros.sh --local` (`pixi.toml:224`). That sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and clears static peers (`tools/ros.sh:37-43`), so `--car` only lifts the check, not the isolation. `talk --car` has the same problem (`pixi.toml:238`, and `agent.py:31-38` passes `--car` to a child process in that same environment). The `.mcp.json` entry uses the same task.
- The invariant says nothing moves "while twist_mux's report says locked". The code checks only the `estop` lock in that report (`aav_language/drive_tools.py:147`). A `stop_sign` or `sensors` lock does not block `drive`, `turn` or `go_to_gps`.
- The invariant "Models run on this computer only" isn't enforced. `OpenAiChat` accepts any `base_url` and an `api_key` (`aav_language/llm_openai.py:102-109`), and `run_eval --base-url` passes any URL through (`run_eval.py:162`).
- The drive server's own options `--max-speed` (default 1.0) and `--fake` aren't mentioned (`drive_server.py:141-143`). Neither are `talk`'s `--model`, `--max-speed`, `--json-actions` and `-v` (`agent.py:159-168`). The `react.py` row leaves out `ReactAgent` (`react.py:351`), the class `talk` actually runs.

## ros/drivers/aav_navigation/README.md

- The invariant says collision_monitor's stop zone is "front bumper to 1 m ahead". `nav2.yaml` defines a `velocity_polygon` with two zones: `forward`, and `backward`, which runs from the rear bumper to 1 m behind and is used when reversing (`config/nav2.yaml:478-493`; `aav_description/vehicle.py:129-134`). Its sources are not only radar or lidar: the ghost and the camera obstacles are sources too (`nav2.yaml:509-512`, plus the `camera` source).

## ros/drivers/aav_rc/README.md

No drift detected.

## ros/drivers/aav_routes/README.md

No drift detected.

## ros/drivers/aav_scripted/README.md

- The invariant "One goal at a time" is only loosely enforced. `busy` is set in `execute`, not in `on_goal` (`aav_scripted/scripted_driver.py:64` checks it, `:89` sets it). A second goal accepted before the first one's `execute` runs is not refused. `aav_goto` sets `busy` in `on_goal` for exactly this reason (`aav_goto/goto_driver.py:221-222`).
- A goal with a negative `hold_time` and `hold_distance` 0 is accepted, because `on_goal` doesn't check it (`scripted_driver.py:62-75`). `SpeedProfile` then raises `ValueError` (`speed_profile.py:21-22`) after `busy = True` (`scripted_driver.py:89-92`), outside the `try/finally` that resets it (`:132-136`). The server then rejects every later goal. The README's rejection list doesn't cover this input.

## ros/drivers/aav_teleop/README.md

No drift detected.

## ros/command_mux/README.md

No drift detected.

## ros/command_mux/aav_mux/README.md

No drift detected.

## ros/ego/README.md

- "Limits come from `vehicle.yaml`" is only partly true. The speed, acceleration, deceleration, jerk, steering-rate, lateral and timeout limits come from `ros/ego/aav_ego/config/ego.yaml` (`config/ego.yaml:9-19`; `launch/ego.launch.py:36`). `vehicle.yaml` supplies only the wheelbase, steering angle, reverse speed and the body (`aav_description/vehicle.py:169-182`).

## ros/ego/aav_ego/README.md

- `/ego/actuator_cmd`'s `acceleration` is described as "the limit used". While speeding up with `max_jerk > 0`, ego publishes the eased, current acceleration instead (`aav_ego/ackermann.py:162-167`: `accel = abs(signed)` from `ease_speed`).
- "plain `Twist` with `use_stamped:=false`" reads like a launch argument, but `ego.launch.py` declares only `use_sim_time`, `max_speed` and `sim_max_speed` (`launch/ego.launch.py:62-67`). `use_stamped` can only be set in `config/ego.yaml` (`:22`) or as a node parameter.

## ros/io_mux/README.md

No drift detected.

## ros/io_mux/aav_io_mux/README.md

No drift detected.

## ros/perception/README.md

- Invariant "The stop-sign lock (twist_mux priority 30) holds Nav2 and the AI slot only" is incomplete. It also holds the `goto` driver, which sits at priority 25: `ros/command_mux/aav_mux/config/twist_mux.yaml:46-49` ("Below the stop-sign and sensor locks (30): it drives on GPS, so they hold it").
- Invariant "Camera and lidar positions come from `vehicle.yaml` only" is not enforced for the lane finder. If it is run without its launch file, it falls back to positions written in the code: `camera_xyz` [1.80, 0.29, 1.20] and `camera_rpy` at `ros/perception/aav_lane_finder/aav_lane_finder/lane_finder.py:51-52`, and `wheelbase` 1.57 at `aav_lane_finder/lane_lines.py:50`.

## ros/perception/aav_detector/README.md

- The README says the launch file takes "Arguments as the parameters, plus `node_name`, `use_sim_time`". `launch/detector.launch.py:44-58` declares no `image_size` and no `publish_image` argument, so neither can be set from the launch file. The arguments it does declare beyond the parameters are `image`, `picture`, `node_name` and `use_sim_time`.
- Invariant "It holds Nav2 (20) and the AI slot (10)" leaves out `goto` (25). The lock holds it too: `ros/command_mux/aav_mux/config/twist_mux.yaml:46-49,63-66`.

## ros/perception/aav_lane_finder/README.md

- Invariant "The camera's place comes from `vehicle.yaml` only, never from this package" is not enforced. The node declares fallback defaults in the code: `camera_xyz` [1.80, 0.29, 1.20] and `camera_rpy` [0, 0, 0] at `aav_lane_finder/lane_finder.py:51-52`, and `Settings.wheelbase = 1.57` at `aav_lane_finder/lane_lines.py:50`. The README's table says these values come "from `vehicle.yaml` (launch)", but they only do when the launch file sets them.

## ros/perception/aav_lane_geometry/README.md

- Invariant "No lines seen, or no message for 2 s: the car slows at `decel_rate` and stops" is only half true:
  - **No message for 2 s:** the watchdog sends a zero `Twist` straight away (`scripts/control_interface.py:239-242`), which is a sudden stop, not a slow-down.
  - **No lines seen:** the slow-down steps only run inside `geometry_cb` (`:106-124`), so it slows only while messages keep arriving.
- Invariant "The scripts run through `python3`, not their `#!` line" is broken by `launch/control_test.launch.py:13-26`. That file starts both scripts with no `prefix='python3'`; only `lane_follower.launch.py:29,31` sets it.
- The parameter `stop_wait_s` (listed as 2 s) is declared and read (`scripts/control_interface.py:58,74`), but nothing uses `self.stop_wait`, so it has no effect.
- `fake_lane_geometry_pub.py` has a public parameter the README doesn't mention: `lane_width_m`, default 3.6 (`scripts/fake_lane_geometry_pub.py:15`). The README lists only `mode` and `rate_hz`.

## ros/perception/aav_lidar/README.md

No drift detected.

## ros/perception/aav_radar_clusters/README.md

No drift detected.

## ros/localization/README.md

No drift detected.

## ros/localization/aav_gps_heading/README.md

No drift detected.

## ros/localization/aav_localization/README.md

No drift detected.

## ros/safety/README.md

- Invariant "The watchdog lock (priority 30) holds Nav2 and the AI slot only" is incomplete. It also holds `goto` at priority 25: `ros/command_mux/aav_mux/config/twist_mux.yaml:46-49,67-70`.

## ros/safety/aav_ghost/README.md

No drift detected.

## ros/safety/aav_safety/README.md

- The state table lists `NO FIX` as a state, but the watchdog never reports it. `NO FIX` is only an internal flag (`aav_safety/watchdog.py:164-166`):
  - before `max_age` runs out, the input still reads `OK`;
  - after that it reports `STALE` with the words "no fix ..." (`watchdog.py:210-214`).
  - The HUD, the log and `/diagnostics` never show `NO FIX`.
- Invariant "The lock holds Nav2 (20) and the AI slot (10)" leaves out `goto` (25), which the sensors lock (priority 30) also holds: `ros/command_mux/aav_mux/config/twist_mux.yaml:46-49,67-70`.

## ros/ecolo_msgs/README.md

No drift detected.

## ros/ecolo_msgs/aav_interfaces/README.md

- "Every message starts with `std_msgs/Header header`" is not true for one message. `msg/SensorSource.msg` has no header; its first field is the constant `REAL=0`, then `name`, `topic`, `source`.

## ros/sim/README.md

- The invariant "A command older than 0.2 s means speed 0" is only true for `gazebo_adapter` and `carla_adapter`.
  - `turtlesim_adapter` has no command timeout. `ros/sim/aav_turtlesim/aav_turtlesim/turtlesim_adapter.py:60-69` forwards each command and nothing ever zeroes it.
  - `spine_sim` (bench) runs spine-core with `..SpineConfig::default()` (`ros/sim/aav_spine_sim/crates/spine-sim/src/main.rs:111-114`). That default is `cmd_timeout_ms: 100` (`firmware/spine/crates/spine-core/src/spine.rs:146`), so the bench's timeout is 0.1 s, not 0.2 s.
- The invariant "limits come from `vehicle.yaml`" does not hold for the bench spine. `spine_sim` uses spine-core's built-in limits (`max_speed: 3.0`, `max_accel: 1.5` and so on, `spine.rs:143-150`). From `vehicle.yaml` it only gets `wheelbase`, `track`, `wheel_radius` and `max_steering`, and those go to the model car (`ros/bringup/aav_bringup/launch/bench.launch.py:112-116`).

## ros/sim/aav_bench/README.md

- `frame_id` is a public parameter that the README never mentions. `mock_imu`, `mock_gps`, `mock_camera` and `mock_radar` each declare it (`aav_bench/sensors.py:73`, `:112`, `:178`, `:248`).
- The heading says "Parameters (defaults in `config/bench.yaml`)", but several defaults are not in that file:
  - `datum` comes from `navsat.yaml`, passed in by `bench.launch.py:121`;
  - `seed`, `host`, `port` and `frame_id` are only defaults in the code (`sensors.py:77,116,264,309-310`).

## ros/sim/aav_carla/README.md

No drift detected.

## ros/sim/aav_gazebo/README.md

- The nodes table says `gazebo_radar` runs "only with `radar:=true` in the road world". In fact it runs in any world that renders sensors, and `radar` defaults to true:
  - `radar = sensors and flag(context, 'radar')` (`launch/gazebo.launch.py:133`, node at `:182-184`);
  - `test_lot.sdf` loads the Sensors system (`worlds/test_lot.sdf:25`);
  - so the test lot gets the radar too.
  - The same applies to the lidar (`:132`, default `true` at `:238`). The README's "(with `lidar:=true`)" under the road world reads as if it were opt-in and road-only.
- The table says `gazebo_camera` runs "only with `faults:=true`". It also needs a world that renders sensors: `relay = sensors and flag(context, 'faults')` (`gazebo.launch.py:134`).
- The invariant "No command for `cmd_timeout` (0.2 s): target speed 0, as the spine's watchdog" gets the spine's number wrong. The spine's watchdog is 0.1 s (`firmware/spine/crates/spine-core/src/spine.rs:146`).

## ros/sim/aav_spine_sim/README.md

- The `bench-car` table leaves out public API that the node depends on:
  - `Bench::outputs()`, which `spine_sim` calls in `car:=gazebo` (`crates/spine-sim/src/main.rs:165`);
  - `Bench::pose()`, `Bench::car()` and `Bench::car_mut()` (`crates/bench-car/src/bench.rs:69,187,192,197`);
  - the `Frame` type (`bench.rs:38`);
  - the re-exports `SpeedLoopProfile` and `SpineConfig` (`crates/bench-car/src/lib.rs:14-21`).
- The invariants mention only `PlantConfig::default`. They don't say that the spine itself also runs on `SpineConfig::default()` (`main.rs:111-114`). That gives a 100 ms command watchdog and the firmware's own speed and acceleration caps (`spine.rs:146-150`), none of them from `vehicle.yaml`.

## ros/sim/aav_turtlesim/README.md

No drift detected.

## ros/sources/README.md

No drift detected.

## ros/sources/aav_camera_stream/README.md

- The launch "Arguments" list leaves out five arguments that the launch file declares: `transport`, `preview_width`, `publish_image`, `publish_camera_info` and `use_sim_time` (`launch/camera_stream.launch.py:50,61-63,66`).
- The README says `/sensors/camera/<camera>/camera_info` goes out "with each detection". The publisher is only created when there is a calibration: `if publish_info and self.lens` (`aav_camera_stream/camera_stream.py:175-176`).
  - Both cameras have `calibration: ''` in `config/cameras.yaml`, so no `camera_info` is published today.
- The invariant "The default decoder is picked per machine; the Orin uses `gst-nvidia`" is misleading. `decoders.choose('auto')` never picks `gst-nvidia`: it gives gst-mac on a Mac, gst-soft on Linux, and ffmpeg otherwise (`aav_camera_stream/decoders.py:164-192`). The Orin gets `gst-nvidia` from `car.launch.py`'s machine settings (`ros/bringup/aav_bringup/launch/car.launch.py:305`), not from the node.
- One side effect is missing: the launch file writes a temporary parameters file for each camera and deletes it at exit (`camera_stream.launch.py:108-111`).

## ros/sources/aav_imu_yahboom/README.md

No drift detected.

## ros/sources/aav_sensors/README.md

- The Overview lists only `/sensors/gps/fix` for the GPS. The drivers publish more:
  - `ublox_gps` also publishes `/sensors/gps/fix_velocity` (`launch/sensors.launch.py:82-83`);
  - `gps_nmea` also publishes `/sensors/gps/vel` and `/sensors/gps/time_reference` (`launch/gps_nmea.launch.py:36-37`).

## ros/sources/battery_monitor/README.md

- The README says "Both publish once per good block that holds `V`, `I`, `SOC` and `VS`", but the parser keeps values from earlier blocks:
  - `VEDirect.set_block` merges each good block into `self.data` (`battery_monitor/VEDirect.py:88`);
  - `dataChanged` checks that merged dict, not the block that just arrived (`battery_monitor/battery_monitor.py:27-33`);
  - so once all four keys have been seen, every good block publishes, even one without those keys. The message then carries old values for the keys that block didn't hold.

## ros/sources/ti_ros2_driver/README.md

- The README says a cfg "named `3d_tracking_*`" selects tracking mode. The code checks for `'tracking' in cfg_path_.split('.')[0]` (`ti_ros2_driver/TI.py:57`). That means:
  - any path containing "tracking" switches mode, so `cfg/AOP_6m_tracking_default.cfg` does too;
  - a `.` in a directory name cuts the path short before the file name is checked.
- The mode table gives the tracking-mode fields as "x, y, z, velocity_xyz, acceleration_xyz". In tracking mode the main `<topic>` still carries 5 columns, published as x, y, z, intensity, velocity (`TI.py:259`; `pc_publisher.py:207`, `:247-252`). The 9-field layout appears only on `<topic>_target` (`pc_publisher.py:316-324`).
- `mock_radar_publisher`'s parameters are not documented: `topic` (`/ti_radar_0/ti_mmwave_0`), `frame_id` (`radar_link`), `publish_rate_hz` (10.0) and `mode` (`moving`) (`ti_ros2_driver/mock_radar_publisher.py:16-19`).
- The "Drift and known gaps" section says "9 m range". `base.cfg` sets 8.4 m (`cfg/base.cfg:50`, `cfarFovCfg -1 0 0 8.40`), which is what README line 72 also says.

## ros/README.md

- The area table leaves out two packages. The `drivers` row lists "joystick, RC, tests, scripted, goto, Nav2, language" but not `aav_routes`. That package has the `street_router` node (`ros/drivers/aav_routes/setup.py`, console script `street_router = aav_routes.street_router:main`), which publishes street routes as GPS waypoints and as a path. The `sim` row lists "turtlesim, Gazebo, bench mode, CARLA" but never names `aav_spine_sim` (`ros/sim/aav_spine_sim/`). That is the Rust `spine_sim` node, which bench mode and `sim spine:=true` run.

## ros/bringup/README.md

No drift detected.

## ros/bringup/aav_bringup/README.md

No drift detected.

## ros/bringup/aav_description/README.md

- The `actuators` section is described as "the actuator model for Gazebo", but other code reads it too:
  - `vehicle.ego_params()` passes `actuators.max_accel` and `max_brake` to ego as `full_throttle_accel` and `full_brake_decel` (`ros/bringup/aav_description/aav_description/vehicle.py:171-181`);
  - `ros/drivers/aav_goto/launch/goto.launch.py` and `ros/sim/aav_carla/aav_carla/carla_adapter.py` read it;
  - `vehicle.yaml:58` says `steer_rate` is spine_sim's.
- The `vehicle.py` function table leaves out some public symbols:
  - `config_path()` (`vehicle.py:31`), which the README only mentions under Invariants;
  - `footprint_points(v)` (`:89`) and `polygon(points)` (`:98`);
  - `CONTROLLERS = ('rpp', 'mppi')` and `DEFAULT_CONTROLLER = 'mppi'` (`:227-228`), which `sim.launch.py` uses as the default of its `controller` argument;
  - the constants `STOP_ZONE_AHEAD`, `STOP_ZONE_BEHIND`, `STOP_ZONE_MARGIN`, `SEES_BEHIND` and `PLAN_TURN_MARGIN`.

## ros/can_bridge/README.md

- Invariant "A stale command or an e-stop sends brake frames; the car brakes within 0.2 s" is not true for `legacy_uart_bridge`, today's backend:
  - its `cmd_timeout` is 0.3 s (`ros/can_bridge/aav_legacy/config/legacy.yaml:8`; `legacy_uart_bridge.py:60`);
  - with `frame_version: 1`, the car's current setting (`legacy.yaml:14`), the ESP32 has no timeout at all (`legacy_uart_bridge.py:30-31`).

  The 0.2 s bound holds only for `spine_bridge`: 0.1 s plus the spine's 100 ms watchdog (`aav_spine/config/spine.yaml:5-8`).
- Invariant "Bridges read only `/control/car/actuator_cmd`" is too narrow. `spine_bridge` reads `/control/bench/actuator_cmd` or `/control/gazebo/actuator_cmd` when `source` is `bench` or `sim` (`ros/can_bridge/aav_spine/aav_spine/spine_bridge.py:76-77, 130`; the Rust twin is `crates/bridge-node/src/main.rs:377-379`). The diagram shows only `/control/car/actuator_cmd` too. The sibling `aav_spine` README already says `/control/<target>/actuator_cmd`.

## ros/can_bridge/aav_legacy/README.md

- The parameter table leaves out `encoder_udp`'s `speed_variance` (0.05, the (m/s)² covariance on `/vehicle/odom`). It is in `ros/can_bridge/aav_legacy/config/legacy.yaml:42` and `aav_legacy/encoder_udp.py:37`.

## ros/can_bridge/aav_spine/README.md

- The table headed "Parameters (`config/spine.yaml`)" lists `wheel_timeout` (0.1), `source` and `role`, but none of them is in `config/spine.yaml`:
  - `wheel_timeout` and `source` come only from the node's own defaults (`aav_spine/spine_bridge.py:98, 106`);
  - `role` comes from the node default (`:109`) or from `spine.launch.py:49`.
- `spine.launch.py` also declares `use_sim_time` (default `false`, `launch/spine.launch.py:77`), which the launch line doesn't list.

## ros/can_bridge/aav_spine_bridge/README.md

No drift detected.

## ros/can_bridge/aav_spine_bridge/crates/spine-bridge/README.md

- Some public items are missing from the Types table: `UnknownRole` (`src/bridge.rs:71`), `UnknownTarget` (`src/gate.rs:48`), `wheels::Side` (`src/wheels.rs:60`), and the constant `Role::NAMES` (`src/bridge.rs:61`). `lib.rs` re-exports the first three at the crate root.
- The README says it re-exports "the ids" from `spine_core::protocol`. In fact it re-exports only `DRIVE_CMD_ID` and `ESTOP_CMD_ID` (`src/lib.rs`, the last `pub use` line), not the feedback, status or wheel ids.

## ros/hud/README.md

- Invariant "Topic names live in config files, not in code" holds for `aav_hud_raylib` but not for `aav_hud`. `hud_server.py` hard-codes every topic and action name:
  - `/vehicle/status`, `/localization/gps/fix`, `/perception/stop_sign`, `/vehicle/battery`, `/safety/estop` and `/follow_gps_waypoints` (`ros/hud/aav_hud/aav_hud/hud_server.py:115-122`);
  - `/sensors/camera/front_left/image_raw` (`:182-183`).

  `config/hud_server.yaml` has no topic parameters, and the GPS topic can only be changed through a launch remapping (`launch/hud.launch.py:47`).

## ros/hud/aav_hud/README.md

- The `events.py` row lists `speed_event`, `vehicle_event`, `position_event`, `battery_event` and `Throttle`. It leaves out the public `stop_sign_event` (`aav_hud/events.py:116`), `fix_position` (`:94`), `gear` (`:54`) and `fault_names` (`:70`). `stop_sign_event` makes the `stop_sign` event the README lists.

## ros/hud/aav_hud_raylib/README.md

- The `pixi run map` row gives `[fullscreen:=true] [streets:=] [places:=]`, but `map.launch.py` also declares `monitor` (`launch/map.launch.py:60`), the same argument the `hud` row lists.

## ros/hud/aav_hud_raylib/crates/hud-core/README.md

No drift detected.

## ros/hud/aav_hud_raylib/crates/map-core/README.md

- The README never says that map-core depends on `hud-core`. It uses `hud_core::health`, `origin`, `units` and `watchdog` (`crates/map-core/Cargo.toml`, the `hud-core = { path = "../hud-core" }` line; `src/state.rs:10-15`; `src/trip.rs:9`). The invariant "No ROS and no raylib" still holds, but the dependency list is incomplete.

## firmware/spine/README.md

- The Tests column is wrong for five crates. Counting `#[test]` functions plus doc tests, the same way the other rows count (aav-inputs 9 = 8 + 1 doc test; wheel-encoder 12 = 11 + 1 in `tests/`):
  - `spine-core`: the README says 70; there are 83. That is 70 in `src/`, 12 in `tests/vectors.rs` and 1 in `tests/speed_loop_vectors.rs`.
  - `legacy-link`: the README says 31; there are 72. That is 23 in `src/` and 49 in `crates/legacy-link/tests/vectors.rs`.
  - `spine-board`: the README says 57; there are 79, all in `src/`: actuators 13, spine_board 32, link 14, wheel 7, calibration 8, stand_in 3, board 2.
  - `ibus`: the README says 24; there are 25. The extra one is the doc test at `crates/ibus/src/lib.rs:17`.
  - `jrk-g2`: the README says 14; there are 15. The extra one is the doc test at `crates/jrk-g2/src/lib.rs:20`.
- Line 103 says the ESP32 takes commands "from the Motor-Pi's UART (default) or CAN (`--features can`)". `--features can` on its own fails to build:
  - `boards/esp32-umod/Cargo.toml:20` makes `legacy-uart` a default feature;
  - `boards/esp32-umod/src/main.rs:58-61` stops the build with `compile_error!` when `legacy-uart` and `can` are both on;
  - the right flags are `--no-default-features --features can`, which is what the message there and the `esp build` task's description (`pixi.toml:95`) say.
- Lines 62-63 say "`pixi run test` checks it" (every crate builds for `thumbv8m.main-none-eabihf`), and lines 91-92 say the same for the board programs. `tools/test.sh:48` only runs those builds when that target is installed. If it isn't, they are skipped and nothing is printed.

## firmware/spine/crates/aav-inputs/README.md

No drift detected.

## firmware/spine/crates/ad5311/README.md

No drift detected.

## firmware/spine/crates/hd44780/README.md

No drift detected.

## firmware/spine/crates/ibus/README.md

- `RcMap` has public settings fields the README never mentions (`crates/ibus/src/rc.rs:36-58`): `mode_switch`, `throttle`, `steering`, `gear`, `valid_above`, `valid_below`, `direct_above`, `via_ros_above`, `forward_below`, `reverse_above`, `centre`, `half_travel`, `throttle_deadband`. The README describes only the two presets, `TODAY` and `THREE_POSITION`.

## firmware/spine/crates/jrk-g2/README.md

- Line 26 says `feedback_mode()` returns `None`, `Analog` or `Frequency`. `FeedbackMode` has a fourth variant, `Other(u8)` (`crates/jrk-g2/src/lib.rs:53`), and a public `FeedbackMode::from_byte` (`lib.rs:58`). Neither is mentioned.
- The `ErrorFlags` bit constants are public but not listed (`crates/jrk-g2/src/flags.rs:13-32`: `AWAITING_COMMAND`, `NO_POWER`, `MOTOR_DRIVER`, ... `HARD_OVERCURRENT`). These are the `bits` that `ErrorFlags::contains(bits)` takes.

## firmware/spine/crates/legacy-link/README.md

- Public items the README never names:
  - `Flags::to_byte` and `Flags::from_byte` (`src/v2.rs:36,42`);
  - `Heartbeat::new` and `Heartbeat::last` (`src/v2.rs:139,151`);
  - `Parser::new` and `ParserV2::new` (`src/parser.rs:17,58`).
- Line 75 lists a bad "kind" as its own error. In the code a wrong kind byte comes back as `FrameError::Version` (`src/uplink.rs:180-181`), because `FrameError` has no `Kind` variant. The README's list of `FrameError` variants is correct; the invariant just suggests a separate error that doesn't exist.

## firmware/spine/crates/mcp2515/README.md

- Public types the README never names:
  - `Crystal` (`MHZ_8`, `MHZ_16`, `from_hz`) and `Bitrate` (`KBPS_125`, `KBPS_250`, `KBPS_500`, `MBPS_1`, `from_bps`) in `src/timing.rs:19-49`. These are the arguments `BitTiming::new` takes.
  - `FilterError` (`NoIds`, `TooManyIds`), `Filters::ids()` and `MAX_IDS` (`src/filter.rs:14,30,72`).
  - The type names `ErrorState` and `BusStatus`, which `bus_status()` returns (`src/lib.rs:152,179`).
  - `BitTiming::quanta()`, `segments()` and `bitrate()` (`src/timing.rs:122,134,145`).
  - `Frame::id()`, `data()`, `dlc()` and `is_remote()` (`src/frame.rs:62-80`).

## firmware/spine/crates/pca9555/README.md

No drift detected.

## firmware/spine/crates/sentinel/README.md

- Page 9 (Computers) is shown as `JET+ MPI+ RPI-`, which suggests three computers. It shows five: `UPI` and `CPI` are on its second line (`src/telemetry.rs:174-180`; the test at `src/pages.rs:267` expects `["JET+ MPI+ RPI+  ", "UPI- CPI+       "]`).
- The board program needs the public `Knob` (`new`, `update` → `Option<Turn>`, `src/controls.rs:34-51`) and `Button` (`new`, `pressed`, `STABLE_MS = 20`, `src/controls.rs:85-96`) to turn pins into `Input`. Neither is in the Interface table; only the `controls` module is listed.

## firmware/spine/crates/spine-board/README.md

- Line 44 says each tick does "3 reads, 4 writes". `Actuators::read` does up to 4 reads a tick (`src/actuators.rs:128-157`): each Jrk's error flags, plus each Jrk's scaled feedback when it has analog feedback. There is also a feedback-mode read once per Jrk (`src/actuators.rs:286-297`).
- Line 24 (One tick, step 5) says it reads "the steering Jrk's feedback". It reads both Jrks' positions: `Positions { steering, brake }` (`src/actuators.rs:62-68,128-157`). Line 64 (`Actuators::read()`, "the steering angle and the Jrks' faults") also leaves out the brake position it returns.
- The "Today's numbers" table leaves out `Calibration.wheel.timeout_ms = 1000`: with no wheel count for that long, the car counts as stopped (`src/calibration.rs:157`).
- Public items the README never names:
  - `FEEDBACK_PERIOD_MS` and `STATUS_PERIOD_MS` (`src/spine_board.rs:30-31`);
  - `SpineBoard::board()`, `link()` and `actuators()` (`src/spine_board.rs:145-155`);
  - `LegacyLink::stats()` and `uplink_frame()` (`src/link.rs:223,228`);
  - `Actuators::new`, `init`, `throttle_code`, `reverse`, `devices` and `release` (`src/actuators.rs:100-207`; `init` appears only in the Ported-from table);
  - `CanFrame::new`, `id`, `data` and `MAX_ID` (`src/board.rs:27-49`);
  - `WheelSpeed::new` (`src/wheel.rs:21`).

## firmware/spine/crates/spine-core/README.md

- The invariant at line 86, "Reverse never faster than `MAX_REVERSE_SPEED` (0.5 m/s), whoever drives", isn't enforced for open-loop `DirectCmd` commands from the UART link:
  - `Spine::direct` (`src/spine.rs:526-555`) passes `cmd.throttle.clamp(0.0, 1.0)` with `cmd.reverse` and no reverse cap;
  - the cap is applied only on the speed paths (`src/spine.rs:456`, `:491`);
  - with `SpeedFeedback::None` (today's ESP32) the overspeed check is off too (`src/spine.rs:406-411`), so nothing limits reverse speed;
  - with a wheel speed, the overspeed fault only trips above 0.5 + 0.5 = 1.0 m/s.
- Public items the README never mentions:
  - the CAN ID constants `ESTOP_CMD_ID` … `WHEEL_SPEED_R_ID` (`src/protocol.rs:8-13`);
  - `protocol::FrameError`, `Incoming` and `Report` (`src/protocol.rs:16,137,153`);
  - `SpeedLoop::open_loop` and `reset`, the `Drive` return type, `AntiWindup`, `UnknownProfile` and `SpeedLoopConfig::smooth()` (`src/speed_loop.rs:22,78,123,144,169,255`);
  - the `EstopSteering` and `RcGearBrake` enum types (`src/spine.rs:84,93`; only their values appear).

## firmware/spine/crates/wheel-core/README.md

No drift detected.

## firmware/spine/crates/wheel-encoder/README.md

- The Interface table lists `Reporter::report` but not `Reporter::new(now: WheelCount)`, which you must call first (`src/packet.rs:76`). It also leaves out:
  - `SpokeSensors::new(lockout_ms, direction_window_ms)` (`src/spokes.rs:36`);
  - the `Direction` enum that `direction()` returns (`src/packet.rs:23`);
  - `SpokeDelta::LEN` (`src/packet.rs:31`).

## firmware/README.md

- The invariant "Safe after any reset: throttle 0, brake on, latched" is only partly true. Throttle 0 and latched are enforced, but the brake does not stay on with the shared default settings:
  - `IDLE_BRAKE` is `0.0` (`firmware/spine/crates/spine-core/src/spine.rs:65`), and `SpineConfig::default()` uses it (`spine.rs:155`).
  - After boot the spine is in Manual mode. Latched alone does not take the stop path (`spine.rs:428`), so the Manual branch outputs `c.idle_brake`, which is 0 (`spine.rs:443`).
  - `spine-board` applies that brake value on every tick (`firmware/spine/crates/spine-board/src/spine_board.rs:131-132`). So the brake that `init` sets is released on the first tick.
  - The test `the_idle_brake_after_boot` asserts brake `0.0` after boot (`spine.rs:1065-1068`).
  - Only `esp32-umod` overrides this to `1.0` (`firmware/spine/boards/esp32-umod/src/main.rs:117`).

## firmware/can/README.md

No drift detected.

## firmware/udp/README.md

No drift detected.

## firmware/legacy/README.md

No drift detected.

## firmware/legacy/aav_umod_esp32/README.md

- The Files table says `src/modules/hardware_configs.h` holds "the pins". The Motor-Pi UART1 pins (16/17) and its baud rate are hard-coded in `src/main.cpp:93` (`Serial1.begin(115200, SERIAL_8N1, 16, 17)`), not in `hardware_configs.h` (`hardware_configs.h:6-22` has only the iBUS, LED, DIR and I2C pins).
- The Files table leaves out `src/modules/globals.h`. `main.cpp:2` and every module include it.

## firmware/spine/boards/esp32-umod/README.md

- The Verification section says `tools/esp32.sh check` runs "clippy on every feature mix". It runs clippy on six mixes only (`tools/esp32.sh:172-178`): the default, `can,estop-input`, `qemu`, `link-v1`, `uplink`, and `qemu,uplink`. These legal mixes are never checked:
  - `link-v1` with `uplink`;
  - `can` without `estop-input`;
  - `estop-input` on the UART link.

## firmware/spine/boards/pico2/README.md

- The invariant "Safe after any reset: `SpineBoard::init` puts the throttle at 0 and the brake on" does not hold past the first tick.
  - This board uses `SpineConfig::default()` (`firmware/spine/boards/pico2/src/main.rs:222`), whose `idle_brake` is `IDLE_BRAKE = 0.0` (`spine-core/src/spine.rs:65,155`).
  - On the first tick after boot, the spine is in Manual mode and outputs brake 0 (`spine.rs:443`; tested at `spine.rs:1065-1068`). `spine-board` writes that out (`spine_board.rs:132`).
  - Throttle 0 and the boot latch do hold.

## firmware/spine/boards/sentinel-pico2/README.md

No drift detected.

## firmware/spine/boards/wheel-encoder-pico2/README.md

No drift detected.

## examples/README.md

No drift detected.

## examples/rust-listener/README.md

No drift detected.

## tools/README.md

- **`mcp_parts.py` is not standard-library only.** The Overview table says "the maths behind one (`topic_load.py`, `stalls.py`, `drive_metrics.py`, `mcp_parts.py`) | Python 3, standard library". But `tools/mcp_parts.py:20` does `import yaml` (PyYAML). The other three use only the standard library (`topic_load.py:19-20`, `stalls.py:19-23`, `drive_metrics.py:23-25`).
- **`replay_commands.sh` has no `--sim`.** The Side Effects row says "`actuator_test.sh`, `goto.sh`, `keyboard.sh`, `replay_commands.sh` without `--sim` | move the real car". `tools/replay_commands.sh:9-12` takes no flags. It always runs `ros2 bag play` on whatever shares the domain, and never goes through `tools/ros.sh` or `--local`. `keyboard.sh` also stays local with `--local`, not only `--sim` (`tools/keyboard.sh:8-11`).
- **`goto.sh` caps no speed itself.** The invariant says `actuator_test.sh` and `goto.sh` "cap the speed (1 m/s; 0.3 m/s on `--stands`)".
  - `tools/goto.sh:7` says "ego caps the speed at 1 m/s". Neither `goto.sh` nor `aav_goto/rules.py` enforces a cap.
  - `goto.sh` has no `--stands`.
  - Only `actuator_test` caps its own speed (`ros/drivers/aav_actuator_tests/aav_actuator_tests/actuator_test.py:14-15`).
- **`car_day_build.sh` puts the docs PDF in `docs/html/`, not `car-day/`.** The row says "builds the ESP32 firmware, QEMU check, docs PDF into `car-day/`". The PDF goes to `docs/html/aav-docs.pdf` (`tools/car_day_build.sh:93`). The script is also missing from Side Effects: it writes `car-day/esp32/` and `docs/html/`.
- **`legacy_firmware.sh` writes a file.** With no `src/secrets.h`, it copies `secrets.h.example` into `firmware/legacy/aav_umod_esp32/src/secrets.h` (`tools/legacy_firmware.sh:10-13`). Side Effects lists only `-t upload`.
- **Download targets are incomplete.** Side Effects says `get_*.sh` download into "`vendor/`, `models/`, `maps/`, `car-kit/`". Two more places get written:
  - `get_hud_map.sh` also links or moves `ottawa.db` into `hud/flutter_pi/assets/databases/` (`tools/get_hud_map.sh:2-4,16`).
  - `car_kit.sh` also caches in `vendor/car-kit-cache` (`tools/car_kit.sh:24`).
- **`ros.sh` reads a per-computer file the README doesn't mention.** It appends `~/.config/aav/cyclonedds.xml` to `CYCLONEDDS_URI` when that file exists (`tools/ros.sh:50-53`). The README says only "on Cyclone DDS" with `tools/cyclonedds.xml`.

## tools/foxglove/README.md

- **The `sensors-health.json` description misses two panels.** The README says "every `/diagnostics` status …; a timeline of stops". The layout also has an Indicator and a RawMessages panel on `/system/sensor_mux/state` ("Sources: every sensor as asked"), and a RawMessages panel on `/safety/sensor_watchdog/state` (`tools/foxglove/sensors-health.json`, `Indicator!sources`, `RawMessages!sources`, `RawMessages!watchdog`). Its timeline also tracks the output gate and "sources ok", not only stops.

## tools/screens/README.md

No drift detected.

## tools/udev/README.md

No drift detected.

## hud/README.md

No drift detected.

## hud/flutter_pi/README.md

- **The map page uses the computer's location service (geolocator).** It requests permission and reads a fix from it (`lib/screens/map_page.dart:11,109-143`). Neither Interface nor Side Effects mentions this. The README lists only the back end, OSRM and the tile server.
- **The map page's simulated trace is not mentioned.** It has a "Simulate Trace" feature: paste CSV lat/lon, then play, pause, step or stop a fake position along it (`lib/screens/map_page.dart:507-575,954-989,1048`). The map row lists only "the map, place search, an OSRM route, Lock/Unlock/Clear".
- **`constants.dart` has more settings than listed.** It also defines `kAppTitle` and `kColorSchemeSeed` (`lib/data/constants.dart:13,15`). The table lists four names.

## hud/scripts/README.md

No drift detected.

## README.md

- **"What's where", `ros/` row.** It ends "ecolo_msgs the interfaces profiles". That reads garbled. `ros/ecolo_msgs/` holds `aav_interfaces`, "the team's own message and action types" (`ros/README.md:31`).
- **"What's where", `firmware/` row.** It lists `can/`, `spine/` and `legacy/` only. `firmware/udp/` exists too, with its own README: the wheel encoder's UDP packet vectors (`firmware/udp/README.md`).
- **"What's where", `models/` row.** It says "the YOLO models, downloaded by `pixi run get models`". `tools/get_models.sh:6-9` downloads RF-DETR Nano (the detector's default) and YOLO11n. `models/` also holds exports (`.onnx`, `.mlpackage`).
- **"What's where", `vendor/` row.** It says "the ROS 2 and ros2-rust source". `vendor/` also gets Flutter (`tools/get_flutter.sh`), MediaMTX (`tools/get_mediamtx.sh`), PlatformIO's downloads (`pixi.toml`, `PLATFORMIO_CORE_DIR`), QEMU (`tools/esp32.sh qemu-tools`), `cargo-tools`, `esp` and `car-kit-cache`.
- **"What's where" leaves out `archive/`** (tracked in git: the 2025-26 repos' files) and `.github/` (the CI workflow and PR template).
- **"Global side effects", Downloads.** It says downloads go into "`vendor/`, `models/`, `maps/` and `car-kit/`". `get_hud_map.sh` also writes `hud/flutter_pi/assets/databases/ottawa.db` (`tools/get_hud_map.sh:16`).
- **"System verification", check profiles.** `pixi run check <turtle|sim|car|replay>` lists four profiles. `check_graph.py` and the `pixi.toml` `check` task also take `road`, `spine` and `bench` (`tools/check_graph.py:5`, `pixi.toml` `check` description).

