# aav_legacy

## Overview

The interim backend for the 2025-26 hardware, until the CAN spine drives the car.
`legacy_uart_bridge` drives the ESP32 over UART from ego's commands. `encoder_udp`
turns the 2025-26 UDP wheel encoder into `/vehicle/odom`. `car.launch.py` starts the
bridge with `actuators:=uart` (today's wiring), on the Motor Pi.

**Read docs/firmware.md before driving.** The ESP32's 2025-26 C++ firmware has no
command timeout: if the bridge dies, the last throttle stays on. So with
`frame_version: 1` the bridge locks the programs out (the no-timeout lock, below).

### The speed loop (`speed_loop.mode`)

| Mode | Throttle | Brake |
|---|---|---|
| `open` (default) | ego's speed ÷ `full_throttle_speed`, open loop | only below `stop_speed`: `hold_brake` |
| `pi` | feed-forward (the slope of ego's speed) + PI on `/vehicle/odom`, inverted through the car model | the same loop, whenever it asks to slow faster than coasting does |

`pi` never sends throttle and brake together, and with no fresh wheel speed it runs on
feed-forward alone. Its PI compares the car with ego's speed delayed by `response_lag`
(the actuators' lag), so it doesn't brake harder to win back a lag it expects.
The **smooth stop** (Autoware's design, Apache-2.0, our own code): when ego reaches 0 the car
keeps slowing as it was, then the slowing fades with the speed, so it stops without a
jolt; stopped, the brake rises gently to `stopped_brake` and holds.
In Ravensim: `pixi run sim esp32:=true`, the C++ firmware standing in (`aav_sim_models`). The design: `docs/designs/2026-10-27_control-plan.md`, step 1.

### The adaptive brake (`adaptive_brake`, TODO 258)

Off by default. With `adaptive_brake: true` (`car.launch.py adaptive_brake:=true`) and
`speed_loop.mode: pi`, the sim's adaptive brake (`aav_description` `adaptive_brake.py`; the
Rust bridge runs its twin, `firmware/spine/crates/adaptive-brake`) sits between the pi loop
and the brake actuator: it brakes harder when the car slows less than the car model says,
**never above `vehicle.yaml`'s `brake: safe_max`**.

| Why here | |
|---|---|
| The brake level exists only after the speed loop | ego sends a speed, not a brake |
| Below every lock | the stops (stale command, e-stop, calibration) send `stop_command` as before, never through it |
| Today's car runs its speed loop here | `actuators:=uart`: the Motor Pi, before the ESP32's C++, which isn't ours to change |

With `open` it is refused (no brake to adapt). With `actuators:=can` the spine's firmware
runs the loop: `car.launch.py` refuses it there for now. Its car model (`adaptive_brake.*`)
and limits (`brake.*`) come from `vehicle.yaml` through `legacy.launch.py`.

### The no-timeout lock

| `frame_version` | `allow_programs_without_timeout` | `/safety/legacy_uart_bridge/lock` | Who drives |
|---|---|---|---|
| 1 (the C++, no timeout) | false (the default) | **true** | people (joy, RC, keyboard) and the actuator tests (60) only |
| 1 | true: **supervised stand only**, wheels off the ground | false | everyone |
| 2 or 3 (the Rust, brakes after 100 ms) | either | false | everyone |

twist_mux's `no_timeout` lock (priority 60, timeout 0) reads it. It holds every input
below 60: scripted, replay, point, Nav2, AI. Timeout 0, so where no legacy bridge runs
it never locks.

### Finding the steering actuator's ends and centre (1 Nov)

`frame_version: 3` carries the hardware panel's calibration request to the Rust firmware:
the steering actuator **limp** (its motor off, a person turns the wheels) or a **jog**
target in counts. The uplink sends back each Jrk's position in raw counts. The procedure
and its fill-in sheet: `docs/actuator-tests.md`.

| The bridge passes the request on only when | |
|---|---|
| `steering_calibration` is `stands` or `bench` (default `none`) | the hardware lead's say-so |
| `frame_version: 3`, `uplink: true`, an uplink frame within `uplink_timeout` | |
| no e-stop; ego asks for no speed; the wheel speed (if fresh) under 0.05 m/s | |
| the panel asked within 0.5 s | it asks at 10 Hz while its person holds |

Meanwhile its frames stand the car still: no speed, the brake full, not armed. The
firmware checks again and holds the throttle at 0 (`legacy-link`'s `SteerCal`).

`steering_limits` (no ROS) turns the marked left end, centre and right end into equal
limits each side of the centre (the smaller side less 5 %), the gain and the steering
limit, with warnings:

```sh
ros2 run aav_legacy steering_limits --left 3420 --centre 1890 --right 480 --left-deg 31 --right-deg 29
```

## Interface

### Executables

| Executable | Runs on | Does |
|---|---|---|
| `legacy_uart_bridge` | the Pi wired to the ESP32 (`/dev/ttyAMA0`) | commands to serial frames; with `uplink`, the ESP32's frames back |
| `steering_limits` | any laptop | the steering actuator's limits from its marked ends and centre (no ROS) |
| `encoder_udp` | the Motor Pi (the encoder sends only there) | UDP spoke counts to speed. Off since 23 Oct: the wheels are on CAN |

### `legacy_uart_bridge` topics

| Direction | Topic | Type |
|---|---|---|
| sub | `/control/car/actuator_cmd` | ackermann_msgs/AckermannDriveStamped |
| sub | `/control/car/heartbeat` | aav_interfaces/Heartbeat |
| sub | `/safety/estop` | std_msgs/Bool |
| sub, `speed_loop.mode: pi` only | `/vehicle/odom` | nav_msgs/Odometry: the wheel speed it closes on |
| pub | `/system/source` (latched, REAL) | aav_interfaces/DataSource |
| pub | `/safety/legacy_uart_bridge/lock` (latched, and again at 5 Hz) | std_msgs/Bool |
| pub | `/control/car/link_frame` (`link_frame_rate_hz`, default 10; every frame version) | aav_interfaces/LinkFrame: what really went out, decoded from the bytes written; `written` and `write_errors` count every frame |
| pub, `uplink` only | `/vehicle/status` (10 Hz) | aav_interfaces/VehicleStatus |
| pub, `uplink` only | `/vehicle/steering`, `/vehicle/brake` | aav_interfaces/ActuatorState |
| pub, `uplink` only | `/vehicle/throttle` | aav_interfaces/ActuatorState: command is the uplink's `throttle_out`, before the firmware's ramp; measured is NaN, as no sensor reads the throttle |
| pub, `uplink` only | `/vehicle/esp32/uplink_health` (1 Hz) | aav_interfaces/UplinkHealth: fresh, and the frames, lost and bad counted |
| pub, `uplink` only | `/vehicle/rc` | aav_interfaces/RcSticks |
| pub, `uplink` only | `/vehicle/steering_actuator/raw`, `/vehicle/brake_actuator/raw` (50 Hz) | aav_interfaces/ActuatorRaw: counts, not converted; the calibration state |
| pub, `uplink` only | `/vehicle/steering_actuator/calibration` | std_msgs/String: one line, what the calibration does and why |
| sub, `uplink` only | `/drivers/buttons/steering_calibration` | aav_interfaces/SteeringCalibration: the panel's limp or jog |

`encoder_udp` publishes `/vehicle/odom` (nav_msgs/Odometry, forward speed only, in
`base_link`).

### Parameters (`config/legacy.yaml`)

| Node | Name | Value | Unit |
|---|---|---|---|
| bridge | `port`, `baud` | `/dev/ttyAMA0`, 115200 | |
| bridge | `rate_hz`, `cmd_timeout` | 50.0, 0.3 | Hz, s |
| bridge | `frame_version` | 1 today (C++ firmware); 2 for the Rust firmware; 3 adds the steering calibration | |
| bridge | `steering_calibration` | `none`; `stands` or `bench` only with the hardware lead | |
| bridge | `allow_programs_without_timeout` | false; true only on a supervised stand | |
| bridge | `uplink`, `uplink_timeout` | false, 0.2 | s |
| bridge | `link_frame_rate_hz` | 10.0 (0.0: no samples; the counts still run) | Hz |
| bridge | `max_steering_angle` | from `vehicle.yaml` | rad |
| bridge | `steering_sign` | -1.0 (TODO check) | |
| bridge | `full_throttle_speed`, `stop_speed` | 3.0, 0.05 | m/s |
| bridge | `hold_brake`, `stop_brake` | 0.3, 1.0 | 0..1 |
| bridge | `speed_loop.mode` | `open` (the 2025-26 mapping; the car's, for now) or `pi` | |
| bridge | `speed_loop.kp`, `speed_loop.ki` | 2.0, 0.5 (guesses) | m/s² per m/s, per m/s·s |
| bridge | `speed_loop.max_accel`, `max_brake`, `drag` | from `vehicle.yaml` `actuators:` | m/s², m/s², 1/s |
| bridge | `speed_loop.response_lag` | 0.3 (guess): how far behind ego the car is expected to run | s |
| bridge | `speed_loop.hold_brake`, `speed_loop.odom_timeout` | 0.2, 0.3 | 0..1, s |
| bridge | `adaptive_brake` | false (true needs `pi`; TODO 258) | |
| bridge | `adaptive_brake.max_brake`, `brake_lag`, `brake_dead_time`, `drag`, `throttle_dead_time`, `throttle_lag`; `brake.released`, `touch`, `full`, `safe_max`, `max_hold_time` | from `vehicle.yaml` `actuators:` and `brake:` | m/s², s, 1/s; counts, s |
| bridge | `speed_loop.smooth_stop_speed`, `smooth_stop_rate`, `weak_stop_decel`, `weak_stop_time`, `strong_stop_decel`, `stopped_brake`, `stopped_brake_rate` | 0.3 (0: off), 1.5, 0.2, 2.0, 1.0, 0.5, 1.0 (guesses) | m/s, 1/s, m/s², s, m/s², 0..1, per s |
| encoder | `port`, `bind_address` | 10111, 0.0.0.0 | |
| encoder | `wheel_diameter`, `counts_per_rev` | from `vehicle.yaml` | m, counts |
| encoder | `window_s`, `timeout_s` | 0.5, 0.3 | s |
| encoder | `speed_variance` (on `/vehicle/odom`) | 0.05 | (m/s)² |

### Launch

`legacy.launch.py [uart:=true] [encoder:=true] [rc:=true] [legacy_impl:=rust] [rc_impl:=rust]
[port:=] [frame_version:=] [adaptive_brake:=false]`: the bridge, the encoder, and
`rc_driver` beside the bridge.
`port` and `frame_version` (empty: `legacy.yaml`'s) change one run without editing the file,
in both languages: the bridge has `namespace='/'`, so launch's values beat the YAML in Rust
too (`docs/languages.md`, "Launch overrides"; `aav_bringup`'s `test_rust_overrides.py`).

| Switch | `rust` (the default since 17 Nov, Ben: "Rust tomorrow") | `python` (the fall-back) |
|---|---|---|
| `legacy_impl` | `aav_legacy_rs`'s `legacy_uart_bridge` | this package's |
| `rc_impl` | `aav_rc_rs`'s `rc_driver` | `aav_rc`'s |

The same node names, topics, parameters and frames: `aav_legacy_rs`'s vectors check every
frame byte for byte. The chain tests (`test_heartbeat_chain.py`, `test_uplink_chain.py`,
`test_no_timeout_lock.py`, `test_steering_calibration_chain.py`) run each test with both
(`test/conftest.py`'s `impl`); the heartbeat chain's Rust run uses the Rust muxes too.

### Modules

| Module | ROS? | Holds |
|---|---|---|
| `legacy_uart_bridge.py`, `encoder_udp.py` | yes | the nodes (the bridge's Rust twin, the default: `aav_legacy_rs`) |
| `legacy_protocol.py` | no | the frames: `pack_frame` (v1, 13 bytes), `pack_frame_v2` (17 bytes), `pack_frame_v3` (20 bytes), `unpack_uplink` (35 or 40 bytes), `UplinkParser`, `to_legacy`, `from_pedals`, `stop_command`, `EncoderSpeed` |
| `uart_relay.py` | no | `Relay`: what to write each tick; `programs_locked`: the no-timeout lock |
| `speed_control.py` | no | `SpeedControl`: feed-forward from ego's speed profile plus PI on the wheel speed, out as throttle or brake; the smooth stop |
| `uplink_reader.py` | no | `UplinkReader`, `status_fields`, `steering_state`, `throttle_state`, `brake_state`, `rc_fields`, `raw_fields`, `calibration_line`, `health_fields` |
| `link_report.py` | no | `LinkReport`: counts every frame written, and when a sample is due; `frame_fields`: a written frame's values, decoded from its bytes |
| `steering_limits.py` | no | `compute`: the steering actuator's limits from its marked ends and centre; the `steering_limits` command. The hardware panel has a Rust twin (panel-core); both test against `test/steering_limits_vectors.json` |

## Invariants

- Reads only `/control/car/actuator_cmd` (the output mux, target car), never
  `/ego/actuator_cmd`.
- A stale command (`cmd_timeout`) or an e-stop sends brake frames: throttle 0, brake full.
- The bridge only relays the Orin's heartbeat counter, never makes one up. The Rust
  ESP32 firmware brakes when it hasn't changed for 100 ms.
- Version 2 and 3 frames and the uplink use CRC-8/SAE-J1850, as `aav.dbc`. Both sides check
  `firmware/spine/crates/legacy-link/vectors.json`.
- With `frame_version: 1` and no override, the lock says true: no program drives
  through firmware with no command timeout. It is sent again at 5 Hz, as twist_mux's
  lock subscription is volatile: a mux started (or restarted) later still hears it.
- ROS's steering is + left; `steering_sign` flips it for the ESP32.
- `encoder_udp` stops publishing after `timeout_s` of silence, rather than say 0 m/s.
- Never run `encoder_udp` where the `spine` part runs: two `/vehicle/odom` publishers
  stop the output mux.
- `/control/car/link_frame` reports the bytes written, not what the relay meant. A failed or
  short write never counts as written. Reporting can't stop the frames. `written` means the
  kernel took the bytes: only the ESP32's own count says they arrived.
- The logic modules have no ROS imports.

## Side Effects

- Writes and reads the serial port (pyserial).
- `encoder_udp` binds a UDP socket on port 10111.
- Drives the real car's throttle, brake and steering through the ESP32.

## Verification

```sh
pixi run build && colcon test --packages-select aav_legacy && colcon test-result --verbose
```

`pixi run test` runs every package's tests.
