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
In Gazebo: `pixi run sim esp32:=true`, the C++ firmware standing in (`aav_gazebo`). The design: `docs/designs/2026-10-27_control-plan.md`, step 1.

### The no-timeout lock

| `frame_version` | `allow_programs_without_timeout` | `/safety/legacy_uart_bridge/lock` | Who drives |
|---|---|---|---|
| 1 (the C++, no timeout) | false (the default) | **true** | people (joy, RC, keyboard) and the actuator tests (60) only |
| 1 | true: **supervised stand only**, wheels off the ground | false | everyone |
| 2 (the Rust, brakes after 100 ms) | either | false | everyone |

twist_mux's `no_timeout` lock (priority 60, timeout 0) reads it. It holds every input
below 60: scripted, replay, point, Nav2, AI. Timeout 0, so where no legacy bridge runs
it never locks.

## Interface

### Executables

| Executable | Runs on | Does |
|---|---|---|
| `legacy_uart_bridge` | the Pi wired to the ESP32 (`/dev/ttyAMA0`) | commands to serial frames; with `uplink`, the ESP32's frames back |
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
| pub, `uplink` only | `/vehicle/status` (10 Hz) | aav_interfaces/VehicleStatus |
| pub, `uplink` only | `/vehicle/steering`, `/vehicle/brake` | aav_interfaces/ActuatorState |
| pub, `uplink` only | `/vehicle/rc` | aav_interfaces/RcSticks |

`encoder_udp` publishes `/vehicle/odom` (nav_msgs/Odometry, forward speed only, in
`base_link`).

### Parameters (`config/legacy.yaml`)

| Node | Name | Value | Unit |
|---|---|---|---|
| bridge | `port`, `baud` | `/dev/ttyAMA0`, 115200 | |
| bridge | `rate_hz`, `cmd_timeout` | 50.0, 0.3 | Hz, s |
| bridge | `frame_version` | 1 today (C++ firmware); 2 for the Rust firmware | |
| bridge | `allow_programs_without_timeout` | false; true only on a supervised stand | |
| bridge | `uplink`, `uplink_timeout` | false, 0.2 | s |
| bridge | `max_steering_angle` | from `vehicle.yaml` | rad |
| bridge | `steering_sign` | -1.0 (TODO check) | |
| bridge | `full_throttle_speed`, `stop_speed` | 3.0, 0.05 | m/s |
| bridge | `hold_brake`, `stop_brake` | 0.3, 1.0 | 0..1 |
| bridge | `speed_loop.mode` | `open` (the 2025-26 mapping; the car's, for now) or `pi` | |
| bridge | `speed_loop.kp`, `speed_loop.ki` | 2.0, 0.5 (guesses) | m/s² per m/s, per m/s·s |
| bridge | `speed_loop.max_accel`, `max_brake`, `drag` | from `vehicle.yaml` `actuators:` | m/s², m/s², 1/s |
| bridge | `speed_loop.response_lag` | 0.3 (guess): how far behind ego the car is expected to run | s |
| bridge | `speed_loop.hold_brake`, `speed_loop.odom_timeout` | 0.2, 0.3 | 0..1, s |
| bridge | `speed_loop.smooth_stop_speed`, `smooth_stop_rate`, `weak_stop_decel`, `weak_stop_time`, `strong_stop_decel`, `stopped_brake`, `stopped_brake_rate` | 0.3 (0: off), 1.5, 0.2, 2.0, 1.0, 0.5, 1.0 (guesses) | m/s, 1/s, m/s², s, m/s², 0..1, per s |
| encoder | `port`, `bind_address` | 10111, 0.0.0.0 | |
| encoder | `wheel_diameter`, `counts_per_rev` | from `vehicle.yaml` | m, counts |
| encoder | `window_s`, `timeout_s` | 0.5, 0.3 | s |
| encoder | `speed_variance` (on `/vehicle/odom`) | 0.05 | (m/s)² |

### Launch

`legacy.launch.py [uart:=true] [encoder:=true] [rc:=true]`: the bridge, the encoder,
and `aav_rc`'s `rc_driver` beside the bridge.

### Modules

| Module | ROS? | Holds |
|---|---|---|
| `legacy_uart_bridge.py`, `encoder_udp.py` | yes | the nodes |
| `legacy_protocol.py` | no | the frames: `pack_frame` (v1, 13 bytes), `pack_frame_v2` (17 bytes), `unpack_uplink` (35 bytes), `UplinkParser`, `to_legacy`, `from_pedals`, `stop_command`, `EncoderSpeed` |
| `uart_relay.py` | no | `Relay`: what to write each tick; `programs_locked`: the no-timeout lock |
| `speed_control.py` | no | `SpeedControl`: feed-forward from ego's speed profile plus PI on the wheel speed, out as throttle or brake; the smooth stop |
| `uplink_reader.py` | no | `UplinkReader`, `status_fields`, `steering_state`, `brake_state`, `rc_fields` |

## Invariants

- Reads only `/control/car/actuator_cmd` (the output mux, target car), never
  `/ego/actuator_cmd`.
- A stale command (`cmd_timeout`) or an e-stop sends brake frames: throttle 0, brake full.
- The bridge only relays the Orin's heartbeat counter, never makes one up. The Rust
  ESP32 firmware brakes when it hasn't changed for 100 ms.
- Version 2 frames and the uplink use CRC-8/SAE-J1850, as `aav.dbc`. Both sides check
  `firmware/spine/crates/legacy-link/vectors.json`.
- With `frame_version: 1` and no override, the lock says true: no program drives
  through firmware with no command timeout. It is sent again at 5 Hz, as twist_mux's
  lock subscription is volatile: a mux started (or restarted) later still hears it.
- ROS's steering is + left; `steering_sign` flips it for the ESP32.
- `encoder_udp` stops publishing after `timeout_s` of silence, rather than say 0 m/s.
- Never run `encoder_udp` where the `spine` part runs: two `/vehicle/odom` publishers
  stop the output mux.
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
