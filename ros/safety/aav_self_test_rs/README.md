# aav_self_test_rs

## Overview

The **boot self-test** (TODO 255). Ben, 19 Nov: "Make sure we have control over
everything with a basic boot self test." One node, `boot_self_test`, next to the mux on
the car: is every part that keeps the car safe here and answering? It says **READY** or
**NOT READY, and why**. `spine_bridge` refuses `/spine/arm` until READY.

```text
 sensor watchdog's line ─┐
 each lock's line ───────┤
 the stop zone (graph) ──┼─▶ boot_self_test ─▶ /safety/self_test/status ─▶ spine_bridge:
 /vehicle/status or      │                     READY / NOT READY + why       /spine/arm
   the UART link ────────┤                                                   refused until READY
 stamped sensors ────────┘
```

It only reads. It moves nothing, and never stands between a person and the e-stop.

## Interface

### Node `boot_self_test`

| Direction | Topic | Type |
|---|---|---|
| sub | `/safety/sensor_watchdog/state`, `/safety/geofence/state`, `/safety/output_mux/state` | std_msgs/String |
| sub | `/vehicle/status` | aav_interfaces/VehicleStatus |
| sub | `/control/car/link_frame` | aav_interfaces/LinkFrame |
| sub (each in `clocks`) | `/sensors/imu/data`, `/sensors/gps/fix`, `/vehicle/odom` | Imu, NavSatFix, Odometry |
| graph | a publisher on `/safety/stop_zone/state` | |
| pub | `/safety/self_test/status` (2 Hz) | diagnostic_msgs/DiagnosticStatus |

| Check | Passes when |
|---|---|
| `sensors` | the sensor watchdog spoke within 1 s and doesn't say STOP: every sensor at its rate, by its own rules |
| `actuators` | `status`: `/vehicle/status` within 0.5 s, no steering, brake, throttle or CAN driver fault. `link`: the UART bridge writes frames. `none`: not checked |
| `watchdog`, `geofence`, `output_gate` | its line within 1 s: alive (whatever it says, it holds what it must) |
| `stop_zone` | a publisher on its state (it speaks only on commands) |
| `clock imu`, `gps`, `odom` | a stamp within 0.5 s of this computer's clock |

READY once every check has passed for `settle` s (2) without a break. Any failure: NOT
READY at once.

| Parameter | Default |
|---|---|
| `actuators` | `status` (`link`, `none`) |
| `locks` | `[watchdog, geofence, output_gate]` (and `stop_zone`) |
| `clocks` | `[]` (`imu`, `gps`, `odom`) |
| `settle`, `line_timeout`, `status_timeout`, `max_clock_offset` | 2.0, 1.0, 0.5, 0.5 s |
| `rate_hz` | 2.0 |

### Launch

`self_test.launch.py [self_test_actuators:=status] [self_test_locks:=watchdog,geofence,output_gate] [self_test_clocks:=]`.
`car.launch.py` starts it wherever the mux runs:

| Car | `actuators` | locks | clocks |
|---|---|---|---|
| `actuators:=uart` (today) | `link` | watchdog, geofence, output gate, the stop zone where navigation runs | imu, odom, gps |
| `actuators:=can` | `status` | the same | the same |

**Per profile:** only the car runs it. Ravensim and the bench don't, and their
`spine_bridge` doesn't wait for it (`require_ready:=false`; `arm_when_open` arms them).

### Crates

| Crate | Holds | ROS? |
|---|---|---|
| `crates/self-test-core` | `SelfTest`, `Config`, `Verdict`, `Lock`, `Actuators`, `ClockSource` | no: `cargo test` |
| `crates/self-test-node` | the node, binary `boot_self_test` | yes (rclrs 0.8) |

## Invariants

- It only reads; it never publishes a command, a lock or an e-stop.
- Silence is failure: a quiet input is NOT READY, and `spine_bridge` treats a quiet
  self-test as NOT READY.
- **What it can't see (untested on the car):** with `actuators:=uart` and the uplink off,
  nothing arms (the UART bridge has no arm), so READY is for people (card 1); and
  `link` only says frames go out, not that the ESP32 heard them. The spine's own
  self-test passes after one tick (TODO 0b, Arsh's firmware): a fault-free status says
  only that no driver reported a fault yet.
- Rust API Guidelines; `cargo fmt` and `cargo clippy` clean.

## Side Effects

- Publishes `/safety/self_test/status`. Reads the ROS graph.

## Verification

```sh
cd ros/safety/aav_self_test_rs && cargo test -p self-test-core
```

`colcon test --packages-select aav_self_test_rs` also starts the node: NOT READY, READY
as its inputs speak, NOT READY when one goes quiet.
