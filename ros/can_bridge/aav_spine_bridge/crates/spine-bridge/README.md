# spine-bridge

## Overview

`spine_bridge`'s logic with no ROS: what the Orin sends the spine over CAN, and what it
makes of the frames it hears. The ROS node (`bridge-node`) only turns messages into
calls here, and `Update`s back into messages. A port of `aav_spine`'s
`spine_bridge.py`, `wheel_speed.py` and `output_gate.py`; its tests carry their cases.

## Interface

### `Bridge`

| Item | Does |
|---|---|
| `Bridge::new(Settings)` | a bridge: role, geometry, timeouts, target |
| `command(Command, now)` | note ego's command (m/s, rad + left, m/s², rad/s) |
| `drive(now) -> Drive` | at each 50 Hz tick: `DRIVE_CMD` to send, or nothing (never commanded, or stale) |
| `arm(bool, now) -> Result<Arming, ArmError>` | `/spine/arm`: refused during an e-stop, while the gate is shut, or (with the ready gate) until the boot self-test says READY |
| `with_ready_gate(timeout)`, `self_test(ready, message, at)` | the car's ready gate (TODO 255): one `/safety/self_test/status` |
| `set_estop(bool) -> Option<EstopCmd>` | `ESTOP_CMD` at once, on a change |
| `estop_frame() -> EstopCmd` | the 10 Hz `ESTOP_CMD` heartbeat |
| `output_mux_state(line, at)` | one line from `/safety/output_mux/state` |
| `receive(id, data, now) -> Result<Option<Update>, BadFrame>` | `VEHICLE_FEEDBACK`, `VEHICLE_STATUS`, `WHEEL_SPEED_L/R` into motion, status or wheels; other ids are `Ok(None)` |
| `bad_frames()` | frames that failed their checks, so far |

### Types

| Type | Holds |
|---|---|
| `Settings`, `Geometry` | the parameters; `Geometry::yaw_rate`, `wheel_angles` (Ackermann) |
| `Role`, `Role::NAMES`, `UnknownRole` | `Spine` (drives) or `Sensors` (listens to the wheels, never sends); the `role` parameter's names; a name that is neither |
| `Command`, `Drive`, `Arming`, `ArmError` | commands in, frames and answers out |
| `Motion`, `Status`, `Update`, `BadFrame` | what the frames say, in ROS's units |
| `OutputGate`, `Target`, `UnknownTarget`, `GateShut`, `GATE_SHUT` | the safety gate (`gate.rs`); `UnknownTarget`: not car, bench or sim |
| `ReadyGate`, `NotReady`, `NOT_READY` | the boot self-test's word (`ready.rs`); the same words as `output_gate.py`'s |
| `wheels::FrontWheels`, `Reading`, `Side`, `Direction`, `rear_axle_speed` | the front wheel nodes (`wheels.rs`); `Side`: one wheel in a `Reading` |

`UnknownRole`, `UnknownTarget`, `Side`, `Reading` and `Direction` are also at the crate
root. It re-exports `spine_core::protocol`'s `DriveCmd`, `EstopCmd`, `Mode`,
`DRIVE_CMD_ID` and `ESTOP_CMD_ID` (only the two ids it sends).

## Invariants

- No ROS. The frames are `firmware/can/aav.dbc`, packed by `spine-core`
  (`../../../../../firmware/spine/crates/spine-core`).
- `drive` returns a frame only while the last command is younger than `cmd_timeout`.
- Arming sends `armed=0` frames before `armed=1`: the spine arms on the edge.
- `arm(true)` needs the gate OPEN for this `Target`, heard within `gate_timeout`, on a
  steady clock (`Instant`). Disarming always works.
- Refusals start with `GATE_SHUT`, the same words as `output_gate.py`.
- Motion speed: the front wheels' mean while both are fresh, else the spine's
  `measured_speed`.
- `Role::Sensors` never produces a frame to send.
- No `unwrap` in library code.

## Side Effects

None: pure logic. The tests read `firmware/can/test_vectors.json`.

## Verification

```sh
cd ros/can_bridge/aav_spine_bridge && cargo test --locked -p spine-bridge
```

The tests: `tests/bridge.rs` (the Python node's cases), `tests/gate.rs`, `tests/ready.rs`, and
`tests/vectors.rs` (the shared CAN vectors through the whole bridge).
