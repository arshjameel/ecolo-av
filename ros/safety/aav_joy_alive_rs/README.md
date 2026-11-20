# aav_joy_alive_rs

## Overview

`joy_alive` in Rust (rclrs): **the default since 17 Nov** (Ben: "I don't think we want
to do this again later"), in place of `aav_safety`'s Python node. The same node name,
topics, parameters and answers. The Python was removed on 20 Nov (tag `save/python-safety-nodes`).

```
joy_node ─► /drivers/joy/joy ─► joy_alive ─► /drivers/joy/joy_checked ─► teleop_twist_joy
                                    └──────► /drivers/joy/alive  (OK, WAITING, SILENT, FROZEN)
```

It stops a joystick that goes silent (0.5 s) or freezes while driving (2 s), until the
deadman is released: joy_node repeats the last state while the receiver is in, so a pad
turned off could keep driving (TODO 74a). The rules: `aav_safety`'s README.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/joy-alive-core` | `JoyAlive` (`on_joy`, `tick`, `state`), `Settings`, `Trip` (`Waiting`, `Silent`, `Frozen`), `SettingsError`, `g` (Python's `:g`) | no: `cargo test` on any laptop |
| `crates/joy-alive-node` | the process, binary `joy_alive` | yes (rclrs 0.8, `ros-env` 0.3) |

### Node `joy_alive`

In the namespace `/drivers/joy` (`aav_teleop` `teleop.launch.py`).

| In / out | Topic | Type |
|---|---|---|
| in | `/drivers/joy/joy` | `sensor_msgs/Joy` |
| out | `/drivers/joy/joy_checked` | `sensor_msgs/Joy`: each message, or a released copy (all 0) while stopped; once more when silence trips it |
| out | `/drivers/joy/alive`, at `rate_hz` | `std_msgs/String`: `STATE - why` |

Parameters (`aav_teleop` `config/joy_alive.yaml`): `enable_button` 0, `motion_axes` [1],
`motion_threshold` 0.05, `silent_after` 0.5 s, `frozen_after` 2.0 s (0 off), `rate_hz` 10.

### Which one runs

`teleop.launch.py joy_alive_impl:=rust` (`vehicle.JOY_ALIVE_IMPLS`, default
`rust`); `car.launch.py` passes it on. Not in `stacks.yaml`: the same algorithm in
another language.

## Invariants

- The same answers as `joy_alive.py`: `tests/vectors.rs` reads `vectors.json`, which
  `gen_vectors.py` writes from it (6,982 events: Joy messages and ticks, four settings;
  every answer, trip, size and line the same). `aav_safety`'s `test_joy_alive_vectors.py`
  fails if the Python's answers drift from the file.
- Only ever releases the joystick; never makes a command of its own.
- `joy-alive-core` has no ROS; no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## Where it differs from the Python

| Python | Rust |
|---|---|
| a negative number in `motion_axes` reads from the end of the axes | refused at start-up: not an axis number |

## Side Effects

- Publishes the two topics above; logs each change of state (OK as info, the rest as
  errors). Stops cleanly on SIGINT, SIGTERM or SIGHUP.

## Verification

```sh
cd ros/safety/aav_joy_alive_rs && cargo test --locked -p joy-alive-core    # no ROS; tools/test.sh
python3 ros/safety/aav_joy_alive_rs/gen_vectors.py                         # after changing joy_alive.py
pixi run build && colcon test --packages-select aav_joy_alive_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `joy-alive-core` unit tests | waiting, frozen, silence, bad settings, `g` |
| `tests/vectors.rs` | the Python's answers, event by event |
| `aav_teleop`'s `test/test_joy_chain.py` | the chain, both languages: a pretend pad → joy_alive → teleop_twist_joy → twist_mux → ego. A live hand drives; frozen stops (and stays stopped with the trigger held); released and pressed drives again; silent stops |
