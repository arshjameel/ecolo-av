# aav_mux_state

## Overview

**Who is driving?** twist_mux passes on one driver's commands, but never says whose.
`command_mux_state` works it out beside it and says so (TODO 247, the debug screen).

```
/drivers/*/cmd_vel ──┬──▶ twist_mux ──────────▶ /control/cmd_vel ──▶ ego
locks (Bool) ────────┤
                     └──▶ command_mux_state ──▶ /control/command_mux/state ──▶ the map window
                          (the same files, the same rule)
```

It reads twist_mux's own parameter files, in the same order, so it follows the same
inputs, priorities, timeouts and locks. Nothing to keep in step by hand.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/mux-state-core` | `MuxConfig` (the files), `Mux` (the rule), `Choice` (the answer, as a line and as rows) | no: `cargo test` on any laptop |
| `crates/mux-state-node` | the process, binary `command_mux_state` | yes (rclrs 0.8, `ros-env` 0.3) |

### Node `command_mux_state`

Started by `aav_command_mux`'s `mux.launch.py`, beside twist_mux, in every profile.

| In / out | Topic | Type |
|---|---|---|
| in | every input in the files: `/drivers/<driver>/cmd_vel` | `geometry_msgs/TwistStamped` |
| in | every lock in the files: `/safety/estop`, `/safety/sensor_watchdog/lock`... | `std_msgs/Bool` |
| out | `/control/command_mux/state`, 10 Hz and on each change | `diagnostic_msgs/DiagnosticStatus` |

The message:

| Field | Example |
|---|---|
| `level` | OK (0); WARN (1) when nobody drives while a lock holds someone |
| `message` | `keyboard (90)`, `nobody; held: nav by sensors 30 silent`, `nobody sending` |
| value `driver` | `keyboard`, `nav`... or `none` |
| value `priority`, `topic` | `90`, `/drivers/keyboard/cmd_vel` |
| value `age_s` | `0.05`: seconds since its last command |
| value `locks` | `sensors 30 silent, stop_sign 30`, or `none` |
| value `lock_priority` | `30` |
| value `held` | `nav, ai`: sending, but held by a lock; or `none` |

| Parameter | Default | Means |
|---|---|---|
| `config_files` | (required) | twist_mux's parameter files, in its order |
| `rate_hz` | 10 | how often it says it |
| `state_topic` | `/control/command_mux/state` | where |

## The rule (twist_mux 4.5.0)

| Step | Rule |
|---|---|
| 1 | A lock is **on** while its last value was true, or it has been silent past its timeout (0 = never) |
| 2 | The **lock priority** is the highest priority among the locks that are on |
| 3 | An input counts while heard within its timeout (by arrival time; stamps ignored) |
| 4 | Among those not below the lock priority, the **highest priority wins**; a tie goes to the first name |

## Invariants

- It only listens and says: it never commands anything.
- It reads the files twist_mux reads (`mux.launch.py` hands both the same list).
- `aav_command_mux`'s `test_mux_runtime.py` runs it beside a real twist_mux: it must name
  the driver twist_mux passes, and the lock that holds navigation.
- `mux-state-core` has no ROS; no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## Side Effects

- Publishes `/control/command_mux/state`. Nothing else.

## Verification

```sh
cd ros/command_mux/aav_mux_state && cargo test -p mux-state-core   # tools/test.sh runs it
colcon test --packages-select aav_mux_state aav_command_mux        # beside a real twist_mux
ros2 topic echo /control/command_mux/state                         # against a running sim
```
