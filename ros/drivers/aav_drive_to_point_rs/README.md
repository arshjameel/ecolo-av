# aav_drive_to_point_rs

## Overview

`point_driver` and `drive_to_point` in Rust (rclrs): the **default** since 18 Nov, in
place of `aav_drive_to_point`'s Python ones. The same node name, action, topics, QoS and
parameters. The only ones since 22 Nov, when the Python node and client were removed
(tag `save/python-fallbacks-1`); `aav_drive_to_point` keeps the launch file, the config and
the Python maths.

```
pixi run drive-to-point ─/drive_to_point─> point_driver ─/drivers/point/cmd_vel─> twist_mux (25)
                                           speed up, hold, slow by the distance left; stop at B
```

## Interface

| Crate | Is | ROS? |
|---|---|---|
| `crates/point-core` | the maths and rules: `approach` (the speed), `line` (pure pursuit, the corridor), `drive` (one tick), `rules` (refusals, stops), `run` (one drive, plan to result), `cli` (the command's line and words) | no: `cargo test` |
| `crates/point-node` | binaries `point_driver` (the action server) and `drive_to_point` (its command-line client) | yes (rclrs 0.8) |

The node, its parameters and rules: `aav_drive_to_point`'s README. `drive_to_point.launch.py`
starts it (`drive_to_point_impl:=rust`, the default); `sim.launch.py`, `bench.launch.py` and
`car.launch.py` pass the switch on (`rust` only: a challenger's hook).
`tools/drive_to_point.sh` runs this client.

## Invariants

- **The same answers as the Python**: `tests/vectors.rs` reads `vectors.json`, which
  `gen_vectors.py` writes from `approach.py`, `line.py`, `drive.py`, `rules.py` and
  `run.py` (whole drives: to B, an e-stop, a cancel, a stuck car, speeding, a GPS jump,
  the stopping law, someone else driving, a stale pose, a turn round). Numbers to 1e-12,
  words exactly. `aav_drive_to_point`'s `test_rust_vectors.py` checks the Python still
  gives the file. Change one, change both, then
  `pixi run python ros/drivers/aav_drive_to_point_rs/gen_vectors.py`.
- TODO 94 (it stops 0.61 m past B once a run) is kept as it is: a port, not a fix.
- Reasons are enums (`Stop`, `Refusal`, `Halt`, `Breach`) whose words are the Python's.
- `point-core` has no ROS; `cargo fmt` and `cargo clippy -D warnings` clean.

## Side Effects

- Publishes `/drivers/point/cmd_vel` while a goal runs, `/drivers/point/plan` (latched).
- Calls `/fromLL` and `/ego/get_parameters`. No files, no hardware.

## Verification

```sh
cd ros/drivers/aav_drive_to_point_rs && cargo test -p point-core   # tools/test.sh runs it
```

The process: `pixi run sim-check --drive-to-point` (Ravensim, drives at 1 and 3 m/s).
