# aav_safety_rs

## Overview

Three of `aav_safety`'s locks in Rust (rclrs): **the defaults since 19 Nov** (Ben, 16 Nov:
"lets remove python were possible. I trust rust more then python"). The same node names,
topics, QoS, parameters and answers as the Python; each is one switch away.

| Node | Lock | Switch |
|---|---|---|
| `geofence` | twist_mux's geofence lock (51): every program held before its stopping paths reach the fence | `geofence_impl:=rust` |
| `stop_hold` | twist_mux's stop_zone lock (21): a stop zone's stop held until the way stays clear | `stop_hold_impl:=rust` |
| `goal_leash` | a laptop driver's goals cancelled when its beats stop | `goal_leash_impl:=rust` |

What each one does, its states and its rules: `aav_safety`'s README and
`docs/geofence.md`.

## Interface

### Crates

| Crate | Is | ROS? |
|---|---|---|
| `crates/safety-core` | `geofence` (`Geofence`, `State`, `Circle`, `Polygon`, `Fence`, `site_fence`, `Motion`, `Settings`, `Pose`, `Mode`, `stopping_distance`, `along_arc`), `hold` (`StopHold`), `leash` (`Leash`), `pyfmt` (numbers as Python writes and sums them, Python's `hypot`) | no: `cargo test` on any laptop |
| `crates/safety-node` | the three processes: binaries `geofence`, `stop_hold`, `goal_leash` | yes (rclrs 0.8, `ros-env` 0.3) |

### The nodes

Topics, services and parameters: the Python's, name for name (`aav_safety`'s README).
One more for the geofence: `vehicle_yaml` (`''`: `aav_description`'s, found on
`AMENT_PREFIX_PATH`), for the car's corners (`vehicle.footprint_points`).

### Which one runs

| Switch | Where it is read | Default |
|---|---|---|
| `geofence_impl` | `aav_safety` `geofence.launch.py`; car, sim and bench pass it on | `rust` |
| `stop_hold_impl` | `aav_navigation` `navigation.launch.py`; car, sim and bench pass it on | `rust` |
| `goal_leash_impl` | `car.launch.py`, `sim.launch.py`, `bench.launch.py` | `rust` |

`vehicle.SAFETY_IMPLS` (`GEOFENCE_IMPLS`, `STOP_HOLD_IMPLS`, `GOAL_LEASH_IMPLS`) and
`DEFAULT_*_IMPL`. The geofence and stop_hold Nodes carry `namespace='/'`, so launch's
values beat the YAML in Rust too (`docs/languages.md`).

## Invariants

- **The same answers as the Python**, checked by `tests/vectors.rs` against
  `vectors.json`, which `gen_vectors.py` writes from `geofence.py`, `hold.py` and
  `leash.py`:

| Vectors | Count | Matched |
|---|---|---|
| geofence runs: poses, fixes, checks, clears; 5 fences (a concave one), 3 motions, 2 settings, 4 modes (gps:=false, coasting, stands) | 17 runs, 17,143 events, every state | lock, state and line, letter for letter |
| its maths: depth, outlines, stopping distances, arcs, the stopping paths' corners, `hypot` (zeros, subnormals, the largest float, inf, NaN) | 600 depths, 400 stops, 400 arcs, 2,292 corners, 469 `hypot`s | **to the bit** |
| the fences, settings and motions refused | 34 | the Python's error lines |
| stop hold: 4 `clear_time`s; refusals (NaN, inf, negative) | 8,419 events | every answer |
| leash: 4 timeouts | 8,982 events | every trip and armed |

  `aav_safety`'s `test_safety_rs_vectors.py` fails if the Python drifts from the file.
- To match to the bit, `safety-core` writes each sum in the Python's order, sums the
  polygon's area as Python 3.12's `sum()` does (Neumaier), and uses CPython's own
  `math.hypot` (`pyfmt::hypot`), which differs from the C library's in the last bit.
- Only ever stops: none of the three makes a command. The geofence's clear refuses while
  the car is outside; the leash only cancels.
- `safety-core` has no ROS; no `unwrap` in library code; `cargo fmt` and
  `cargo clippy -D warnings` clean. `Cargo.lock` committed; builds use `--locked`.

## Where it differs from the Python

| Python | Rust |
|---|---|
| `repr` of a datum in the site's error line: exponent form below 1e-4 | plain decimals (no latitude or longitude reaches it) |
| the geofence reads the footprint through `aav_description.vehicle` | reads `vehicle.yaml`'s `vehicle.body` itself (`vehicle_yaml`) |

## Side Effects

- Publishes the locks and states; the geofence its fence (transient local), and serves
  `/safety/geofence/clear`; the leash calls each action's `_action/cancel_goal`. Logs each
  change of state. Stops cleanly on SIGINT, SIGTERM or SIGHUP.

## Verification

```sh
cd ros/safety/aav_safety_rs && cargo test --locked -p safety-core   # no ROS; tools/test.sh
pixi run python ros/safety/aav_safety_rs/gen_vectors.py              # after changing the Python
pixi run build && colcon test --packages-select aav_safety_rs && colcon test-result --verbose
```

| Test | Covers |
|---|---|
| `safety-core` unit tests | stopping distances, depths, latching, the hold, the leash, Python's numbers |
| `tests/vectors.rs` | the Python's answers, event by event, bit by bit |
| `test/test_safety_rs_nodes.py` | the processes: the fence arrives; OK, LOST (fixes on a remapped topic stop), OUTSIDE latched, a clear refused outside and taken inside; a held stop let go after `clear_time`; a quiet laptop's goals cancelled once |
