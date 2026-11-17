# aav_stop_zone

## Overview

The stop zone, a **safety lock**, in Rust: Nav2's `collision_monitor` stop decision ported
with no ROS (TODO 182: "ours in Rust with its own tests first"): the decision
(`crates/stop-zone`) and a node (`crates/stop-zone-node`, `stop_zone`). **The car's stop
zone since 17 Nov**, in every stack and on the car: it sits between the smoother and
twist_mux (`/control/cmd_vel_smoothed` -> `/drivers/nav/cmd_vel`), with the settings and
sensors of the tuned `nav2.yaml`'s `collision_monitor` block (`vehicle.stop_zone_params`).

It ran in shadow first, beside Nav2's `collision_monitor` on every obstacle check, and
stopped every time Nav2's did, no later
([Nav2, as it was](../../../docs/history/nav2-as-it-was.md#the-swap-of-the-stop-zone)).
Then it took the lock, and Nav2 went (TODO 184).

```text
 command ─▶ the box for its direction ─▶ points in it (all sensors) ≥ min_points ─▶ zero
 each sensor's newest reading older than source_timeout (or none) ──────────────▶ zero
 after a stop: zeros for stop_pub_timeout s, then nothing (twist_mux's slot times out)
```

| Setting (nav2.yaml `collision_monitor`) | Ours |
|---|---|
| `velocity_polygons` forward / backward | `SpeedBand`s, the first that holds the command; none: the last |
| `min_points` (3) | `Settings::min_points`, all sensors' points together |
| `source_timeout` (1 s) | `Settings::source_timeout`: a silent sensor stops the car |
| `stop_pub_timeout` (2 s) | `Settings::stop_pub_timeout` |
| a source's `min_height`, `max_height`, `min_range` | `SourceKind::Cloud`, `Source::min_range` (scans: none) |

## Interface

| Item | What |
|---|---|
| `StopZone::new(Settings, sources)` | `SettingsError` for no box, a box under 3 corners, min_points 0 |
| `StopZone::step(now, Command, readings)` | `Decision { out, stop, band }`: the command to send (or none), why it stopped |
| `Polygon::contains` | the ray-crossing point test |

The node: `cmd_in`, `cmd_out`, `state` (`nav2_msgs/CollisionMonitorState`, on each change,
as collision_monitor's: `stop_hold` reads it), `status` (`DiagnosticStatus`, each command),
`box` (`PolygonStamped`, the box in use). Its defaults are the shadow's topics, so a stop
zone started by hand drives nothing; `navigation.launch.py` passes the live ones. Not yet: mounts from
`/tf_static` (the launch gives the radar's from vehicle.yaml; the rest send base_link);
the `slowdown`, `limit` and `approach` actions (our config uses none); sensors with roll
or pitch.

## Testing

`cargo test` here (`tools/test.sh` runs it, with `cargo fmt` and `cargo clippy`). `crates/stop-zone/tests/props.rs` checks the lock's rules on thousands of random points and commands (proptest): it only passes or stops, 3 points in the box always stop, points outside never do, more points never clear a stop, a silent sensor always stops, NaN points are dropped.
