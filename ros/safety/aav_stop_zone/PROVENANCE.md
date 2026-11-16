# Where aav_stop_zone came from

Ported, not copied: Nav2's `nav2_collision_monitor` (Apache-2.0, Samsung R&D Institute
Russia), https://github.com/ros-planning/navigation2.git at
645abd95f2be02a13ca539b29c2ddc065db34f89 (vendor/ros2-src, Jazzy), rewritten in Rust.

| Ours (`crates/stop-zone/src/lib.rs`) | Nav2's |
|---|---|
| `StopZone::step` | `CollisionMonitor::process`, the `stop` action only |
| `StopZone::publish` | `CollisionMonitor::publishVelocity` |
| `SpeedBand::holds` | `VelocityPolygon::isInRange`, not holonomic |
| `Polygon::contains` | `Polygon::isPointInside` (Shimrat 1962) |
| `in_base` | `PointCloud::getData`, `Scan::getData` (roll and pitch left out) |
| `StopZone::silent` | `Source::sourceValid` |

## Changes since

- 15 Nov 2026 (Ben): ported; the decision only, with its own tests (TODO 182).
- 2026-11-15 (TODO 176): the Rust API Guidelines pass: `#[must_use]` on pure functions, doc comments, `Debug`, and the lints that keep them; no change in behaviour.
